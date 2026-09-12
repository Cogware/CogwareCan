// SPDX-License-Identifier: GPL-3.0-only
//! Global bus state and the node-to-master message channel.
//!
//! The master broadcasts one `BusState` frame carrying the mode every node
//! should be in and what the bus is currently busy with. Nodes talk back on
//! their own ID — a node's message ID *is* its node address — so two nodes
//! never contend for one ID and arbitration orders them by address.
//!
//! Master side:
//!
//! ```ignore
//! if let Some(msg) = bus.feed(&frame) { /* an error or an offer to act on */ }
//! if let Some(f) = bus.announce() { can.send_message(f).ok(); }  // state changed
//! ```
//!
//! Node side:
//!
//! ```ignore
//! view.feed(&frame);
//! if view.update_in_progress() { draw_update_screen(view.progress()); }
//! if button_pressed { can.send_message(view.request_mode(MODE_SPORT)?).ok(); }
//! // A tell-tale on one of this node's own pins. It is not lit here: the
//! // master folds it into the bank and it comes back on the next broadcast.
//! if stalk_changed { can.send_message(view.report_lamp(Indicator::TurnLeft, on)?).ok(); }
//! ```

use crate::indicators::{self, Indicator};
use crate::protocol::{frame, id_of, msg_node, node_msg_id, BUS_STATE_ID, NODE_BROADCAST};
use embedded_hal_0_2::can::Frame;
use mcp2515::frame::CanFrame;

/// The mode every node starts in and falls back to.
pub const MODE_NORMAL: u8 = 0;

/// A file transfer is running; nodes should show it and stay quiet.
pub const FLAG_UPDATE: u8 = 1 << 0;
/// The master has a live ECU link, so an unset gauge means "not supplied".
pub const FLAG_ECU_LINK: u8 = 1 << 1;
/// The master is refusing mode changes, usually because an update is running.
pub const FLAG_MODE_LOCKED: u8 = 1 << 2;

/// Payload length of a `BusState` frame.
pub const STATE_LEN: usize = 4;

/// What a node is telling the master. The opcode is the first payload byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Opcode {
    /// `[1, mode]` — asks the master to move the whole bus to `mode`.
    ModeRequest = 1,
    /// `[2, code]` — this node has hit an error worth showing elsewhere.
    Error = 2,
    /// `[3, kind, size u32 LE]` — this node holds an image the master may pull.
    UpdateOffer = 3,
    /// `[4]` — this node has just booted and wants the state broadcast again.
    Hello = 4,
    /// `[5, lamp..]` — a tell-tale wired to an input on this node has changed.
    ///
    /// A display with the turn stalk on a spare pin reports it here; the
    /// master folds it into `INDICATORS1`/`INDICATORS2` and broadcasts those
    /// as it broadcasts any gauge, so every screen lights the lamp together
    /// and none of them has to be the one that decided. See [`pack_lamp`] for
    /// the byte, and up to seven of them fit one frame.
    Input = 5,
}

impl Opcode {
    /// The opcode `v` names, or `None` if this build does not know it.
    pub const fn from_u8(v: u8) -> Option<Opcode> {
        Some(match v {
            1 => Opcode::ModeRequest,
            2 => Opcode::Error,
            3 => Opcode::UpdateOffer,
            4 => Opcode::Hello,
            5 => Opcode::Input,
            _ => return None,
        })
    }
}

/// The most tell-tales one `Input` message carries.
///
/// Seven, because a node message has seven bytes after its opcode and a lamp
/// packs into one. A display with a full loom reports the lot in a single
/// frame, which matters more than the byte it saves: a dash that arrives in
/// two halves is a dash that is briefly wrong.
pub const LAMPS_PER_MSG: usize = 7;

/// A tell-tale and its state, as one byte of an `Input` message.
///
/// `bit 7` is the state, `bit 5` the bank (0 for `INDICATORS1`), and `bits
/// 0..=4` the bit within it. Bit 6 is spare and written zero.
///
/// Packed rather than sent as a `[bank, bit, on]` triple so that seven fit a
/// frame instead of two. The layout is fixed by the shape of the table -- two
/// banks of thirty-two -- and `indicators` is where that is decided, so it
/// cannot drift without this failing to build.
pub const fn pack_lamp(ind: Indicator, on: bool) -> u8 {
    // The table's banks are 1 and 2, and one bit distinguishes them.
    let bank = (ind.bank() - 1) & 0x01;
    (on as u8) << 7 | bank << 5 | (ind.bit() & 0x1F)
}

/// Read a byte [`pack_lamp`] wrote.
///
/// `None` for a lamp this build's table does not have, which is what a display
/// running older firmware sees when a newer master reports one that was added
/// after it shipped. Skipped rather than guessed: bits are never renumbered,
/// so an unknown one is genuinely unknown and lighting the wrong lamp would be
/// worse than lighting none.
///
/// A byte with the spare bit set is refused for the same reason. It cannot
/// have come from this build's `pack_lamp`, and if that bit is ever spent --
/// on a third bank, most likely -- reading it as though it were not would
/// light a bank-one lamp for a bank-three one. Refusing now is what makes
/// spending it later safe.
pub const fn unpack_lamp(byte: u8) -> Option<(Indicator, bool)> {
    if byte & SPARE_BIT != 0 {
        return None;
    }
    let on = byte & 0x80 != 0;
    let bank = ((byte >> 5) & 0x01) + 1;
    match Indicator::from_bank_bit(bank, byte & 0x1F) {
        Some(ind) => Some((ind, on)),
        None => None,
    }
}

/// The bit of a packed lamp that nothing has spent yet.
pub const SPARE_BIT: u8 = 1 << 6;

/// One message from a node, with the sender taken from the frame's ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeMsg {
    /// Node that sent it, which is also the CAN ID the frame arrived on.
    pub node: u8,
    /// What the node is saying.
    pub opcode: Opcode,
    args: [u8; 7],
    len: u8,
}

impl NodeMsg {
    /// Parse a node message, or `None` if this frame is not one.
    pub fn parse(f: &CanFrame) -> Option<NodeMsg> {
        let node = msg_node(id_of(f)?)?;
        let data = &f.data()[..f.dlc()];
        let opcode = Opcode::from_u8(*data.first()?)?;
        let mut args = [0u8; 7];
        let len = (data.len() - 1).min(7);
        args[..len].copy_from_slice(&data[1..1 + len]);
        Some(NodeMsg { node, opcode, args, len: len as u8 })
    }

    /// Everything after the opcode byte.
    pub fn args(&self) -> &[u8] {
        &self.args[..self.len as usize]
    }

    /// The mode asked for, if this is a `ModeRequest`.
    pub fn mode(&self) -> Option<u8> {
        if self.opcode != Opcode::ModeRequest {
            return None;
        }
        self.args().first().copied()
    }

    /// The error code, if this is an `Error`.
    pub fn error(&self) -> Option<u8> {
        if self.opcode != Opcode::Error {
            return None;
        }
        self.args().first().copied()
    }

    /// The image kind and size, if this is an `UpdateOffer`.
    pub fn offer(&self) -> Option<(u8, u32)> {
        let a = self.args();
        if self.opcode != Opcode::UpdateOffer || a.len() < 5 {
            return None;
        }
        Some((a[0], u32::from_le_bytes([a[1], a[2], a[3], a[4]])))
    }

    /// The tell-tales this message reports, if it is an `Input`.
    ///
    /// Empty for any other opcode, and a byte naming a lamp this build does
    /// not have is skipped rather than guessed at.
    pub fn lamps(&self) -> impl Iterator<Item = (Indicator, bool)> + '_ {
        let carried = if self.opcode == Opcode::Input {
            self.args()
        } else {
            &[]
        };
        carried.iter().filter_map(|b| unpack_lamp(*b))
    }
}

/// Build the frame `node` sends to say `opcode` with `args`.
/// `None` if `node` cannot own a message ID or `args` is over seven bytes.
pub fn message(node: u8, opcode: Opcode, args: &[u8]) -> Option<CanFrame> {
    let id = node_msg_id(node)?;
    if args.len() > 7 {
        return None;
    }
    let mut buf = [0u8; 8];
    buf[0] = opcode as u8;
    buf[1..1 + args.len()].copy_from_slice(args);
    frame(id, &buf[..1 + args.len()])
}

/// What the master says the bus is doing. Travels as `[mode, flags, progress, target]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct BusState {
    /// Scene mode every node should be in; 0 is `MODE_NORMAL`.
    pub mode: u8,
    /// `FLAG_UPDATE`, `FLAG_ECU_LINK`, `FLAG_MODE_LOCKED`.
    pub flags: u8,
    /// Percent of the running transfer, 0 when none is running.
    pub progress: u8,
    /// Node the running transfer is addressed to, or `NODE_BROADCAST`.
    pub target: u8,
}

impl BusState {
    /// Read a state frame, or `None` if this frame is not one.
    pub fn parse(f: &CanFrame) -> Option<BusState> {
        if id_of(f)? != BUS_STATE_ID || f.dlc() < STATE_LEN {
            return None;
        }
        let d = f.data();
        Some(BusState { mode: d[0], flags: d[1], progress: d[2], target: d[3] })
    }

    /// The frame carrying this state.
    pub fn to_frame(self) -> Option<CanFrame> {
        frame(BUS_STATE_ID, &[self.mode, self.flags, self.progress, self.target])
    }

    /// True if a file transfer is running anywhere on the bus.
    pub fn update_in_progress(&self) -> bool {
        self.flags & FLAG_UPDATE != 0
    }

    /// True if the master is currently refusing mode changes.
    pub fn mode_locked(&self) -> bool {
        self.flags & FLAG_MODE_LOCKED != 0
    }

    /// True if the master reports a live ECU link.
    pub fn ecu_link(&self) -> bool {
        self.flags & FLAG_ECU_LINK != 0
    }
}

// ===================== master side =====================

/// The master's view: it owns the mode and the update flag, and answers the
/// node message channel.
#[derive(Clone, Copy, Debug, Default)]
pub struct Bus {
    state: BusState,
    dirty: bool,
}

impl Bus {
    /// A bus in `MODE_NORMAL` with nothing running.
    pub const fn new() -> Self {
        Bus {
            state: BusState { mode: MODE_NORMAL, flags: 0, progress: 0, target: 0 },
            dirty: false,
        }
    }

    /// The state as it stands.
    pub fn state(&self) -> BusState {
        self.state
    }

    /// The mode the bus is in.
    pub fn mode(&self) -> u8 {
        self.state.mode
    }

    /// Move the bus to `mode`. Returns false if it was already there or if
    /// mode changes are locked.
    pub fn set_mode(&mut self, mode: u8) -> bool {
        if self.state.mode == mode || self.state.mode_locked() {
            return false;
        }
        self.state.mode = mode;
        self.dirty = true;
        true
    }

    /// Raise or lower a flag; `FLAG_UPDATE` is better set through
    /// `update_started` and `update_finished`.
    pub fn set_flag(&mut self, flag: u8, on: bool) {
        let next = if on { self.state.flags | flag } else { self.state.flags & !flag };
        if next != self.state.flags {
            self.state.flags = next;
            self.dirty = true;
        }
    }

    /// Announce a transfer to `target`, which also locks the mode until it ends.
    pub fn update_started(&mut self, target: u8) {
        self.state.flags |= FLAG_UPDATE | FLAG_MODE_LOCKED;
        self.state.progress = 0;
        self.state.target = target;
        self.dirty = true;
    }

    /// Update the announced percentage. Only a whole percent moves the state,
    /// so a byte-by-byte caller does not flood the bus with state frames.
    pub fn update_progress(&mut self, done: u32, total: u32) {
        let pct = (done.min(total) * 100).checked_div(total).unwrap_or(0) as u8;
        if pct != self.state.progress {
            self.state.progress = pct;
            self.dirty = true;
        }
    }

    /// The transfer is over, whether it succeeded or not.
    pub fn update_finished(&mut self) {
        self.state.flags &= !(FLAG_UPDATE | FLAG_MODE_LOCKED);
        self.state.progress = 0;
        self.state.target = 0;
        self.dirty = true;
    }

    /// Feed every frame off the bus. A `ModeRequest` is applied here and a
    /// `Hello` re-announces the state; every message is returned either way,
    /// so the firmware still sees errors and offers.
    pub fn feed(&mut self, f: &CanFrame) -> Option<NodeMsg> {
        let msg = NodeMsg::parse(f)?;
        match msg.opcode {
            Opcode::ModeRequest => {
                // Announce either way: a node whose request was refused needs
                // to be told the mode it is actually in.
                self.dirty = true;
                if let Some(mode) = msg.mode() {
                    self.set_mode(mode);
                }
            }
            Opcode::Hello => self.dirty = true,
            // The master owns the banks, so an input reported by a node is
            // applied here and goes out on the next gauge broadcast like any
            // other reading. A node that lit its own lamp instead would be
            // the one screen in the car that disagreed with the others.
            Opcode::Input => {
                for (ind, on) in msg.lamps() {
                    indicators::set(ind, on);
                }
            }
            _ => {}
        }
        Some(msg)
    }

    /// The state frame, but only once per change. Call every loop; the
    /// firmware should also send `state_frame` on a slow timer so a node that
    /// missed one still converges.
    pub fn announce(&mut self) -> Option<CanFrame> {
        if !core::mem::take(&mut self.dirty) {
            return None;
        }
        self.state_frame()
    }

    /// The state frame unconditionally.
    pub fn state_frame(&self) -> Option<CanFrame> {
        self.state.to_frame()
    }
}

// ===================== node side =====================

/// A node's view of the bus: the last state the master announced, and the
/// frames this node sends back.
#[derive(Clone, Copy, Debug)]
pub struct BusView {
    node: u8,
    state: BusState,
    synced: bool,
}

impl BusView {
    /// `node` is this node's bus address, fixed by the firmware.
    pub const fn new(node: u8) -> Self {
        BusView {
            node,
            state: BusState { mode: MODE_NORMAL, flags: 0, progress: 0, target: 0 },
            synced: false,
        }
    }

    /// Feed every frame off the bus. Returns true if the state changed, which
    /// is the display's cue to redraw.
    pub fn feed(&mut self, f: &CanFrame) -> bool {
        let Some(state) = BusState::parse(f) else { return false };
        let changed = !self.synced || state != self.state;
        self.state = state;
        self.synced = true;
        changed
    }

    /// The last state the master announced.
    pub fn state(&self) -> BusState {
        self.state
    }

    /// The mode this node should be showing.
    pub fn mode(&self) -> u8 {
        self.state.mode
    }

    /// True once a state frame has been seen, so a node can tell
    /// `MODE_NORMAL` from "the master has not spoken yet".
    pub fn is_synced(&self) -> bool {
        self.synced
    }

    /// True while a transfer is running, whichever node it targets.
    pub fn update_in_progress(&self) -> bool {
        self.state.update_in_progress()
    }

    /// True while a transfer targeting this node is running.
    pub fn update_is_mine(&self) -> bool {
        self.state.update_in_progress()
            && (self.state.target == self.node || self.state.target == NODE_BROADCAST)
    }

    /// Percent of the running transfer.
    pub fn progress(&self) -> u8 {
        self.state.progress
    }

    /// Ask the master to move the whole bus to `mode`.
    pub fn request_mode(&self, mode: u8) -> Option<CanFrame> {
        message(self.node, Opcode::ModeRequest, &[mode])
    }

    /// Tell the master this node has hit `code`.
    pub fn report_error(&self, code: u8) -> Option<CanFrame> {
        message(self.node, Opcode::Error, &[code])
    }

    /// Tell the master this node holds an image it may pull.
    pub fn offer_update(&self, kind: u8, size: u32) -> Option<CanFrame> {
        let s = size.to_le_bytes();
        message(self.node, Opcode::UpdateOffer, &[kind, s[0], s[1], s[2], s[3]])
    }

    /// Ask the master to re-announce the state, after a boot or a bus drop.
    pub fn hello(&self) -> Option<CanFrame> {
        message(self.node, Opcode::Hello, &[])
    }

    /// Tell the master that a tell-tale wired to an input on this node has
    /// changed.
    ///
    /// The lamp is not lit here. The master folds the bit into the bank and
    /// broadcasts it, and the lamp comes back on the bus -- on this display
    /// and on every other one at the same time, which is the whole reason the
    /// report goes out rather than straight onto the screen.
    pub fn report_lamp(&self, ind: Indicator, on: bool) -> Option<CanFrame> {
        self.report_lamps(&[(ind, on)])
    }

    /// Report several at once, up to [`LAMPS_PER_MSG`] of them.
    ///
    /// One frame rather than several, so a node coming back from a bus drop
    /// hands over its whole loom in a single message: a dash that arrives in
    /// two halves is a dash that is briefly wrong. `None` for more than a
    /// frame holds, rather than a silent truncation that would leave a lamp
    /// stuck at whatever it was last time.
    pub fn report_lamps(&self, lamps: &[(Indicator, bool)]) -> Option<CanFrame> {
        if lamps.is_empty() || lamps.len() > LAMPS_PER_MSG {
            return None;
        }
        let mut args = [0u8; LAMPS_PER_MSG];
        for (i, (ind, on)) in lamps.iter().enumerate() {
            args[i] = pack_lamp(*ind, *on);
        }
        message(self.node, Opcode::Input, &args[..lamps.len()])
    }

    /// Forget the announced state, so `is_synced` reports false again.
    pub fn reset(&mut self) {
        self.synced = false;
    }
}
