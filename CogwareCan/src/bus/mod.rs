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
//!
//! # The server's radios
//!
//! One set of aerials serves the whole car and they live on the server, which
//! is the board with no screen, no buttons and no easy way in once the car is
//! together. So the way they get switched on is a display asking:
//!
//! ```ignore
//! // On a display, from a menu:
//! can.send_message(view.request_pairing()?).ok();
//! if view.pairing() { show("hold your phone near the car"); }
//! ```
//!
//! ```ignore
//! // On the server:
//! if let Some((radios, action)) = msg.radio() {
//!     let on = bring_up_the_aerial(radios, action);   // this board's own code
//!     bus.set_radio(radios, on);                      // report what happened
//! }
//! ```
//!
//! A radio request is answered but not applied, unlike a mode request. A mode
//! is a number the master owns outright; a radio is hardware that takes time
//! to come up and can fail to. So the state follows the hardware rather than
//! leading it, and every screen in the car says Bluetooth is on only once it
//! is. What the radio *does* once it is on is no business of this crate: it
//! carries the same `xfer` bytes the bus does, and the board that has the
//! aerial is the board that knows how.

use crate::indicators::{self, Indicator};
use crate::protocol::{frame, id_of, msg_node, node_msg_id, BUS_STATE_ID, NODE_BROADCAST};
use embedded_can::Frame;
use crate::frame::CanFrame;

/// The mode every node starts in and falls back to.
pub const MODE_NORMAL: u8 = 0;

/// A file transfer is running; nodes should show it and stay quiet.
pub const FLAG_UPDATE: u8 = 1 << 0;
/// The master has a live ECU link, so an unset gauge means "not supplied".
pub const FLAG_ECU_LINK: u8 = 1 << 1;
/// The master is refusing mode changes, usually because an update is running.
pub const FLAG_MODE_LOCKED: u8 = 1 << 2;

/// Shortest `BusState` frame this build will read.
///
/// The frame has grown once already, and the rule that made that safe is kept
/// here: a master older than a node sends only the bytes it knows about, and
/// anything past this is read as absent rather than as rubbish.
pub const STATE_LEN: usize = 4;

/// Where the radio byte sits in a `BusState` frame.
pub const RADIO_BYTE: usize = 4;

/// Where the radio outcome sits in a `BusState` frame.
pub const RADIO_FAULT_BYTE: usize = 5;

/// Length of a `BusState` frame from this build.
pub const STATE_LEN_FULL: usize = 6;

/// The last radio request did what was asked, or nothing has been asked.
pub const RADIO_FAULT_NONE: u8 = 0;
/// The server refused. Its own policy -- locked during an update, or a radio
/// this car is not allowed to use -- rather than anything that went wrong.
pub const RADIO_FAULT_DENIED: u8 = 1;
/// The server has no such radio fitted.
pub const RADIO_FAULT_NO_RADIO: u8 = 2;
/// The radio was asked to come up and did not.
pub const RADIO_FAULT_WOULD_NOT_START: u8 = 3;

/// Why a radio request came to nothing.
///
/// A node watching a bit stay clear cannot tell a refusal from a radio that
/// tried and failed, and the difference is the whole of what a person needs
/// to be told: one of them is worth asking again, and the other is worth
/// looking at the car.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioFault {
    /// The server refused.
    Denied,
    /// No such radio is fitted.
    NoRadio,
    /// The radio would not start.
    WouldNotStart,
    /// The server said nothing at all within the time allowed.
    ///
    /// The one fault a node works out for itself: every other kind arrives in
    /// a broadcast, and this is the absence of one.
    NoAnswer,
    /// A fault this build does not know, from a newer server.
    ///
    /// Carried rather than discarded so a display can at least say the number
    /// instead of claiming everything is well.
    Unknown(u8),
}

impl RadioFault {
    /// Read the byte a server broadcasts, or `None` for "no fault".
    pub const fn from_u8(v: u8) -> Option<RadioFault> {
        Some(match v {
            RADIO_FAULT_NONE => return None,
            RADIO_FAULT_DENIED => RadioFault::Denied,
            RADIO_FAULT_NO_RADIO => RadioFault::NoRadio,
            RADIO_FAULT_WOULD_NOT_START => RadioFault::WouldNotStart,
            other => RadioFault::Unknown(other),
        })
    }

    /// The byte a server broadcasts for this fault.
    ///
    /// `NoAnswer` has none: it is the absence of a broadcast, so a server has
    /// no way to send it and never should. It maps to "no fault" rather than
    /// to a number that would mean something else at the far end.
    pub const fn to_u8(self) -> u8 {
        match self {
            RadioFault::Denied => RADIO_FAULT_DENIED,
            RadioFault::NoRadio => RADIO_FAULT_NO_RADIO,
            RadioFault::WouldNotStart => RADIO_FAULT_WOULD_NOT_START,
            RadioFault::NoAnswer => RADIO_FAULT_NONE,
            RadioFault::Unknown(v) => v,
        }
    }

    /// What to put in front of a person, in words rather than a code.
    pub const fn describe(self) -> &'static str {
        match self {
            RadioFault::Denied => "the server refused",
            RadioFault::NoRadio => "the server has no such radio",
            RadioFault::WouldNotStart => "the radio would not start",
            RadioFault::NoAnswer => "the server stopped answering",
            RadioFault::Unknown(_) => "the server reported a fault this display does not know",
        }
    }
}

/// The server's Bluetooth radio.
///
/// One value, used both for "which radio" in a request and "which is on" in
/// the state, so there is one vocabulary rather than two that must agree.
pub const RADIO_BLUETOOTH: u8 = 1 << 0;
/// The server's WiFi radio.
pub const RADIO_WIFI: u8 = 1 << 1;
/// The server is accepting a new device -- a Bluetooth pairing, or whatever
/// joining means for the radio in question.
///
/// Separate from the radio bits because it is a state a radio is *in*, not a
/// radio: a server can have Bluetooth on and be refusing new devices, which is
/// what it should be doing almost all of the time.
pub const RADIO_PAIRING: u8 = 1 << 2;

/// Turn the named radios off.
pub const RADIO_OFF: u8 = 0;
/// Turn the named radios on, without accepting new devices.
pub const RADIO_ON: u8 = 1;
/// Turn the named radios on and accept a new device.
pub const RADIO_PAIR: u8 = 2;

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
    /// `[6, radios, action]` — asks the master to switch a radio, or to accept
    /// a new device on one.
    ///
    /// `radios` is a mask of [`RADIO_BLUETOOTH`] and [`RADIO_WIFI`]; `action`
    /// is [`RADIO_OFF`], [`RADIO_ON`] or [`RADIO_PAIR`].
    ///
    /// The radio is the *server's*, not the asking node's. One set of aerials
    /// serves the whole car, and it lives on the board with no screen, no
    /// buttons and no easy way in once the car is together -- so the way it
    /// gets switched on is a display asking it to.
    RadioRequest = 6,
    /// `[7, fps, compose us lo, compose us hi, px lo, px hi, rects]` — how
    /// fast this node is drawing, and what a frame of it costs.
    ///
    /// Diagnostics, on the bus rather than on a wire nobody is reading: a
    /// display's own console is not reachable once the car is together, and
    /// a rate drawn on the glass costs a repaint of its own every window to
    /// report itself. The master relays these to whichever host is listening,
    /// so the numbers land in the editor instead of on the screen.
    ///
    /// Nothing here is control traffic: a master that never sees one loses
    /// nothing but the readout, and a node sends it about once a second,
    /// which is one frame in the hundreds a busy bus already carries.
    Health = 7,
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
            6 => Opcode::RadioRequest,
            7 => Opcode::Health,
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

/// What a node's [`Opcode::Health`] message says about its own drawing.
///
/// Averages over whatever window the node chose, which is its business: the
/// reader wants the shape of the number, not its bookkeeping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Health {
    /// Frames the node put on its panel per second.
    pub fps: u8,
    /// What composing one frame cost, in microseconds.
    pub compose_us: u16,
    /// Pixels one frame repainted.
    pub painted: u16,
    /// Damage rectangles one frame repainted.
    pub rects: u8,
}

impl Health {
    /// The seven argument bytes this report travels as.
    #[must_use]
    pub fn encode(&self) -> [u8; 7] {
        let (cu, px) = (self.compose_us.to_le_bytes(), self.painted.to_le_bytes());
        [self.fps, cu[0], cu[1], px[0], px[1], self.rects, 0]
    }

    /// The report those argument bytes carry, or `None` if they are short.
    #[must_use]
    pub fn decode(args: &[u8]) -> Option<Health> {
        if args.len() < 6 {
            return None;
        }
        Some(Health {
            fps: args[0],
            compose_us: u16::from_le_bytes([args[1], args[2]]),
            painted: u16::from_le_bytes([args[3], args[4]]),
            rects: args[5],
        })
    }
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

    /// The drawing report this message carries, if it is a [`Opcode::Health`].
    #[must_use]
    pub fn health(&self) -> Option<Health> {
        match self.opcode {
            Opcode::Health => Health::decode(self.args()),
            _ => None,
        }
    }

    /// The mode asked for, if this is a `ModeRequest`.
    pub fn mode(&self) -> Option<u8> {
        if self.opcode != Opcode::ModeRequest {
            return None;
        }
        self.args().first().copied()
    }

    /// The radios and the action asked for, if this is a `RadioRequest`.
    ///
    /// `None` for a message that is not one, or one too short to carry both --
    /// which a node this build does not understand could send, and which must
    /// not be read as "turn everything off".
    pub fn radio(&self) -> Option<(u8, u8)> {
        if self.opcode != Opcode::RadioRequest {
            return None;
        }
        let args = self.args();
        match (args.first(), args.get(1)) {
            (Some(&radios), Some(&action)) => Some((radios, action)),
            _ => None,
        }
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

/// What the master says the bus is doing.
///
/// Travels as `[mode, flags, progress, target, radio]`.
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
    /// Which of the server's radios are on, and whether one is accepting a
    /// new device: `RADIO_BLUETOOTH`, `RADIO_WIFI`, `RADIO_PAIRING`.
    ///
    /// A byte of its own rather than more bits in `flags`, because the two
    /// describe different things -- `flags` is what the *bus* is doing, this
    /// is what the server's aerials are doing -- and because this one will
    /// grow: a second radio, a station-or-access-point distinction, a count of
    /// what is connected. Re-carving `flags` later would cost every node.
    pub radio: u8,
    /// Why the last radio request came to nothing; 0 for none.
    ///
    /// One server, one set of aerials, so this is the server's own business
    /// rather than any node's, and every display sees the same answer. It is
    /// cleared the moment a new request arrives, so a node that has just asked
    /// can tell "nothing has gone wrong yet" from "something went wrong once,
    /// a while ago".
    pub radio_fault: u8,
}

impl BusState {
    /// Read a state frame, or `None` if this frame is not one.
    pub fn parse(f: &CanFrame) -> Option<BusState> {
        if id_of(f)? != BUS_STATE_ID || f.dlc() < STATE_LEN {
            return None;
        }
        let d = f.data();
        Some(BusState {
            mode: d[0],
            flags: d[1],
            progress: d[2],
            target: d[3],
            // A master built before the bus had radios sends four bytes. Read
            // as "every radio off", which is both the truth about that master
            // and the safe way to be wrong: a node may then ask for one, and
            // a master that cannot answer simply does not.
            radio: if f.dlc() > RADIO_BYTE { d[RADIO_BYTE] } else { 0 },
            radio_fault: if f.dlc() > RADIO_FAULT_BYTE {
                d[RADIO_FAULT_BYTE]
            } else {
                RADIO_FAULT_NONE
            },
        })
    }

    /// The frame carrying this state.
    pub fn to_frame(self) -> Option<CanFrame> {
        frame(
            BUS_STATE_ID,
            &[
                self.mode,
                self.flags,
                self.progress,
                self.target,
                self.radio,
                self.radio_fault,
            ],
        )
    }

    /// Why the last radio request came to nothing, or `None` if nothing has.
    pub const fn fault(&self) -> Option<RadioFault> {
        RadioFault::from_u8(self.radio_fault)
    }

    /// True if the server's Bluetooth radio is on.
    pub fn bluetooth_on(&self) -> bool {
        self.radio & RADIO_BLUETOOTH != 0
    }

    /// True if the server's WiFi radio is on.
    pub fn wifi_on(&self) -> bool {
        self.radio & RADIO_WIFI != 0
    }

    /// True while the server is accepting a new device.
    pub fn pairing(&self) -> bool {
        self.radio & RADIO_PAIRING != 0
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

mod master;
mod node;

pub use master::Bus;
pub use node::{BusView, RadioAsk, RadioWait, RADIO_WAIT_MS};
