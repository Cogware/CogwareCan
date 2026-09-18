// SPDX-License-Identifier: GPL-3.0-only
//! A node's side of the bus: the last state the master announced, and the
//! frames this node sends back.

use super::{
    RadioFault, RADIO_FAULT_NONE,
    message, BusState, CanFrame, Indicator, Opcode, LAMPS_PER_MSG, MODE_NORMAL, NODE_BROADCAST,
    RADIO_BLUETOOTH, RADIO_OFF, RADIO_ON, RADIO_PAIR,
};
use super::pack_lamp;
use super::Health;

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
            state: BusState {
                mode: MODE_NORMAL,
                flags: 0,
                progress: 0,
                target: 0,
                radio: 0,
                radio_fault: RADIO_FAULT_NONE,
            },
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

    /// Ask the master to switch a radio, or to accept a new device on one.
    ///
    /// `radios` is a mask of [`RADIO_BLUETOOTH`] and [`super::RADIO_WIFI`]; `action`
    /// is [`RADIO_OFF`], [`RADIO_ON`] or [`RADIO_PAIR`].
    ///
    /// Nothing happens here and nothing is assumed: the radio belongs to the
    /// server, and this node learns what came of the request the same way
    /// every other node does, from the next state broadcast.
    pub fn request_radio(&self, radios: u8, action: u8) -> Option<CanFrame> {
        message(self.node, Opcode::RadioRequest, &[radios, action])
    }

    /// Ask the master to turn its Bluetooth radio on or off.
    pub fn request_bluetooth(&self, on: bool) -> Option<CanFrame> {
        self.request_radio(
            RADIO_BLUETOOTH,
            if on { RADIO_ON } else { RADIO_OFF },
        )
    }

    /// Ask the master to accept a new Bluetooth device, turning the radio on
    /// if it is not already.
    ///
    /// This is the one a display's menu sends when somebody wants to pair a
    /// phone: the server has no screen to confirm on and no button to hold,
    /// so the display it is paired *from* is what asks.
    pub fn request_pairing(&self) -> Option<CanFrame> {
        self.request_radio(RADIO_BLUETOOTH, RADIO_PAIR)
    }

    /// True if the server's Bluetooth radio is on.
    pub fn bluetooth_on(&self) -> bool {
        self.state.bluetooth_on()
    }

    /// True if the server's WiFi radio is on.
    pub fn wifi_on(&self) -> bool {
        self.state.wifi_on()
    }

    /// True while the server is accepting a new device, which is a display's
    /// cue to say so: it is the only place a person can be told.
    pub fn pairing(&self) -> bool {
        self.state.pairing()
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

    /// Tell the master how fast this node is drawing.
    ///
    /// The master is the only node every display and every host already
    /// talks to, so it is where a report about this node's own panel goes:
    /// relayed out to whichever host is listening, rather than drawn on the
    /// glass it is describing.
    pub fn report_health(&self, health: &Health) -> Option<CanFrame> {
        let args = health.encode();
        message(self.node, Opcode::Health, &args)
    }

    /// Forget the announced state, so `is_synced` reports false again.
    pub fn reset(&mut self) {
        self.synced = false;
    }
}

/// Where a radio request has got to.
///
/// What a display's menu draws from: a throbber while it is [`Waiting`], the
/// radio's new state when it is [`Done`], and words when it is [`Failed`].
///
/// [`Waiting`]: RadioWait::Waiting
/// [`Done`]: RadioWait::Done
/// [`Failed`]: RadioWait::Failed
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioWait {
    /// Asked, and the server has not finished answering.
    Waiting,
    /// The radios are doing what was asked.
    Done,
    /// They are not, and this is why.
    Failed(RadioFault),
}

/// One radio request, from asking to knowing.
///
/// The server owns the aerials, so a display that asks for one learns nothing
/// by looking at itself: the answer arrives in a broadcast, or fails to. This
/// watches for it, and puts a limit on how long a person is left looking at a
/// throbber.
///
/// It holds no clock, because this crate runs on boards whose timers have
/// nothing in common. The caller brings the time, in milliseconds, the same
/// way it does everywhere else here.
///
/// ```ignore
/// let mut ask = RadioAsk::pairing(now_ms);
/// can.send_message(view.request_pairing()?).ok();
/// // then, every pass:
/// match ask.poll(&view, now_ms) {
///     RadioWait::Waiting => menu.throbber(),
///     RadioWait::Done => menu.say("hold your phone near the car"),
///     RadioWait::Failed(why) => menu.error(why.describe()),
/// }
/// ```
#[derive(Clone, Copy, Debug)]
pub struct RadioAsk {
    radios: u8,
    action: u8,
    deadline_ms: u64,
    state: RadioWait,
}

/// How long a server has to answer before a display gives up on it.
///
/// An aerial takes a moment to come up and the server may be part way through
/// a bus round when it is asked, so this is generous. What it is not is
/// unbounded: a throbber that can spin for ever is a display that cannot tell
/// a slow server from a dead one, and so neither can the person watching it.
pub const RADIO_WAIT_MS: u64 = 5_000;

impl RadioAsk {
    /// Start waiting for a request just sent.
    pub const fn new(radios: u8, action: u8, now_ms: u64) -> Self {
        Self {
            radios,
            action,
            deadline_ms: now_ms + RADIO_WAIT_MS,
            state: RadioWait::Waiting,
        }
    }

    /// Start waiting for [`BusView::request_bluetooth`].
    pub const fn bluetooth(on: bool, now_ms: u64) -> Self {
        Self::new(
            RADIO_BLUETOOTH,
            if on { RADIO_ON } else { RADIO_OFF },
            now_ms,
        )
    }

    /// Start waiting for [`BusView::request_pairing`].
    pub const fn pairing(now_ms: u64) -> Self {
        Self::new(RADIO_BLUETOOTH, RADIO_PAIR, now_ms)
    }

    /// Where the request has got to, given what the master last announced.
    ///
    /// Settles once: a request that has finished, one way or the other, does
    /// not go back to waiting because a later broadcast happened to differ.
    /// What a person was told is what remains true until they ask again.
    pub fn poll(&mut self, view: &BusView, now_ms: u64) -> RadioWait {
        if self.state != RadioWait::Waiting {
            return self.state;
        }
        let state = view.state();
        // A fault first: the server saying why is worth more than a bit that
        // happens to be in the right place.
        if let Some(fault) = state.fault() {
            self.state = RadioWait::Failed(fault);
        } else if self.satisfied(&state) {
            self.state = RadioWait::Done;
        } else if now_ms >= self.deadline_ms {
            // Nothing wrong was announced and nothing right happened either,
            // which from here is indistinguishable from a server that has
            // stopped listening -- and is the same thing to the person waiting.
            self.state = RadioWait::Failed(RadioFault::NoAnswer);
        }
        self.state
    }

    /// Where it has got to, without advancing anything.
    pub const fn state(&self) -> RadioWait {
        self.state
    }

    /// True while a display should still be showing a throbber.
    pub const fn waiting(&self) -> bool {
        matches!(self.state, RadioWait::Waiting)
    }

    /// Whether the announced state is what was asked for.
    fn satisfied(&self, state: &BusState) -> bool {
        let on = state.radio & self.radios == self.radios;
        match self.action {
            RADIO_OFF => state.radio & self.radios == 0,
            RADIO_ON => on,
            // Pairing is two things at once and both have to be true: a radio
            // that came up but is not accepting anything leaves a person
            // holding a phone at a car that will not talk to it.
            RADIO_PAIR => on && state.pairing(),
            // An action this build does not know cannot be checked for, so it
            // is never satisfied and times out rather than reporting a success
            // it did not verify.
            _ => false,
        }
    }
}
