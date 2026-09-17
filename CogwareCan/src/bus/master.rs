// SPDX-License-Identifier: GPL-3.0-only
//! The master's side of the bus: it owns the mode, the update flag and the
//! server's radios, and answers the node message channel.

use super::{
    RadioFault, RADIO_FAULT_NONE,
    indicators, BusState, CanFrame, NodeMsg, Opcode, FLAG_MODE_LOCKED, FLAG_UPDATE, MODE_NORMAL,
    RADIO_BLUETOOTH, RADIO_PAIRING, RADIO_WIFI,
};

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
            state: BusState {
                mode: MODE_NORMAL,
                flags: 0,
                progress: 0,
                target: 0,
                radio: 0,
                radio_fault: RADIO_FAULT_NONE,
            },
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

    /// Which of the server's radios are on, and whether one is accepting a
    /// new device.
    pub fn radio(&self) -> u8 {
        self.state.radio
    }

    /// Record that `radios` are now on or off.
    ///
    /// Reports what the hardware *is*, not what was asked for, and that is the
    /// whole reason a radio request is not applied the way a mode request is.
    /// A mode is a number this board owns outright, so it can simply take it.
    /// A radio is hardware that takes time to come up and can fail to: the
    /// firmware brings it up, and calls this once it knows. A state that led
    /// the hardware would have every screen in the car saying Bluetooth was on
    /// while it was not.
    ///
    /// Turning a radio off also stops it accepting new devices, because a
    /// radio that is off cannot be pairing and a bit saying otherwise would
    /// outlive the thing it described.
    pub fn set_radio(&mut self, radios: u8, on: bool) {
        let mut next = if on {
            self.state.radio | radios
        } else {
            self.state.radio & !radios
        };
        if next & (RADIO_BLUETOOTH | RADIO_WIFI) == 0 {
            next &= !RADIO_PAIRING;
        }
        if next != self.state.radio {
            self.state.radio = next;
            // Doing what was asked is what clears a fault: a state that said
            // both "Bluetooth is on" and "the radio would not start" would be
            // telling a display two things and letting it choose.
            self.state.radio_fault = RADIO_FAULT_NONE;
            self.dirty = true;
        }
    }

    /// Record that the server is, or is no longer, accepting a new device.
    ///
    /// Refused while every radio is off: there is nothing for a device to
    /// arrive on, and saying otherwise would put a pairing notice on every
    /// screen for a pairing that could not happen. Returns whether it took.
    pub fn set_pairing(&mut self, on: bool) -> bool {
        if on && self.state.radio & (RADIO_BLUETOOTH | RADIO_WIFI) == 0 {
            return false;
        }
        let next = if on {
            self.state.radio | RADIO_PAIRING
        } else {
            self.state.radio & !RADIO_PAIRING
        };
        if next != self.state.radio {
            self.state.radio = next;
            self.dirty = true;
        }
        true
    }

    /// Say why the last radio request came to nothing.
    ///
    /// The firmware calls this when it could not do what was asked -- it
    /// refused, there is no such aerial, or the radio would not come up. A
    /// node watching a bit stay clear cannot tell those apart, and which one
    /// it was decides whether a person should try again or go and look at the
    /// car.
    pub fn radio_failed(&mut self, fault: RadioFault) {
        let code = fault.to_u8();
        if code != self.state.radio_fault {
            self.state.radio_fault = code;
            self.dirty = true;
        }
    }

    /// Feed every frame off the bus. A `ModeRequest` is applied here and a
    /// `Hello` re-announces the state; every message is returned either way,
    /// so the firmware still sees errors and offers.
    ///
    /// A `RadioRequest` is deliberately *not* applied here -- see
    /// [`Bus::set_radio`] -- but does mark the state for re-announcement, so
    /// the node that asked is told what the radios are actually doing whether
    /// or not the firmware acts on it.
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
            // Answered, not applied: the firmware owns the aerials. Announcing
            // either way is what tells a node its request was heard, and what
            // tells it the truth when nothing came of it.
            Opcode::RadioRequest => {
                // Cleared as the request arrives, so the node that asked can
                // tell "nothing has gone wrong yet" from a fault left over
                // from somebody else's request ten minutes ago. Its own
                // outcome replaces this, one way or the other.
                self.state.radio_fault = RADIO_FAULT_NONE;
                self.dirty = true;
            }
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
