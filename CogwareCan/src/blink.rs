// SPDX-License-Identifier: GPL-3.0-only
//! Lamps the master drives over time rather than holding steady.
//!
//! A turn signal is asked for as one thing -- the stalk is pushed -- and shown
//! as another: a bulb going on and off. The car does not send a bit per flash
//! and no display invents its own rhythm, because two screens flashing out of
//! step is worse than either of them being wrong.
//!
//! So the master keeps what was asked for here, flips a phase on a timer, and
//! writes the product into the indicator bank. Everything downstream is
//! unchanged: a display subscribes to `INDICATORS1` exactly as before and sees
//! a bit that goes on and off, and a relay wired to a real bulb flashes for
//! the same reason the drawn one does.
//!
//! The bulb-check wave a dash runs at ignition-on is deliberately not here.
//! It belongs to the board whose bulbs they are -- see `copilot::gpio::Pins`
//! -- because a master running it centrally would be sweeping lamps at a node
//! that has not subscribed yet, or one that is not powered, which is exactly
//! when somebody is watching.
//!
//! ```ignore
//! let mut blink = Blink::new();                  // turn signals and hazards
//! for (ind, on) in msg.lamps() { blink.request(ind, on); }   // a node reported
//! blink.tick(now_us);                            // every loop
//! ```

use crate::indicators::{self, Indicator};

/// Half a flash: how long the lamp is on, and then off.
///
/// Ninety flashes a minute, which is the rate a road car's relay runs and the
/// middle of the 60-120 a type-approved one is held to. A flash is a full
/// cycle, so ninety a minute is 1.5 Hz and a third of a second each way.
pub const HALF_PERIOD_US: u64 = 1_000_000 / 3;

/// The lamps that flash unless told otherwise: the two turn signals and the
/// hazards, which are the only ones on a car that a steady bulb would misreport.
pub const DEFAULT: &[Indicator] = &[
    Indicator::TurnLeft,
    Indicator::TurnRight,
    Indicator::Hazard,
];

/// What has been asked for, and which of it flashes.
///
/// One bank word per [`Indicator`] bank, so a lamp is a bit and the whole
/// thing is four words: this sits in a loop that also drives a bus and has no
/// business allocating.
pub struct Blink {
    /// What the car has asked for, before any flashing is applied.
    want: [u32; 2],
    /// Which lamps flash rather than hold.
    flashing: [u32; 2],
    /// Whether flashing lamps are currently lit.
    lit: bool,
    /// When the phase last flipped.
    since_us: u64,
    /// Half a flash, so the full period is twice this.
    half_us: u64,
}

impl Default for Blink {
    fn default() -> Self {
        Self::new()
    }
}

impl Blink {
    /// Flash [`DEFAULT`] at [`HALF_PERIOD_US`].
    #[must_use]
    pub fn new() -> Self {
        Self::with(DEFAULT, HALF_PERIOD_US)
    }

    /// Flash exactly `lamps`, each half-flash lasting `half_us`.
    ///
    /// A `half_us` of zero holds every lamp steady, which is the honest way to
    /// turn flashing off: the lamps still light, they simply do not flash.
    #[must_use]
    pub fn with(lamps: &[Indicator], half_us: u64) -> Self {
        let mut flashing = [0u32; 2];
        for ind in lamps {
            flashing[bank_index(*ind)] |= ind.mask();
        }
        Self {
            want: [0; 2],
            flashing,
            lit: true,
            since_us: 0,
            half_us,
        }
    }

    /// Record what the car is asking for, and show it straight away.
    ///
    /// A lamp turning on lights at once rather than waiting for the phase:
    /// a stalk pushed and a bulb that stays dark for most of a second reads as
    /// a fault.
    pub fn request(&mut self, ind: Indicator, on: bool) {
        let i = bank_index(ind);
        if on {
            self.want[i] |= ind.mask();
            // Start of a flash, not the middle of whatever was already
            // running, so the first one is a full length.
            if self.flashing[i] & ind.mask() != 0 {
                self.lit = true;
            }
        } else {
            self.want[i] &= !ind.mask();
        }
        self.show(ind);
    }

    /// Whether the car is asking for `ind`, whatever the bulb is doing.
    #[must_use]
    pub fn wanted(&self, ind: Indicator) -> bool {
        self.want[bank_index(ind)] & ind.mask() != 0
    }

    /// Flip the phase if it is due, and write every flashing lamp.
    ///
    /// `now_us` is a monotonic microsecond count. Call it every loop; it does
    /// nothing until a half period has passed.
    pub fn tick(&mut self, now_us: u64) {
        if self.half_us == 0 {
            return;
        }
        // Only a clock that moved forward but not far enough holds the phase.
        // One that went backwards flips instead: a lamp frozen until a wrapped
        // counter catches up is worse than one that flashed out of step once.
        if now_us >= self.since_us && now_us - self.since_us < self.half_us {
            return;
        }
        self.since_us = now_us;
        self.lit = !self.lit;
        for ind in indicators::ALL_INDICATORS {
            if self.flashing[bank_index(*ind)] & ind.mask() != 0 {
                self.show(*ind);
            }
        }
    }

    /// Put one lamp into the bank, flashing applied.
    fn show(&self, ind: Indicator) {
        let i = bank_index(ind);
        let wanted = self.want[i] & ind.mask() != 0;
        let flashes = self.flashing[i] & ind.mask() != 0;
        indicators::set(ind, wanted && (!flashes || self.lit));
    }
}

/// Which word of [`Blink::want`] a lamp lives in. Bank 1 is index 0.
const fn bank_index(ind: Indicator) -> usize {
    (ind.bank() as usize) - 1
}
