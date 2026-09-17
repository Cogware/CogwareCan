// SPDX-License-Identifier: GPL-3.0-only
//! A turn signal is asked for once and shown as a flash.
//!
//! These take the `bus()` guard because the indicator banks are the gauge
//! table's, which is process-wide.

use cogware_can::blink::{Blink, HALF_PERIOD_US};
use cogware_can::indicators::{self, Indicator};

mod common;
use common::bus;

/// Microseconds, as the master's loop would pass them.
const HALF: u64 = HALF_PERIOD_US;

#[test]
fn a_turn_signal_lights_the_moment_it_is_asked_for() {
    let _bus = bus();
    let mut blink = Blink::new();
    // Not on the next phase flip: a stalk pushed and a bulb that stays dark
    // for most of a second reads as a fault.
    blink.request(Indicator::TurnLeft, true);
    assert!(indicators::get(Indicator::TurnLeft));
}

#[test]
fn a_turn_signal_goes_on_and_off_a_second_at_a_time() {
    let _bus = bus();
    let mut blink = Blink::new();
    blink.request(Indicator::TurnLeft, true);

    let mut now = 0;
    assert!(indicators::get(Indicator::TurnLeft), "lit when asked");

    // A tick before the half period is up changes nothing.
    now += HALF - 1;
    blink.tick(now);
    assert!(indicators::get(Indicator::TurnLeft), "still lit just before");

    now += 1;
    blink.tick(now);
    assert!(!indicators::get(Indicator::TurnLeft), "dark after a second");

    now += HALF;
    blink.tick(now);
    assert!(indicators::get(Indicator::TurnLeft), "lit again after two");
}

#[test]
fn letting_go_of_the_stalk_puts_the_lamp_out_at_once() {
    let _bus = bus();
    let mut blink = Blink::new();
    blink.request(Indicator::TurnLeft, true);
    blink.request(Indicator::TurnLeft, false);
    assert!(!indicators::get(Indicator::TurnLeft));

    // And it stays out however long the phase runs: a lamp nobody asked for
    // must not come back on the next flip.
    let mut now = 0;
    for _ in 0..4 {
        now += HALF;
        blink.tick(now);
        assert!(!indicators::get(Indicator::TurnLeft));
    }
}

#[test]
fn a_lamp_that_does_not_flash_is_held_steady() {
    let _bus = bus();
    let mut blink = Blink::new();
    // Oil pressure is a fault, not a rhythm. It must not be flashed by a
    // mechanism built for stalks.
    blink.request(Indicator::OilPressure, true);
    let mut now = 0;
    for _ in 0..4 {
        now += HALF;
        blink.tick(now);
        assert!(indicators::get(Indicator::OilPressure), "held on");
    }
}

#[test]
fn both_signals_flash_together_rather_than_against_each_other() {
    let _bus = bus();
    let mut blink = Blink::new();
    // Hazards are both stalks at once, and a car whose two sides alternate is
    // a car with a fault, not a car with its hazards on.
    blink.request(Indicator::TurnLeft, true);
    blink.request(Indicator::TurnRight, true);
    let mut now = 0;
    for _ in 0..5 {
        assert_eq!(
            indicators::get(Indicator::TurnLeft),
            indicators::get(Indicator::TurnRight),
            "the two sides agree"
        );
        now += HALF;
        blink.tick(now);
    }
}

#[test]
fn the_request_is_remembered_whatever_the_bulb_is_doing() {
    let _bus = bus();
    let mut blink = Blink::new();
    blink.request(Indicator::TurnLeft, true);
    blink.tick(HALF);
    // Dark, but still asked for: this is what a node would be told if it asked
    // what the car wants, rather than what the bulb happens to be showing.
    assert!(!indicators::get(Indicator::TurnLeft));
    assert!(blink.wanted(Indicator::TurnLeft));
}

#[test]
fn a_half_period_of_zero_holds_every_lamp_steady() {
    let _bus = bus();
    // The honest way to turn flashing off: the lamps still light.
    let mut blink = Blink::with(&[Indicator::TurnLeft], 0);
    blink.request(Indicator::TurnLeft, true);
    let mut now = 0;
    for _ in 0..4 {
        now += HALF;
        blink.tick(now);
        assert!(indicators::get(Indicator::TurnLeft));
    }
}

#[test]
fn a_clock_that_went_backwards_flips_rather_than_freezing() {
    let _bus = bus();
    let mut blink = Blink::new();
    blink.request(Indicator::TurnLeft, true);
    blink.tick(10 * HALF);
    let after = indicators::get(Indicator::TurnLeft);
    // A lamp stuck because a counter wrapped is worse than one that flashed
    // out of step for a moment.
    blink.tick(0);
    assert_ne!(indicators::get(Indicator::TurnLeft), after);
}

#[test]
fn the_flash_rate_is_ninety_a_minute() {
    // A flash is a full cycle, on and off, so ninety a minute is 1.5 Hz. The
    // rate is what a person compares against the car next to them, so it is
    // worth pinning rather than leaving to whatever the constant drifts to.
    let per_minute = 60_000_000 / (HALF_PERIOD_US * 2);
    assert_eq!(per_minute, 90);
}
