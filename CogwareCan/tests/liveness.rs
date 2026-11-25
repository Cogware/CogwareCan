// SPDX-License-Identifier: GPL-3.0-only
//! Telling a sender that has died from one holding steady.
//!
//! A gauge holds its last reading for ever, so by value alone a failed
//! coolant sensor and a healthy one at operating temperature are the same
//! number. The write count is the only thing that separates them, and
//! `NO_DATA` is what a reader sees once nothing is filling a channel at all.
//!
//! These take the `bus()` guard because the gauge table is process-wide: a
//! test asserting that one gauge's count did *not* move cannot share the
//! table with a test writing it.

use cogware_can::*;

mod common;
use common::bus;

#[test]
fn writing_a_gauge_advances_its_count() {
    let _bus = bus();
    let before = RPM.writes();
    RPM.set(3000);
    assert_eq!(RPM.writes(), before.wrapping_add(1));
    RPM.set(3000);
    // The same value again still counts: what is being asked is whether
    // anything is writing it, not whether the number moved. A sender holding
    // steady is alive; one that has died is not, and they look identical by
    // value.
    assert_eq!(RPM.writes(), before.wrapping_add(2));
}

#[test]
fn a_gauge_nobody_writes_stays_where_it_was() {
    let _bus = bus();
    let before = CLNT.writes();
    RPM.set(1000);
    assert_eq!(
        CLNT.writes(),
        before,
        "another gauge's write was counted here"
    );
}

#[test]
fn clearing_a_gauge_is_not_a_write() {
    let _bus = bus();
    // Clearing says there is no reading, which is the opposite of one.
    let before = CLNT.writes();
    CLNT.clear();
    assert_eq!(CLNT.writes(), before);
    assert!(!CLNT.is_set());
}

#[test]
fn every_gauge_has_a_count_of_its_own() {
    let _bus = bus();
    // Sharing one would make a busy gauge vouch for a dead one.
    for gauge in ALL_GAUGES {
        let before = gauge.writes();
        gauge.set(1);
        assert_eq!(gauge.writes(), before.wrapping_add(1), "{}", gauge.name);
    }
}

/// A gauge of this file's own, outside the table, so the sentinel checks
/// disturb nothing that reads the table. The spec is a separate static
/// because that half of a gauge lives in flash; see `GaugeSpec`.
static SPARE_SPEC: GaugeSpec =
    GaugeSpec::new("SPARE", 0xFFFF, Wire::I32, Unit::RAW, Source::Standalone);
static SPARE: GaugeData = GaugeData::new(&SPARE_SPEC);

/// Staying out of the table buys nothing against the tests that share this
/// gauge with each other: one setting a value while another clears it fails
/// whichever loses. They take this the way the others take `bus()`.
static SPARE_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn spare() -> std::sync::MutexGuard<'static, ()> {
    SPARE_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

#[test]
fn a_gauge_nothing_has_fed_reads_as_no_data() {
    let _spare = spare();
    SPARE.clear();
    assert_eq!(SPARE.raw(), NO_DATA);
    assert_eq!(SPARE.get(), None);
}

#[test]
fn a_reading_reads_as_itself() {
    let _spare = spare();
    SPARE.set(875);
    assert_eq!(SPARE.raw(), 875);
}

#[test]
fn writing_the_sentinel_stores_absence_rather_than_the_number() {
    let _spare = spare();
    // Otherwise the next reader cannot tell "no source" from a source that
    // happened to send this, and the whole point of the value is that those
    // two are the same answer.
    SPARE.set(875);
    SPARE.set(NO_DATA);
    assert_eq!(SPARE.get(), None);
    assert_eq!(SPARE.raw(), NO_DATA);
}

#[test]
fn saying_there_is_no_reading_still_counts_as_a_write() {
    let _spare = spare();
    // A source repeating "I have nothing on this channel" is alive. Only
    // silence is death, and staleness is judged on the count.
    SPARE.set(1);
    let before = SPARE.writes();
    SPARE.set(NO_DATA);
    assert_eq!(SPARE.writes(), before.wrapping_add(1));
}

#[test]
fn the_sentinel_is_not_a_reading_any_gauge_could_hold() {
    // Every gauge is fixed point, so for the sentinel to be mistaken for data
    // it would have to be a physical value some sensor produces. The coarsest
    // scale in the table is a thousand counts per unit, and even there it
    // lands near a million units below zero.
    for gauge in ALL_GAUGES {
        let physical = NO_DATA as f32 / gauge.unit.scale as f32;
        assert!(
            physical < -100_000.0,
            "{} would read {physical} {}, which is not absurd enough",
            gauge.name,
            gauge.unit.symbol
        );
    }
}

#[test]
fn the_new_exhaust_gauges_are_reachable_by_id() {
    for (name, id) in [("EGT2", 0x7F), ("EGT5", 0x82), ("EGT8", 0x85)] {
        let gauge = gauge_by_id(id).unwrap_or_else(|| panic!("{name} missing"));
        assert_eq!(gauge.name, name);
    }
}
