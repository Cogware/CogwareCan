// SPDX-License-Identifier: GPL-3.0-only
//! One way in, and one table out the other side.
//!
//! These prove the promise the display side relies on: whatever is plumbed
//! into the car fills the channels it can without being named first, and
//! every channel nothing filled reads as `NO_DATA` rather than as zero.

use cogware_can::*;
use embedded_can::{Frame as _, Id, StandardId};

mod common;
use common::bus;

fn frame(id: u16, data: &[u8]) -> CanFrame {
    CanFrame::new(Id::Standard(StandardId::new(id).expect("id")), data).expect("frame")
}

#[test]
fn a_broadcast_ecu_is_decoded_without_being_named_first() {
    let _bus = bus();
    // rusEFI Speeds at its default base: rpm in the first word.
    let f = frame(0x201, &[0x70, 0x17, 0, 0, 0, 0, 0, 0]);
    assert_eq!(
        ingest(&f),
        Fed::Ecu {
            source: "rusEFI",
            gauges: 6
        }
    );
    assert_eq!(RPM.get(), Some(6000));
}

#[test]
fn a_different_ecu_lands_in_the_same_gauges() {
    let _bus = bus();
    // MaxxECU 0x520, rpm 3000, TPS 25.0 %, MAP 150.0 kPa. Nothing was
    // configured differently to read this than to read the rusEFI above:
    // the ID is the whole of how the source is known.
    let f = frame(0x520, &[0xB8, 0x0B, 0xFA, 0x00, 0xDC, 0x05, 0, 0]);
    assert_eq!(
        ingest(&f),
        Fed::Ecu {
            source: "MaxxECU",
            gauges: 3
        }
    );
    assert_eq!(RPM.get(), Some(3000));
    assert_eq!(TPS.as_f32(), Some(25.0));
    assert_eq!(MAP.as_f32(), Some(150.0));
}

#[test]
fn an_obd2_reply_fills_the_same_table_as_a_standalone() {
    let _bus = bus();
    // Mode 01 PID 0x05, coolant, 90 °C sent as 130.
    assert_eq!(
        ingest(&frame(0x7E8, &[0x03, 0x41, 0x05, 130, 0, 0, 0, 0])),
        Fed::Obd2 { gauges: 1 }
    );
    assert_eq!(CLNT.as_f32(), Some(90.0));
    // PID 0x0C, rpm in quarters: 3000 rpm is 12000.
    assert_eq!(
        ingest(&frame(0x7E8, &[0x04, 0x41, 0x0C, 0x2E, 0xE0, 0, 0, 0])),
        Fed::Obd2 { gauges: 1 }
    );
    assert_eq!(RPM.get(), Some(3000));
}

#[test]
fn a_reply_to_a_pid_nothing_decodes_is_not_a_fault() {
    let _bus = bus();
    // The car answers what it likes. An unknown PID wrote no gauges, which
    // is a different thing from the frame being wrong.
    assert_eq!(
        ingest(&frame(0x7E8, &[0x03, 0x41, 0xB7, 1, 0, 0, 0, 0])),
        Fed::Obd2 { gauges: 0 }
    );
}

#[test]
fn a_display_bus_gauge_frame_still_goes_where_it_always_did() {
    let _bus = bus();
    CLNT.set(875);
    let f = CLNT.to_frame().expect("frame");
    CLNT.clear();
    assert_eq!(ingest(&f), Fed::Gauge(Gauge::Clnt.data()));
    assert_eq!(CLNT.get(), Some(875));
}

#[test]
fn a_channel_no_source_filled_reads_as_no_data() {
    let _bus = bus();
    // rusEFI has no tank float and no GPS receiver. Those gauges say so,
    // rather than reading zero and looking like an empty tank and a car
    // standing still.
    ingest(&frame(0x201, &[0x70, 0x17, 0, 0, 0, 0, 0, 0]));
    assert_eq!(RPM.raw(), 6000);
    assert_eq!(FUEL_VOLUME.raw(), NO_DATA);
    assert_eq!(GPS_SPEED.raw(), NO_DATA);
    assert_eq!(GPS_LOCK.raw(), NO_DATA);
}

#[test]
fn a_second_source_fills_what_the_first_one_cannot() {
    let _bus = bus();
    // The mixed case, which is the normal one: an ECU on the bus and a GPS
    // receiver wired to the server. The display reads one table and never
    // learns there were two senders.
    ingest(&frame(0x201, &[0x70, 0x17, 0, 0, 0, 0, 0, 0]));
    assert_eq!(GPS_SPEED.raw(), NO_DATA);
    GPS_SPEED.set(880);
    GPS_LOCK.set(3);
    assert_eq!(GPS_SPEED.as_f32(), Some(88.0));
    assert_eq!(RPM.get(), Some(6000), "the ECU's channels were left alone");
}

#[test]
fn every_exhaust_probe_arrives_not_just_the_first() {
    let _bus = bus();
    // Eight bytes, 5 °C each. The last probe is as interesting as the first
    // when one cylinder is running lean, and it is the disagreement between
    // them that says so -- which needs all eight.
    let f = frame(0x209, &[100, 101, 102, 103, 104, 105, 106, 140]);
    assert_eq!(
        ingest(&f),
        Fed::Ecu {
            source: "rusEFI",
            gauges: 8
        }
    );
    assert_eq!(EGT1.as_f32(), Some(500.0));
    assert_eq!(
        EGT8.as_f32(),
        Some(700.0),
        "the eighth probe is 200 °C hotter"
    );
}

#[test]
fn an_exhaust_probe_nothing_reported_is_not_a_cold_cylinder() {
    let _bus = bus();
    // A four-cylinder sends four. The other four must not read 0 °C, which
    // would be a plausible-looking answer and a wrong one.
    EGT1.set(5000);
    EGT2.set(5100);
    EGT3.set(5050);
    EGT4.set(4950);
    assert_eq!(EGT5.raw(), NO_DATA);
    assert_eq!(EGT8.raw(), NO_DATA);
}

#[test]
fn clearing_the_table_puts_every_gauge_back_to_no_data() {
    let _bus = bus();
    // What a server does when the ECU link drops: the displays should show
    // absence, not the last frame before the wire came off.
    ingest(&frame(0x201, &[0x70, 0x17, 0, 0, 0, 0, 0, 0]));
    assert_ne!(RPM.raw(), NO_DATA);
    clear_all();
    for gauge in ALL_GAUGES {
        assert_eq!(
            gauge.raw(),
            NO_DATA,
            "{} survived the table being cleared",
            gauge.name
        );
    }
}

#[test]
fn a_source_saying_it_has_no_reading_is_not_the_same_as_saying_zero() {
    let _bus = bus();
    // A sender that knows its sensor is open-circuit reports the sentinel.
    // The gauge goes back to absent, and the write still counts, because
    // something is plainly still running out there.
    OIL_PRES.set(3500);
    let writes = OIL_PRES.writes();
    OIL_PRES.set(NO_DATA);
    assert_eq!(OIL_PRES.get(), None);
    assert_eq!(OIL_PRES.raw(), NO_DATA);
    assert_eq!(OIL_PRES.writes(), writes.wrapping_add(1));
}
