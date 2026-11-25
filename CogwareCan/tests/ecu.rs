// SPDX-License-Identifier: GPL-3.0-only
//! CAN-broadcast ECU converters. These prove each table decodes what it
//! declares; the declared layouts were checked against vendor documents and
//! firmware source (see each module header).

use cogware_can::CanFrame;
use cogware_can::*;
use embedded_can::{Frame, Id, StandardId};

mod common;
use common::bus;

fn frame(id: u16, data: &[u8]) -> CanFrame {
    CanFrame::new(Id::Standard(StandardId::new(id).unwrap()), data).unwrap()
}

fn be(words: &[i16]) -> Vec<u8> {
    words.iter().flat_map(|w| w.to_be_bytes()).collect()
}

fn le(words: &[i16]) -> Vec<u8> {
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

#[test]
fn haltech_frames() {
    let _bus = bus();
    // 0x360: rpm 3500, MAP 45.0 kPa, TPS 50.0 %, coolant pressure 221.3 kPa abs = 120.0 gauge
    assert_eq!(
        haltech::SOURCE.feed_default(&frame(0x360, &be(&[3500, 450, 500, 2213]))),
        Some(4)
    );
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(MAP.as_f32(), Some(45.0));
    assert_eq!(TPS.as_f32(), Some(50.0));
    assert_eq!(COOLANT_PRES.as_f32(), Some(120.0));
    // 0x361: fuel 401.3 abs -> 300.0 gauge, oil 351.3 -> 250.0; offset 4 (engine demand) ignored
    haltech::SOURCE.feed_default(&frame(0x361, &be(&[4013, 3513, 999, 1013])));
    assert_eq!(FUEL_PRES.as_f32(), Some(300.0));
    assert_eq!(OIL_PRES.as_f32(), Some(250.0));
    assert_eq!(WASTEGATE_PRES.as_f32(), Some(0.0));
    assert_eq!(PEDAL.get(), None);
    // 0x3E0: Kelvin*10: 87.0 °C = 360.15 K -> 3601
    haltech::SOURCE.feed_default(&frame(0x3E0, &be(&[3601, 2981, 3231, 3701])));
    assert_eq!(CLNT.get(), Some(870));
    assert_eq!(IAT.get(), Some(250));
    assert_eq!(FLEX_FUEL_TEMP.get(), Some(500));
    assert_eq!(OIL_TEMP.get(), Some(970));
    // 0x368 lambda 0.850 -> AFR 12.50; 0x372 battery at 0, boost target at 4, baro at 6
    haltech::SOURCE.feed_default(&frame(0x368, &be(&[850, 1000])));
    assert_eq!(AFR_PRI.as_f32(), Some(12.5));
    assert_eq!(AFR_SEC.as_f32(), Some(14.7));
    haltech::SOURCE.feed_default(&frame(0x372, &be(&[138, 0, 2000, 1013])));
    assert_eq!(BAT_VOL.get(), Some(13_800));
    assert_eq!(BOOST_TARGET.as_f32(), Some(200.0));
    assert_eq!(BARO.as_f32(), Some(101.3));
    // 0x370 cams at 4 and 6; 0x470 gear byte 7; 0x471 pedal at 2
    haltech::SOURCE.feed_default(&frame(0x370, &be(&[880, 0, -50, 120])));
    assert_eq!(VSS.as_f32(), Some(88.0));
    assert_eq!(VVT_ANGLE.as_f32(), Some(-5.0));
    assert_eq!(VVT_ANGLE2.as_f32(), Some(12.0));
    haltech::SOURCE.feed_default(&frame(0x470, &[0, 0, 0, 0, 0, 0, 0, (-1i8) as u8]));
    assert_eq!(GEAR.get(), Some(-1));
    haltech::SOURCE.feed_default(&frame(0x471, &be(&[0, 755])));
    assert_eq!(PEDAL.as_f32(), Some(75.5));
    haltech::SOURCE.feed_default(&frame(0x3E2, &be(&[425, 0, 0, 0])));
    assert_eq!(FUEL_VOLUME.as_f32(), Some(42.5));
    assert_eq!(
        FUEL_VOLUME
            .reading()
            .unwrap()
            .gallons()
            .map(|g| (g * 100.0).round() / 100.0),
        Some(11.23)
    );
    assert_eq!(FUEL_LEVEL.get(), None);
    assert_eq!(haltech::SOURCE.feed_default(&frame(0x200, &[0; 8])), None);
}

#[test]
fn megasquirt_frames() {
    let _bus = bus();
    // 1512: MAP 45.0 kPa, rpm 3500, CLT 188.6 °F (=87.0 °C), TPS 50.0 %
    assert_eq!(
        megasquirt::SOURCE.feed_default(&frame(1512, &be(&[450, 3500, 1886, 500]))),
        Some(4)
    );
    assert_eq!(MAP.as_f32(), Some(45.0));
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(CLNT.get(), Some(870));
    assert_eq!(TPS.as_f32(), Some(50.0));
    // 1513: PW1 2750 µs, PW2 0, MAT 77.0 °F (=25 °C), advance -3.0°
    megasquirt::SOURCE.feed_default(&frame(1513, &be(&[2750, 0, 770, -30])));
    assert_eq!(PULSE_WIDTH1.as_f32(), Some(2.75));
    assert_eq!(IAT.get(), Some(250));
    assert_eq!(CUR_SPARK_ADVANCE.as_f32(), Some(-3.0));
    // 1514: afr target 14.7, afr 12.5, ego 102.3 %, egt 1472 °F (=800 °C)
    let mut d = vec![147u8, 125];
    d.extend(be(&[1023, 14720, 0]));
    megasquirt::SOURCE.feed_default(&frame(1514, &d));
    assert_eq!(AFR_TARGET.as_f32(), Some(14.7));
    assert_eq!(AFR_PRI.as_f32(), Some(12.5));
    assert_eq!(
        EGO_CORRECT.as_f32(),
        Some(2.3),
        "100-centred MS value becomes 0-centred trim"
    );
    assert_eq!(EGT1.get(), Some(8000));
    // 1516: VSS1 25.0 m/s = 90.0 km/h
    megasquirt::SOURCE.feed_default(&frame(1516, &be(&[250, 0, 0, 0])));
    assert_eq!(VSS.as_f32(), Some(90.0));
    // A relocated base still decodes
    clear_all();
    assert_eq!(
        megasquirt::SOURCE.feed(0x600, &frame(0x601, &be(&[2750, 0, 770, -30]))),
        Some(4)
    );
    assert_eq!(PULSE_WIDTH1.get(), Some(2750));
    assert_eq!(
        megasquirt::SOURCE.feed_default(&frame(0x601, &[0; 8])),
        None
    );
}

#[test]
fn rusefi_frames() {
    let _bus = bus();
    // 0x200: warnings 3, last error 0x1234, gear 4 at byte 5
    let mut d = le(&[3, 0x1234]);
    d.extend([0, 4, 0, 0]);
    rusefi::SOURCE.feed_default(&frame(0x200, &d));
    assert_eq!(ERROR_COUNT.get(), Some(3));
    assert_eq!(NEXT_ERROR.get(), Some(0x1234));
    assert_eq!(GEAR.get(), Some(4));
    // 0x201: rpm 3500, timing 12.5° (*50), inj duty 30 % (*2), coil duty, vss 88, flex 85 %
    let mut d = le(&[3500, 625]);
    d.extend([60, 20, 88, 85]);
    assert_eq!(rusefi::SOURCE.feed_default(&frame(0x201, &d)), Some(6));
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(CUR_SPARK_ADVANCE.as_f32(), Some(12.5));
    assert_eq!(INJ_DUTY.as_f32(), Some(30.0));
    assert_eq!(COIL_DUTY.as_f32(), Some(10.0));
    assert_eq!(VSS.as_f32(), Some(88.0));
    assert_eq!(ETHANOL_PERCENT.as_f32(), Some(85.0));
    assert_eq!(IDLE_DUTY.get(), None);
    // 0x203: MAP 45 kPa (*30 = 1350), CLT 87 °C (+40), IAT 25 °C, fuel level 60 % (*2)
    let mut d = le(&[1350]);
    d.extend([127, 65, 0, 0, 0, 120]);
    rusefi::SOURCE.feed_default(&frame(0x203, &d));
    assert_eq!(MAP.as_f32(), Some(45.0));
    assert_eq!(CLNT.as_f32(), Some(87.0));
    assert_eq!(IAT.as_f32(), Some(25.0));
    assert_eq!(FUEL_LEVEL.as_f32(), Some(60.0));
    // 0x204: pad, oil 250 kPa (*30), oil temp 97 °C, fuel temp 30 °C, 13800 mV
    let mut d = le(&[0, 7500]);
    d.extend([137, 70]);
    d.extend(13_800u16.to_le_bytes());
    rusefi::SOURCE.feed_default(&frame(0x204, &d));
    assert_eq!(OIL_PRES.as_f32(), Some(250.0));
    assert_eq!(OIL_TEMP.as_f32(), Some(97.0));
    assert_eq!(FLEX_FUEL_TEMP.as_f32(), Some(30.0));
    assert_eq!(BAT_VOL.as_f32(), Some(13.8));
    assert_eq!(AFR_PRI.get(), None, "pad bytes must not feed AFR");
    // 0x205 pulse width 2.75 ms (*300 = 825); 0x206 fuel trim +2.5 % (*100)
    rusefi::SOURCE.feed_default(&frame(0x205, &le(&[0, 0, 825, 0])));
    assert_eq!(PULSE_WIDTH1.get(), Some(2750));
    rusefi::SOURCE.feed_default(&frame(0x206, &le(&[0, 0, 250, 0])));
    assert_eq!(EGO_CORRECT.as_f32(), Some(2.5));
    // 0x207 lambda 0.8500 (*10000), fuel pressure 300 kPa (*30)
    rusefi::SOURCE.feed_default(&frame(0x207, &le(&[8500, 10000, 9000, 0])));
    assert_eq!(AFR_PRI.as_f32(), Some(12.5));
    assert_eq!(AFR_SEC.as_f32(), Some(14.7));
    assert_eq!(FUEL_PRES.as_f32(), Some(300.0));
    // 0x208 cams as signed whole degrees; 0x209 EGT in 5 °C steps
    rusefi::SOURCE.feed_default(&frame(0x208, &[(-5i8) as u8, 10, 0, 0, 12, 15, 0, 0]));
    assert_eq!(VVT_ANGLE.as_f32(), Some(-5.0));
    assert_eq!(VVT_TARGET_ANGLE.as_f32(), Some(10.0));
    assert_eq!(VVT_ANGLE2.as_f32(), Some(12.0));
    rusefi::SOURCE.feed_default(&frame(0x209, &[160, 0, 0, 0, 0, 0, 0, 0]));
    assert_eq!(EGT1.as_f32(), Some(800.0));
    // Short frame: only the fields present are written
    clear_all();
    assert_eq!(
        rusefi::SOURCE.feed_default(&frame(0x201, &3500u16.to_le_bytes())),
        Some(1)
    );
    assert_eq!(CUR_SPARK_ADVANCE.get(), None);
}

#[test]
fn maxxecu_frames() {
    let _bus = bus();
    // 0x520: rpm, TPS 50.0 %, MAP 45.0 kPa; 0x521 lambda A/B, timing 12.5°
    assert_eq!(
        maxxecu::SOURCE.feed_default(&frame(0x520, &le(&[3500, 500, 450, 1000]))),
        Some(3)
    );
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(TPS.as_f32(), Some(50.0));
    assert_eq!(MAP.as_f32(), Some(45.0));
    maxxecu::SOURCE.feed_default(&frame(0x521, &le(&[850, 1000, 125, 0])));
    assert_eq!(AFR_PRI.as_f32(), Some(12.5));
    assert_eq!(CUR_SPARK_ADVANCE.as_f32(), Some(12.5));
    // 0x522: PW 2.75 ms (*100), duty 30.0 %, cut, vss 88.0
    maxxecu::SOURCE.feed_default(&frame(0x522, &le(&[275, 300, 0, 880])));
    assert_eq!(PULSE_WIDTH1.get(), Some(2750));
    assert_eq!(INJ_DUTY.as_f32(), Some(30.0));
    assert_eq!(VSS.as_f32(), Some(88.0));
    // 0x530: 13.80 V (*100), baro 101.3, IAT 25.0, CLT -10.0
    maxxecu::SOURCE.feed_default(&frame(0x530, &le(&[1380, 1013, 250, -100])));
    assert_eq!(BAT_VOL.get(), Some(13_800));
    assert_eq!(BARO.as_f32(), Some(101.3));
    assert_eq!(IAT.as_f32(), Some(25.0));
    assert_eq!(CLNT.as_f32(), Some(-10.0));
    // 0x531 EGT whole °C; 0x536 gear, boost duty, oil pressure, oil temp; 0x537 pressures
    maxxecu::SOURCE.feed_default(&frame(0x531, &le(&[0, 850, 0, 800])));
    assert_eq!(ETHANOL_PERCENT.as_f32(), Some(85.0));
    assert_eq!(EGT1.as_f32(), Some(800.0));
    maxxecu::SOURCE.feed_default(&frame(0x536, &le(&[3, 455, 2500, 970])));
    assert_eq!(GEAR.get(), Some(3));
    assert_eq!(BOOST_PWM.as_f32(), Some(45.5));
    assert_eq!(OIL_PRES.as_f32(), Some(250.0));
    assert_eq!(OIL_TEMP.as_f32(), Some(97.0));
    maxxecu::SOURCE.feed_default(&frame(0x537, &le(&[3000, 500, 1200, 2000])));
    assert_eq!(FUEL_PRES.as_f32(), Some(300.0));
    assert_eq!(BOOST_TARGET.as_f32(), Some(200.0));
    maxxecu::SOURCE.feed_default(&frame(0x540, &le(&[0, 425, 850, 0])));
    assert_eq!(FUEL_VOLUME.as_f32(), Some(42.5));
    assert_eq!(TRANS_TEMP.as_f32(), Some(85.0));
}

#[test]
fn ecumaster_frames() {
    let _bus = bus();
    // 0x600: rpm 3500, TPS 100 (=50.0 %), IAT 25, MAP 45, PW 62*2.75=170.5 -> 171
    let mut d = le(&[3500]);
    d.extend([100, 25]);
    d.extend(le(&[45, 171]));
    assert_eq!(ecumaster::SOURCE.feed_default(&frame(0x600, &d)), Some(5));
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(TPS.as_f32(), Some(50.0));
    assert_eq!(IAT.as_f32(), Some(25.0));
    assert_eq!(MAP.as_f32(), Some(45.0));
    assert_eq!(PULSE_WIDTH1.get(), Some(2758)); // 171/62 ms
                                                // 0x602: vss 88, baro 101, oil temp 97, oil 2.5 bar (*16 = 40), fuel 3 bar (48), CLT -10
    let mut d = le(&[88]);
    d.extend([101, 97, 40, 48]);
    d.extend(le(&[-10]));
    ecumaster::SOURCE.feed_default(&frame(0x602, &d));
    assert_eq!(VSS.as_f32(), Some(88.0));
    assert_eq!(BARO.as_f32(), Some(101.0));
    assert_eq!(OIL_TEMP.as_f32(), Some(97.0));
    assert_eq!(OIL_PRES.as_f32(), Some(250.0));
    assert_eq!(FUEL_PRES.as_f32(), Some(300.0));
    assert_eq!(CLNT.as_f32(), Some(-10.0));
    // 0x603: advance 25 (=12.5°), dwell 60 (=3.0 ms), lambda 128 (=1.00), ego 200 (=100 %), EGT 800
    let mut d = vec![25u8, 60, 128, 200];
    d.extend(le(&[800, 0]));
    ecumaster::SOURCE.feed_default(&frame(0x603, &d));
    assert_eq!(CUR_SPARK_ADVANCE.as_f32(), Some(12.5));
    assert_eq!(DWELL.as_f32(), Some(3.0));
    assert_eq!(AFR_PRI.as_f32(), Some(14.7));
    assert_eq!(EGO_CORRECT.as_f32(), Some(100.0));
    assert_eq!(EGT1.as_f32(), Some(800.0));
    // 0x604: gear 3, ecu temp, battery 511*0.027 = 13.797 V, flags, ethanol 85 %
    let mut d = vec![3u8, 40];
    d.extend(le(&[511, 0]));
    d.extend([0, 85]);
    ecumaster::SOURCE.feed_default(&frame(0x604, &d));
    assert_eq!(GEAR.get(), Some(3));
    assert_eq!(BAT_VOL.get(), Some(13_797));
    assert_eq!(ETHANOL_PERCENT.as_f32(), Some(85.0));
    // 0x607: boost target 200 kPa, lambda target 0.85
    let mut d = le(&[200, 0]);
    d.extend([85, 0, 0, 0]);
    ecumaster::SOURCE.feed_default(&frame(0x607, &d));
    assert_eq!(BOOST_TARGET.as_f32(), Some(200.0));
    assert_eq!(AFR_TARGET.as_f32(), Some(12.5));
    // Relocated base
    assert_eq!(
        ecumaster::SOURCE.feed(0x700, &frame(0x700, &le(&[900, 0, 0, 0]))),
        Some(5)
    );
    assert_eq!(RPM.get(), Some(900));
}

#[test]
fn registry_dispatches_by_id() {
    let _bus = bus();
    let hit = |f: &CanFrame| {
        CAN_SOURCES
            .iter()
            .find_map(|s| s.feed_default(f).map(|_| s.name))
    };
    assert_eq!(hit(&frame(0x360, &be(&[900, 0, 0, 0]))), Some("Haltech"));
    assert_eq!(hit(&frame(1512, &be(&[0, 900, 0, 0]))), Some("Megasquirt"));
    assert_eq!(hit(&frame(0x201, &900u16.to_le_bytes())), Some("rusEFI"));
    assert_eq!(hit(&frame(0x520, &le(&[900, 0, 0, 0]))), Some("MaxxECU"));
    assert_eq!(hit(&frame(0x600, &le(&[900, 0, 0, 0]))), Some("ECUMaster"));
    assert_eq!(hit(&frame(0x7DF, &[0])), None);
    assert_eq!(RPM.get(), Some(900));
    // Every source's IDs are distinct from each other at default bases.
    let mut all: Vec<u16> = CAN_SOURCES
        .iter()
        .flat_map(|s| s.ids(s.default_base))
        .collect();
    let n = all.len();
    all.sort();
    all.dedup();
    assert_eq!(all.len(), n, "overlapping default IDs between sources");
}

#[test]
fn can_sources_never_write_speeduino_only_gauges() {
    for s in CAN_SOURCES {
        for m in s.frames {
            for f in m.fields {
                assert!(
                    f.gauge.source <= Source::Standalone,
                    "{} writes {} which is {:?}",
                    s.name,
                    f.gauge.name,
                    f.gauge.source
                );
            }
        }
    }
}

#[test]
fn every_common_gauge_has_a_can_source() {
    // A gauge classed Common must be reachable from at least one converter here.
    let fed: std::collections::HashSet<&str> = CAN_SOURCES
        .iter()
        .flat_map(|s| {
            s.frames
                .iter()
                .flat_map(|m| m.fields.iter().map(|f| f.gauge.name))
        })
        .collect();
    let orphans: Vec<&str> = ALL_GAUGES
        .iter()
        .filter(|g| g.source == Source::Common && !fed.contains(g.name))
        .map(|g| g.name)
        .collect();
    assert_eq!(
        orphans,
        vec!["STA_TIME", "FUEL_LOAD", "IGN_LOAD", "ODOMETER"],
        "Common gauges only Speeduino/OBD2 feed"
    );
}

#[test]
fn gps_gauges_need_a_receiver_not_an_ecu() {
    // Classing these Common or Standalone would tell a display that fitting
    // any ECU is enough to get them, which it is not.
    for g in [&GPS_LOCK, &GPS_SATS, &GPS_SPEED] {
        assert_eq!(g.source, Source::Gps, "{}", g.name);
        assert!(
            g.source > Source::Standalone,
            "{} must fail a <= Standalone check",
            g.name
        );
    }
    // No converter in this crate feeds them, so none may claim to.
    let fed: std::collections::HashSet<&str> = CAN_SOURCES
        .iter()
        .flat_map(|s| {
            s.frames
                .iter()
                .flat_map(|m| m.fields.iter().map(|f| f.gauge.name))
        })
        .collect();
    for name in ["GPS_LOCK", "GPS_SATS", "GPS_SPEED"] {
        assert!(!fed.contains(name), "{name} has no CAN-broadcast source");
    }
}

#[test]
fn no_gps_lock_is_the_only_value_meaning_do_not_trust_the_rest() {
    let _bus = bus();
    // Unset and zero are different: nothing has said anything yet, versus a
    // receiver that is powered and reporting no fix.
    assert!(!GPS_LOCK.is_set());
    GPS_LOCK.set(0);
    assert_eq!(GPS_LOCK.get(), Some(0));
    assert!(GPS_LOCK.is_set());

    GPS_SATS.set(11);
    GPS_LOCK.set(3);
    assert_eq!((GPS_LOCK.get(), GPS_SATS.get()), (Some(3), Some(11)));

    // Both are single bytes, so a satellite count cannot run past 255.
    GPS_SATS.set(9999);
    let f = frame_for(Gauge::GpsSats.id()).unwrap();
    assert_eq!(f.dlc(), 1);
    GPS_SATS.clear();
    feed_frame(&f).unwrap();
    assert_eq!(
        GPS_SATS.get(),
        Some(255),
        "clamped, never wrapped to nothing"
    );
}

/// The channels added after the first pass, kept in one place so the scaling
/// each was verified against is recorded beside the number it produces.
///
/// rusEFI's are read off its own firmware: `can_verbose.cpp` for the layout
/// and `rusefi_config_shared.txt` for the packing multipliers. MaxxECU's and
/// Haltech's are off their published protocol tables.
#[test]
fn rusefi_channels_added_later() {
    let _bus = bus();
    // 0x200 byte 6: trip distance, sent as metres/100 -- which is already
    // what a tenths-of-a-kilometre count is.
    let mut d = le(&[0, 0]);
    d.extend([0, 0]);
    d.extend(1234u16.to_le_bytes());
    rusefi::SOURCE.feed_default(&frame(0x200, &d));
    assert_eq!(TRIP_DIST.as_f32(), Some(123.4));

    // 0x202: pedal, TPS1, TPS2, wastegate, each percent*100.
    rusefi::SOURCE.feed_default(&frame(0x202, &le(&[1000, 2500, 2450, 0])));
    assert_eq!(TPS.as_f32(), Some(25.0));
    assert_eq!(
        TPS2.as_f32(),
        Some(24.5),
        "the second blade disagreeing is the fault"
    );

    // 0x203 bytes 4-6: the two spare temperature inputs and the ECU's own
    // board, all +40 in a byte like coolant and intake.
    let mut d = le(&[1350]);
    d.extend([127, 65, 100, 110, 85, 0]);
    rusefi::SOURCE.feed_default(&frame(0x203, &d));
    assert_eq!(AUX_TEMP1.as_f32(), Some(60.0));
    assert_eq!(AUX_TEMP2.as_f32(), Some(70.0));
    assert_eq!(ECU_TEMP.as_f32(), Some(45.0));

    // 0x205: air mass per cylinder (grams*1000, so milligrams already),
    // airflow (kg/h*100) and the knock counter.
    rusefi::SOURCE.feed_default(&frame(0x205, &le(&[450, 3500, 0, 7])));
    assert_eq!(
        CYL_AIRMASS.get(),
        Some(450),
        "450 mg of air in one cylinder"
    );
    assert_eq!(AIRFLOW.as_f32(), Some(35.0));
    assert_eq!(KNOCK_COUNT.get(), Some(7));

    // 0x206: grams burned, (g/s)*200 burning, and a trim for each bank.
    // 400 counts is 2 g/s, which is 7.2 kg/h.
    rusefi::SOURCE.feed_default(&frame(0x206, &le(&[1500, 400, 250, -300])));
    assert_eq!(FUEL_USED.to(Unit::GRAMS), Some(1500.0));
    assert_eq!(FUEL_FLOW.as_f32(), Some(7.2));
    assert_eq!(
        FUEL_FLOW.to(Unit::G_PER_S),
        Some(2.0),
        "the same flow, stated the other way"
    );
    assert_eq!(EGO_CORRECT.as_f32(), Some(2.5));
    assert_eq!(
        EGO_CORRECT2.as_f32(),
        Some(-3.0),
        "banks drifting apart is the fault"
    );

    // 0x207 byte 6: direct injection rail, bar*10 -> 150 bar is 15000 kPa.
    rusefi::SOURCE.feed_default(&frame(0x207, &le(&[0, 0, 0, 1500])));
    assert_eq!(FUEL_PRES_HIGH.as_f32(), Some(15_000.0));

    // 0x208: the exhaust cams sit between the intake pair, not after them.
    rusefi::SOURCE.feed_default(&frame(0x208, &[1, 2, 3, 4, 5, 6, 7, 8]));
    assert_eq!(VVT_ANGLE.as_f32(), Some(1.0));
    assert_eq!(VVT_TARGET_ANGLE.as_f32(), Some(2.0));
    assert_eq!(VVT_EXH_ANGLE.as_f32(), Some(3.0));
    assert_eq!(VVT_EXH_TARGET.as_f32(), Some(4.0));
    assert_eq!(VVT_ANGLE2.as_f32(), Some(5.0));
    assert_eq!(VVT_EXH_ANGLE2.as_f32(), Some(7.0));

    // 0x20A: a signed knock level per cylinder. One cylinder well above the
    // rest is a different fault from all of them climbing together.
    rusefi::SOURCE.feed_default(&frame(0x20A, &[2, 3, 2, 31, 2, 3, 2, 2]));
    assert_eq!(KNOCK1.get(), Some(2));
    assert_eq!(KNOCK4.get(), Some(31), "cylinder four is the one knocking");
    assert_eq!(KNOCK8.get(), Some(2));
}

#[test]
fn maxxecu_channels_added_later() {
    let _bus = bus();
    // 0x523: undriven and driven wheel speed, slip and its target.
    maxxecu::SOURCE.feed_default(&frame(0x523, &le(&[880, 1050, 193, 100])));
    assert_eq!(WHEEL_SPEED_UNDRIVEN.as_f32(), Some(88.0));
    assert_eq!(WHEEL_SPEED_DRIVEN.as_f32(), Some(105.0));
    assert_eq!(WHEEL_SLIP.as_f32(), Some(19.3));
    assert_eq!(SLIP_TARGET.as_f32(), Some(10.0));

    // 0x527: the accelerometer, hundredths of a G.
    maxxecu::SOURCE.feed_default(&frame(0x527, &le(&[95, -42, 105, 0])));
    assert_eq!(ACCEL_FWD.as_f32(), Some(0.95));
    assert_eq!(ACCEL_RIGHT.as_f32(), Some(-0.42));
    assert_eq!(ACCEL_UP.as_f32(), Some(1.05));

    // 0x528: knock, in the detail a lamp cannot carry.
    maxxecu::SOURCE.feed_default(&frame(0x528, &le(&[1450, -32, 12, 3])));
    assert_eq!(KNOCK_LEVEL.get(), Some(1450));
    assert_eq!(
        KNOCK_RETARD.as_f32(),
        Some(-3.2),
        "timing pulled back 3.2 degrees"
    );
    assert_eq!(KNOCK_COUNT.get(), Some(12));
    assert_eq!(KNOCK_CYL.get(), Some(3));

    // 0x532 and 0x533: probes two to eight, whole degrees here.
    maxxecu::SOURCE.feed_default(&frame(0x532, &le(&[700, 710, 705, 690])));
    maxxecu::SOURCE.feed_default(&frame(0x533, &le(&[695, 880, 700, 0])));
    assert_eq!(EGT2.as_f32(), Some(700.0));
    assert_eq!(EGT5.as_f32(), Some(690.0));
    assert_eq!(
        EGT7.as_f32(),
        Some(880.0),
        "the seventh probe is running hot"
    );
    assert_eq!(EGT8.as_f32(), Some(700.0));

    // 0x534: the ECU's own temperature and the sync-loss counter.
    // Whole degrees on this channel, unlike most of the protocol's tenths.
    maxxecu::SOURCE.feed_default(&frame(0x534, &le(&[0, 52, 2, 9])));
    assert_eq!(ECU_TEMP.as_f32(), Some(52.0));
    assert_eq!(LOST_SYNC.get(), Some(9), "a trigger that keeps losing sync");

    // 0x541: cam positions, intake and exhaust, both banks.
    maxxecu::SOURCE.feed_default(&frame(0x541, &le(&[120, -55, 118, -50])));
    assert_eq!(VVT_ANGLE.as_f32(), Some(12.0));
    assert_eq!(VVT_EXH_ANGLE.as_f32(), Some(-5.5));
    assert_eq!(VVT_ANGLE2.as_f32(), Some(11.8));
    assert_eq!(VVT_EXH_ANGLE2.as_f32(), Some(-5.0));

    // 0x526 is a frame of limiter bits with one number in it.
    maxxecu::SOURCE.feed_default(&frame(0x526, &le(&[0, 0, 7200, 0])));
    assert_eq!(REV_LIMIT.get(), Some(7200));
}

#[test]
fn haltech_channels_added_later() {
    let _bus = bus();
    // 0x369: trigger health. Counters, so it is the climbing that matters.
    haltech::SOURCE.feed_default(&frame(0x369, &be(&[4, 15000, 1200, 3])));
    assert_eq!(MISS_COUNT.get(), Some(4));
    assert_eq!(TRIGGER_COUNT.get(), Some(15_000));
    assert_eq!(HOME_COUNT.get(), Some(1200));
    assert_eq!(TRIG_SINCE_HOME.get(), Some(3));

    // 0x362: both injector stages and both ignition angles. The trailing
    // angle is a rotary's second plug, not a duplicate of the leading one.
    haltech::SOURCE.feed_default(&frame(0x362, &be(&[550, 300, 180, -50])));
    assert_eq!(INJ_DUTY.as_f32(), Some(55.0));
    assert_eq!(INJ_DUTY2.as_f32(), Some(30.0));
    assert_eq!(CUR_SPARK_ADVANCE.as_f32(), Some(18.0));
    assert_eq!(IGN_ANGLE_TRAIL.as_f32(), Some(-5.0));

    // 0x373 and 0x374: probes one to eight, tenths of a Kelvin.
    // 900 °C is 1173.15 K, sent as 11732 after rounding.
    haltech::SOURCE.feed_default(&frame(0x373, &be(&[11732, 11700, 11650, 11800])));
    assert_eq!(EGT1.as_f32(), Some(900.1));
    assert_eq!(EGT4.as_f32(), Some(906.9));
    haltech::SOURCE.feed_default(&frame(0x374, &be(&[11732, 11732, 11732, 11732])));
    assert_eq!(EGT8.as_f32(), Some(900.1));

    // 0x3E2: litres in the tank, then hundredths of a litre an hour out of it.
    haltech::SOURCE.feed_default(&frame(0x3E2, &be(&[350, 1850, 0, 0])));
    assert_eq!(FUEL_VOLUME.as_f32(), Some(35.0));
    assert_eq!(FUEL_RATE.as_f32(), Some(18.5));
}

/// Status bits from a standalone, and the lamps they were mapped to.
///
/// The mappings are judgements, written out in each converter's module
/// header. These pin them down so that changing one is a deliberate act with
/// a failing test attached, rather than a quiet re-meaning of a dash lamp.
#[test]
fn rusefi_status_bits_drive_lamps() {
    let _bus = bus();
    use cogware_can::indicators::{get, Indicator};

    // Byte 4: rev limit, fuel pump, check engine and the first fan.
    let mut d = le(&[0, 0]);
    d.extend([0b1110_1101, 0, 0, 0]);
    rusefi::SOURCE.feed_default(&frame(0x200, &d));
    assert!(get(Indicator::RevLimit));
    assert!(get(Indicator::FuelPump));
    assert!(get(Indicator::CheckEngine));
    assert!(get(Indicator::LeanWarn), "the lambda monitor is cutting");
    assert!(get(Indicator::Fan));

    // A bit going clear puts its lamp out. The ECU is the authority on its
    // own state, and a lamp that only ever lit would stay on for good.
    let mut d = le(&[0, 0]);
    d.extend([0, 0, 0, 0]);
    rusefi::SOURCE.feed_default(&frame(0x200, &d));
    assert!(!get(Indicator::RevLimit));
    assert!(!get(Indicator::CheckEngine));
    assert!(!get(Indicator::Fan));
}

#[test]
fn one_lamp_fed_by_two_bits_lights_for_either() {
    let _bus = bus();
    use cogware_can::indicators::{get, Indicator};
    // rusEFI has two fan outputs and this table has one fan lamp. The second
    // fan running must light it even with the first one off, which it would
    // not if the bits were written in order as they were read.
    let mut d = le(&[0, 0]);
    d.extend([0b1000_0000, 0, 0, 0]);
    rusefi::SOURCE.feed_default(&frame(0x200, &d));
    assert!(get(Indicator::Fan), "fan 2 alone must light the fan lamp");
}

#[test]
fn maxxecu_status_bits_drive_lamps() {
    let _bus = bus();
    use cogware_can::indicators::{get, Indicator};
    // 0x526 byte 0: shiftcut, anti-lag and knock; byte 1: brake and nitrous.
    // Byte 4 is the limiter's rpm, a number in the same frame.
    let mut d = [0u8; 8];
    d[0] = 0b1000_0101;
    d[1] = 0b0100_0001;
    d[4..6].copy_from_slice(&7200i16.to_le_bytes());
    assert_eq!(maxxecu::SOURCE.feed_default(&frame(0x526, &d)), Some(11));
    assert!(get(Indicator::GearShiftCut));
    assert!(get(Indicator::Antilag));
    assert!(get(Indicator::Knock));
    assert!(get(Indicator::Brake));
    assert!(get(Indicator::NitrousActive));
    assert!(!get(Indicator::LaunchActive));
    assert_eq!(REV_LIMIT.get(), Some(7200), "the number in the frame too");
}

#[test]
fn a_frame_of_nothing_but_status_bits_is_still_claimed() {
    let _bus = bus();
    use cogware_can::indicators::{get, Indicator};
    // rusEFI base+11 carries one bit and no readings. A converter that only
    // looked at its readings table would hand it to the next source.
    assert_eq!(rusefi::SOURCE.feed_default(&frame(0x20B, &[1])), Some(1));
    assert!(get(Indicator::Brake));
}

#[test]
fn every_lamp_a_converter_drives_is_one_this_table_has() {
    // The macro would have refused an unknown name, so this is really about
    // the reverse: a lamp mapped twice within one frame is either the
    // deliberate two-bits-one-lamp case or a copy-and-paste mistake, and the
    // header table is where the difference is recorded.
    for s in CAN_SOURCES {
        for m in s.lamps {
            for f in m.flags {
                assert!(
                    f.bit < 8,
                    "{} maps a bit past the end of a byte at 0x{:03X}",
                    s.name,
                    m.id_offset
                );
                assert!(
                    cogware_can::indicators::ALL_INDICATORS.contains(&f.lamp),
                    "{} drives {}, which is not in the table",
                    s.name,
                    f.lamp.name()
                );
            }
        }
    }
}
