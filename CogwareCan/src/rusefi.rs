// SPDX-License-Identifier: GPL-3.0-only
//! rusEFI CAN broadcast converter ("rusEFI verbose" dash output).
//!
//! Layout verified against `firmware/controllers/can/can_verbose.cpp` and
//! `rusEFI_CAN_verbose.dbc` on rusEFI master (2025). Frames are little-endian
//! at a configurable base ID, 0x200 by default (11-bit; a 29-bit variant with
//! the same low bits exists). `scaled_channel` encodings: pressure kPa*30,
//! angle deg*50, percent %*100, temperature °C+40 in a byte, voltage mV,
//! lambda λ*10000, pulse width ms*300, cam angles whole degrees in a signed
//! byte, EGT 5 °C per unsigned byte.
//!
//! Packing constants are rusEFI's own: pressure kPa*30, angle deg*50, percent
//! %*100, temperature +40 in a byte, voltage mV, lambda *10000, pulse width
//! ms*300, high-pressure bar*10, fuel flow (g/s)*200, fuel mass in grams.
//!
//! Not mapped: the raw MAF voltage.
//!
//! # Status bits, and the lamps they drive
//!
//! Byte 4 of base+0, bit by bit, and the whole of base+11. Each row is a
//! decision that this protocol's flag and this crate's lamp mean the same
//! thing, so each is written down rather than left to be inferred:
//!
//! | bit | rusEFI               | lamp           | why |
//! |-----|----------------------|----------------|-----|
//! | 0   | `revLimit`           | `RevLimit`     | set when the limiter is cutting either injection or ignition |
//! | 1   | `mainRelay`          | --             | the ECU being powered is not news to a dash the ECU is feeding |
//! | 2   | `fuelPump`           | `FuelPump`     | the pump relay's own output |
//! | 3   | `checkEngine`        | `CheckEngine`  | the ECU's own MIL output |
//! | 4   | `o2Heater`           | --             | a heater warming up is normal running, not a condition to show |
//! | 5   | `lambdaProtectActive`| `LeanWarn`     | rusEFI's lambda monitor cuts on a lean mixture, which is what this lamp is for |
//! | 6   | `fan`                | `Fan`          | |
//! | 7   | `fan2`               | `Fan`          | one lamp for either fan: a dash with one bulb should light it for whichever is running |
//!
//! base+11 bit 0 is `brakePedal`, which drives `Brake`. That lamp is the
//! pedal being pressed here, not a fault: this protocol has no brake warning
//! and a display wanting one should not read this.

use crate::decode::{conv::*, CanFrameMap, CanSource, Field, Flag, LampFrame};
use crate::{field, flag};

pub const DEFAULT_BASE: u16 = 0x200;

/// kPa*30 into tenths of a kPa.
fn kpa_x30(v: i32) -> i32 {
    div_round(v, 3)
}
/// deg*50 into tenths of a degree.
fn deg_x50(v: i32) -> i32 {
    div_round(v, 5)
}
/// %*2 into tenths of a percent.
fn pct_x2(v: i32) -> i32 {
    v * 5
}
/// ms*300 into microseconds.
fn ms_x300(v: i32) -> i32 {
    div_round(v * 10, 3)
}
/// bar*10 into tenths of a kPa.
fn bar_x10(v: i32) -> i32 {
    v * 100
}
/// (g/s)*200 into hundredths of a kg/h: 1 g/s is 3.6 kg/h.
fn gps_x200(v: i32) -> i32 {
    div_round(v * 18, 10)
}
/// Grams into milligrams, which is what the mass gauges store.
fn grams(v: i32) -> i32 {
    v * 1000
}

// base+0 Status
// Offset 4 is a byte of status bits (rev limit, relays, fans, lambda
// protection); those belong in `indicators`, not in a gauge of their own.
static STATUS: &[Field] = &[
    field!(0, U16, ERROR_COUNT, raw),
    field!(2, U16, NEXT_ERROR, raw),
    field!(5, U8, GEAR, raw),
    // Trip metres divided by 100, which is already what a KILOMETRES count is.
    field!(6, U16, TRIP_DIST, raw),
];

// base+1 Speeds
static SPEEDS: &[Field] = &[
    field!(0, U16, RPM, raw),
    field!(2, S16, CUR_SPARK_ADVANCE, deg_x50),
    field!(4, U8, INJ_DUTY, pct_x2),
    field!(5, U8, COIL_DUTY, pct_x2),
    field!(6, U8, VSS, x10),
    field!(7, U8, ETHANOL_PERCENT, x10),
];

// base+2 PedalAndTps (offset 6 is wastegate valve position, shown as boost duty)
static PEDAL_AND_TPS: &[Field] = &[
    field!(0, S16, PEDAL, hundredths_to_tenths),
    field!(2, S16, TPS, hundredths_to_tenths),
    field!(4, S16, TPS2, hundredths_to_tenths),
    field!(6, S16, BOOST_PWM, hundredths_to_tenths),
];

// base+3 Sensors1
static SENSORS1: &[Field] = &[
    field!(0, U16, MAP, kpa_x30),
    field!(2, U8, CLNT, celsius_plus40),
    field!(3, U8, IAT, celsius_plus40),
    field!(4, U8, AUX_TEMP1, celsius_plus40),
    field!(5, U8, AUX_TEMP2, celsius_plus40),
    field!(6, U8, ECU_TEMP, celsius_plus40),
    field!(7, U8, FUEL_LEVEL, pct_x2),
];

// base+4 Sensors2 (offsets 0-1 are padding)
static SENSORS2: &[Field] = &[
    field!(2, U16, OIL_PRES, kpa_x30),
    field!(4, U8, OIL_TEMP, celsius_plus40),
    field!(5, U8, FLEX_FUEL_TEMP, celsius_plus40),
    field!(6, U16, BAT_VOL, raw),
];

// base+5 Fueling. Air mass per cylinder arrives as grams*1000, which is
// milligrams, and that is what the mass gauges store.
static FUELING: &[Field] = &[
    field!(0, U16, CYL_AIRMASS, raw),
    field!(2, U16, AIRFLOW, raw),
    field!(4, S16, PULSE_WIDTH1, ms_x300),
    field!(6, U16, KNOCK_COUNT, raw),
];

// base+6 Fueling2: fuel burned and burning, then trim for each bank
static FUELING2: &[Field] = &[
    field!(0, U16, FUEL_USED, grams),
    field!(2, U16, FUEL_FLOW, gps_x200),
    field!(4, S16, EGO_CORRECT, hundredths_to_tenths),
    field!(6, S16, EGO_CORRECT2, hundredths_to_tenths),
];

// base+7 Fueling3
static FUELING3: &[Field] = &[
    field!(0, U16, AFR_PRI, lambda_ten_thousandths),
    field!(2, U16, AFR_SEC, lambda_ten_thousandths),
    field!(4, U16, FUEL_PRES, kpa_x30),
    field!(6, U16, FUEL_PRES_HIGH, bar_x10),
];

// base+8 Cams: bank1 intake actual/target, exhaust actual/target, then bank 2
static CAMS: &[Field] = &[
    field!(0, S8, VVT_ANGLE, x10),
    field!(1, S8, VVT_TARGET_ANGLE, x10),
    field!(2, S8, VVT_EXH_ANGLE, x10),
    field!(3, S8, VVT_EXH_TARGET, x10),
    field!(4, S8, VVT_ANGLE2, x10),
    field!(5, S8, VVT_TARGET_ANGLE2, x10),
    field!(6, S8, VVT_EXH_ANGLE2, x10),
    field!(7, S8, VVT_EXH_TARGET2, x10),
];

// base+9 Egts: eight bytes, 5 °C each, one per cylinder. All eight are read:
// on a full set the reason to look is which probe disagrees with the others,
// and keeping only the first turns that into a single number that cannot.
static EGTS: &[Field] = &[
    field!(0, U8, EGT1, x50),
    field!(1, U8, EGT2, x50),
    field!(2, U8, EGT3, x50),
    field!(3, U8, EGT4, x50),
    field!(4, U8, EGT5, x50),
    field!(5, U8, EGT6, x50),
    field!(6, U8, EGT7, x50),
    field!(7, U8, EGT8, x50),
];

// base+0 byte 4, and base+11 bit 0. See the table in the module header for
// what each bit was taken to mean.
static STATUS_LAMPS: &[Flag] = &[
    flag!(4, 0, RevLimit),
    flag!(4, 2, FuelPump),
    flag!(4, 3, CheckEngine),
    flag!(4, 5, LeanWarn),
    flag!(4, 6, Fan),
    flag!(4, 7, Fan),
];

static BRAKE_LAMP: &[Flag] = &[flag!(0, 0, Brake)];

// base+10 PerCylinderKnock: a signed level per cylinder
static KNOCK: &[Field] = &[
    field!(0, S8, KNOCK1, raw),
    field!(1, S8, KNOCK2, raw),
    field!(2, S8, KNOCK3, raw),
    field!(3, S8, KNOCK4, raw),
    field!(4, S8, KNOCK5, raw),
    field!(5, S8, KNOCK6, raw),
    field!(6, S8, KNOCK7, raw),
    field!(7, S8, KNOCK8, raw),
];

pub static SOURCE: CanSource = CanSource {
    name: "rusEFI",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap {
            id_offset: 0,
            fields: STATUS,
        },
        CanFrameMap {
            id_offset: 1,
            fields: SPEEDS,
        },
        CanFrameMap {
            id_offset: 2,
            fields: PEDAL_AND_TPS,
        },
        CanFrameMap {
            id_offset: 3,
            fields: SENSORS1,
        },
        CanFrameMap {
            id_offset: 4,
            fields: SENSORS2,
        },
        CanFrameMap {
            id_offset: 5,
            fields: FUELING,
        },
        CanFrameMap {
            id_offset: 6,
            fields: FUELING2,
        },
        CanFrameMap {
            id_offset: 7,
            fields: FUELING3,
        },
        CanFrameMap {
            id_offset: 8,
            fields: CAMS,
        },
        CanFrameMap {
            id_offset: 9,
            fields: EGTS,
        },
        CanFrameMap {
            id_offset: 10,
            fields: KNOCK,
        },
    ],
    lamps: &[
        LampFrame {
            id_offset: 0,
            flags: STATUS_LAMPS,
        },
        LampFrame {
            id_offset: 11,
            flags: BRAKE_LAMP,
        },
    ],
};
