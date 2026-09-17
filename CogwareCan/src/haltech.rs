// SPDX-License-Identifier: GPL-3.0-only
//! Haltech CAN Broadcast Protocol converter (Elite and Nexus, protocol V2.35).
//!
//! Every channel is a big-endian 16-bit word at a fixed 11-bit ID (the base is
//! not configurable). Temperatures are Kelvin*10, pressures kPa*10 with
//! sensor pressures sent *absolute* (subtract 101.3 kPa for gauge pressure),
//! percentages %*10, angles deg*10, wideband lambda*1000, battery V*10,
//! speed km/h*10.
//!
//! Fuel level arrives in litres and feeds `FUEL_VOLUME`, not `FUEL_LEVEL`.
//! Not mapped: injection stages 3-4, launch and traction data.
//!
//! The exhaust probes are the one place here where the layout is inferred
//! rather than read off a per-offset table: the protocol allots EGT 1-12 to
//! 0x373..=0x375 at a tenth of a Kelvin each, and four 16-bit channels per
//! eight-byte frame is the only arrangement that fits. Probes past the eighth
//! have no gauge, so 0x375 is left alone.

use crate::decode::{conv::*, CanFrameMap, CanSource, Field};
use crate::field;

/// IDs are fixed, so the base is zero and each `id_offset` is the absolute ID.
pub const DEFAULT_BASE: u16 = 0;

static F360: &[Field] = &[
    field!(0, U16BE, RPM, raw),
    field!(2, U16BE, MAP, raw),
    field!(4, U16BE, TPS, raw),
    field!(6, U16BE, COOLANT_PRES, kpa_abs_tenths_to_gauge),
];

// offset 4 is Engine Demand, not pedal position
static F361: &[Field] = &[
    field!(0, U16BE, FUEL_PRES, kpa_abs_tenths_to_gauge),
    field!(2, U16BE, OIL_PRES, kpa_abs_tenths_to_gauge),
    field!(6, U16BE, WASTEGATE_PRES, kpa_abs_tenths_to_gauge),
];

static F362: &[Field] = &[
    field!(0, U16BE, INJ_DUTY, raw),
    field!(2, U16BE, INJ_DUTY2, raw),
    field!(4, S16BE, CUR_SPARK_ADVANCE, raw),
    field!(6, S16BE, IGN_ANGLE_TRAIL, raw),
];

static F368: &[Field] = &[
    field!(0, U16BE, AFR_PRI, lambda_thousandths),
    field!(2, U16BE, AFR_SEC, lambda_thousandths),
];

// Trigger health. Counters, so what matters is whether they climb.
static F369: &[Field] = &[
    field!(0, U16BE, MISS_COUNT, raw),
    field!(2, U16BE, TRIGGER_COUNT, raw),
    field!(4, U16BE, HOME_COUNT, raw),
    field!(6, U16BE, TRIG_SINCE_HOME, raw),
];

static F36F: &[Field] = &[field!(2, U16BE, BOOST_PWM, raw)];

// offsets 2-3 are undefined
static F370: &[Field] = &[
    field!(0, U16BE, VSS, raw),
    field!(4, S16BE, VVT_ANGLE, raw),
    field!(6, S16BE, VVT_ANGLE2, raw),
];

// offsets 2-3 are undefined
static F372: &[Field] = &[
    field!(0, U16BE, BAT_VOL, tenths_to_thousandths),
    field!(4, U16BE, BOOST_TARGET, raw),
    field!(6, U16BE, BARO, raw),
];

static F373: &[Field] = &[
    field!(0, U16BE, EGT1, kelvin_tenths),
    field!(2, U16BE, EGT2, kelvin_tenths),
    field!(4, U16BE, EGT3, kelvin_tenths),
    field!(6, U16BE, EGT4, kelvin_tenths),
];

static F374: &[Field] = &[
    field!(0, U16BE, EGT5, kelvin_tenths),
    field!(2, U16BE, EGT6, kelvin_tenths),
    field!(4, U16BE, EGT7, kelvin_tenths),
    field!(6, U16BE, EGT8, kelvin_tenths),
];

static F3E0: &[Field] = &[
    field!(0, U16BE, CLNT, kelvin_tenths),
    field!(2, U16BE, IAT, kelvin_tenths),
    field!(4, U16BE, FLEX_FUEL_TEMP, kelvin_tenths),
    field!(6, U16BE, OIL_TEMP, kelvin_tenths),
];

static F3E1: &[Field] = &[
    field!(0, U16BE, TRANS_TEMP, kelvin_tenths),
    field!(4, U16BE, ETHANOL_PERCENT, raw),
];

static F3E2: &[Field] = &[
    field!(0, U16BE, FUEL_VOLUME, raw),
    field!(2, U16BE, FUEL_RATE, raw),
];

static F3E3: &[Field] = &[field!(0, S16BE, EGO_CORRECT, raw)];

static F3E9: &[Field] = &[field!(4, U16BE, AFR_TARGET, lambda_thousandths)];

static F3EB: &[Field] = &[
    field!(4, S16BE, ADVANCE1, raw),
    field!(6, S16BE, ADVANCE2, raw),
];

// byte 7: -1 reverse, 0 neutral, 1.. forward
static F470: &[Field] = &[field!(7, S8, GEAR, raw)];

static F471: &[Field] = &[field!(2, U16BE, PEDAL, raw)];

pub static SOURCE: CanSource = CanSource {
    name: "Haltech",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap { id_offset: 0x360, fields: F360 },
        CanFrameMap { id_offset: 0x361, fields: F361 },
        CanFrameMap { id_offset: 0x362, fields: F362 },
        CanFrameMap { id_offset: 0x368, fields: F368 },
        CanFrameMap { id_offset: 0x369, fields: F369 },
        CanFrameMap { id_offset: 0x36F, fields: F36F },
        CanFrameMap { id_offset: 0x370, fields: F370 },
        CanFrameMap { id_offset: 0x372, fields: F372 },
        CanFrameMap { id_offset: 0x373, fields: F373 },
        CanFrameMap { id_offset: 0x374, fields: F374 },
        CanFrameMap { id_offset: 0x3E0, fields: F3E0 },
        CanFrameMap { id_offset: 0x3E1, fields: F3E1 },
        CanFrameMap { id_offset: 0x3E2, fields: F3E2 },
        CanFrameMap { id_offset: 0x3E3, fields: F3E3 },
        CanFrameMap { id_offset: 0x3E9, fields: F3E9 },
        CanFrameMap { id_offset: 0x3EB, fields: F3EB },
        CanFrameMap { id_offset: 0x470, fields: F470 },
        CanFrameMap { id_offset: 0x471, fields: F471 },
    ],
    // No status bits mapped: this protocol's flag words are not documented
    // per-bit in the material this converter was written from, and a lamp lit
    // on a guess is worse than one left dark.
    lamps: &[],
};
