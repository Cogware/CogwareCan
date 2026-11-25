// SPDX-License-Identifier: GPL-3.0-only
//! MaxxECU default CAN output converter (protocol V1.3, 2020-09-29).
//!
//! Every channel is a little-endian `int16` at a fixed 11-bit ID, 500 kbit/s.
//! Most channels are already in tenths (kPa*10, %*10, °C*10, deg*10, km/h*10)
//! and map straight onto the gauge table; lambda is λ*1000, battery V*100,
//! pulse width ms*100, EGT whole °C.
//!
//! Not mapped: the twenty user-defined channels (0x525, 0x535, 0x538, 0x539
//! and 0x540's table selectors), which mean whatever the tuner configured and
//! so cannot be given a name or a unit here.
//!
//! # Status bits, and the lamps they drive
//!
//! Frame 0x526 carries two bytes of flags. Each row below is a decision that
//! MaxxECU's flag and this crate's lamp mean the same thing, so each is
//! written down rather than left to be inferred:
//!
//! | byte.bit | MaxxECU              | lamp              | why |
//! |----------|----------------------|-------------------|-----|
//! | 0.0      | shiftcut active      | `GearShiftCut`    | |
//! | 0.1      | rev-limit active     | `RevLimit`        | |
//! | 0.2      | anti-lag active      | `Antilag`         | |
//! | 0.3      | launch control active| `LaunchActive`    | active, not armed: this bit is the launch running |
//! | 0.4      | traction power limit | `TractionControl` | the limiter cutting power is what the lamp means on a road car |
//! | 0.5      | throttle blip active | --                | a downshift blip lasts a few hundred milliseconds; a lamp cannot show it and nobody would act on it |
//! | 0.6      | AC / idle up active  | --                | this is the idle controller doing its job, not a condition; `IdleControl` is kept for an ECU that reports idle control as a state |
//! | 0.7      | knock detected       | `Knock`           | |
//! | 1.0      | brake pedal active   | `Brake`           | the pedal pressed, not a brake fault |
//! | 1.1      | clutch pedal active  | --                | no lamp in the table, and a clutch light on a dash would read as a fault |
//! | 1.2      | speed limit active   | `PitLimiter`      | a speed limiter engaged is what that lamp is for |
//! | 1.3      | GP limiter active    | --                | general purpose: what it limits is whatever the tuner configured, so no lamp can be true |
//! | 1.4      | user cut active      | --                | same reason |
//! | 1.5      | ECU logging active   | `SdLogging`       | |
//! | 1.6      | nitrous active       | `NitrousActive`   | |

use crate::decode::{conv::*, CanFrameMap, CanSource, Field, Flag, LampFrame};
use crate::{field, flag};

pub const DEFAULT_BASE: u16 = 0;

static F520: &[Field] = &[
    field!(0, S16, RPM, raw),
    field!(2, S16, TPS, raw),
    field!(4, S16, MAP, raw),
];

static F521: &[Field] = &[
    field!(0, S16, AFR_PRI, lambda_thousandths),
    field!(2, S16, AFR_SEC, lambda_thousandths),
    field!(4, S16, CUR_SPARK_ADVANCE, raw),
    field!(6, S16, IGN_CUT, x10),
];

static F522: &[Field] = &[
    field!(0, S16, PULSE_WIDTH1, hundredths_to_thousandths),
    field!(2, S16, INJ_DUTY, raw),
    field!(4, S16, FUEL_CUT, x10),
    field!(6, S16, VSS, raw),
];

// Driven against undriven is the whole of what traction control sees.
static F523: &[Field] = &[
    field!(0, S16, WHEEL_SPEED_UNDRIVEN, raw),
    field!(2, S16, WHEEL_SPEED_DRIVEN, raw),
    field!(4, S16, WHEEL_SLIP, raw),
    field!(6, S16, SLIP_TARGET, raw),
];

// lambda correction bank A; offset 6 is the firmware version
static F524: &[Field] = &[
    field!(0, S16, TRACTION_LIMIT, raw),
    field!(2, S16, EGO_CORRECT, raw),
    field!(4, S16, EGO_CORRECT2, raw),
];

// 0x526 is mostly limiter and driver-input bits; only the limiter's rpm is a
// number. See the table in the module header for what each bit was taken to
// mean, and which were deliberately left alone.
static F526: &[Field] = &[field!(4, S16, REV_LIMIT, raw)];

static F526_LAMPS: &[Flag] = &[
    flag!(0, 0, GearShiftCut),
    flag!(0, 1, RevLimit),
    flag!(0, 2, Antilag),
    flag!(0, 3, LaunchActive),
    flag!(0, 4, TractionControl),
    flag!(0, 7, Knock),
    flag!(1, 0, Brake),
    flag!(1, 2, PitLimiter),
    flag!(1, 5, SdLogging),
    flag!(1, 6, NitrousActive),
];

static F527: &[Field] = &[
    field!(0, S16, ACCEL_FWD, raw),
    field!(2, S16, ACCEL_RIGHT, raw),
    field!(4, S16, ACCEL_UP, raw),
    field!(6, S16, AFR_TARGET, lambda_thousandths),
];

static F528: &[Field] = &[
    field!(0, S16, KNOCK_LEVEL, raw),
    field!(2, S16, KNOCK_RETARD, raw),
    field!(4, S16, KNOCK_COUNT, raw),
    field!(6, S16, KNOCK_CYL, raw),
];

static F530: &[Field] = &[
    field!(0, S16, BAT_VOL, hundredths_to_thousandths),
    field!(2, S16, BARO, raw),
    field!(4, S16, IAT, raw),
    field!(6, S16, CLNT, raw),
];

static F531: &[Field] = &[
    field!(2, S16, ETHANOL_PERCENT, raw),
    field!(4, S16, IGN_COMP, raw),
    field!(6, S16, EGT1, x10),
];

// Whole degrees per probe here, unlike the tenths most of this protocol uses.
static F532: &[Field] = &[
    field!(0, S16, EGT2, x10),
    field!(2, S16, EGT3, x10),
    field!(4, S16, EGT4, x10),
    field!(6, S16, EGT5, x10),
];

// Offset 6 is the highest of the probes, which a display can work out itself.
static F533: &[Field] = &[
    field!(0, S16, EGT6, x10),
    field!(2, S16, EGT7, x10),
    field!(4, S16, EGT8, x10),
];

// Offset 0 is the spread between the probes, also derivable from the eight.
static F534: &[Field] = &[
    field!(2, S16, ECU_TEMP, x10),
    field!(4, S16, ERROR_COUNT, raw),
    field!(6, S16, LOST_SYNC, raw),
];

static F536: &[Field] = &[
    field!(0, S16, GEAR, raw),
    field!(2, S16, BOOST_PWM, raw),
    field!(4, S16, OIL_PRES, raw),
    field!(6, S16, OIL_TEMP, raw),
];

static F537: &[Field] = &[
    field!(0, S16, FUEL_PRES, raw),
    field!(2, S16, WASTEGATE_PRES, raw),
    field!(4, S16, COOLANT_PRES, raw),
    field!(6, S16, BOOST_TARGET, raw),
];

static F540: &[Field] = &[
    field!(2, S16, FUEL_VOLUME, raw),
    field!(4, S16, TRANS_TEMP, raw),
    field!(6, S16, DIFF_TEMP, raw),
];

// cam positions, then the targets
static F541: &[Field] = &[
    field!(0, S16, VVT_ANGLE, raw),
    field!(2, S16, VVT_EXH_ANGLE, raw),
    field!(4, S16, VVT_ANGLE2, raw),
    field!(6, S16, VVT_EXH_ANGLE2, raw),
];
static F542: &[Field] = &[
    field!(0, S16, VVT_TARGET_ANGLE, raw),
    field!(2, S16, VVT_EXH_TARGET, raw),
    field!(4, S16, NEXT_ERROR, raw),
];

pub static SOURCE: CanSource = CanSource {
    name: "MaxxECU",
    default_base: DEFAULT_BASE,
    frames: &[
        CanFrameMap {
            id_offset: 0x520,
            fields: F520,
        },
        CanFrameMap {
            id_offset: 0x521,
            fields: F521,
        },
        CanFrameMap {
            id_offset: 0x522,
            fields: F522,
        },
        CanFrameMap {
            id_offset: 0x523,
            fields: F523,
        },
        CanFrameMap {
            id_offset: 0x524,
            fields: F524,
        },
        CanFrameMap {
            id_offset: 0x526,
            fields: F526,
        },
        CanFrameMap {
            id_offset: 0x527,
            fields: F527,
        },
        CanFrameMap {
            id_offset: 0x528,
            fields: F528,
        },
        CanFrameMap {
            id_offset: 0x530,
            fields: F530,
        },
        CanFrameMap {
            id_offset: 0x531,
            fields: F531,
        },
        CanFrameMap {
            id_offset: 0x532,
            fields: F532,
        },
        CanFrameMap {
            id_offset: 0x533,
            fields: F533,
        },
        CanFrameMap {
            id_offset: 0x534,
            fields: F534,
        },
        CanFrameMap {
            id_offset: 0x536,
            fields: F536,
        },
        CanFrameMap {
            id_offset: 0x537,
            fields: F537,
        },
        CanFrameMap {
            id_offset: 0x540,
            fields: F540,
        },
        CanFrameMap {
            id_offset: 0x541,
            fields: F541,
        },
        CanFrameMap {
            id_offset: 0x542,
            fields: F542,
        },
    ],
    lamps: &[LampFrame {
        id_offset: 0x526,
        flags: F526_LAMPS,
    }],
};
