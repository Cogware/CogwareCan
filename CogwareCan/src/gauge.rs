// SPDX-License-Identifier: GPL-3.0-only
//! The gauge specification: every value the displays can show, its CAN ID,
//! its canonical unit, and how it travels on the wire.
//!
//! This table is the single source of truth. Converters (see `speeduino`)
//! normalise whatever an ECU sends into the canonical unit and store it here;
//! displays read from here and never need to know which ECU produced it.
//!
//! Stored values are fixed point: `physical value * unit.scale`. A coolant
//! reading of 87.5 °C is stored as 875 because `Unit::CELSIUS.scale == 10`.
//! Use `GaugeData::as_f32` when a float is wanted for display.

use crate::frame::CanFrame;
use crate::pack::Wire;
use crate::units::{Reading, Unit};
use core::cell::Cell;
use critical_section::Mutex;
use embedded_can::{Frame, Id, StandardId};
use paste::paste;

/// The value a gauge reads as when nothing has fed it.
///
/// Every reader that can carry absence should use [`GaugeData::get`] and its
/// `Option`. This exists for the ones that cannot: a scene binding wanting a
/// plain number, a debug view printing a column, a frame payload with a fixed
/// width. Those need one agreed value meaning "no source is filling this", and
/// this is it.
///
/// It is deliberately absurd rather than merely unusual. Gauges are fixed
/// point, so a reader that mistakes it for a reading gets -88,927,571.4 as a
/// temperature or a pressure -- a number no sensor fault produces and nobody
/// mistakes for data. A tidier sentinel like zero or -1 is a real reading for
/// half the table: zero rpm is a stalled engine, -1 °C is a cold morning.
///
/// The sentinel is never stored. [`GaugeData::set`] treats it as "I have no
/// reading for this channel" and stores absence, so a gauge holds a reading or
/// holds nothing, and this value only ever appears on the way out.
pub const NO_DATA: i32 = 0xCAFE_BABE_u32 as i32;

/// Which kind of source can populate a gauge. A display should only rely on
/// a gauge being present if the attached source is at least this specific.
/// Ordered, so `gauge.source <= Source::Standalone` is a valid check.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Source {
    /// Available from a stock car over OBD2 mode 01 as well as every standalone.
    Common,
    /// Standalone ECUs (Speeduino, Megasquirt, rusEFI, Haltech, MaxxECU,
    /// ECUMaster, ...) broadcast it; OBD2 has no PID for it.
    Standalone,
    /// Speeduino-only: status bitfields, firmware diagnostics, and the
    /// individual fuel-correction terms other ECUs only send summed.
    Speeduino,
    /// Bus housekeeping, produced by the server itself.
    Protocol,
    /// Needs a GPS receiver wired to the server. No ECU supplies it, so this
    /// sits outside the ladder above rather than further along it.
    Gps,
}

/// Everything about a gauge that never changes: what it is called, where it
/// sits on the bus, how it is packed, what it means.
///
/// Split from the value so that it can live in flash. A gauge holding a value
/// needs interior mutability, which forces the whole static into RAM; this
/// half has none, so the linker puts it with the other constants and the RAM
/// cost of a gauge falls from forty-eight bytes to sixteen.
///
/// That is not tidiness. The table is the vocabulary of the car and it grows
/// every time an ECU is added; on a part with under two hundred kilobytes of
/// internal RAM shared between the radio's heap, two stacks and everything
/// static, the difference decides whether a build boots.
///
/// Reached through `Deref` on [`GaugeData`], so `gauge.name` and `gauge.unit`
/// read as though the fields were still there.
pub struct GaugeSpec {
    pub name: &'static str,
    pub id: u16,
    pub wire: Wire,
    pub unit: Unit,
    pub source: Source,
}

pub struct GaugeData {
    spec: &'static GaugeSpec,
    value: Mutex<Cell<Option<i32>>>,
    /// How many times this gauge has been written.
    ///
    /// A count rather than a time, because this crate has no clock and the
    /// boards it runs on do not agree about what one would be. A reader that
    /// remembers the count and looks again later learns the one thing a value
    /// cannot tell it: whether anything is still writing this gauge.
    ///
    /// That distinction is the whole of knowing a dead sender from a steady
    /// one. A coolant sensor that has failed holds its last reading for ever,
    /// and the number looks perfectly reasonable.
    writes: Mutex<Cell<u32>>,
}

impl core::fmt::Debug for GaugeData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{}(0x{:03X}) = {:?}{}",
            self.name,
            self.id,
            self.get(),
            self.unit.symbol
        )
    }
}

/// Two gauges are equal only if they are the same table entry.
impl PartialEq for GaugeData {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self, other)
    }
}
impl Eq for GaugeData {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// The payload length does not match the gauge's wire width.
    BadLength { expected: usize, got: usize },
}

impl core::ops::Deref for GaugeData {
    type Target = GaugeSpec;
    fn deref(&self) -> &GaugeSpec {
        self.spec
    }
}

impl GaugeSpec {
    pub const fn new(name: &'static str, id: u16, wire: Wire, unit: Unit, source: Source) -> Self {
        GaugeSpec {
            name,
            id,
            wire,
            unit,
            source,
        }
    }
}

impl GaugeData {
    /// A gauge holding nothing, described by a spec that lives in flash.
    pub const fn new(spec: &'static GaugeSpec) -> Self {
        GaugeData {
            spec,
            value: Mutex::new(Cell::new(None)),
            writes: Mutex::new(Cell::new(0)),
        }
    }

    /// Fixed-point value, or `None` if nothing has written this gauge yet.
    pub fn get(&self) -> Option<i32> {
        critical_section::with(|cs| self.value.borrow(cs).get())
    }

    pub fn get_or(&self, default: i32) -> i32 {
        self.get().unwrap_or(default)
    }

    /// The fixed-point value as a reader that cannot carry absence sees it:
    /// the counts, or [`NO_DATA`] when nothing has fed this gauge.
    ///
    /// This is the read for a display: it never has to ask whether a source
    /// is attached, only whether the number it got is the sentinel.
    pub fn raw(&self) -> i32 {
        self.get_or(NO_DATA)
    }

    /// Value in physical units (counts divided by `unit.scale`).
    pub fn as_f32(&self) -> Option<f32> {
        self.reading().map(|r| r.value())
    }

    /// Value with its unit attached, for conversion: `MAP.reading()?.psi()`.
    pub fn reading(&self) -> Option<Reading> {
        self.get().map(|counts| Reading {
            counts,
            unit: self.unit,
        })
    }

    /// Value converted to `target`: `MAP.to(Unit::PSI)`.
    /// `None` if unset or if `target` is a different physical dimension.
    pub fn to(&self, target: Unit) -> Option<f32> {
        self.reading()?.to(target)
    }

    /// Store a reading, or [`NO_DATA`] to say this source has none.
    ///
    /// Either way it counts as a write. A source that keeps saying it has
    /// nothing for a channel is still a source that is running, and that is
    /// a different thing from one that has died -- which is the whole
    /// question `writes` exists to answer. [`clear`](Self::clear) is the
    /// other case: the link dropped and nothing at all is being said.
    pub fn set(&self, value: i32) {
        critical_section::with(|cs| {
            let stored = if value == NO_DATA { None } else { Some(value) };
            self.value.borrow(cs).set(stored);
            let writes = self.writes.borrow(cs);
            // Wrapping, and that is fine: a reader compares for difference,
            // never for order, so the only way round is the one in a hundred
            // million where a gauge is written exactly 2^32 times between two
            // looks.
            writes.set(writes.get().wrapping_add(1));
        });
    }

    /// How many times this gauge has been written since the board started.
    ///
    /// Compare it with what it was a moment ago: unchanged means nothing is
    /// writing this gauge, whatever its value says.
    pub fn writes(&self) -> u32 {
        critical_section::with(|cs| self.writes.borrow(cs).get())
    }

    pub fn clear(&self) {
        critical_section::with(|cs| self.value.borrow(cs).set(None));
    }

    pub fn is_set(&self) -> bool {
        self.get().is_some()
    }

    /// Frame carrying this gauge, or `None` if it has never been set.
    pub fn to_frame(&self) -> Option<CanFrame> {
        let value = self.get()?;
        let mut buf = [0u8; 4];
        let n = self.wire.encode(value, &mut buf);
        CanFrame::new(Id::Standard(StandardId::new(self.id)?), &buf[..n])
    }

    /// Store the payload of a frame addressed to this gauge.
    pub fn set_from_frame(&self, frame: &CanFrame) -> Result<i32, FrameError> {
        let data = &frame.data()[..frame.dlc()];
        let value = self.wire.decode(data).ok_or(FrameError::BadLength {
            expected: self.wire.len(),
            got: data.len(),
        })?;
        self.set(value);
        Ok(value)
    }
}

/// Generates, from one row per gauge: the `static` for each gauge, the
/// `Gauge` enum with matching discriminants, ID lookup, and `ALL_GAUGES`.
macro_rules! gauges {
    ($($name:ident = $id:literal : $wire:ident, $unit:ident, $source:ident;)+) => {
        // The specs go in their own module so that a gauge called, say,
        // `RPM` and its spec can share a name without colliding, and so that
        // nothing outside reaches for a spec when it wants the gauge.
        mod spec {
            use super::*;
            $(
                pub static $name: GaugeSpec = GaugeSpec::new(
                    stringify!($name), $id, Wire::$wire, Unit::$unit, Source::$source);
            )+
        }
        $(
            pub static $name: GaugeData = GaugeData::new(&spec::$name);
            // The client request protocol carries gauge IDs as single bytes.
            const _: () = assert!($id <= 0xFF, "gauge ID must fit in one request byte");
        )+

        /// Every gauge in table order.
        pub static ALL_GAUGES: &[&'static GaugeData] = &[$(&$name),+];

        paste! {
            #[repr(u16)]
            #[derive(Clone, Copy, Debug, PartialEq, Eq)]
            pub enum Gauge {
                $([<$name:camel>] = $id,)+
            }

            impl Gauge {
                pub const fn data(self) -> &'static GaugeData {
                    match self {
                        $(Gauge::[<$name:camel>] => &$name,)+
                    }
                }

                pub const fn id(self) -> u16 {
                    self as u16
                }

                pub const fn from_id(id: u16) -> Option<Gauge> {
                    match id {
                        $($id => Some(Gauge::[<$name:camel>]),)+
                        _ => None,
                    }
                }
            }
        }
    };
}

/// Look up a gauge by its CAN ID.
pub fn gauge_by_id(id: u16) -> Option<&'static GaugeData> {
    Gauge::from_id(id).map(Gauge::data)
}

// CAN ID map. 0x000..=0x01F is reserved for protocol frames (see `protocol`).
// Lower IDs win bus arbitration, so more urgent gauges belong at lower IDs.
gauges! {
    STA_TIME            = 0x20: U8,  SECONDS,       Common;
    STA_STATUS1         = 0x21: U8,  RAW,           Speeduino;
    STA_ENG             = 0x22: U8,  RAW,           Speeduino;
    DWELL               = 0x23: U16, MS,            Standalone;
    MAP                 = 0x24: U16, KPA,           Common;
    IAT                 = 0x25: I16, CELSIUS,       Common;
    CLNT                = 0x26: I16, CELSIUS,       Common;
    BAT_CORRECT         = 0x27: U16, PERCENT,       Speeduino;
    BAT_VOL             = 0x28: U16, VOLT,          Common;
    AFR_PRI             = 0x29: U16, AFR,           Common;
    // Closed-loop fuel trim, 0 = no correction (Speeduino's 100-centred value is shifted).
    EGO_CORRECT         = 0x2A: I16, PERCENT,       Common;
    IAT_CORRECT         = 0x2B: U16, PERCENT,       Speeduino;
    WUE_CORRECT         = 0x2C: U16, PERCENT,       Speeduino;
    RPM                 = 0x2D: U16, RPM,           Common;
    ACCEL_ENRICH        = 0x2E: U16, PERCENT,       Speeduino;
    GAMME_E             = 0x2F: U16, PERCENT,       Speeduino;
    VE                  = 0x30: U16, PERCENT,       Standalone;
    AFR_TARGET          = 0x31: U16, AFR,           Common;
    PULSE_WIDTH1        = 0x32: U16, MS,            Standalone;
    TPS_DOT             = 0x33: I16, PERCENT_PER_S, Standalone;
    CUR_SPARK_ADVANCE   = 0x34: I16, DEGREES,       Common;
    TPS                 = 0x35: U16, PERCENT,       Common;
    LOOP_PS             = 0x36: U16, HZ,            Speeduino;
    FREE_MEM            = 0x37: U16, BYTES,         Speeduino;
    BOOST_TARGET        = 0x38: U16, KPA,           Standalone;
    BOOST_PWM           = 0x39: U16, PERCENT,       Standalone;
    STA_SPARK           = 0x3A: U8,  RAW,           Speeduino;
    RPM_DOT             = 0x3B: I16, RPM_PER_S,     Standalone;
    ETHANOL_PERCENT     = 0x3C: U16, PERCENT,       Common;
    FLEX_CORRECT        = 0x3D: U16, PERCENT,       Speeduino;
    FLEX_IGN_CORRECT    = 0x3E: I16, DEGREES,       Speeduino;
    IDLE_LOAD           = 0x3F: U8,  RAW,           Standalone;
    TEST_OUTPUTS        = 0x40: U8,  RAW,           Speeduino;
    AFR_SEC             = 0x41: U16, AFR,           Standalone;
    BARO                = 0x42: U16, KPA,           Common;
    TPS_ADC             = 0x43: U8,  RAW,           Speeduino;
    NEXT_ERROR          = 0x44: U8,  RAW,           Common;
    STA_LAUNCH_CORRECT  = 0x45: U16, PERCENT,       Standalone;
    PULSE_WIDTH2        = 0x46: U16, MS,            Standalone;
    PULSE_WIDTH3        = 0x47: U16, MS,            Standalone;
    PULSE_WIDTH4        = 0x48: U16, MS,            Standalone;
    STA_STATUS2         = 0x49: U8,  RAW,           Speeduino;
    ENG_PROTECT_STA     = 0x4A: U8,  RAW,           Speeduino;
    FUEL_LOAD           = 0x4B: I16, RAW,           Common;
    IGN_LOAD            = 0x4C: I16, RAW,           Common;
    INJ_ANGLE           = 0x4D: U16, DEGREES,       Standalone;
    IDLE_DUTY           = 0x4E: U16, PERCENT,       Standalone;
    CL_IDLE_TARGET      = 0x4F: U16, RPM,           Standalone;
    MAP_DOT             = 0x50: I16, KPA_PER_S,     Standalone;
    VVT_ANGLE           = 0x51: I16, DEGREES,       Standalone;
    VVT_TARGET_ANGLE    = 0x52: U16, DEGREES,       Standalone;
    VVT_DUTY            = 0x53: U16, PERCENT,       Standalone;
    FLEX_BOOST_CORRECT  = 0x54: I16, KPA,           Speeduino;
    BARO_CORRECTION     = 0x55: U16, PERCENT,       Speeduino;
    ASE                 = 0x56: U16, PERCENT,       Speeduino;
    VSS                 = 0x57: U16, KMH,           Common;
    GEAR                = 0x58: U8,  RAW,           Common;
    FUEL_PRES           = 0x59: U16, KPA,           Common;
    OIL_PRES            = 0x5A: U16, KPA,           Standalone;
    WMI_PW              = 0x5B: U16, PERCENT,       Standalone;
    STA_STATUS4         = 0x5C: U8,  RAW,           Speeduino;
    VVT_ANGLE2          = 0x5D: I16, DEGREES,       Standalone;
    VVT_TARGET_ANGLE2   = 0x5E: U16, DEGREES,       Standalone;
    VVT_DUTY2           = 0x5F: U16, PERCENT,       Standalone;
    STATUS_OUT_STA      = 0x60: U8,  RAW,           Speeduino;
    FLEX_FUEL_TEMP      = 0x61: I16, CELSIUS,       Standalone;
    FUEL_TEMP_CORRECT   = 0x62: U16, PERCENT,       Speeduino;
    VE1                 = 0x63: U16, PERCENT,       Standalone;
    VE2                 = 0x64: U16, PERCENT,       Standalone;
    ADVANCE1            = 0x66: I16, DEGREES,       Standalone;
    ADVANCE2            = 0x67: I16, DEGREES,       Standalone;
    NITRO_STA           = 0x68: U8,  RAW,           Speeduino;
    SD_STA              = 0x69: U8,  RAW,           Speeduino;
    // Dashboard tell-tales, one bit each; see `indicators` for the bit names.
    INDICATORS1         = 0x6A: I32, RAW,           Protocol;
    INDICATORS2         = 0x6B: I32, RAW,           Protocol;
    MASTERALIVE         = 0x70: U8,  RAW,           Protocol;
    // Common channels other ECUs broadcast that Speeduino does not.
    OIL_TEMP            = 0x71: I16, CELSIUS,       Common;
    EGT1                = 0x72: I16, CELSIUS,       Common;
    FUEL_LEVEL          = 0x73: U16, PERCENT,       Common;
    COOLANT_PRES        = 0x74: U16, KPA,           Standalone;
    PEDAL               = 0x75: U16, PERCENT,       Common;
    TRANS_TEMP          = 0x76: I16, CELSIUS,       Standalone;
    INJ_DUTY            = 0x77: U16, PERCENT,       Standalone;
    WASTEGATE_PRES      = 0x78: U16, KPA,           Standalone;
    ERROR_COUNT         = 0x79: U16, RAW,           Common;
    // Fuel remaining in litres; FUEL_LEVEL is the percent form.
    FUEL_VOLUME         = 0x7A: U16, LITRES,        Standalone;
    // Distance travelled. OBD2 PID 0xA6 supplies it on MY2019 and later cars;
    // anything older needs a trip counter kept by whatever feeds the bus.
    ODOMETER            = 0x7B: I32, KILOMETRES,    Common;
    // 0 no fix, 1 2D, 2 3D, 3 differential or better. Zero is the only value
    // that means "do not trust the other GPS gauges".
    GPS_LOCK            = 0x7C: U8,  RAW,           Gps;
    GPS_SATS            = 0x7D: U8,  RAW,           Gps;
    // Speed over ground, which is not VSS: it survives a wheel-speed fault
    // and disagrees with it under wheelspin.
    GPS_SPEED           = 0x7E: U16, KMH,           Gps;
    // Exhaust gas temperature per cylinder. EGT1 sits back at 0x72 with the
    // other common channels; an engine with one probe uses that and stops
    // there. These are for the sources that send a full set, where the point
    // is the disagreement between them rather than any one number.
    EGT2                = 0x7F: I16, CELSIUS,       Standalone;
    EGT3                = 0x80: I16, CELSIUS,       Standalone;
    EGT4                = 0x81: I16, CELSIUS,       Standalone;
    EGT5                = 0x82: I16, CELSIUS,       Standalone;
    EGT6                = 0x83: I16, CELSIUS,       Standalone;
    EGT7                = 0x84: I16, CELSIUS,       Standalone;
    EGT8                = 0x85: I16, CELSIUS,       Standalone;

    // Knock. A standalone knows far more about this than a lamp can say, and
    // it is the one thing a driver cannot hear over the engine.
    KNOCK_LEVEL         = 0x86: U16, RAW,           Standalone;
    // Timing the ECU has pulled back because of knock, as a signed angle.
    KNOCK_RETARD        = 0x87: I16, DEGREES,       Standalone;
    KNOCK_COUNT         = 0x88: U16, RAW,           Standalone;
    // Which cylinder knocked last, 1-based; 0 means none has.
    KNOCK_CYL           = 0x89: U8,  RAW,           Standalone;
    // Per-cylinder knock level. One cylinder well above the others is a
    // different fault from all eight climbing together, and the sum of them
    // cannot tell the two apart.
    KNOCK1              = 0x8A: I8,  RAW,           Standalone;
    KNOCK2              = 0x8B: I8,  RAW,           Standalone;
    KNOCK3              = 0x8C: I8,  RAW,           Standalone;
    KNOCK4              = 0x8D: I8,  RAW,           Standalone;
    KNOCK5              = 0x8E: I8,  RAW,           Standalone;
    KNOCK6              = 0x8F: I8,  RAW,           Standalone;
    KNOCK7              = 0x90: I8,  RAW,           Standalone;
    KNOCK8              = 0x91: I8,  RAW,           Standalone;

    // Second throttle blade, and the duties the first two frames leave out.
    TPS2                = 0x92: U16, PERCENT,       Standalone;
    COIL_DUTY           = 0x93: U16, PERCENT,       Standalone;
    INJ_DUTY2           = 0x94: U16, PERCENT,       Standalone;
    // Rotary trailing ignition. Leading is CUR_SPARK_ADVANCE.
    IGN_ANGLE_TRAIL     = 0x95: I16, DEGREES,       Standalone;

    // The ECU's own board temperature, and whatever else the tuner wired to
    // the spare temperature inputs.
    ECU_TEMP            = 0x96: I16, CELSIUS,       Standalone;
    AUX_TEMP1           = 0x97: I16, CELSIUS,       Standalone;
    AUX_TEMP2           = 0x98: I16, CELSIUS,       Standalone;
    DIFF_TEMP           = 0x99: I16, CELSIUS,       Standalone;

    // Air in and fuel out, by mass. CYL_AIRMASS is what the model thinks one
    // cylinder took; AIRFLOW is the whole engine.
    CYL_AIRMASS         = 0x9A: U16, GRAMS,         Standalone;
    AIRFLOW             = 0x9B: U16, KG_PER_H,      Standalone;
    FUEL_USED           = 0x9C: I32, GRAMS,         Standalone;
    FUEL_FLOW           = 0x9D: U16, KG_PER_H,      Standalone;
    FUEL_RATE           = 0x9E: U16, L_PER_H,       Standalone;

    // Closed-loop trim on the second bank; EGO_CORRECT is the first.
    EGO_CORRECT2        = 0x9F: I16, PERCENT,       Standalone;
    // Direct injection rail pressure. FUEL_PRES is the low-pressure side.
    FUEL_PRES_HIGH      = 0xA0: U16, KPA,           Standalone;

    // Exhaust cams. VVT_ANGLE and VVT_ANGLE2 are the intake pair.
    VVT_EXH_ANGLE       = 0xA1: I16, DEGREES,       Standalone;
    VVT_EXH_TARGET      = 0xA2: I16, DEGREES,       Standalone;
    VVT_EXH_ANGLE2      = 0xA3: I16, DEGREES,       Standalone;
    VVT_EXH_TARGET2     = 0xA4: I16, DEGREES,       Standalone;

    // Wheel speeds as the ECU sees them. Driven against undriven is what
    // traction control acts on, and VSS alone cannot show it.
    WHEEL_SPEED_UNDRIVEN = 0xA5: U16, KMH,          Standalone;
    WHEEL_SPEED_DRIVEN  = 0xA6: U16, KMH,           Standalone;
    WHEEL_SLIP          = 0xA7: I16, PERCENT,       Standalone;
    SLIP_TARGET         = 0xA8: I16, PERCENT,       Standalone;
    TRACTION_LIMIT      = 0xA9: U16, PERCENT,       Standalone;

    // What the ECU is taking away, and where the limiter sits.
    IGN_CUT             = 0xAA: U16, PERCENT,       Standalone;
    FUEL_CUT            = 0xAB: U16, PERCENT,       Standalone;
    REV_LIMIT           = 0xAC: U16, RPM,           Standalone;
    // Total ignition compensation the corrections have added up to.
    IGN_COMP            = 0xAD: I16, DEGREES,       Standalone;

    // Accelerometer, as a driver reads it: forward, right, up.
    ACCEL_FWD           = 0xAE: I16, G_FORCE,       Standalone;
    ACCEL_RIGHT         = 0xAF: I16, G_FORCE,       Standalone;
    ACCEL_UP            = 0xB0: I16, G_FORCE,       Standalone;

    // Trigger diagnostics. A dash that shows these turns "it stumbles
    // sometimes" into a number that either climbs or does not.
    LOST_SYNC           = 0xB1: U16, RAW,           Standalone;
    MISS_COUNT          = 0xB2: U16, RAW,           Standalone;
    TRIGGER_COUNT       = 0xB3: U16, RAW,           Standalone;
    HOME_COUNT          = 0xB4: U16, RAW,           Standalone;
    TRIG_SINCE_HOME     = 0xB5: U16, RAW,           Standalone;

    // Distance this trip, which is not the odometer: it resets.
    TRIP_DIST           = 0xB6: I32, KILOMETRES,    Standalone;
}
