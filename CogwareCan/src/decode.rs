// SPDX-License-Identifier: GPL-3.0-only
//! Table-driven decoding shared by every ECU converter.
//!
//! Adding a data source means writing tables, not code:
//!
//! ```ignore
//! // A CAN-broadcast ECU: one `Field` per channel, grouped by frame.
//! pub static SOURCE: CanSource = CanSource {
//!     name: "MyEcu",
//!     default_base: 0x600,
//!     frames: &[
//!         CanFrameMap { id_offset: 0, fields: &[
//!             field!(0, U16BE, RPM, raw),
//!             field!(2, U16BE, MAP, x10),
//!         ]},
//!     ],
//! };
//! // then, per received frame:  SOURCE.feed(base, &frame)
//! ```
//!
//! A serial ECU uses the same `Field` table with `apply_fields`; see `speeduino`.
//!
//! # Status bits
//!
//! Most standalones broadcast a byte or two of flags -- rev limit reached,
//! fan on, knock heard -- which are lamps rather than readings. Those go in a
//! second table, `lamps`, one [`Flag`] per bit naming the [`Indicator`] it
//! drives, and the converter writes them through `indicators` exactly as a
//! server would.
//!
//! Every such mapping is written out bit by bit in the converter that owns
//! it, because it is a judgement and not a decode: a protocol's "speed limit
//! active" and this crate's `PitLimiter` are one person's decision that the
//! two mean the same thing, and the next person to read it deserves to see
//! that decision rather than infer it. Bits with no lamp worth lighting are
//! listed there too, as bits deliberately left alone.

use crate::frame::CanFrame;
use crate::gauge::GaugeData;
use crate::indicators::{self, Indicator, IndicatorSet};
use embedded_can::{Frame, Id};

/// How a raw channel is packed in a byte buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Raw {
    U8,
    S8,
    /// Little-endian 16-bit.
    U16,
    S16,
    /// Big-endian 16-bit (Haltech, Megasquirt).
    U16BE,
    S16BE,
    U32,
    U32BE,
}

impl Raw {
    /// How many bytes this width occupies on the wire.
    // No `is_empty` beside it: a width is one, two or four bytes and never
    // none, so the method clippy asks for could only ever return false.
    #[allow(clippy::len_without_is_empty)]
    pub const fn len(self) -> usize {
        match self {
            Raw::U8 | Raw::S8 => 1,
            Raw::U16 | Raw::S16 | Raw::U16BE | Raw::S16BE => 2,
            Raw::U32 | Raw::U32BE => 4,
        }
    }

    /// Read the channel at `off`, or `None` if the buffer is too short.
    pub fn read(self, data: &[u8], off: usize) -> Option<i32> {
        let b = data.get(off..off + self.len())?;
        Some(match self {
            Raw::U8 => b[0] as i32,
            Raw::S8 => b[0] as i8 as i32,
            Raw::U16 => u16::from_le_bytes([b[0], b[1]]) as i32,
            Raw::S16 => i16::from_le_bytes([b[0], b[1]]) as i32,
            Raw::U16BE => u16::from_be_bytes([b[0], b[1]]) as i32,
            Raw::S16BE => i16::from_be_bytes([b[0], b[1]]) as i32,
            Raw::U32 => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as i32,
            Raw::U32BE => u32::from_be_bytes([b[0], b[1], b[2], b[3]]) as i32,
        })
    }
}

/// One channel: where it sits, how it is packed, which gauge it feeds, and
/// the conversion from raw counts to that gauge's canonical fixed point.
pub struct Field {
    pub off: usize,
    pub raw: Raw,
    pub gauge: &'static GaugeData,
    pub conv: fn(i32) -> i32,
}

/// Build a `Field` from `(offset, Raw variant, gauge static, conversion fn)`.
#[macro_export]
macro_rules! field {
    ($off:literal, $raw:ident, $gauge:ident, $conv:expr) => {
        $crate::decode::Field {
            off: $off,
            raw: $crate::decode::Raw::$raw,
            gauge: &$crate::$gauge,
            conv: $conv,
        }
    };
}

/// Write every field present in `data` into its gauge. Fields that fall past
/// the end of `data` are skipped. Returns how many gauges were written.
pub fn apply_fields(fields: &[Field], data: &[u8]) -> usize {
    let mut written = 0;
    for f in fields {
        if let Some(v) = f.raw.read(data, f.off) {
            f.gauge.set((f.conv)(v));
            written += 1;
        }
    }
    written
}

/// One status bit: where it sits, and the lamp it drives.
pub struct Flag {
    pub off: usize,
    /// Bit position within that byte, 0 is least significant.
    pub bit: u8,
    pub lamp: Indicator,
}

/// Build a `Flag` from `(offset, bit, Indicator variant)`.
#[macro_export]
macro_rules! flag {
    ($off:literal, $bit:literal, $lamp:ident) => {
        $crate::decode::Flag {
            off: $off,
            bit: $bit,
            lamp: $crate::indicators::Indicator::$lamp,
        }
    };
}

/// The status bits carried by one CAN frame of a broadcast protocol.
pub struct LampFrame {
    /// Frame ID minus the protocol's base ID.
    pub id_offset: u16,
    pub flags: &'static [Flag],
}

/// Light or extinguish every lamp this frame speaks for. Returns how many
/// lamps were written.
///
/// A bit that is clear puts its lamp out, not just leaves it: the ECU is the
/// authority on its own state, and a lamp that only ever lit would stay on
/// after the condition passed.
///
/// Gathered before being written because two bits may drive one lamp -- a car
/// with two cooling fans has one fan lamp -- and writing as it went would let
/// the second bit put out what the first one lit.
pub fn apply_flags(flags: &[Flag], data: &[u8]) -> usize {
    let mut lit = IndicatorSet::new();
    let mut spoken_for = IndicatorSet::new();
    for f in flags {
        let Some(byte) = data.get(f.off) else {
            continue;
        };
        spoken_for.set(f.lamp, true);
        if byte & (1 << f.bit) != 0 {
            lit.set(f.lamp, true);
        }
    }
    let mut written = 0;
    for lamp in spoken_for.iter() {
        indicators::set(lamp, lit.contains(lamp));
        written += 1;
    }
    written
}

/// The channels carried by one CAN frame of a broadcast protocol.
pub struct CanFrameMap {
    /// Frame ID minus the protocol's base ID.
    pub id_offset: u16,
    pub fields: &'static [Field],
}

/// A CAN-broadcast ECU protocol.
pub struct CanSource {
    pub name: &'static str,
    /// Base ID the ECU ships with; pass a different one to `feed` if reconfigured.
    pub default_base: u16,
    pub frames: &'static [CanFrameMap],
    /// Status bits, kept apart from the readings because they are lamps and
    /// not numbers, and because the mapping from a protocol's flag to one of
    /// this crate's lamps is a judgement that wants stating in one place.
    pub lamps: &'static [LampFrame],
}

impl CanSource {
    /// Decode `frame` if it belongs to this protocol at `base`.
    /// Returns the number of gauges written, or `None` if the ID is not ours.
    pub fn feed(&self, base: u16, frame: &CanFrame) -> Option<usize> {
        let id = match frame.id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(e) => u16::try_from(e.as_raw()).ok()?,
        };
        let offset = id.checked_sub(base)?;
        let data = &frame.data()[..frame.dlc()];
        let readings = self.frames.iter().find(|m| m.id_offset == offset);
        let lamps = self.lamps.iter().find(|m| m.id_offset == offset);
        // A frame carrying only status bits is still ours, and one carrying
        // both is counted once for each thing it wrote.
        if readings.is_none() && lamps.is_none() {
            return None;
        }
        let mut written = 0;
        if let Some(map) = readings {
            written += apply_fields(map.fields, data);
        }
        if let Some(map) = lamps {
            written += apply_flags(map.flags, data);
        }
        Some(written)
    }

    /// Decode with the protocol's default base ID.
    pub fn feed_default(&self, frame: &CanFrame) -> Option<usize> {
        self.feed(self.default_base, frame)
    }

    /// Every CAN ID this protocol uses at `base`, status-bit frames included
    /// and each ID given once.
    ///
    /// A frame can carry readings and status bits together, so it appears in
    /// both tables; a caller setting up hardware filters wants the ID once,
    /// not once per reason.
    pub fn ids(&self, base: u16) -> impl Iterator<Item = u16> + '_ {
        let readings = self.frames.iter().map(move |m| base + m.id_offset);
        let lamps_only = self
            .lamps
            .iter()
            .filter(|l| !self.frames.iter().any(|m| m.id_offset == l.id_offset))
            .map(move |m| base + m.id_offset);
        readings.chain(lamps_only)
    }
}

/// Raw-to-canonical conversions shared by the converters. Each is named for
/// the raw scaling it undoes; the target scale is the gauge's `Unit::scale`.
pub mod conv {
    /// Raw already matches the canonical scale.
    pub fn raw(v: i32) -> i32 {
        v
    }
    /// Whole units into a tenths unit (%, °, kPa, km/h).
    pub fn x10(v: i32) -> i32 {
        v * 10
    }
    /// Whole units into a hundredths unit.
    pub fn x100(v: i32) -> i32 {
        v * 100
    }
    /// Whole units into a thousandths unit (V into mV, ms into µs).
    pub fn x1000(v: i32) -> i32 {
        v * 1000
    }
    /// 0.5-unit steps into a tenths unit.
    pub fn half(v: i32) -> i32 {
        v * 5
    }
    /// 2-unit steps into a tenths unit.
    pub fn x20(v: i32) -> i32 {
        v * 20
    }
    /// 5-unit steps into a tenths unit (rusEFI EGT).
    pub fn x50(v: i32) -> i32 {
        v * 50
    }
    /// Raw in hundredths into a thousandths unit (0.01 V into mV, 0.01 ms into µs).
    pub fn hundredths_to_thousandths(v: i32) -> i32 {
        v * 10
    }
    /// Raw in tenths into a hundredths unit (AFR sent as AFR*10).
    pub fn tenths_to_hundredths(v: i32) -> i32 {
        v * 10
    }
    /// Raw in tenths into a thousandths unit (0.1 V into mV, 0.1 ms into µs).
    pub fn tenths_to_thousandths(v: i32) -> i32 {
        v * 100
    }
    /// Raw in hundredths into a tenths unit (rounded).
    pub fn hundredths_to_tenths(v: i32) -> i32 {
        div_round(v, 10)
    }
    /// Raw in thousandths into a hundredths unit (rounded).
    pub fn thousandths_to_hundredths(v: i32) -> i32 {
        div_round(v, 10)
    }
    /// Whole °C sent with a +40 offset, into tenths of °C.
    pub fn celsius_plus40(v: i32) -> i32 {
        (v - 40) * 10
    }
    /// Kelvin in tenths into tenths of °C.
    pub fn kelvin_tenths(v: i32) -> i32 {
        div_round(v * 10 - 27_315, 10)
    }
    /// Fahrenheit in tenths into tenths of °C.
    pub fn fahrenheit_tenths(v: i32) -> i32 {
        div_round((v - 320) * 5, 9)
    }
    /// Whole percent centred on 100 (Speeduino corrections) into tenths of trim, 0-centred.
    pub fn centred100_to_trim_tenths(v: i32) -> i32 {
        (v - 100) * 10
    }
    /// Tenths of percent centred on 100 (Megasquirt) into tenths of trim, 0-centred.
    pub fn centred1000_to_trim_tenths(v: i32) -> i32 {
        v - 1000
    }
    /// Absolute pressure in tenths of kPa into gauge pressure (Haltech).
    pub fn kpa_abs_tenths_to_gauge(v: i32) -> i32 {
        v - 1013
    }
    /// Metres per second in tenths into tenths of km/h (Megasquirt VSS1).
    pub fn mps_tenths_to_kmh_tenths(v: i32) -> i32 {
        div_round(v * 36, 10)
    }
    /// Whole psi into tenths of kPa.
    pub fn psi(v: i32) -> i32 {
        div_round(v * 6895, 100)
    }
    /// psi in tenths into tenths of kPa.
    pub fn psi_tenths(v: i32) -> i32 {
        div_round(v * 6895, 1000)
    }
    /// Lambda in thousandths into hundredths of AFR (gasoline stoich 14.7).
    pub fn lambda_thousandths(v: i32) -> i32 {
        div_round(v * 147, 100)
    }
    /// Lambda in hundredths into hundredths of AFR.
    pub fn lambda_hundredths(v: i32) -> i32 {
        div_round(v * 147, 10)
    }
    /// Lambda in ten-thousandths into hundredths of AFR.
    pub fn lambda_ten_thousandths(v: i32) -> i32 {
        div_round(v * 147, 1000)
    }

    /// Integer division rounding to nearest, away from zero on ties.
    pub fn div_round(v: i32, d: i32) -> i32 {
        let half = d / 2;
        if v >= 0 {
            (v + half) / d
        } else {
            (v - half) / d
        }
    }
}
