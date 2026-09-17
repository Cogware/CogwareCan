// SPDX-License-Identifier: GPL-3.0-only
//! How a gauge value is packed into the bytes of a CAN frame.
//!
//! A gauge stores an `i32` whatever its width on the wire, so this is the
//! narrowing: one, two or four bytes, little endian, signed or not. It is the
//! only place that decides what happens to a value too big for its width, and
//! it clamps rather than wraps -- an over-range reading should read as the top
//! of the scale, not as the bottom of it.

/// How a value is packed into a CAN frame: little endian, exactly this many bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wire {
    U8,
    I8,
    U16,
    I16,
    I32,
}

impl Wire {
    /// How many bytes this width occupies in a frame's payload.
    // No `is_empty` beside it: a width is one, two or four bytes and never
    // none, so the method clippy asks for could only ever return false.
    #[allow(clippy::len_without_is_empty)]
    pub const fn len(self) -> usize {
        match self {
            Wire::U8 | Wire::I8 => 1,
            Wire::U16 | Wire::I16 => 2,
            Wire::I32 => 4,
        }
    }

    /// Pack `value` into `out` and return how many bytes were used.
    /// Values outside the wire range are clamped rather than wrapped.
    pub fn encode(self, value: i32, out: &mut [u8; 4]) -> usize {
        let n = self.len();
        let clamped = match self {
            Wire::U8 => value.clamp(0, u8::MAX as i32),
            Wire::I8 => value.clamp(i8::MIN as i32, i8::MAX as i32),
            Wire::U16 => value.clamp(0, u16::MAX as i32),
            Wire::I16 => value.clamp(i16::MIN as i32, i16::MAX as i32),
            Wire::I32 => value,
        };
        out[..n].copy_from_slice(&clamped.to_le_bytes()[..n]);
        n
    }

    /// Unpack a payload. Returns `None` unless `data.len()` matches exactly.
    pub fn decode(self, data: &[u8]) -> Option<i32> {
        if data.len() != self.len() {
            return None;
        }
        Some(match self {
            Wire::U8 => data[0] as i32,
            Wire::I8 => data[0] as i8 as i32,
            Wire::U16 => u16::from_le_bytes([data[0], data[1]]) as i32,
            Wire::I16 => i16::from_le_bytes([data[0], data[1]]) as i32,
            Wire::I32 => i32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        })
    }
}
