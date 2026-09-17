// SPDX-License-Identifier: GPL-3.0-only
//! The record a node writes around an image it has been given, so a slot of
//! flash or a file on a card says for itself what it holds and whether the
//! write finished.
//!
//! The header goes down *after* the payload. Erased flash reads as `0xFF` and
//! never matches [`MAGIC`], so a node interrupted part way through a transfer
//! comes back with a slot that reads as empty rather than as half an image.

use super::Image;
use crate::crc::crc32;

/// Magic at the head of a stored slot.
pub const MAGIC: [u8; 4] = *b"CWST";
/// Record layout version, bumped only if this framing changes.
pub const VERSION: u8 = 1;
/// Bytes the header occupies in front of the payload.
pub const HEADER_LEN: usize = 16;

/// What a stored slot holds, as written in front of the payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotHeader {
    /// One of the constants in [`super::kind`].
    pub kind: u8,
    /// Which slot of that kind this is; see [`super::kind`] for what a slot
    /// number means for each.
    pub slot: u8,
    /// Payload length in bytes.
    pub len: u32,
    /// CRC-32 of the payload, the same value `xfer` verified on arrival.
    pub crc: u32,
}

impl SlotHeader {
    /// A header for an image whose bytes are all in hand.
    ///
    /// A node that streamed the payload straight to flash has no such slice
    /// and should build the header from a running [`crate::crc::Crc32`]
    /// instead.
    pub fn for_payload(image: Image, payload: &[u8]) -> SlotHeader {
        SlotHeader {
            kind: image.kind,
            slot: image.slot,
            len: payload.len() as u32,
            crc: crc32(payload),
        }
    }

    /// Pack into the bytes that precede the payload.
    pub fn encode(&self) -> [u8; HEADER_LEN] {
        let mut out = [0u8; HEADER_LEN];
        out[0..4].copy_from_slice(&MAGIC);
        out[4] = VERSION;
        out[5] = self.kind;
        out[6] = self.slot;
        out[8..12].copy_from_slice(&self.len.to_le_bytes());
        out[12..16].copy_from_slice(&self.crc.to_le_bytes());
        out
    }

    /// Read a header back, or `None` if the slot is erased, is another
    /// version's, or is too short to hold one.
    pub fn decode(bytes: &[u8]) -> Option<SlotHeader> {
        if bytes.len() < HEADER_LEN || bytes[0..4] != MAGIC || bytes[4] != VERSION {
            return None;
        }
        Some(SlotHeader {
            kind: bytes[5],
            slot: bytes[6],
            len: u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            crc: u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]),
        })
    }

    /// True if `payload` is the length and the CRC this header claims.
    pub fn verify(&self, payload: &[u8]) -> bool {
        payload.len() as u32 == self.len && crc32(payload) == self.crc
    }

    /// Bytes this record occupies, header included.
    pub fn stored_len(&self) -> usize {
        HEADER_LEN + self.len as usize
    }

    /// What a transfer of this record would declare in its `Begin`.
    pub fn image(&self) -> Image {
        Image {
            size: self.len,
            kind: self.kind,
            slot: self.slot,
        }
    }
}

/// The header and payload of a stored slot, or `None` if the slot is erased,
/// truncated or does not match its own CRC.
///
/// This is the only way a node should read a slot back: a record that fails
/// here is one the node must fall back from, not one it can half trust.
pub fn read(slot: &[u8]) -> Option<(SlotHeader, &[u8])> {
    let header = SlotHeader::decode(slot)?;
    let payload = slot.get(HEADER_LEN..header.stored_len())?;
    header.verify(payload).then_some((header, payload))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xfer::kind;

    fn record(kind: u8, slot: u8, payload: &[u8]) -> Vec<u8> {
        let image = Image {
            size: payload.len() as u32,
            kind,
            slot,
        };
        let mut out = SlotHeader::for_payload(image, payload).encode().to_vec();
        out.extend_from_slice(payload);
        out
    }

    #[test]
    fn a_written_record_reads_back_whole() {
        let payload = b"{ \"scene\": 1 }";
        let stored = record(kind::SCENE, 2, payload);
        let (header, read_back) = read(&stored).expect("a whole record reads back");
        assert_eq!(header.kind, kind::SCENE);
        assert_eq!(header.slot, 2);
        assert_eq!(read_back, payload);
        assert_eq!(
            header.image(),
            Image {
                size: payload.len() as u32,
                kind: kind::SCENE,
                slot: 2
            }
        );
    }

    #[test]
    fn an_erased_slot_is_not_a_record() {
        assert!(read(&[0xFF; 64]).is_none());
        assert!(read(&[0x00; 64]).is_none());
        assert!(read(&[]).is_none());
    }

    #[test]
    fn a_record_written_without_its_header_is_not_a_record() {
        // What an interrupted transfer leaves behind: the payload is down but
        // the header never went, so the slot still reads as erased.
        let mut slot = vec![0xFFu8; HEADER_LEN];
        slot.extend_from_slice(b"half an image");
        assert!(read(&slot).is_none());
    }

    #[test]
    fn a_corrupted_payload_fails_its_crc() {
        let mut stored = record(kind::RIG, 0, b"display 0x02");
        stored[HEADER_LEN] ^= 0x20;
        assert!(read(&stored).is_none());
    }

    #[test]
    fn a_truncated_payload_is_refused() {
        let stored = record(kind::SCENE, 0, b"0123456789");
        assert!(read(&stored[..stored.len() - 1]).is_none());
    }

    #[test]
    fn a_slot_longer_than_its_record_reads_only_the_record() {
        // A slot is a fixed span of flash, so the bytes after a short image
        // are whatever the last image left there.
        let mut stored = record(kind::SCENE, 1, b"short");
        stored.extend_from_slice(b"older, longer image left behind");
        let (header, payload) = read(&stored).expect("the record is still whole");
        assert_eq!(header.len, 5);
        assert_eq!(payload, b"short");
    }

    #[test]
    fn another_version_is_refused_rather_than_guessed_at() {
        let mut stored = record(kind::CONFIG, 0, b"CFG1");
        stored[4] = VERSION + 1;
        assert!(read(&stored).is_none());
    }

    #[test]
    fn an_empty_image_is_a_valid_record() {
        let stored = record(kind::SCENE, 3, b"");
        let (header, payload) = read(&stored).expect("an empty image still has a header");
        assert_eq!(header.len, 0);
        assert!(payload.is_empty());
        assert_eq!(header.stored_len(), HEADER_LEN);
    }
}
