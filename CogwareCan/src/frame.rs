//! The frame this crate builds and parses.
//!
//! Nothing here knows what controller will put the frame on the wire. A board
//! brings its own driver -- an SPI part, a peripheral built into the chip, a
//! test double -- and converts at that one edge with [`CanFrame::to_frame`]
//! and [`CanFrame::from_frame`]. Both sides of the conversion are
//! [`embedded_can::Frame`], so any driver written against that trait fits
//! without this crate naming it.
//!
//! Owning the type rather than borrowing a driver's is what makes that true:
//! a crate that returned a particular driver's frame would force every board
//! onto that driver, whatever controller the board actually has.

use embedded_can::{Frame, Id};

/// Longest payload a classic CAN frame carries.
pub const MAX_DATA: usize = 8;

/// A CAN frame, held in the shape the protocol code reads it in.
///
/// `Copy`, and eight bytes of payload wide, so passing one by value costs
/// about what passing a pointer would and no caller has to think about
/// ownership on the hot path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanFrame {
    id: Id,
    /// Declared length. Kept as a byte because the wire field is four bits;
    /// `embedded_can` speaks in `usize`, so the accessor widens it.
    dlc: u8,
    remote: bool,
    data: [u8; MAX_DATA],
}

impl CanFrame {
    /// The payload, as a slice of exactly the declared length.
    ///
    /// Empty for a remote frame, which declares a length but carries nothing.
    /// This has to agree with [`embedded_can::Frame::data`], or a remote
    /// request would go out with a payload of zeros behind it.
    pub fn as_bytes(&self) -> &[u8] {
        if self.remote {
            &[]
        } else {
            &self.data[..self.dlc as usize]
        }
    }

    /// Copy a driver's frame into this one.
    ///
    /// `None` if the frame declares more than eight bytes, which classic CAN
    /// cannot carry and this crate never produces. A driver is trusted to
    /// report its own hardware honestly, but not checking here would turn a
    /// surprising length into a panic on the slice below.
    pub fn from_frame<F: Frame>(other: &F) -> Option<Self> {
        if other.dlc() > MAX_DATA {
            return None;
        }
        if other.is_remote_frame() {
            Frame::new_remote(other.id(), other.dlc())
        } else {
            Frame::new(other.id(), other.data())
        }
    }

    /// Copy this frame into whatever type the board's driver sends.
    ///
    /// `None` only if that type refuses the frame; every driver that carries
    /// classic CAN accepts everything this crate builds.
    pub fn to_frame<F: Frame>(&self) -> Option<F> {
        if self.remote {
            F::new_remote(self.id, self.dlc as usize)
        } else {
            F::new(self.id, self.as_bytes())
        }
    }
}

impl Frame for CanFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > MAX_DATA {
            return None;
        }
        let mut buf = [0u8; MAX_DATA];
        buf[..data.len()].copy_from_slice(data);
        Some(Self {
            id: id.into(),
            dlc: data.len() as u8,
            remote: false,
            data: buf,
        })
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > MAX_DATA {
            return None;
        }
        Some(Self {
            id: id.into(),
            dlc: dlc as u8,
            remote: true,
            data: [0; MAX_DATA],
        })
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }

    fn is_remote_frame(&self) -> bool {
        self.remote
    }

    fn id(&self) -> Id {
        self.id
    }

    fn dlc(&self) -> usize {
        self.dlc as usize
    }

    fn data(&self) -> &[u8] {
        self.as_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{ExtendedId, StandardId};

    /// Stands in for a board's driver frame: a different type, reachable only
    /// through the trait, so the round trips below prove the conversion uses
    /// nothing this crate happens to know about its own layout.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Foreign {
        id: Id,
        dlc: usize,
        remote: bool,
        data: [u8; MAX_DATA],
    }

    impl Frame for Foreign {
        fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
            if data.len() > MAX_DATA {
                return None;
            }
            let mut buf = [0u8; MAX_DATA];
            buf[..data.len()].copy_from_slice(data);
            Some(Self {
                id: id.into(),
                dlc: data.len(),
                remote: false,
                data: buf,
            })
        }
        fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
            (dlc <= MAX_DATA).then_some(Self {
                id: id.into(),
                dlc,
                remote: true,
                data: [0; MAX_DATA],
            })
        }
        fn is_extended(&self) -> bool {
            matches!(self.id, Id::Extended(_))
        }
        fn is_remote_frame(&self) -> bool {
            self.remote
        }
        fn id(&self) -> Id {
            self.id
        }
        fn dlc(&self) -> usize {
            self.dlc
        }
        fn data(&self) -> &[u8] {
            if self.remote {
                &[]
            } else {
                &self.data[..self.dlc]
            }
        }
    }

    fn standard(id: u16) -> Id {
        // A literal inside the test's own range; `new` only rejects IDs past
        // 11 bits and every caller here stays well inside that.
        Id::Standard(StandardId::new(id).expect("test IDs are 11-bit"))
    }

    #[test]
    fn a_data_frame_survives_the_trip_to_a_driver_and_back() {
        let mine = CanFrame::new(standard(0x018), &[1, 2, 3, 4, 5]).expect("5 bytes fit");
        let theirs: Foreign = mine.to_frame().expect("a classic frame is always accepted");
        let back = CanFrame::from_frame(&theirs).expect("8 bytes or fewer come back");
        assert_eq!(back, mine);
    }

    #[test]
    fn an_empty_frame_survives_the_trip() {
        let mine = CanFrame::new(standard(0x000), &[]).expect("zero bytes fit");
        let theirs: Foreign = mine.to_frame().expect("an empty frame is accepted");
        assert_eq!(theirs.dlc(), 0);
        assert_eq!(CanFrame::from_frame(&theirs), Some(mine));
    }

    #[test]
    fn a_full_frame_survives_the_trip() {
        let mine = CanFrame::new(standard(0x7FF), &[9; MAX_DATA]).expect("8 bytes fit");
        let theirs: Foreign = mine.to_frame().expect("a full frame is accepted");
        assert_eq!(CanFrame::from_frame(&theirs), Some(mine));
    }

    #[test]
    fn an_extended_id_survives_the_trip() {
        let id = Id::Extended(ExtendedId::new(0x12AB_CDEF).expect("29-bit literal"));
        let mine = CanFrame::new(id, &[7, 7]).expect("2 bytes fit");
        assert!(mine.is_extended());
        let theirs: Foreign = mine.to_frame().expect("extended IDs are accepted");
        assert_eq!(CanFrame::from_frame(&theirs), Some(mine));
    }

    #[test]
    fn a_remote_frame_keeps_its_length_and_carries_nothing() {
        let mine = CanFrame::new_remote(standard(0x020), 4).expect("a 4-byte request");
        assert!(mine.is_remote_frame());
        assert_eq!(mine.dlc(), 4);
        assert!(mine.as_bytes().is_empty());
        let theirs: Foreign = mine.to_frame().expect("remote requests are accepted");
        assert!(theirs.is_remote_frame());
        assert_eq!(theirs.dlc(), 4);
        assert_eq!(CanFrame::from_frame(&theirs), Some(mine));
    }

    #[test]
    fn a_payload_longer_than_the_bus_carries_is_refused() {
        assert_eq!(CanFrame::new(standard(0x001), &[0; MAX_DATA + 1]), None);
        assert_eq!(CanFrame::new_remote(standard(0x001), MAX_DATA + 1), None);
    }

    #[test]
    fn a_driver_frame_declaring_more_than_the_bus_carries_is_refused() {
        // Not reachable through `Foreign::new`, which refuses it too, so the
        // length is forced past the limit the way a driver reading a bad DLC
        // field off its own hardware would report one.
        let long = Foreign {
            id: standard(0x001),
            dlc: 12,
            remote: false,
            data: [0; MAX_DATA],
        };
        assert_eq!(CanFrame::from_frame(&long), None);
    }
}
