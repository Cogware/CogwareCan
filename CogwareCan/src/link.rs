//! The seam between this crate and whatever controller a board has.
//!
//! This crate builds and parses frames; it never touches hardware. [`Link`]
//! is the one shape it asks a board for in return: queue a frame, and hand
//! back a frame that has arrived if one has. Everything a node's main loop
//! does with a controller is one of those two calls plus the decision that
//! falls out of [`SendError::Busy`] -- hold the frame and offer it again --
//! so a board that can do these two things can run the whole protocol.
//!
//! # Implementing it for a board
//!
//! A controller that already speaks [`embedded_can::nb::Can`] needs no code
//! at all: wrap it in [`NbLink`]. A peripheral built into the chip is the
//! usual case, and the wrapper is where its frame type is converted to this
//! crate's, once per frame, at the only place that knows both.
//!
//! A driver with a blocking or bespoke API -- anything whose receive waits
//! for a frame rather than reporting that none has come -- cannot be adapted
//! generically, because [`Link::poll`] must return rather than block. Such a
//! driver gets a newtype in the board's own crate:
//!
//! ```ignore
//! struct MyLink(MyDriver);
//!
//! impl Link for MyLink {
//!     type Error = MyDriverError;
//!
//!     fn send(&mut self, frame: &CanFrame) -> Result<(), SendError<Self::Error>> {
//!         let out = frame.to_frame().ok_or(SendError::Mismatch)?;
//!         self.0.transmit(&out).map_err(|e| match e {
//!             MyDriverError::TxBusy => SendError::Busy,
//!             other => SendError::Failed(other),
//!         })
//!     }
//!
//!     fn poll(&mut self) -> Result<Option<CanFrame>, RecvError<Self::Error>> {
//!         match self.0.read_if_any().map_err(RecvError::Failed)? {
//!             Some(f) => Ok(Some(CanFrame::from_frame(&f).ok_or(RecvError::Mismatch)?)),
//!             None => Ok(None),
//!         }
//!     }
//! }
//! ```
//!
//! The newtype is needed because Rust only lets a crate implement a trait for
//! a type when it owns one of the two, and a board owns neither this trait
//! nor a driver it pulled in. It costs nothing at run time.

use crate::frame::CanFrame;

/// Why a frame could not be queued.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SendError<E> {
    /// Every transmit buffer is full. The frame was not queued and nothing is
    /// wrong: hold it and offer it again. A sender that treats this as a
    /// failure drops frames under load, which is the one way this protocol
    /// stalls without saying so.
    Busy,
    /// The driver's frame type would not hold this frame. Unreachable for a
    /// controller carrying classic CAN, which accepts everything this crate
    /// builds; it is here so the conversion has somewhere honest to fail.
    Mismatch,
    /// The controller reported an error.
    Failed(E),
}

/// Why a frame could not be collected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecvError<E> {
    /// A frame arrived declaring more than eight bytes, which classic CAN
    /// does not carry. The frame is lost; the link is still usable.
    Mismatch,
    /// The controller reported an error.
    Failed(E),
}

/// A CAN controller, as this crate needs it.
pub trait Link {
    /// Whatever the board's driver reports when it goes wrong.
    type Error;

    /// Queue a frame for transmission.
    ///
    /// Returns once the frame is queued, not once it is on the wire. A bus
    /// with no other node on it never acknowledges, so a frame can sit in a
    /// buffer indefinitely and this call still returns `Ok`.
    fn send(&mut self, frame: &CanFrame) -> Result<(), SendError<Self::Error>>;

    /// Take one frame that has arrived, or `Ok(None)` if none has.
    ///
    /// Must not block. A caller drains in a loop until this returns `None`,
    /// which is what keeps a controller with a shallow receive buffer from
    /// overflowing between passes of a main loop that also draws a screen.
    fn poll(&mut self) -> Result<Option<CanFrame>, RecvError<Self::Error>>;
}

/// Any controller that speaks [`embedded_can::nb::Can`], as a [`Link`].
///
/// This is the adapter a chip's built-in controller usually needs. The frame
/// conversion happens here and nowhere else.
pub struct NbLink<C>(pub C);

impl<C> NbLink<C> {
    /// Wrap a controller.
    pub const fn new(can: C) -> Self {
        Self(can)
    }

    /// Borrow the controller, for the settings this trait does not cover --
    /// error counters, bus-off recovery, filters.
    pub fn controller(&mut self) -> &mut C {
        &mut self.0
    }

    /// Give the controller back.
    pub fn into_inner(self) -> C {
        self.0
    }
}

impl<C: embedded_can::nb::Can> Link for NbLink<C> {
    type Error = C::Error;

    fn send(&mut self, frame: &CanFrame) -> Result<(), SendError<Self::Error>> {
        let out = frame.to_frame().ok_or(SendError::Mismatch)?;
        // A controller that makes room by evicting a lower-priority frame
        // hands that frame back. This crate queues in priority order and
        // holds on `Busy`, so it never asks for an eviction; a frame handed
        // back here would be one this node did not queue, and re-queueing it
        // would be this link deciding another sender's retry policy.
        match self.0.transmit(&out) {
            Ok(_) => Ok(()),
            Err(nb::Error::WouldBlock) => Err(SendError::Busy),
            Err(nb::Error::Other(e)) => Err(SendError::Failed(e)),
        }
    }

    fn poll(&mut self) -> Result<Option<CanFrame>, RecvError<Self::Error>> {
        match self.0.receive() {
            Ok(f) => Ok(Some(CanFrame::from_frame(&f).ok_or(RecvError::Mismatch)?)),
            Err(nb::Error::WouldBlock) => Ok(None),
            Err(nb::Error::Other(e)) => Err(RecvError::Failed(e)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{ErrorKind, Frame, Id, StandardId};

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Fault;

    impl embedded_can::Error for Fault {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// A controller with one transmit slot and a queue of frames to hand out,
    /// which is enough to exercise every arm of the adapter.
    struct Fake {
        full: bool,
        broken: bool,
        sent: Option<CanFrame>,
        waiting: Option<CanFrame>,
    }

    impl Fake {
        fn idle() -> Self {
            Self {
                full: false,
                broken: false,
                sent: None,
                waiting: None,
            }
        }
    }

    impl embedded_can::nb::Can for Fake {
        type Frame = CanFrame;
        type Error = Fault;

        fn transmit(&mut self, frame: &CanFrame) -> nb::Result<Option<CanFrame>, Fault> {
            if self.broken {
                return Err(nb::Error::Other(Fault));
            }
            if self.full {
                return Err(nb::Error::WouldBlock);
            }
            self.sent = Some(*frame);
            Ok(None)
        }

        fn receive(&mut self) -> nb::Result<CanFrame, Fault> {
            if self.broken {
                return Err(nb::Error::Other(Fault));
            }
            self.waiting.take().ok_or(nb::Error::WouldBlock)
        }
    }

    fn frame(id: u16, data: &[u8]) -> CanFrame {
        CanFrame::new(
            Id::Standard(StandardId::new(id).expect("11-bit literal")),
            data,
        )
        .expect("no test builds a payload past eight bytes")
    }

    #[test]
    fn a_queued_frame_reaches_the_controller_unchanged() {
        let mut link = NbLink::new(Fake::idle());
        let f = frame(0x018, &[1, 2, 3]);
        assert_eq!(link.send(&f), Ok(()));
        assert_eq!(link.0.sent, Some(f));
    }

    #[test]
    fn a_full_controller_reports_busy_rather_than_failure() {
        let mut link = NbLink::new(Fake {
            full: true,
            ..Fake::idle()
        });
        assert_eq!(link.send(&frame(0x018, &[0])), Err(SendError::Busy));
        // Nothing was queued, so the caller still owns the frame and the
        // retry that follows is not a duplicate.
        assert_eq!(link.0.sent, None);
    }

    #[test]
    fn a_controller_error_is_passed_through_on_send() {
        let mut link = NbLink::new(Fake {
            broken: true,
            ..Fake::idle()
        });
        assert_eq!(
            link.send(&frame(0x018, &[0])),
            Err(SendError::Failed(Fault))
        );
    }

    #[test]
    fn an_empty_controller_polls_as_nothing_rather_than_an_error() {
        let mut link = NbLink::new(Fake::idle());
        assert_eq!(link.poll(), Ok(None));
    }

    #[test]
    fn a_waiting_frame_is_collected_once() {
        let f = frame(0x001, &[7, 7]);
        let mut link = NbLink::new(Fake {
            waiting: Some(f),
            ..Fake::idle()
        });
        assert_eq!(link.poll(), Ok(Some(f)));
        assert_eq!(link.poll(), Ok(None));
    }

    #[test]
    fn a_controller_error_is_passed_through_on_poll() {
        let mut link = NbLink::new(Fake {
            broken: true,
            ..Fake::idle()
        });
        assert_eq!(link.poll(), Err(RecvError::Failed(Fault)));
    }
}
