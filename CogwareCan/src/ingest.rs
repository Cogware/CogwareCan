// SPDX-License-Identifier: GPL-3.0-only
//! One way in, whatever is talking.
//!
//! A server does not get told which ECU is attached. It gets frames, and the
//! ID says who sent them: the display bus owns 0x020..=0x0FF, OBD2 answers on
//! 0x7E8..=0x7EF, and every broadcast ECU this crate knows sits in a band of
//! its own further up. [`ingest`] reads the ID and routes accordingly, so a
//! car with a rusEFI fills every channel rusEFI sends, a stock car over OBD2
//! fills every PID it answers, and neither needs configuring first.
//!
//! Whatever no source filled stays unfilled, and an unfilled gauge reads as
//! [`NO_DATA`](crate::NO_DATA). That is the whole of the contract a display
//! needs: it asks for a value and either gets a reading or gets the sentinel,
//! without knowing or caring what is plumbed into the other end.
//!
//! Mixing sources is fine and is the point. A rusEFI has no fuel level and no
//! GPS; a sender on a tank float and a GPS receiver write those gauges
//! directly, and the display sees one table rather than three.
//!
//! # Base IDs
//!
//! Each ECU is tried at the base it ships with. Two of them (rusEFI,
//! ECUMaster) can be reconfigured to another base; a server told that has
//! happened should call [`CanSource::feed`](crate::CanSource::feed) with the
//! base it was given rather than come through here.

use crate::frame::CanFrame;
use crate::gauge::GaugeData;
use crate::protocol::{id_of, is_gauge_range};
use crate::{feed_frame, obd2, FeedError, CAN_SOURCES};

/// What [`ingest`] made of a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fed {
    /// A display-bus gauge frame, stored in the gauge named.
    Gauge(&'static GaugeData),
    /// A broadcast ECU frame, claimed by the source named, writing this many
    /// gauges. The name is for a log or a debug view; nothing should branch
    /// on it, or the source has been specified after all.
    Ecu { source: &'static str, gauges: usize },
    /// An OBD2 mode 01 reply, writing this many gauges.
    Obd2 { gauges: usize },
    /// A gauge-range frame that would not decode. Kept separate from
    /// `Ignored` because it means a real fault -- a truncated frame, an ID
    /// colliding with ours -- rather than traffic that is simply not ours.
    Rejected(FeedError),
    /// Nobody recognised it: bus control, an ECU with no converter here, a
    /// PID nothing polls, an extended ID.
    Ignored,
}

/// Feed a frame to whichever source owns it.
///
/// Safe to call on every frame off the wire: anything unrecognised comes back
/// as [`Fed::Ignored`] rather than an error, and nothing here panics.
pub fn ingest(frame: &CanFrame) -> Fed {
    let Some(id) = id_of(frame) else {
        // Extended IDs: no protocol here uses one at its default base.
        return Fed::Ignored;
    };

    // The display bus first, and by range rather than by lookup, so that a
    // malformed frame at one of our own IDs is reported instead of being
    // offered to every ECU converter in turn.
    if is_gauge_range(id) {
        return match feed_frame(frame) {
            Ok(gauge) => Fed::Gauge(gauge),
            Err(e) => Fed::Rejected(e),
        };
    }

    if obd2::is_response(frame) {
        // A reply to a PID nothing polls is not a fault; the car answers what
        // it likes and we decode the ones we know.
        return Fed::Obd2 { gauges: obd2::feed_response(frame).unwrap_or(0) };
    }

    for source in CAN_SOURCES {
        if let Some(gauges) = source.feed_default(frame) {
            return Fed::Ecu { source: source.name, gauges };
        }
    }

    Fed::Ignored
}

#[cfg(test)]
mod tests {
    // Only checks that touch no gauges live here; the gauge table is
    // process-wide, and anything reading or writing it needs the serialising
    // guard the integration tests hold. See `tests/ingest.rs`.
    use super::*;
    use crate::protocol::frame;
    use crate::Gauge;

    #[test]
    fn a_truncated_frame_at_one_of_our_ids_is_a_fault_not_silence() {
        // CLNT is two bytes wide; one byte is a broken sender, and saying
        // "ignored" would hide it.
        let f = frame(Gauge::Clnt.id(), &[0x01]).expect("frame");
        assert!(matches!(ingest(&f), Fed::Rejected(FeedError::BadLength { .. })));
    }

    #[test]
    fn traffic_belonging_to_nobody_is_ignored_rather_than_rejected() {
        let f = frame(0x123, &[0xFF; 8]).expect("frame");
        assert_eq!(ingest(&f), Fed::Ignored);
    }

    #[test]
    fn no_two_sources_claim_the_same_id() {
        // `ingest` hands a frame to the first source that claims it, so an
        // overlap would not be an ambiguity to resolve later -- it would be
        // one ECU's frame silently decoded with another's layout, which
        // reads as plausible nonsense. Catch it when a source is added.
        for (i, a) in CAN_SOURCES.iter().enumerate() {
            for b in &CAN_SOURCES[i + 1..] {
                for id in a.ids(a.default_base) {
                    assert!(
                        !b.ids(b.default_base).any(|other| other == id),
                        "{} and {} both claim 0x{id:03X}",
                        a.name,
                        b.name
                    );
                }
            }
        }
    }

    #[test]
    fn no_source_reaches_into_the_display_bus_or_obd2() {
        for source in CAN_SOURCES {
            for id in source.ids(source.default_base) {
                assert!(
                    !is_gauge_range(id)
                        && !(obd2::RESPONSE_ID_MIN..=obd2::RESPONSE_ID_MAX).contains(&id),
                    "{} claims 0x{id:03X}, which is already spoken for",
                    source.name
                );
            }
        }
    }
}
