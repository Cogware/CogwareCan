// SPDX-License-Identifier: GPL-3.0-only
//! CogwareCan: one crate holding the gauge spec, the ECU converters, and the
//! CAN framing, so a server or display only ever talks to this crate.
//!
//! Server side (ECU in, CAN out):
//! ```ignore
//! ingest(&frame);                            // any ECU or OBD2 -> gauges
//! speeduino::parse_n(&uart_packet)?;         // serial ECU bytes -> gauges
//! if let Some(f) = frame_for(Gauge::Rpm.id()) // gauge -> CAN frame
//!     { can.send_message(f)?; }
//! ```
//!
//! Display side (CAN in, gauges out):
//! ```ignore
//! feed_frame(&frame)?;                       // CAN frame -> gauge
//! let rpm = RPM.raw();                       // counts, or NO_DATA if unfed
//! let clt = CLNT.as_f32();                   // 87.5 for 87.5 °C, None if unset
//! let psi = MAP.to(Unit::PSI);               // unit conversion, see `units`
//! ```
//!
//! # One table, whatever is feeding it
//!
//! The gauge table is the whole vocabulary of the car, and nothing in it
//! belongs to a particular ECU. [`ingest`] routes a frame by its ID, so
//! whichever source is attached fills every channel it can send and needs no
//! configuring to do it; a second source -- a GPS receiver, a tank float on a
//! GPIO -- writes the gauges the first one cannot, straight into the same
//! table. What nothing filled reads as [`NO_DATA`], which is how a display
//! tells an absent source from a sensor sitting at zero. See `ingest`.
//!
//! Adding an ECU: one new module holding a `CanSource` table (CAN broadcast)
//! or a `Field` table plus a packet parser (serial). See `decode`. OBD2 is
//! request/response and lives in `obd2`.
//!
//! Bus housekeeping: `protocol` holds the ID map and node addressing,
//! `subscribe` the display-to-master gauge subscriptions, `bus` the global
//! mode and the node-to-master message channel, and `xfer` the master-to-node
//! file transfer used for over-the-air updates. `indicators` packs the
//! dashboard tell-tales into two bitfield gauges. All are frame in, frame
//! out, with no driver or timer of their own. A node with a tell-tale wired
//! to one of its own pins reports it with `bus::BusView::report_lamp`; the
//! master applies it and broadcasts the bank, so every display lights the
//! lamp together and none of them is the one that decided.
//!
//! `config` is the one module that is not about the bus: it describes the
//! settings a display holds well enough for an editor to render them, and
//! packs them into the blob `xfer` carries as `kind::CONFIG`.
//!
//! # Which controller a board has is the board's business
//!
//! This crate names no driver and touches no hardware. It owns its frame
//! type, [`CanFrame`], and every function above takes or returns that one
//! type; a board converts once, at its driver, with [`CanFrame::to_frame`]
//! and [`CanFrame::from_frame`]. Both sides of that conversion are stated as
//! [`embedded_can::Frame`], so a controller wired over SPI, one built into
//! the chip, and a test double are all equally acceptable and none of them
//! appears in this crate.
//!
//! That is deliberate rather than tidiness. A crate that handed back one
//! driver's frame type would force every board onto that driver whatever
//! controller the board actually has, and changing a node's controller would
//! mean changing the protocol crate every other node shares.
//!
//! [`link`] carries the other half: [`Link`], the two calls a node's main
//! loop makes of a controller, and [`NbLink`], which turns anything speaking
//! [`embedded_can::nb::Can`] into one. A driver with a blocking or bespoke
//! API gets a short newtype in the board's own crate; `link` shows the shape.

#![cfg_attr(not(test), no_std)]

pub mod bus;
pub mod config;
pub mod crc;
pub mod decode;
pub mod ecumaster;
pub mod frame;
mod gauge;
pub mod haltech;
pub mod blink;
pub mod drive;
pub mod trace;
pub mod indicators;
pub mod ingest;
pub mod link;
pub mod maxxecu;
pub mod megasquirt;
pub mod obd2;
pub mod pack;
pub mod rusefi;
pub mod speeduino;
pub mod subscribe;
pub mod units;
pub mod wire;
pub mod xfer;

pub use decode::CanSource;
pub use frame::CanFrame;
pub use gauge::*;
pub use pack::Wire;
pub use ingest::{ingest, Fed};
pub use link::{Link, NbLink, RecvError, SendError};
pub use units::{Quantity, Reading, Unit};

/// Every CAN-broadcast ECU this crate understands. A server that does not
/// know which ECU is attached can try each in turn:
/// `CAN_SOURCES.iter().find_map(|s| s.feed_default(&frame))`.
pub static CAN_SOURCES: &[&CanSource] = &[
    &rusefi::SOURCE,
    &haltech::SOURCE,
    &megasquirt::SOURCE,
    &maxxecu::SOURCE,
    &ecumaster::SOURCE,
];

use embedded_can::{Frame, Id, StandardId};

/// The CAN ID map of the display bus.
///
/// | Range           | Use                                                    |
/// |-----------------|--------------------------------------------------------|
/// | 0x000..=0x01F   | Reserved: bus control and node addressing (this module) |
/// | 0x020..=0x0FF   | Gauges, one ID each (see `gauge`)                       |
/// | 0x700..=0x7FF   | Left clear for OBD2 (0x7DF request, 0x7E8.. replies)    |
///
/// Inside the reserved range: 0x000 master ack, 0x001..=0x00F one message ID
/// per node (see `bus`), 0x010..=0x012 file transfer (see `xfer`), 0x015
/// client subscribe request, 0x018 bus state. A node's message ID *is* its
/// node address, so two nodes never contend for one ID and the lower address
/// wins arbitration. Every reserved ID sits below every gauge ID, so control
/// traffic pre-empts a gauge broadcast already in flight.
pub mod protocol {
    use super::*;

    /// Lowest and highest IDs reserved for bus control.
    pub const RESERVED_ID_MIN: u16 = 0x000;
    /// Top of the reserved range.
    pub const RESERVED_ID_MAX: u16 = 0x01F;
    /// Server acknowledges a client's subscription; payload is the gauge ID echoed back.
    pub const MASTER_ACK_ID: u16 = 0x000;
    /// Lowest node-to-master message ID; see `bus`.
    pub const NODE_MSG_MIN: u16 = 0x001;
    /// Highest node-to-master message ID.
    pub const NODE_MSG_MAX: u16 = 0x00F;
    /// Master broadcasts the global mode and bus flags here; see `bus`.
    pub const BUS_STATE_ID: u16 = 0x018;
    /// Client asks the server to start broadcasting gauges; payload is up to 8 gauge IDs.
    pub const CLIENT_REQUEST_ID: u16 = 0x015;
    /// First ID available to gauges.
    pub const GAUGE_ID_MIN: u16 = 0x020;
    /// Top of the gauge range. Gauge IDs travel as single bytes in a
    /// subscribe request, so the range cannot reach past 0x0FF.
    pub const GAUGE_ID_MAX: u16 = 0x0FF;

    // Control traffic only pre-empts a gauge broadcast because every
    // reserved ID sorts below every gauge ID.
    const _: () = assert!(NODE_MSG_MAX < GAUGE_ID_MIN);
    const _: () = assert!(BUS_STATE_ID <= RESERVED_ID_MAX);
    const _: () = assert!(RESERVED_ID_MAX < GAUGE_ID_MIN);

    /// Node address of the master (the ECU-facing server).
    pub const NODE_MASTER: u8 = 0x00;
    /// Highest address a node may take, fixed by the one-ID-per-node map.
    pub const NODE_MAX: u8 = 0x0F;
    /// Node address that every node answers to.
    pub const NODE_BROADCAST: u8 = 0xFF;

    /// True if `id` is reserved for bus control rather than a gauge.
    pub fn is_reserved(id: u16) -> bool {
        (RESERVED_ID_MIN..=RESERVED_ID_MAX).contains(&id)
    }

    /// True if `id` falls in the range gauges are allocated from.
    pub fn is_gauge_range(id: u16) -> bool {
        (GAUGE_ID_MIN..=GAUGE_ID_MAX).contains(&id)
    }

    /// The CAN ID `node` speaks on, or `None` for the master and for
    /// addresses past `NODE_MAX`.
    pub const fn node_msg_id(node: u8) -> Option<u16> {
        if node == NODE_MASTER || node > NODE_MAX {
            None
        } else {
            Some(node as u16)
        }
    }

    /// The node that owns message ID `id`, or `None` if it is not one.
    pub const fn msg_node(id: u16) -> Option<u8> {
        if id >= NODE_MSG_MIN && id <= NODE_MSG_MAX {
            Some(id as u8)
        } else {
            None
        }
    }

    /// Build a standard-ID frame; `None` if the ID is over 11 bits or the payload over 8 bytes.
    pub fn frame(id: u16, data: &[u8]) -> Option<CanFrame> {
        CanFrame::new(Id::Standard(StandardId::new(id)?), data)
    }

    /// Standard ID of a frame, or `None` for extended IDs.
    pub fn id_of(frame: &CanFrame) -> Option<u16> {
        match frame.id() {
            Id::Standard(s) => Some(s.as_raw()),
            Id::Extended(_) => None,
        }
    }

    /// Frame a client sends to subscribe to `ids` (at most 8 per frame).
    pub fn request_frame(ids: &[u8]) -> Option<CanFrame> {
        frame(CLIENT_REQUEST_ID, ids)
    }

    /// Frame the server sends to acknowledge subscription to `id`.
    pub fn ack_frame(id: u8) -> Option<CanFrame> {
        frame(MASTER_ACK_ID, &[id])
    }

    /// True if this frame is a client subscribe request.
    pub fn is_request(frame: &CanFrame) -> bool {
        id_of(frame) == Some(CLIENT_REQUEST_ID)
    }

    /// True if this frame is a master subscription acknowledgement.
    pub fn is_ack(frame: &CanFrame) -> bool {
        id_of(frame) == Some(MASTER_ACK_ID)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedError {
    /// Extended (29-bit) IDs are not part of the spec.
    ExtendedId,
    /// No gauge is defined at this ID (protocol frames land here too).
    UnknownId(u16),
    /// Payload length does not match the gauge's wire width.
    BadLength { id: u16, expected: usize, got: usize },
}

/// Store an incoming gauge frame. Frames that are not gauges are rejected,
/// never panicked on, so this is safe to call on every frame off the bus.
pub fn feed_frame(frame: &CanFrame) -> Result<&'static GaugeData, FeedError> {
    let id = match frame.id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return Err(FeedError::ExtendedId),
    };
    let gauge = gauge_by_id(id).ok_or(FeedError::UnknownId(id))?;
    gauge
        .set_from_frame(frame)
        .map_err(|FrameError::BadLength { expected, got }| FeedError::BadLength {
            id,
            expected,
            got,
        })?;
    Ok(gauge)
}

/// Frame for the gauge at `id`, or `None` if the ID is unknown or the gauge
/// has not been set yet.
pub fn frame_for(id: u16) -> Option<CanFrame> {
    gauge_by_id(id)?.to_frame()
}

/// Reset every gauge to unset. Useful when the ECU link drops.
pub fn clear_all() {
    for g in ALL_GAUGES {
        g.clear();
    }
}
