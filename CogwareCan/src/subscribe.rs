// SPDX-License-Identifier: GPL-3.0-only
//! Which gauges each display wants, and how it asks for them.
//!
//! Both halves are pure state machines over CAN frames. They touch no driver,
//! no timer and no pin, so they are the same on every board and every CAN
//! controller. The firmware owns the loop: it hands frames in and transmits
//! the frames that come out, and it alone decides retry pacing.
//!
//! Display side:
//!
//! ```ignore
//! let mut sub = Subscription::new(&WANTED);
//! while !sub.is_complete() {
//!     for f in sub.request() { can.send_message(f).ok(); }   // only what is still missing
//!     while waiting {
//!         if let Ok(f) = can.read_message() { sub.feed(&f); }
//!     }
//! }
//! ```
//!
//! Master side:
//!
//! ```ignore
//! for ack in publisher.feed(&frame) { can.send_message(ack).ok(); }
//! for f in publisher.broadcast() { can.send_message(f).ok(); }
//! ```

use crate::protocol::{ack_frame, is_ack, is_request, request_frame};
use crate::{frame_for, gauge_by_id};
use embedded_can::Frame;
use crate::frame::CanFrame;

/// Gauge IDs that fit in one request frame.
pub const IDS_PER_FRAME: usize = 8;

/// A set of gauge IDs. IDs are one byte, so the whole set is 32 bytes and
/// there is no capacity to run out of. Iteration is in ID order, which is
/// also CAN arbitration order.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct IdSet {
    bits: [u8; 32],
}

impl IdSet {
    pub const fn new() -> Self {
        IdSet { bits: [0; 32] }
    }

    /// Add an ID. Returns true if it was not already present.
    pub fn insert(&mut self, id: u8) -> bool {
        let (i, m) = (id as usize / 8, 1 << (id % 8));
        let new = self.bits[i] & m == 0;
        self.bits[i] |= m;
        new
    }

    pub fn remove(&mut self, id: u8) {
        self.bits[id as usize / 8] &= !(1 << (id % 8));
    }

    pub fn contains(&self, id: u8) -> bool {
        self.bits[id as usize / 8] & (1 << (id % 8)) != 0
    }

    pub fn len(&self) -> usize {
        self.bits.iter().map(|b| b.count_ones() as usize).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.bits.iter().all(|b| *b == 0)
    }

    pub fn clear(&mut self) {
        self.bits = [0; 32];
    }

    pub fn iter(&self) -> IdIter {
        IdIter { set: *self, next: 0 }
    }
}

impl IntoIterator for IdSet {
    type Item = u8;
    type IntoIter = IdIter;

    fn into_iter(self) -> IdIter {
        IdIter { set: self, next: 0 }
    }
}

impl Default for IdSet {
    fn default() -> Self {
        Self::new()
    }
}

/// Yields the IDs in a set, lowest first. Holds its own copy of the set, so
/// the set it came from stays free to change.
pub struct IdIter {
    set: IdSet,
    next: u16,
}

impl Iterator for IdIter {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        while self.next < 256 {
            let id = self.next as u8;
            self.next += 1;
            if self.set.contains(id) {
                return Some(id);
            }
        }
        None
    }
}

// ===================== master side =====================

/// Tracks what the displays on the bus have asked for.
#[derive(Clone, Copy, Debug, Default)]
pub struct Publisher {
    wanted: IdSet,
}

impl Publisher {
    pub const fn new() -> Self {
        Publisher { wanted: IdSet::new() }
    }

    /// Feed every frame off the bus. Requests are recorded and the returned
    /// iterator yields one acknowledgement per ID asked for; everything else
    /// yields nothing.
    ///
    /// Every requested ID is acknowledged, including one this build has no
    /// gauge for, so a display running an older spec finishes subscribing
    /// instead of retrying forever. Only IDs that name a real gauge are
    /// recorded, so `broadcast` never wastes bus time on them.
    pub fn feed(&mut self, frame: &CanFrame) -> Acks {
        if !is_request(frame) {
            return Acks::empty();
        }
        let ids = &frame.data()[..frame.dlc().min(IDS_PER_FRAME)];
        for &id in ids {
            if gauge_by_id(id as u16).is_some() {
                self.wanted.insert(id);
            }
        }
        Acks::new(ids)
    }

    /// Record a subscription directly, for a master with a fixed set of
    /// displays that does not wait to be asked.
    pub fn subscribe(&mut self, id: u8) -> bool {
        gauge_by_id(id as u16).is_some() && self.wanted.insert(id)
    }

    pub fn wanted(&self) -> &IdSet {
        &self.wanted
    }

    pub fn is_empty(&self) -> bool {
        self.wanted.is_empty()
    }

    pub fn len(&self) -> usize {
        self.wanted.len()
    }

    /// Forget every subscription, so displays must ask again.
    pub fn clear(&mut self) {
        self.wanted.clear();
    }

    /// One frame per subscribed gauge that has a value, lowest ID first.
    /// A gauge that has never been written yields no frame, so a display can
    /// tell "this ECU does not supply it" from "the value is zero".
    pub fn broadcast(&self) -> Broadcast {
        Broadcast { ids: self.wanted.into_iter() }
    }

    /// One round, carrying only what has moved or is due to be repeated.
    pub fn broadcast_fresh<'a>(&self, fresh: &'a mut Fresh, since_ms: u16) -> BroadcastFresh<'a> {
        BroadcastFresh { ids: self.wanted.into_iter(), fresh, since_ms, offered: None }
    }
}

/// Acknowledgements to transmit, at most one frame's worth.
pub struct Acks {
    ids: [u8; IDS_PER_FRAME],
    len: u8,
    at: u8,
}

impl Acks {
    fn empty() -> Self {
        Acks { ids: [0; IDS_PER_FRAME], len: 0, at: 0 }
    }

    fn new(ids: &[u8]) -> Self {
        let mut a = Acks::empty();
        a.len = ids.len().min(IDS_PER_FRAME) as u8;
        a.ids[..a.len as usize].copy_from_slice(&ids[..a.len as usize]);
        a
    }
}

impl Iterator for Acks {
    type Item = CanFrame;

    fn next(&mut self) -> Option<CanFrame> {
        while self.at < self.len {
            let id = self.ids[self.at as usize];
            self.at += 1;
            if let Some(f) = ack_frame(id) {
                return Some(f);
            }
        }
        None
    }
}

pub struct Broadcast {
    ids: IdIter,
}

impl Iterator for Broadcast {
    type Item = CanFrame;

    fn next(&mut self) -> Option<CanFrame> {
        loop {
            let id = self.ids.next()?;
            if let Some(f) = frame_for(id as u16) {
                return Some(f);
            }
        }
    }
}

/// The same round, skipping whatever the bus has already been told.
///
/// Holds the record by mutable borrow, so the values it compares against are
/// the ones it is about to send and cannot drift from them.
///
/// Nothing is recorded as told until the caller says it went out. A frame the
/// controller refuses is offered again on the next round, rather than being
/// written off and waited out: the transmit buffer is busy for a hundred
/// microseconds and the keepalive is five hundred milliseconds, so charging a
/// refusal to the keepalive froze a gauge for five thousand times as long as
/// the thing that went wrong.
pub struct BroadcastFresh<'a> {
    ids: IdIter,
    fresh: &'a mut Fresh,
    since_ms: u16,
    /// The id and value of the frame last handed out, until it is kept.
    offered: Option<(u8, i32)>,
}

impl BroadcastFresh<'_> {
    /// Record the frame just taken as sent, so it is not offered again until
    /// it moves or falls due.
    ///
    /// Call it once a send has actually succeeded. A round dropped without
    /// calling it leaves the channel exactly as it was.
    pub fn keep(&mut self) {
        if let Some((id, value)) = self.offered.take() {
            self.fresh.record(id, value);
        }
    }
}

impl Iterator for BroadcastFresh<'_> {
    type Item = CanFrame;

    fn next(&mut self) -> Option<CanFrame> {
        loop {
            let id = self.ids.next()?;
            let Some(gauge) = gauge_by_id(id as u16) else {
                continue;
            };
            let value = gauge.raw();
            // The clock is charged once per id per round whether or not the
            // frame is sent, so the keepalive counts rounds gone by; what is
            // held back is only the record of having told the bus.
            if !self.fresh.is_due(id, value, self.since_ms) {
                continue;
            }
            if let Some(f) = frame_for(id as u16) {
                self.offered = Some((id, value));
                return Some(f);
            }
        }
    }
}

// ===================== display side =====================

/// Asks the master for a fixed list of gauges and tracks the replies.
#[derive(Clone, Copy, Debug)]
pub struct Subscription<'a> {
    wanted: &'a [u8],
    acked: IdSet,
}

impl<'a> Subscription<'a> {
    /// `wanted` is the display's gauge list, fixed at build time.
    pub const fn new(wanted: &'a [u8]) -> Self {
        Subscription { wanted, acked: IdSet::new() }
    }

    /// Frames asking for everything not yet acknowledged, eight IDs each.
    /// Call again after a timeout to retry; the list shrinks as acks arrive.
    ///
    /// The iterator holds its own snapshot, so acks may be fed while it is
    /// still being drained.
    pub fn request(&self) -> Requests<'a> {
        Requests { wanted: self.wanted, acked: self.acked, at: 0 }
    }

    /// Feed every frame off the bus. Returns true if this frame acknowledged
    /// a gauge that was still outstanding.
    pub fn feed(&mut self, frame: &CanFrame) -> bool {
        if !is_ack(frame) || frame.dlc() < 1 {
            return false;
        }
        let id = frame.data()[0];
        self.wanted.contains(&id) && self.acked.insert(id)
    }

    /// Forget that the master acknowledged anything, so the node asks again.
    ///
    /// A subscription lives in the master's memory and nowhere else. If that
    /// board restarts, every node still believes it is subscribed and has
    /// nothing left to say, while the master no longer knows anyone wants
    /// anything -- and the bus goes quiet with both ends waiting for the
    /// other. A node that stops hearing its gauges calls this and asks again.
    pub fn forget(&mut self) {
        self.acked.clear();
    }

    pub fn is_complete(&self) -> bool {
        self.wanted.iter().all(|id| self.acked.contains(*id))
    }

    /// How many gauges are still waiting to be acknowledged.
    pub fn pending(&self) -> usize {
        self.wanted.iter().filter(|id| !self.acked.contains(**id)).count()
    }

    pub fn wanted(&self) -> &'a [u8] {
        self.wanted
    }

    /// Start over, after the master reset or the bus dropped.
    pub fn reset(&mut self) {
        self.acked.clear();
    }
}

pub struct Requests<'a> {
    wanted: &'a [u8],
    acked: IdSet,
    at: usize,
}

impl Iterator for Requests<'_> {
    type Item = CanFrame;

    fn next(&mut self) -> Option<CanFrame> {
        let mut buf = [0u8; IDS_PER_FRAME];
        let mut n = 0;
        while self.at < self.wanted.len() && n < IDS_PER_FRAME {
            let id = self.wanted[self.at];
            self.at += 1;
            if !self.acked.contains(id) {
                buf[n] = id;
                n += 1;
            }
        }
        if n == 0 {
            return None;
        }
        request_frame(&buf[..n])
    }
}

/// What was last put on the bus, so a channel that has not moved is not sent
/// again.
///
/// A dash broadcasting every subscribed gauge every round sends the same
/// number over and over: measured on this bus, twelve gauges at fifty rounds a
/// second came to five hundred and thirty frames a second, of which almost all
/// carried a value the displays already had. It also makes a readout flicker
/// through values faster than anybody can read one.
///
/// Slow channels then go quiet, so each is still repeated on a timer: a node
/// that joins late, or that missed one, must not wait for a coolant gauge to
/// move before it learns the temperature.
pub struct Fresh {
    /// The last value sent for each gauge id.
    last: [i32; 256],
    /// Milliseconds left before this id is due again whatever it reads.
    due: [u16; 256],
}

/// How long a channel may stay quiet before it is repeated anyway.
pub const KEEPALIVE_MS: u16 = 500;

impl Default for Fresh {
    fn default() -> Self {
        Self::new()
    }
}

impl Fresh {
    /// Nothing sent yet, so the first round sends everything.
    #[must_use]
    pub const fn new() -> Self {
        Self { last: [0; 256], due: [0; 256] }
    }

    /// Whether `id` wants sending, given what it reads now.
    ///
    /// `elapsed_ms` is the time since the last call, so the caller's own loop
    /// period is the clock and this needs no timer of its own. Asking does not
    /// record anything: a caller that asks and then fails to send must be able
    /// to ask again, so [`record`] is separate and follows a send that worked.
    ///
    /// [`record`]: Self::record
    pub fn is_due(&mut self, id: u8, value: i32, elapsed_ms: u16) -> bool {
        let slot = usize::from(id);
        self.due[slot] = self.due[slot].saturating_sub(elapsed_ms);
        self.last[slot] != value || self.due[slot] == 0
    }

    /// Note that the bus has been told `value` for `id`.
    pub fn record(&mut self, id: u8, value: i32) {
        let slot = usize::from(id);
        self.last[slot] = value;
        self.due[slot] = KEEPALIVE_MS;
    }

    /// Ask and record in one go, for a caller with nothing that can refuse.
    pub fn worth_sending(&mut self, id: u8, value: i32, elapsed_ms: u16) -> bool {
        if !self.is_due(id, value, elapsed_ms) {
            return false;
        }
        self.record(id, value);
        true
    }
}

#[cfg(test)]
mod fresh_tests {
    use super::*;

    #[test]
    fn a_value_that_has_not_moved_is_not_sent_again() {
        let mut fresh = Fresh::new();
        assert!(fresh.worth_sending(0x2D, 3000, 20), "the first one always goes");
        assert!(!fresh.worth_sending(0x2D, 3000, 20), "and the same one does not");
        assert!(fresh.worth_sending(0x2D, 3100, 20), "a change does");
    }

    #[test]
    fn a_frame_that_never_went_out_is_offered_again() {
        // The bug this guards: asking used to record, so a frame the CAN
        // controller refused was written off as told and the channel then sat
        // silent for a whole keepalive -- half a second of a frozen needle
        // bought by a transmit buffer that was busy for a hundred microseconds.
        let mut fresh = Fresh::new();
        assert!(fresh.is_due(0x2D, 3000, 20), "a value never sent is due");
        assert!(
            fresh.is_due(0x2D, 3000, 20),
            "and stays due while nothing has confirmed it went out"
        );
        fresh.record(0x2D, 3000);
        assert!(!fresh.is_due(0x2D, 3000, 20), "only a send settles it");
    }

    #[test]
    fn a_quiet_channel_is_repeated_on_a_timer() {
        // Or a node that joined late would wait for the coolant to move before
        // it learned the temperature.
        let mut fresh = Fresh::new();
        assert!(fresh.worth_sending(0x26, 890, 20));
        for _ in 0..(KEEPALIVE_MS / 20) - 1 {
            assert!(!fresh.worth_sending(0x26, 890, 20), "quiet until it is due");
        }
        assert!(fresh.worth_sending(0x26, 890, 20), "and then again anyway");
    }

    #[test]
    fn one_gauge_moving_does_not_send_another() {
        let mut fresh = Fresh::new();
        assert!(fresh.worth_sending(0x2D, 3000, 20));
        assert!(fresh.worth_sending(0x26, 890, 20));
        assert!(fresh.worth_sending(0x2D, 3200, 20), "revs moved");
        assert!(!fresh.worth_sending(0x26, 890, 20), "coolant did not");
    }
}

#[cfg(test)]
mod restart_tests {
    use super::*;

    #[test]
    fn a_node_can_ask_again_after_the_master_forgets_it() {
        // The master keeps subscriptions in memory. When it restarts it knows
        // nothing, and a node that thinks it is still subscribed never speaks
        // again -- so both ends wait for the other and the bus stays quiet.
        let wanted = [0x2Du8, 0x26];
        let mut sub = Subscription::new(&wanted);
        let mut master = Publisher::new();
        for request in sub.request() {
            for ack in master.feed(&request) {
                sub.feed(&ack);
            }
        }
        assert!(sub.is_complete(), "subscribed to begin with");
        assert_eq!(sub.request().count(), 0, "and has nothing to ask for");

        sub.forget();
        assert!(!sub.is_complete(), "and asks again once it has forgotten");
        assert!(sub.request().count() > 0);
    }
}
