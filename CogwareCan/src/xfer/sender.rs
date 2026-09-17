// SPDX-License-Identifier: GPL-3.0-only
//! Master side of a transfer: turn an image into frames and track replies.

use super::{check, Cmd, Image, Status, BLOCK_LEN, CMD_ID, DATA_ID, REPLY_ID};
use crate::protocol::{frame, id_of, NODE_BROADCAST};
use crate::frame::CanFrame;
use embedded_can::Frame;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxState {
    /// `next_frame` yields `Begin`.
    Begin,
    /// Waiting for `Ready` from every target.
    WaitReady,
    /// `next_frame` yields data blocks, then `Check`.
    Data,
    /// Waiting for `Ok`/`Missing` from every target.
    WaitCheck,
    /// `next_frame` yields `End`.
    End,
    /// Waiting for `Done` from every target.
    WaitDone,
    /// `next_frame` yields `Commit`.
    Commit,
    Done,
    Failed(Status),
}

/// Where a sender takes the image from.
///
/// A slice is the obvious one and is implemented below. The other is a window
/// onto an image too large to hold at once: a master with a few hundred
/// kilobytes of memory cannot keep a firmware image, and does not have to —
/// a transfer never looks further back than the window it is checkpointing.
// `len` here is the size of an image, not the count of a collection, and an
// image of no bytes is refused before a sender is built. An `is_empty` would
// be a question nothing asks.
#[allow(clippy::len_without_is_empty)]
pub trait Source {
    /// Length of the whole image.
    fn len(&self) -> u32;

    /// CRC-32 of the whole image.
    ///
    /// Known before the last byte is sent, because whoever supplied the image
    /// computed it. A sender streaming an image it has never seen whole
    /// cannot work this out for itself.
    fn crc(&self) -> u32;

    /// Copy `out.len()` bytes from `offset`, returning how many were there.
    ///
    /// Short of what was asked means the source has not caught up, and the
    /// sender waits rather than sending a short block: every block but the
    /// last is a whole one, and the rewind arithmetic depends on it.
    fn read(&mut self, offset: u32, out: &mut [u8]) -> usize;
}

impl Source for &[u8] {
    fn len(&self) -> u32 {
        <[u8]>::len(self) as u32
    }

    fn crc(&self) -> u32 {
        crate::crc::crc32(self)
    }

    fn read(&mut self, offset: u32, out: &mut [u8]) -> usize {
        let start = (offset as usize).min(<[u8]>::len(self));
        let end = (start + out.len()).min(<[u8]>::len(self));
        out[..end - start].copy_from_slice(&self[start..end]);
        end - start
    }
}

/// Checkpoints in a row that may deliver nothing before a transfer is given
/// up on.
///
/// A block the far end refuses is sent again, which is what carries an image
/// across a bus that drops one now and then. On a bus that is broken rather
/// than lossy the same block goes back and forth for ever, so the two ends
/// have to agree to stop: enough attempts to ride out a bad patch, few enough
/// that a wall is hit in seconds rather than never.
const STALL_LIMIT: u16 = 32;

/// Master side of a transfer.
pub struct Sender<S> {
    node: u8,
    source: S,
    size: u32,
    kind: u8,
    slot: u8,
    window: u16,
    replies_needed: u8,
    state: TxState,
    seq: u16,
    offset: u32,
    in_window: u16,
    replies: u8,
    rewind_to: Option<u16>,
    blocks: u32,
    crc: u32,
    confirmed: u32,
    /// Checkpoints since the far end last took delivery of anything new.
    stalled: u16,
}

impl<S: Source> Sender<S> {
    /// Push the image in `source` (tagged `kind`, see the `kind` module) to
    /// `node`, or to all nodes with `NODE_BROADCAST`. `window` is blocks
    /// between checkpoints; `replies_needed` is how many nodes must answer
    /// each checkpoint (1 for a single node, the node count for broadcast).
    pub fn new(node: u8, source: S, kind: u8, window: u16, replies_needed: u8) -> Self {
        let size = source.len();
        let crc = source.crc();
        Sender {
            node,
            source,
            size,
            kind,
            slot: 0,
            window: window.max(1),
            replies_needed: replies_needed.max(1),
            state: TxState::Begin,
            seq: 0,
            offset: 0,
            in_window: 0,
            replies: 0,
            rewind_to: None,
            blocks: 0,
            crc,
            confirmed: 0,
            stalled: 0,
        }
    }

    /// Give the source back, so a master can reuse the buffer behind it.
    pub fn into_source(self) -> S {
        self.source
    }

    /// The source, to put more of the image into.
    ///
    /// A master streaming an image it cannot hold owns the buffer the sender
    /// reads from, and has to keep filling it as the host sends more.
    pub fn source_mut(&mut self) -> &mut S {
        &mut self.source
    }

    /// Land the image in `slot` rather than slot 0, for a node that keeps
    /// more than one image of this kind.
    pub fn with_slot(mut self, slot: u8) -> Self {
        self.slot = slot;
        self
    }

    pub fn state(&self) -> TxState {
        self.state
    }

    /// What this sender is pushing.
    pub fn image(&self) -> Image {
        Image {
            size: self.size,
            kind: self.kind,
            slot: self.slot,
        }
    }

    /// Bytes the far end has acknowledged, and the image size.
    ///
    /// Acknowledged, not sent: a block lost on the bus rewinds the sender to
    /// it and everything after it goes again, so bytes sent is a number that
    /// moves both ways. Anything showing this to a person — a progress bar
    /// most of all — needs the one that only ever climbs.
    pub fn progress(&self) -> (u32, u32) {
        (self.confirmed, self.size)
    }

    /// Acknowledged bytes as a percentage, for `bus::Bus::update_progress`.
    pub fn percent(&self) -> u8 {
        if self.size == 0 {
            return 100;
        }
        (self.confirmed.min(self.size) as u64 * 100 / u64::from(self.size)) as u8
    }

    /// Call after a timeout while in a `Wait*` state to resend the last control frame.
    pub fn resend(&mut self) -> Option<CanFrame> {
        self.replies = 0;
        match self.state {
            TxState::WaitReady => self.ctrl(Cmd::Begin),
            TxState::WaitCheck => self.ctrl(Cmd::Check),
            TxState::WaitDone => self.ctrl(Cmd::End),
            _ => None,
        }
    }

    pub fn abort(&mut self) -> Option<CanFrame> {
        self.state = TxState::Failed(Status::Aborted);
        self.ctrl(Cmd::Abort)
    }

    fn ctrl(&self, cmd: Cmd) -> Option<CanFrame> {
        let mut buf = [0u8; 8];
        buf[0] = self.node;
        buf[1] = cmd as u8;
        let len = match cmd {
            Cmd::Begin => {
                buf[2..6].copy_from_slice(&self.size.to_le_bytes());
                buf[6] = self.kind;
                buf[7] = self.slot;
                8
            }
            Cmd::Check => {
                buf[2..4].copy_from_slice(&self.seq.wrapping_sub(1).to_le_bytes());
                4
            }
            Cmd::End => {
                buf[2..6].copy_from_slice(&self.crc.to_le_bytes());
                6
            }
            Cmd::Commit | Cmd::Abort => 2,
        };
        frame(CMD_ID, &buf[..len])
    }

    /// Next frame to transmit, or `None` while waiting for replies or when finished.
    pub fn next_frame(&mut self) -> Option<CanFrame> {
        match self.state {
            TxState::Begin => {
                self.state = TxState::WaitReady;
                self.replies = 0;
                self.ctrl(Cmd::Begin)
            }
            TxState::Data => {
                if self.offset >= self.size || self.in_window >= self.window {
                    self.in_window = 0;
                    self.replies = 0;
                    self.rewind_to = None;
                    self.state = TxState::WaitCheck;
                    return self.ctrl(Cmd::Check);
                }
                let want = (self.size - self.offset).min(BLOCK_LEN as u32) as usize;
                let mut buf = [0u8; 8];
                let got = self.source.read(self.offset, &mut buf[2..2 + want]);
                // Short of a whole block means the source has not caught up.
                // Nothing is sent: every block but the last is a whole one,
                // and the rewind below counts on that.
                if got < want {
                    return None;
                }
                buf[..2].copy_from_slice(&self.seq.to_le_bytes());
                // Over the sequence number as well as the payload: a block
                // believed at the wrong offset is as bad as a wrong byte.
                buf[2 + want] = check(&buf[..2 + want]);
                let f = frame(DATA_ID, &buf[..3 + want]);
                self.offset += want as u32;
                self.seq = self.seq.wrapping_add(1);
                self.blocks += 1;
                self.in_window += 1;
                f
            }
            TxState::End => {
                self.state = TxState::WaitDone;
                self.replies = 0;
                self.ctrl(Cmd::End)
            }
            TxState::Commit => {
                self.state = TxState::Done;
                self.ctrl(Cmd::Commit)
            }
            _ => None,
        }
    }

    /// Feed node replies (frames with `REPLY_ID`); other frames are ignored.
    pub fn feed_reply(&mut self, f: &CanFrame) {
        if id_of(f) != Some(REPLY_ID) || f.dlc() < 4 {
            return;
        }
        let d = f.data();
        if self.node != NODE_BROADCAST && d[0] != self.node {
            return;
        }
        let Some(status) = Status::from_u8(d[1]) else {
            return;
        };
        let seq = u16::from_le_bytes([d[2], d[3]]);
        match (self.state, status) {
            (TxState::WaitReady, Status::Ready) => self.count(TxState::Data),
            (TxState::WaitCheck, Status::Ok) => self.count(TxState::Data),
            (TxState::WaitCheck, Status::Missing) => {
                self.rewind_to = Some(match self.rewind_to {
                    Some(r) if self.seq.wrapping_sub(r) > self.seq.wrapping_sub(seq) => r,
                    _ => seq,
                });
                self.count(TxState::Data);
            }
            (TxState::WaitDone, Status::Done) => self.count(TxState::Commit),
            (_, Status::Refused | Status::CrcFail | Status::Aborted) => {
                self.state = TxState::Failed(status)
            }
            _ => {}
        }
    }

    fn count(&mut self, next: TxState) {
        self.replies += 1;
        if self.replies < self.replies_needed {
            return;
        }
        if let Some(seq) = self.rewind_to.take() {
            // Nothing new got through this window. A few of these is a lossy
            // patch; a run of them is a link that is not going to carry the
            // image however long it is given.
            self.stalled = self.stalled.saturating_add(1);
            if self.stalled >= STALL_LIMIT {
                self.state = TxState::Failed(Status::Missing);
                return;
            }
            let back = u32::from(self.seq.wrapping_sub(seq));
            self.blocks = self.blocks.saturating_sub(back);
            self.offset = (self.blocks * BLOCK_LEN as u32).min(self.size);
            self.seq = seq;
            // What was confirmed stays confirmed. A rewind can reach back past
            // an earlier checkpoint, but the node did answer that checkpoint
            // and the bytes behind it are still the ones it holds -- and a bar
            // that walks backwards reads as a fault to whoever is watching it.
            self.state = next;
            return;
        }
        // Every target answered and none is missing anything, so what has gone
        // out so far really is at the far end. A high-water mark, never a step
        // back: see the rewind above.
        if self.offset > self.confirmed {
            self.stalled = 0;
        }
        self.confirmed = self.confirmed.max(self.offset);
        if next == TxState::Data && self.offset >= self.size {
            self.state = TxState::End;
            return;
        }
        self.state = next;
    }
}
