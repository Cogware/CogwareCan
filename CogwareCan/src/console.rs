// SPDX-License-Identifier: GPL-3.0-only
//! The last of what a node printed, kept so it can be sent somewhere later.
//!
//! A console is a stream nobody may be reading when it is written. [`Tail`]
//! keeps the newest `N` bytes of it and a reader's place in them, so the same
//! text can be trickled onto a bus as it appears and replayed from the oldest
//! byte still held when somebody asks.

/// The newest `N` bytes of a byte stream, and one reader's place in them.
///
/// Writing never fails: the oldest bytes are overwritten, and a reader that
/// had not reached them resumes at the oldest byte that survived.
#[derive(Clone, Debug)]
pub struct Tail<const N: usize> {
    buf: [u8; N],
    written: u64,
    read: u64,
}

impl<const N: usize> Default for Tail<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Tail<N> {
    /// A tail holding nothing.
    pub const fn new() -> Self {
        Tail {
            buf: [0; N],
            written: 0,
            read: 0,
        }
    }

    /// Append `text`, overwriting the oldest bytes once `N` are held.
    pub fn write(&mut self, text: &[u8]) {
        for &byte in text {
            self.buf[(self.written % N as u64) as usize] = byte;
            self.written += 1;
        }
    }

    /// Where the oldest byte still held sits in the stream.
    fn oldest(&self) -> u64 {
        self.written.saturating_sub(N as u64)
    }

    /// Bytes written that the reader has not yet consumed and can still have.
    pub fn pending(&self) -> usize {
        (self.written - self.read.max(self.oldest())) as usize
    }

    /// Copy the next unread bytes into `out` without consuming them, and
    /// return how many. Call [`Tail::consume`] once they have gone.
    pub fn peek(&mut self, out: &mut [u8]) -> usize {
        self.read = self.read.max(self.oldest());
        let n = out.len().min((self.written - self.read) as usize);
        for (i, slot) in out[..n].iter_mut().enumerate() {
            *slot = self.buf[((self.read + i as u64) % N as u64) as usize];
        }
        n
    }

    /// Move the reader past `n` bytes a [`Tail::peek`] returned.
    pub fn consume(&mut self, n: usize) {
        self.read = (self.read + n as u64).min(self.written);
    }

    /// Put the reader back at the oldest byte still held.
    pub fn rewind(&mut self) {
        self.read = self.oldest();
    }
}
