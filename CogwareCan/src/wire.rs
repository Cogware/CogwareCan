// SPDX-License-Identifier: GPL-3.0-only
//! The wire between a host and the server: framing, and the messages both
//! ends must agree on.
//!
//! It began as the USB link alone, and the shape it took there is why it
//! suits the others. UART0 is also the port the ROM bootloader logs on and
//! the port a reflash drives, so the stream starts with bytes that are not
//! ours: every frame carries a sync pattern and a CRC, and the decoder
//! resynchronises rather than trusting what it is handed.
//!
//! That same property is what lets Bluetooth and a network socket carry these
//! frames unchanged. A socket is a stream with no message boundaries of its
//! own, and a radio link drops a connection mid-sentence; a format that can
//! find its own way back into step handles both without knowing which it is
//! on. So an editor pushing a scene writes the same bytes down a cable, over
//! Bluetooth, or into a socket, and the server reads them with one decoder.
//!
//! This lives here, with the bus protocol, rather than in the server, because
//! the server is not the only thing that has to speak it. Two definitions of
//! a wire format is one more than can be kept true.

/// Longest payload a frame can carry.
///
/// The length field is two bytes because a settings blob does not fit in one:
/// the table encodes to a little over three hundred bytes and would otherwise
/// need chunking, which is more protocol than a wider field.
pub const MAX_PAYLOAD: usize = 1024;
/// First byte of the sync pattern.
pub const SYNC0: u8 = 0xAA;
/// Second byte of the sync pattern.
pub const SYNC1: u8 = 0x55;
/// Longest encoded frame, so a caller can size a buffer.
pub const MAX_FRAME: usize = 2 + 1 + 2 + MAX_PAYLOAD + 2;

/// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection
/// and no final xor.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Bytes of frame that are not payload: two sync, one kind, two length, two CRC.
pub const OVERHEAD: usize = 7;

/// Write a whole frame into `out`, returning its length, or `None` if the
/// payload is too long or `out` is too small.
pub fn encode(kind: u8, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    if payload.len() > MAX_PAYLOAD {
        return None;
    }
    let len = OVERHEAD + payload.len();
    if out.len() < len {
        return None;
    }
    out[0] = SYNC0;
    out[1] = SYNC1;
    out[2] = kind;
    out[3..5].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    out[5..5 + payload.len()].copy_from_slice(payload);

    let crc = crc16(&out[2..5 + payload.len()]);
    out[5 + payload.len()..len].copy_from_slice(&crc.to_le_bytes());
    Some(len)
}

/// Where the decoder is in a frame.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    /// Hunting for the first sync byte.
    Sync0,
    /// The first sync byte has arrived.
    Sync1,
    /// Next byte is the kind.
    Kind,
    /// Next byte is the low half of the payload length.
    LenLo,
    /// Next byte is the high half of the payload length.
    LenHi,
    /// Taking payload bytes.
    Payload,
    /// Next byte is the low half of the CRC.
    CrcLo,
    /// Next byte is the high half of the CRC.
    CrcHi,
}

/// Byte-at-a-time frame decoder. It never allocates and never panics,
/// whatever arrives on the wire.
pub struct Decoder {
    state: State,
    kind: u8,
    want: usize,
    got: usize,
    buf: [u8; MAX_PAYLOAD],
    crc: u16,
    crc_errors: u32,
}

impl Decoder {
    /// A decoder hunting for sync.
    pub const fn new() -> Self {
        Self {
            state: State::Sync0,
            kind: 0,
            want: 0,
            got: 0,
            buf: [0; MAX_PAYLOAD],
            crc: 0,
            crc_errors: 0,
        }
    }

    /// Feed one byte, yielding the kind and payload length of a frame whose
    /// CRC has just checked out. The payload is then readable from
    /// [`payload`](Self::payload) until the next call.
    pub fn push(&mut self, byte: u8) -> Option<(u8, usize)> {
        match self.state {
            State::Sync0 => {
                if byte == SYNC0 {
                    self.state = State::Sync1;
                }
            }
            State::Sync1 => {
                self.state = match byte {
                    SYNC1 => State::Kind,
                    // A second 0xAA is the real first sync byte of a frame
                    // whose predecessor was truncated; dropping back to
                    // hunting here would lose it.
                    SYNC0 => State::Sync1,
                    _ => State::Sync0,
                };
            }
            State::Kind => {
                self.kind = byte;
                self.state = State::LenLo;
            }
            State::LenLo => {
                self.want = byte as usize;
                self.state = State::LenHi;
            }
            State::LenHi => {
                self.want |= (byte as usize) << 8;
                self.got = 0;
                // A length past the buffer is noise that happened to carry
                // the sync pattern; taking it would overrun on the next byte.
                self.state = if self.want > MAX_PAYLOAD {
                    State::Sync0
                } else if self.want == 0 {
                    State::CrcLo
                } else {
                    State::Payload
                };
            }
            State::Payload => {
                self.buf[self.got] = byte;
                self.got += 1;
                if self.got == self.want {
                    self.state = State::CrcLo;
                }
            }
            State::CrcLo => {
                self.crc = byte as u16;
                self.state = State::CrcHi;
            }
            State::CrcHi => {
                self.crc |= (byte as u16) << 8;
                self.state = State::Sync0;
                let len = (self.want as u16).to_le_bytes();
                let header = [self.kind, len[0], len[1]];
                let mut computed = crc16_continue(0xFFFF, &header);
                computed = crc16_continue(computed, &self.buf[..self.want]);
                if computed == self.crc {
                    return Some((self.kind, self.want));
                }
                self.crc_errors = self.crc_errors.wrapping_add(1);
            }
        }
        None
    }

    /// Payload of the frame the last [`push`](Self::push) completed.
    pub fn payload(&self) -> &[u8] {
        &self.buf[..self.want]
    }

    /// Frames discarded because their CRC did not match.
    pub fn crc_errors(&self) -> u32 {
        self.crc_errors
    }

    /// Abandon any partial frame and hunt for sync again.
    pub fn reset(&mut self) {
        self.state = State::Sync0;
        self.got = 0;
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Continue a CRC over a further run of bytes, so the decoder can cover the
/// header and the payload without copying them into one buffer.
fn crc16_continue(mut crc: u16, data: &[u8]) -> u16 {
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}


// ===================== what the frames carry =====================

/// A host pushing an image announces it with one of these.
pub const KIND_XFER_BEGIN: u8 = 0x20;
/// Then feeds it with these.
pub const KIND_XFER_DATA: u8 = 0x21;
/// And may give up with this.
pub const KIND_XFER_ABORT: u8 = 0x22;
/// The server says how it is going with these.
pub const KIND_XFER_REPORT: u8 = 0x82;

/// Payload length of a [`KIND_XFER_BEGIN`].
pub const BEGIN_LEN: usize = 11;
/// Bytes of offset in front of a [`KIND_XFER_DATA`] payload.
pub const DATA_HEADER_LEN: usize = 4;
/// Payload length of a [`KIND_XFER_REPORT`].
pub const REPORT_LEN: usize = 22;

/// What a push is aimed at.
///
/// The CRC is the host's, not the server's: an image is streamed through a
/// buffer far smaller than itself, so the thing sending it on never sees all
/// of it at once, and the node at the far end checks against exactly this.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Target {
    /// Node address the image is for.
    pub node: u8,
    /// One of `xfer::kind`.
    pub kind: u8,
    /// Which slot of that kind.
    pub slot: u8,
    /// Image length in bytes.
    pub size: u32,
    /// CRC-32 of the whole image.
    pub crc: u32,
}

impl Target {
    /// Read one out of a [`KIND_XFER_BEGIN`] payload.
    pub fn decode(payload: &[u8]) -> Option<Target> {
        (payload.len() == BEGIN_LEN).then(|| Target {
            node: payload[0],
            kind: payload[1],
            slot: payload[2],
            size: u32::from_le_bytes([payload[3], payload[4], payload[5], payload[6]]),
            crc: u32::from_le_bytes([payload[7], payload[8], payload[9], payload[10]]),
        })
    }

    /// Pack into a [`KIND_XFER_BEGIN`] payload.
    pub fn encode(&self) -> [u8; BEGIN_LEN] {
        let mut out = [0u8; BEGIN_LEN];
        out[0] = self.node;
        out[1] = self.kind;
        out[2] = self.slot;
        out[3..7].copy_from_slice(&self.size.to_le_bytes());
        out[7..11].copy_from_slice(&self.crc.to_le_bytes());
        out
    }
}

/// Where a push has got to, as the server sees it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Report {
    /// 0 idle, 1 staging, 2 sending, 3 done, 4 failed.
    pub phase: u8,
    /// Why it failed; zero unless it did.
    pub reason: u8,
    /// What it is aimed at.
    pub target: Target,
    /// Image bytes the server has taken from the host so far.
    pub filled: u32,
    /// Percent of it the bus has confirmed.
    pub percent: u8,
    /// Bytes the server can take right now.
    ///
    /// The number a host paces to. An image is streamed through a buffer a
    /// fraction of its size, so a host that simply sends is refused for most
    /// of a transfer and spends the link discovering the limit by hitting it.
    pub free: u32,
    /// CRC-32 of what the server actually put on the bus.
    pub sent: u32,
}

/// A push that has finished, one way or the other.
pub const PHASE_DONE: u8 = 3;
/// A push that has failed.
pub const PHASE_FAILED: u8 = 4;

impl Report {
    /// Read one out of a [`KIND_XFER_REPORT`] payload.
    pub fn decode(payload: &[u8]) -> Option<Report> {
        if payload.len() != REPORT_LEN {
            return None;
        }
        Some(Report {
            phase: payload[0],
            reason: payload[1],
            target: Target {
                node: payload[2],
                kind: payload[3],
                slot: payload[4],
                size: u32::from_le_bytes([payload[9], payload[10], payload[11], payload[12]]),
                crc: 0,
            },
            filled: u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]),
            percent: payload[13],
            free: u32::from_le_bytes([payload[14], payload[15], payload[16], payload[17]]),
            sent: u32::from_le_bytes([payload[18], payload[19], payload[20], payload[21]]),
        })
    }

    /// True once the push has stopped, whichever way.
    pub const fn finished(&self) -> bool {
        matches!(self.phase, PHASE_DONE | PHASE_FAILED)
    }
}


/// The host asks for a [`KIND_STATUS`] frame with one of these.
pub const KIND_PING: u8 = 0x10;
/// And the server answers with this.
pub const KIND_STATUS: u8 = 0x80;

/// Payload length of a [`KIND_STATUS`].
pub const STATUS_LEN: usize = 24;

/// The CAN controller is up.
pub const FLAG_CAN_UP: u8 = 1 << 0;
/// The status panel is fitted and answering.
pub const FLAG_PANEL_UP: u8 = 1 << 1;
/// A host has been heard from recently.
pub const FLAG_HOST_UP: u8 = 1 << 2;
/// The controller is in loopback, which is a bench mode and not a bus.
pub const FLAG_LOOPBACK: u8 = 1 << 3;

/// What the server says about itself.
///
/// Sent on a timer and whenever a [`KIND_PING`] asks, so a tool that has just
/// connected need not wait a second to have something to show.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Status {
    /// Milliseconds since boot.
    pub uptime_ms: u32,
    /// Frames put on the CAN bus since boot.
    pub can_tx: u32,
    /// Frames read off the CAN bus since boot.
    pub can_rx: u32,
    /// Host frames accepted since boot.
    pub host_frames: u32,
    /// Host frames discarded on a checksum mismatch.
    ///
    /// Counted rather than hidden: this link shares a cable with a boot log
    /// and a reflash, so a few at startup are ordinary and a number that
    /// climbs is a link worth looking at.
    pub crc_errors: u32,
    /// CAN frames transmitted during the last whole second.
    pub can_tx_rate: u16,
    /// Gauges displays have subscribed to.
    pub subscribers: u8,
    /// `FLAG_*` bits.
    pub flags: u8,
}

impl Status {
    /// Pack into the [`STATUS_LEN`] payload of a [`KIND_STATUS`] frame.
    pub fn encode(&self) -> [u8; STATUS_LEN] {
        let mut out = [0u8; STATUS_LEN];
        out[0..4].copy_from_slice(&self.uptime_ms.to_le_bytes());
        out[4..8].copy_from_slice(&self.can_tx.to_le_bytes());
        out[8..12].copy_from_slice(&self.can_rx.to_le_bytes());
        out[12..16].copy_from_slice(&self.host_frames.to_le_bytes());
        out[16..20].copy_from_slice(&self.crc_errors.to_le_bytes());
        out[20..22].copy_from_slice(&self.can_tx_rate.to_le_bytes());
        out[22] = self.subscribers;
        out[23] = self.flags;
        out
    }

    /// Read one out of a [`KIND_STATUS`] payload.
    pub fn decode(payload: &[u8]) -> Option<Status> {
        if payload.len() != STATUS_LEN {
            return None;
        }
        let word = |at: usize| {
            u32::from_le_bytes([
                payload[at],
                payload[at + 1],
                payload[at + 2],
                payload[at + 3],
            ])
        };
        Some(Status {
            uptime_ms: word(0),
            can_tx: word(4),
            can_rx: word(8),
            host_frames: word(12),
            crc_errors: word(16),
            can_tx_rate: u16::from_le_bytes([payload[20], payload[21]]),
            subscribers: payload[22],
            flags: payload[23],
        })
    }

    /// True if the CAN controller is up.
    pub const fn can_up(&self) -> bool {
        self.flags & FLAG_CAN_UP != 0
    }

    /// True if the status panel is fitted and answering.
    pub const fn panel_up(&self) -> bool {
        self.flags & FLAG_PANEL_UP != 0
    }

    /// True if a host has been heard from recently.
    pub const fn host_up(&self) -> bool {
        self.flags & FLAG_HOST_UP != 0
    }

    /// True if the controller is in loopback, which is a bench mode: frames
    /// come straight back and nothing of them reaches a display.
    pub const fn loopback(&self) -> bool {
        self.flags & FLAG_LOOPBACK != 0
    }
}


/// The server's account of every gauge it currently holds a value for.
///
/// Sent as one or more parts. The payload is a [`Readings`] header byte
/// followed by whole [`Sample`] entries; a reader starts a fresh set at part
/// zero, appends the rest, and shows the set when the part marked last
/// arrives.
///
/// Parts because the smallest link this travels over is a Bluetooth
/// notification, which cannot exceed the agreed ATT payload -- about two
/// hundred and forty bytes. The gauge table is long past what that holds, and
/// a report too big to send was dropped whole: readings simply stopped once a
/// car had enough to say, which is precisely backwards.
pub const KIND_READINGS: u8 = 0x81;

/// A node's own drawing report, relayed by the server: `[node, fps, compose
/// us lo, compose us hi, px lo, px hi, rects]`. The bytes after the node are
/// exactly the [`crate::bus::Health`] argument bytes the node put on the CAN
/// bus, passed through rather than re-encoded: the server is a post office
/// here, not a translator.
pub const KIND_HEALTH: u8 = 0x83;

/// The header byte at the front of a [`KIND_READINGS`] payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Readings {
    /// Which part this is, counting from zero. Part zero replaces whatever
    /// the reader was holding.
    pub part: u8,
    /// Whether this is the last part of the set.
    pub last: bool,
}

/// Bytes of header before the first sample in a [`KIND_READINGS`] payload.
pub const READINGS_HEADER: usize = 1;

/// The top bit of the header byte marks the last part.
const READINGS_LAST: u8 = 0x80;

impl Readings {
    /// The header byte for this part.
    pub const fn encode(&self) -> u8 {
        (self.part & !READINGS_LAST) | if self.last { READINGS_LAST } else { 0 }
    }

    /// Read the header off the front of a payload, or `None` if there is none.
    ///
    /// An empty payload has no header and is not a part of anything: a server
    /// with nothing to report still sends a set, and that set is one part
    /// holding no gauges rather than no bytes at all.
    pub fn decode(payload: &[u8]) -> Option<Readings> {
        let byte = *payload.first()?;
        Some(Readings {
            part: byte & !READINGS_LAST,
            last: byte & READINGS_LAST != 0,
        })
    }
}

/// Bytes one gauge occupies in a [`KIND_READINGS`] payload.
///
/// The same six as a host's gauge write, and the same layout: an identifier
/// and a value. A reading going one way and a write going the other are the
/// same fact, so they are the same bytes.
pub const SAMPLE_LEN: usize = 6;

/// One gauge, as the server holds it.
///
/// `raw` is the stored fixed-point count, not a number of degrees or
/// kilopascals. That is deliberate and is the whole point of this message:
/// the count is what arrived, and the conversion is an opinion applied after.
/// A sensor reading nonsense reads as nonsense here, undivided and unclamped,
/// which is what makes it possible to tell a broken sender from a scale that
/// is wrong.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    /// The gauge's identifier on the bus.
    pub id: u16,
    /// The stored count, exactly as the server has it.
    pub raw: i32,
}

impl Sample {
    /// Pack into a [`SAMPLE_LEN`] entry.
    pub fn encode(&self) -> [u8; SAMPLE_LEN] {
        let mut out = [0u8; SAMPLE_LEN];
        out[0..2].copy_from_slice(&self.id.to_le_bytes());
        out[2..6].copy_from_slice(&self.raw.to_le_bytes());
        out
    }

    /// Read one out of an entry.
    pub fn decode(entry: &[u8]) -> Option<Sample> {
        (entry.len() == SAMPLE_LEN).then(|| Sample {
            id: u16::from_le_bytes([entry[0], entry[1]]),
            raw: i32::from_le_bytes([entry[2], entry[3], entry[4], entry[5]]),
        })
    }
}

/// Every gauge in a [`KIND_READINGS`] payload, header skipped.
///
/// A trailing part-entry is ignored rather than guessed at: a frame that has
/// been cut short is not a frame with one bad gauge in it, and inventing a
/// value for a sensor is exactly the mistake this message exists to avoid.
pub fn samples(payload: &[u8]) -> impl Iterator<Item = Sample> + '_ {
    payload
        .get(READINGS_HEADER..)
        .unwrap_or(&[])
        .as_chunks::<SAMPLE_LEN>()
        .0
        .iter()
        .filter_map(|entry| Sample::decode(entry))
}

/// How many gauges one part can carry.
///
/// Sized to the smallest link this travels over rather than the largest. A
/// Bluetooth notification cannot exceed the agreed ATT payload -- around two
/// hundred and forty bytes -- and a report past that is not truncated but
/// dropped whole, so a car with plenty to say went silent.
///
/// Every transport sends the same modest parts rather than each sizing to its
/// own limit. Simpler, and it means what is exercised on a cable is what goes
/// over a radio, instead of the radio taking a path nothing else does.
pub const SAMPLES_PER_FRAME: usize = 32;

/// The whole of one part: header, samples, and the frame around them.
pub const READINGS_FRAME: usize =
    OVERHEAD + READINGS_HEADER + SAMPLES_PER_FRAME * SAMPLE_LEN;

/// How many gauges a whole set can carry, across every part.
///
/// The part index is seven bits, the eighth marking the last part.
pub const SAMPLES_PER_SET: usize = 128 * SAMPLES_PER_FRAME;

// A part has to fit a Bluetooth notification, which is the tightest link it
// travels over: three bytes of ATT header inside the agreed payload, and the
// largest payload the radio will agree to is 247. A part past that is not
// truncated but dropped, so readings would stop over the radio and nowhere
// else -- silently, and only on a car with enough to say. Checked here rather
// than in a test because it is knowable without running anything.
const _: () = assert!(
    READINGS_FRAME + 3 <= 247,
    "a readings part will not fit a Bluetooth notification"
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sample_survives_the_trip_through_its_entry() {
        // A negative count, because a gauge below its zero is a real reading
        // and the sign is the difference between cold and broken.
        let s = Sample { id: 0x025, raw: -400 };
        assert_eq!(Sample::decode(&s.encode()), Some(s));
    }

    #[test]
    fn a_payload_yields_every_gauge_in_it() {
        let want = [
            Sample { id: 0x024, raw: 1013 },
            Sample { id: 0x025, raw: -400 },
            Sample { id: 0x030, raw: 6500 },
        ];
        let mut payload = vec![Readings { part: 0, last: true }.encode()];
        for s in want {
            payload.extend_from_slice(&s.encode());
        }
        let got: Vec<_> = samples(&payload).collect();
        assert_eq!(got, want);
    }

    #[test]
    fn a_frame_cut_short_drops_the_part_entry_rather_than_guessing() {
        // Inventing a value for a sensor is the one mistake this message
        // exists to avoid.
        let whole = Sample { id: 0x024, raw: 1013 }.encode();
        let mut payload = vec![Readings { part: 0, last: true }.encode()];
        payload.extend_from_slice(&whole);
        payload.extend_from_slice(&whole[..3]);
        let got: Vec<_> = samples(&payload).collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].id, 0x024);
    }

    #[test]
    fn an_empty_payload_is_no_gauges_rather_than_an_error() {
        // A payload of nothing but a header is what a server with nothing set
        // sends, and it is a real answer: it says the car is quiet, not that
        // the link is.
        assert_eq!(samples(&[Readings { part: 0, last: true }.encode()]).count(), 0);
        // And a payload of nothing at all still yields nothing rather than
        // reading past its end.
        assert_eq!(samples(&[]).count(), 0);
    }

    #[test]
    fn every_gauge_this_build_knows_fits_one_set() {
        // A set is as many parts as it takes; what it cannot outgrow is the
        // seven bits the part index has. Finding that out on a bench beats
        // finding it out in a car.
        assert!(
            crate::ALL_GAUGES.len() <= SAMPLES_PER_SET,
            "{} gauges will not fit {SAMPLES_PER_SET} in a set",
            crate::ALL_GAUGES.len()
        );
    }

    #[test]
    fn a_set_reassembles_from_its_parts() {
        let first = Readings { part: 0, last: false };
        let last = Readings { part: 1, last: true };
        assert_eq!(Readings::decode(&[first.encode()]), Some(first));
        assert_eq!(Readings::decode(&[last.encode()]), Some(last));
        // An empty payload is no part at all, which is different from a part
        // carrying no gauges.
        assert_eq!(Readings::decode(&[]), None);
    }

    #[test]
    fn the_header_is_not_read_as_a_gauge() {
        let mut payload = vec![Readings { part: 0, last: true }.encode()];
        payload.extend_from_slice(&Sample { id: 0x24, raw: 1013 }.encode());
        let got: Vec<Sample> = samples(&payload).collect();
        assert_eq!(got, vec![Sample { id: 0x24, raw: 1013 }]);
    }


    #[test]
    fn a_status_survives_the_trip_through_its_payload() {
        let s = Status {
            uptime_ms: 123_456,
            can_tx: 896_744,
            can_rx: 27_000,
            host_frames: 29_616,
            crc_errors: 0,
            can_tx_rate: 2,
            subscribers: 9,
            flags: FLAG_CAN_UP | FLAG_PANEL_UP,
        };
        assert_eq!(Status::decode(&s.encode()), Some(s));
        assert!(s.can_up() && s.panel_up());
        assert!(!s.host_up() && !s.loopback());
    }

    #[test]
    fn a_status_of_the_wrong_length_is_refused() {
        let s = Status::default();
        let good = s.encode();
        assert_eq!(Status::decode(&good[..STATUS_LEN - 1]), None);
    }


    #[test]
    fn a_target_survives_the_trip_through_a_begin_payload() {
        let t = Target { node: 2, kind: 1, slot: 0, size: 862_832, crc: 0x99C4_433F };
        assert_eq!(Target::decode(&t.encode()), Some(t));
    }

    #[test]
    fn a_begin_payload_of_the_wrong_length_is_refused() {
        // A host one version out would otherwise have its image sent to
        // whatever node its bytes happened to line up as.
        let t = Target::default();
        let good = t.encode();
        assert_eq!(Target::decode(&good[..BEGIN_LEN - 1]), None);
    }

    #[test]
    fn a_report_is_read_from_the_layout_the_server_writes() {
        let mut payload = [0u8; REPORT_LEN];
        payload[0] = PHASE_DONE;
        payload[2] = 0x02;
        payload[5..9].copy_from_slice(&1234u32.to_le_bytes());
        payload[9..13].copy_from_slice(&4321u32.to_le_bytes());
        payload[13] = 55;
        payload[14..18].copy_from_slice(&999u32.to_le_bytes());
        payload[18..22].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
        let r = Report::decode(&payload).expect("a full report must decode");
        assert_eq!(r.target.node, 0x02);
        assert_eq!(r.filled, 1234);
        assert_eq!(r.target.size, 4321);
        assert_eq!(r.percent, 55);
        assert_eq!(r.free, 999);
        assert_eq!(r.sent, 0xDEAD_BEEF);
        assert!(r.finished());
    }

    #[test]
    fn a_report_still_running_is_not_taken_as_finished() {
        let mut payload = [0u8; REPORT_LEN];
        payload[0] = 2;
        let r = Report::decode(&payload).expect("a report must decode");
        assert!(!r.finished());
    }

    #[test]
    fn a_truncated_report_is_refused_rather_than_read_past() {
        assert_eq!(Report::decode(&[0u8; REPORT_LEN - 1]), None);
    }


    fn feed(dec: &mut Decoder, bytes: &[u8]) -> Option<(u8, usize)> {
        let mut last = None;
        for &b in bytes {
            if let Some(hit) = dec.push(b) {
                last = Some(hit);
            }
        }
        last
    }

    #[test]
    fn empty_payload_round_trips() {
        let mut out = [0u8; MAX_FRAME];
        let n = encode(0x10, &[], &mut out).unwrap();
        assert_eq!(n, OVERHEAD);
        let mut dec = Decoder::new();
        assert_eq!(feed(&mut dec, &out[..n]), Some((0x10, 0)));
        assert_eq!(dec.payload(), &[] as &[u8]);
    }

    #[test]
    fn full_payload_round_trips() {
        let payload: [u8; MAX_PAYLOAD] = core::array::from_fn(|i| i as u8);
        let mut out = [0u8; MAX_FRAME];
        let n = encode(0x01, &payload, &mut out).unwrap();
        assert_eq!(n, MAX_FRAME);
        let mut dec = Decoder::new();
        assert_eq!(feed(&mut dec, &out[..n]), Some((0x01, MAX_PAYLOAD)));
        assert_eq!(dec.payload(), &payload[..]);
    }

    #[test]
    fn oversize_payload_is_refused() {
        let mut out = [0u8; MAX_FRAME + 1];
        assert!(encode(0x01, &[0u8; MAX_PAYLOAD + 1], &mut out).is_none());
    }

    #[test]
    fn a_length_past_the_buffer_is_abandoned_not_taken() {
        let mut dec = Decoder::new();
        // Noise that happens to carry the sync pattern and a huge length.
        feed(&mut dec, &[SYNC0, SYNC1, 0x01, 0xFF, 0xFF]);
        // The decoder must be hunting again, so a real frame still lands.
        let mut out = [0u8; MAX_FRAME];
        let n = encode(0x02, &[4, 5], &mut out).unwrap();
        assert_eq!(feed(&mut dec, &out[..n]), Some((0x02, 2)));
    }

    #[test]
    fn a_settings_blob_fits_in_one_frame() {
        // The reason the length field is two bytes at all.
        assert!(crate::config::blob_len() <= MAX_PAYLOAD);
    }

    #[test]
    fn short_output_buffer_is_refused() {
        let mut out = [0u8; OVERHEAD - 1];
        assert!(encode(0x01, &[], &mut out).is_none());
    }

    #[test]
    fn leading_garbage_is_skipped() {
        let mut out = [0u8; MAX_FRAME];
        let n = encode(0x02, &[1, 2, 3], &mut out).unwrap();
        let mut dec = Decoder::new();
        feed(&mut dec, b"ets Jun  8 2016 00:22:57\r\n");
        assert_eq!(feed(&mut dec, &out[..n]), Some((0x02, 3)));
        assert_eq!(dec.payload(), &[1, 2, 3]);
    }

    #[test]
    fn repeated_first_sync_byte_still_frames() {
        let mut out = [0u8; MAX_FRAME];
        let n = encode(0x03, &[9], &mut out).unwrap();
        let mut dec = Decoder::new();
        // The extra 0xAA stands in for a truncated frame's sync pattern.
        assert_eq!(dec.push(SYNC0), None);
        assert_eq!(feed(&mut dec, &out[..n]), Some((0x03, 1)));
        assert_eq!(dec.payload(), &[9]);
    }

    #[test]
    fn corrupt_payload_counts_and_does_not_swallow_the_next_frame() {
        let mut good = [0u8; MAX_FRAME];
        let n = encode(0x01, &[7, 7, 7], &mut good).unwrap();
        let mut bad = good;
        bad[6] ^= 0xFF;

        let mut dec = Decoder::new();
        assert_eq!(feed(&mut dec, &bad[..n]), None);
        assert_eq!(dec.crc_errors(), 1);
        assert_eq!(feed(&mut dec, &good[..n]), Some((0x01, 3)));
        assert_eq!(dec.crc_errors(), 1);
    }

    #[test]
    fn reset_abandons_a_partial_frame() {
        let mut out = [0u8; MAX_FRAME];
        let n = encode(0x01, &[1, 2, 3], &mut out).unwrap();
        let mut dec = Decoder::new();
        feed(&mut dec, &out[..5]);
        dec.reset();
        assert_eq!(feed(&mut dec, &out[..n]), Some((0x01, 3)));
    }

    #[test]
    fn frames_match_the_bytes_the_reference_host_produces() {
        // Golden vectors taken from host/simulate.py. The two encoders are
        // written independently, so this is what stops them drifting apart.
        let mut out = [0u8; MAX_FRAME];

        let n = encode(0x10, &[], &mut out).unwrap();
        assert_eq!(&out[..n], &[0xAA, 0x55, 0x10, 0x00, 0x00, 0xFF, 0x8F]);

        let gauges = [
            0x2D, 0x00, 0xAC, 0x0D, 0x00, 0x00, 0x26, 0x00, 0x66, 0x03, 0x00, 0x00,
        ];
        let n = encode(0x01, &gauges, &mut out).unwrap();
        assert_eq!(
            &out[..n],
            &[
                0xAA, 0x55, 0x01, 0x0C, 0x00, 0x2D, 0x00, 0xAC, 0x0D, 0x00, 0x00, 0x26, 0x00, 0x66,
                0x03, 0x00, 0x00, 0xDE, 0x76,
            ]
        );
    }

    #[test]
    fn crc_matches_the_published_check_value() {
        // CRC-16/CCITT-FALSE is defined by its check value over "123456789".
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }
}

#[cfg(test)]
mod from_the_wire {
    use super::*;

    /// Twenty-nine bytes captured off a Bluetooth link: a report the server
    /// sent while a firmware push was running.
    const CAPTURED: [u8; 29] = [
        0xaa, 0x55, 0x82, 0x16, 0x00, 0x02, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70,
        0x2a, 0x0d, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x18,
    ];

    #[test]
    fn a_report_captured_off_a_radio_decodes() {
        let mut d = Decoder::new();
        let mut got = None;
        for byte in CAPTURED {
            if let Some(found) = d.push(byte) {
                got = Some(found);
            }
        }
        let (kind, len) = got.expect("the frame must decode");
        assert_eq!(kind, KIND_XFER_REPORT);
        assert_eq!(len, REPORT_LEN);
        let report = Report::decode(&d.payload()[..len]).expect("and must read as a report");
        assert_eq!(report.target.size, 862_832);
        assert_eq!(report.free, 8192);
        assert_eq!(report.phase, 2);
    }
}
