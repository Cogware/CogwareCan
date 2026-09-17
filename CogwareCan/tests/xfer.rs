// SPDX-License-Identifier: GPL-3.0-only
//! Master-to-node file transfer, driven frame by frame with a lossy "bus".

use cogware_can::protocol::NODE_BROADCAST;
use cogware_can::xfer::*;
use embedded_can::Frame;
use cogware_can::{crc, protocol};
use cogware_can::CanFrame;

fn image(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 7 % 251) as u8).collect()
}

/// Run a session to completion. `drop` decides which master frames (by
/// count) never reach the nodes. Returns the number of frames the master sent.
fn run(tx: &mut Sender<&[u8]>, rxs: &mut [Receiver<BufferSink>], drop: &dyn Fn(usize) -> bool) -> usize {
    run_watching(tx, rxs, drop, &mut |_| {})
}

/// As `run`, but `watch` is called with the sender after every frame, so a
/// test can assert on what it reported as it went rather than only at the end.
fn run_watching(
    tx: &mut Sender<&[u8]>,
    rxs: &mut [Receiver<BufferSink>],
    drop: &dyn Fn(usize) -> bool,
    watch: &mut dyn FnMut(&Sender<&[u8]>),
) -> usize {
    let mut sent = 0;
    let mut idle = 0;
    loop {
        watch(tx);
        match tx.state() {
            TxState::Done | TxState::Failed(_) => return sent,
            _ => {}
        }
        let f: Option<CanFrame> = tx.next_frame().or_else(|| {
            idle += 1;
            if idle > 3 { tx.resend() } else { None }
        });
        let Some(f) = f else {
            assert!(idle < 20, "sender stalled in {:?}", tx.state());
            continue;
        };
        idle = 0;
        sent += 1;
        if drop(sent) {
            continue;
        }
        for rx in rxs.iter_mut() {
            if let Some(reply) = rx.feed(&f) {
                tx.feed_reply(&reply);
            }
        }
    }
}

#[test]
fn clean_transfer_lands_and_commits() {
    let img = image(100);
    let mut buf = [0u8; 128];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], 1, 8, 1);
    let sent = run(&mut tx, std::slice::from_mut(&mut rx), &|_| false);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(rx.state(), RxState::Idle);
    assert!(rx.sink().committed);
    assert_eq!(rx.sink().len, 100);
    assert_eq!(&buf[..100], &img[..]);
    // 20 data blocks + begin + 3 checks + end + commit
    assert_eq!(sent, 20 + 1 + 3 + 1 + 1);
}

#[test]
fn lost_block_is_resent_from_the_gap() {
    let img = image(60);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], 1, 4, 1);
    // frame 1 = begin, 2..5 = blocks 0..3; drop block 1 (frame 3)
    run(&mut tx, std::slice::from_mut(&mut rx), &|n| n == 3);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(&buf[..60], &img[..]);
}

#[test]
fn lost_last_block_of_window_is_detected_by_check() {
    let img = image(60);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], 1, 4, 1);
    // drop block 3 (frame 5), the last one before the first check
    run(&mut tx, std::slice::from_mut(&mut rx), &|n| n == 5);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(&buf[..60], &img[..]);
}

#[test]
fn rewind_across_final_partial_block_is_exact() {
    let img = image(64); // 12 full blocks + 4 bytes
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], 1, 100, 1);
    // begin=1, blocks at 2..=12; drop block 9 (frame 11) so the rewind spans the partial block 10
    run(&mut tx, std::slice::from_mut(&mut rx), &|n| n == 11);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(&buf[..64], &img[..]);
}

#[test]
fn a_block_spoilt_in_flight_is_caught_and_sent_again() {
    // What the checksum in each block is for. The bus signs a frame on its way
    // out of the sending controller, so a byte spoilt before that is signed as
    // good; without a check of our own it lands, and the whole image fails at
    // the end for one bad block.
    let img = image(30);
    let mut buf = [0u8; 32];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], kind::FIRMWARE, 100, 1);

    let mut spoil_once = true;
    let mut guard = 0;
    while !matches!(tx.state(), TxState::Done | TxState::Failed(_)) {
        guard += 1;
        assert!(guard < 1000, "stuck in {:?}", tx.state());
        let Some(f) = tx.next_frame() else { continue };
        let f = if spoil_once && protocol::id_of(&f) == Some(DATA_ID) {
            use embedded_can::Frame;
            spoil_once = false;
            let mut d = f.data()[..f.dlc()].to_vec();
            d[3] ^= 0xFF;
            protocol::frame(DATA_ID, &d).unwrap()
        } else {
            f
        };
        if let Some(r) = rx.feed(&f) {
            tx.feed_reply(&r);
        }
    }

    assert_eq!(tx.state(), TxState::Done, "one spoilt block should not lose the image");
    assert_eq!(rx.spoilt(), 1, "the spoilt block was not noticed");
    assert!(rx.sink().committed);
    assert_eq!(&buf[..30], &img[..], "the image is wrong after a resend");
}

#[test]
fn a_link_that_spoils_every_block_is_given_up_on_rather_than_retried_for_ever() {
    // A block that is refused is sent again, which is right for a bus that
    // drops one now and then and wrong for one that is broken: without a limit
    // the two ends pass the same block back and forth for ever.
    let img = image(30);
    let mut buf = [0u8; 32];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], kind::FIRMWARE, 8, 1);

    let mut guard = 0;
    while !matches!(tx.state(), TxState::Done | TxState::Failed(_)) {
        guard += 1;
        assert!(guard < 100_000, "the sender never gave up");
        let Some(f) = tx.next_frame() else { continue };
        let f = if protocol::id_of(&f) == Some(DATA_ID) {
            use embedded_can::Frame;
            let mut d = f.data()[..f.dlc()].to_vec();
            d[3] ^= 0xFF;
            protocol::frame(DATA_ID, &d).unwrap()
        } else {
            f
        };
        if let Some(r) = rx.feed(&f) {
            tx.feed_reply(&r);
        }
    }

    assert!(matches!(tx.state(), TxState::Failed(_)), "a broken link must end the push");
    assert!(!rx.sink().committed, "nothing may be committed from a broken link");
}

#[test]
fn node_refuses_oversized_image() {
    let img = image(200);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], 1, 8, 1);
    run(&mut tx, std::slice::from_mut(&mut rx), &|_| false);
    assert_eq!(tx.state(), TxState::Failed(Status::Refused));
    assert_eq!(rx.state(), RxState::Idle);
}

#[test]
fn other_nodes_ignore_a_session_not_addressed_to_them() {
    let img = image(20);
    let mut b7 = [0u8; 32];
    let mut b9 = [0u8; 32];
    let mut rxs = [Receiver::new(7, BufferSink::new(&mut b7)), Receiver::new(9, BufferSink::new(&mut b9))];
    let mut tx = Sender::new(7, &img[..], 1, 8, 1);
    run(&mut tx, &mut rxs, &|_| false);
    assert_eq!(tx.state(), TxState::Done);
    assert!(rxs[0].sink().committed);
    assert!(!rxs[1].sink().committed);
    assert_eq!(rxs[1].sink().len, 0);
}

#[test]
fn broadcast_updates_every_node_and_rewinds_for_the_slowest() {
    let img = image(90);
    let mut b7 = [0u8; 96];
    let mut b9 = [0u8; 96];
    let mut rxs = [Receiver::new(7, BufferSink::new(&mut b7)), Receiver::new(9, BufferSink::new(&mut b9))];
    let mut tx = Sender::new(NODE_BROADCAST, &img[..], 1, 5, 2);
    // Drop block 2 for everyone (frame 4); both nodes report it missing.
    run(&mut tx, &mut rxs, &|n| n == 4);
    assert_eq!(tx.state(), TxState::Done);
    assert!(rxs.iter_mut().all(|r| r.sink().committed));
    assert_eq!(&b7[..90], &img[..]);
    assert_eq!(&b9[..90], &img[..]);
}

#[test]
fn abort_resets_the_node() {
    let img = image(40);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img[..], 1, 100, 1);
    let begin = tx.next_frame().unwrap();
    let ready = rx.feed(&begin).unwrap();
    tx.feed_reply(&ready);
    assert_eq!(rx.state(), RxState::Receiving);
    let abort = tx.abort().unwrap();
    let reply = rx.feed(&abort).unwrap();
    assert_eq!(rx.state(), RxState::Idle);
    tx.feed_reply(&reply);
    assert_eq!(tx.state(), TxState::Failed(Status::Aborted));
}

#[test]
fn crc_matches_reference() {
    assert_eq!(crc::crc32(b"123456789"), 0xCBF4_3926);
    let mut c = crc::Crc32::new();
    c.update(b"1234");
    c.update(b"56789");
    assert_eq!(c.finish(), 0xCBF4_3926);
}

#[test]
fn the_slot_and_kind_reach_the_sink() {
    let image = [0x5Au8; 40];
    let mut store = [0u8; 64];
    let mut tx = Sender::new(2, &image[..], kind::SCENE, 8, 1).with_slot(3);
    let mut rx = Receiver::new(2, BufferSink::new(&mut store));

    for _ in 0..64 {
        if let Some(f) = tx.next_frame() {
            if let Some(reply) = rx.feed(&f) {
                tx.feed_reply(&reply);
            }
        }
        if tx.state() == TxState::Done {
            break;
        }
    }
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(rx.image(), Image { size: 40, kind: kind::SCENE, slot: 3 });
    assert!(rx.sink().committed);
    assert_eq!(&rx.sink().buf[..40], &image[..]);
}

#[test]
fn a_begin_without_a_slot_byte_lands_in_slot_zero() {
    // Seven-byte Begin frames predate slots; a node must still accept one
    // rather than refusing the transfer outright.
    let mut store = [0u8; 16];
    let mut rx = Receiver::new(1, BufferSink::new(&mut store));
    let begin = protocol::frame(CMD_ID, &[1, Cmd::Begin as u8, 8, 0, 0, 0, kind::FIRMWARE]).unwrap();

    let reply = rx.feed(&begin).expect("the node answers");
    assert_eq!(reply.data()[1], Status::Ready as u8);
    assert_eq!(rx.image(), Image { size: 8, kind: kind::FIRMWARE, slot: 0 });
}

#[test]
fn reported_progress_only_ever_climbs() {
    // A block lost on the bus rewinds the sender, so bytes *sent* goes
    // backwards. A bar driven by that jumps about and reads as a fault, which
    // is the opposite of what it is for on a screen that says not to power off.
    let image = image(2000);
    let mut buf = [0u8; 2048];
    let mut rxs = [Receiver::new(3, BufferSink::new(&mut buf))];
    let mut tx = Sender::new(3, &image[..], kind::SCENE, 8, 1);

    let mut highest = 0u32;
    let mut seen_full = false;
    run_watching(
        &mut tx,
        &mut rxs,
        // Enough losses, spread out, to force several rewinds.
        &|n| n % 11 == 0 || n % 17 == 0,
        &mut |tx| {
            let (done, total) = tx.progress();
            assert_eq!(total, image.len() as u32);
            assert!(done >= highest, "progress fell from {highest} to {done}");
            assert!(done <= total, "progress {done} past the image at {total}");
            highest = done;
            seen_full |= done == total;
        },
    );

    assert_eq!(tx.state(), TxState::Done);
    assert!(seen_full, "a finished transfer never reported the whole image");
    assert_eq!(tx.percent(), 100);
}

#[test]
fn nothing_is_reported_delivered_before_a_node_has_confirmed_it() {
    // The first blocks are on the wire but unacknowledged, and a bar that
    // counted them would be ahead of what the node actually holds.
    let image = image(600);
    let mut buf = [0u8; 1024];
    let mut rx = Receiver::new(3, BufferSink::new(&mut buf));
    let mut tx = Sender::new(3, &image[..], kind::SCENE, 64, 1);

    // Begin, its reply, then a handful of data blocks and no checkpoint yet.
    let begin = tx.next_frame().expect("begin");
    if let Some(reply) = rx.feed(&begin) {
        tx.feed_reply(&reply);
    }
    for _ in 0..5 {
        if let Some(f) = tx.next_frame() {
            rx.feed(&f);
        }
    }
    assert_eq!(tx.progress(), (0, image.len() as u32));
    assert_eq!(tx.percent(), 0);
}

#[test]
fn a_node_counts_the_blocks_that_arrived_out_of_turn() {
    let image = image(600);
    let mut clean = [0u8; 1024];
    let mut lossy = [0u8; 1024];

    let mut rx = [Receiver::new(3, BufferSink::new(&mut clean))];
    run(&mut Sender::new(3, &image[..], kind::SCENE, 8, 1), &mut rx, &|_| false);
    assert_eq!(rx[0].gaps(), 0, "a bus that lost nothing has no gaps");

    // Frame 1 is the `Begin`, so frame 4 is the third data block and sits well
    // inside the first window: the block after it arrives out of turn, which
    // is what a gap is. A block lost at the *end* of a window is found by the
    // checkpoint instead and leaves no gap behind it.
    let mut rx = [Receiver::new(3, BufferSink::new(&mut lossy))];
    run(&mut Sender::new(3, &image[..], kind::SCENE, 8, 1), &mut rx, &|n| n == 4);
    assert_eq!(rx[0].gaps(), 1, "the block after the lost one arrived out of turn");
}

#[test]
fn progress_is_reported_as_the_transfer_goes_not_only_at_the_end() {
    // A bar that sits at nothing and then jumps to done is no better than no
    // bar, and is what a screen saying "do not power off" must not show.
    let image = image(600);
    let mut buf = [0u8; 1024];
    let mut rxs = [Receiver::new(3, BufferSink::new(&mut buf))];
    let mut tx = Sender::new(3, &image[..], kind::SCENE, 8, 1);

    let mut seen: Vec<u32> = Vec::new();
    run_watching(&mut tx, &mut rxs, &|_| false, &mut |tx| {
        let (done, _) = tx.progress();
        if seen.last() != Some(&done) {
            seen.push(done);
        }
    });

    assert_eq!(tx.state(), TxState::Done);
    let middling = seen.iter().filter(|d| **d > 0 && **d < image.len() as u32).count();
    assert!(
        middling >= 3,
        "progress went {seen:?}; a whole transfer reported nothing in between"
    );
}

/// A source holding only what has arrived so far, as a master streaming an
/// image far larger than its memory would hold only a window of one.
struct Trickle<'a> {
    image: &'a [u8],
    /// Bytes that have arrived from the host. Past this there is nothing yet.
    filled: u32,
}

impl Source for Trickle<'_> {
    fn len(&self) -> u32 {
        self.image.len() as u32
    }

    fn crc(&self) -> u32 {
        crc::crc32(self.image)
    }

    fn read(&mut self, offset: u32, out: &mut [u8]) -> usize {
        let end = offset as usize + out.len();
        if end > self.filled as usize {
            return 0;
        }
        out.copy_from_slice(&self.image[offset as usize..end]);
        out.len()
    }
}

#[test]
fn a_sender_waits_for_a_source_that_has_not_caught_up() {
    // The firmware case: the image is arriving down one link while it goes out
    // on another, and the far end must not be able to tell the difference.
    let image = image(400);
    let mut buf = [0u8; 512];
    let mut rxs = [Receiver::new(4, BufferSink::new(&mut buf))];
    let mut tx = Sender::new(
        4,
        Trickle { image: &image, filled: 0 },
        kind::FIRMWARE,
        8,
        1,
    );

    let mut stalls = 0;
    let mut guard = 0;
    while !matches!(tx.state(), TxState::Done | TxState::Failed(_)) {
        guard += 1;
        assert!(guard < 10_000, "stuck in {:?}", tx.state());
        if let Some(f) = tx.next_frame() {
            for rx in rxs.iter_mut() {
                if let Some(reply) = rx.feed(&f) {
                    tx.feed_reply(&reply);
                }
            }
            continue;
        }
        // Nothing to send. Either the source is behind, which is this test, or
        // a checkpoint is waiting, which cannot happen with no losses.
        if tx.state() == TxState::Data {
            stalls += 1;
            let src = tx.source_mut();
            src.filled = (src.filled + 37).min(image.len() as u32);
        }
    }

    assert_eq!(tx.state(), TxState::Done);
    assert!(stalls > 5, "the source never actually held the sender up");
    let sink = rxs[0].sink();
    assert!(sink.committed, "a streamed image still has to commit");
    assert_eq!(&sink.buf[..image.len()], &image[..], "the image arrived wrong");
}

#[test]
fn a_streamed_image_is_declared_with_the_size_and_crc_the_source_gives() {
    // The sender never sees the whole image, so it cannot work either out: the
    // `Begin` it sends and the `End` it verifies both come from the source.
    let image = image(250);
    let src = Trickle { image: &image, filled: 0 };
    let tx = Sender::new(6, src, kind::FIRMWARE, 8, 1);
    assert_eq!(
        tx.image(),
        Image { size: 250, kind: kind::FIRMWARE, slot: 0 }
    );
}

#[test]
fn an_image_long_enough_to_wrap_the_sequence_number_arrives_intact() {
    // Block numbers are sixteen bits and a block is six bytes, so anything
    // past 393_216 bytes reuses them. A firmware image is twice that and more,
    // and every rewind past the wrap is arithmetic on numbers that have gone
    // round -- which either works or quietly delivers the wrong bytes.
    let image = image(420_000);
    let mut buf = vec![0u8; 420_000];
    let mut rxs = [Receiver::new(9, BufferSink::new(&mut buf))];
    let mut tx = Sender::new(9, &image[..], kind::FIRMWARE, 32, 1);

    // Losses throughout, so rewinds land on both sides of the wrap.
    run(&mut tx, &mut rxs, &|n| n % 997 == 0);

    assert_eq!(tx.state(), TxState::Done, "the transfer did not finish");
    let sink = rxs[0].sink();
    assert!(sink.committed, "the far end refused to commit");
    assert_eq!(sink.buf[..], image[..], "the image arrived wrong");
}
