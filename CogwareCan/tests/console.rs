// SPDX-License-Identifier: GPL-3.0-only
//! A node's console: kept in a tail, carried in node messages, pulled again.

use cogware_can::bus::{
    console_pull, console_pull_target, BusView, NodeMsg, Opcode, CONSOLE_PER_MSG,
};
use cogware_can::console::Tail;
use cogware_can::protocol::{self, NODE_BROADCAST};

fn drain<const N: usize>(tail: &mut Tail<N>, step: usize) -> Vec<u8> {
    let mut all = Vec::new();
    let mut piece = vec![0u8; step];
    loop {
        let n = tail.peek(&mut piece);
        if n == 0 {
            return all;
        }
        all.extend_from_slice(&piece[..n]);
        tail.consume(n);
    }
}

#[test]
fn what_is_written_is_read_once_and_in_order() {
    let mut tail = Tail::<16>::new();
    tail.write(b"boot ok\n");
    assert_eq!(tail.pending(), 8);
    assert_eq!(drain(&mut tail, 3), b"boot ok\n");
    assert_eq!(tail.pending(), 0);
    tail.write(b"more");
    assert_eq!(drain(&mut tail, 3), b"more");
}

#[test]
fn a_peek_that_was_not_consumed_is_offered_again() {
    // A frame the controller refused must not cost the text it carried.
    let mut tail = Tail::<16>::new();
    tail.write(b"abcdef");
    let mut piece = [0u8; 4];
    assert_eq!(tail.peek(&mut piece), 4);
    assert_eq!(tail.peek(&mut piece), 4);
    assert_eq!(&piece, b"abcd");
    tail.consume(4);
    assert_eq!(drain(&mut tail, 4), b"ef");
}

#[test]
fn a_reader_left_behind_resumes_at_the_oldest_byte_kept() {
    let mut tail = Tail::<8>::new();
    tail.write(b"0123456789abcdef");
    assert_eq!(tail.pending(), 8);
    assert_eq!(drain(&mut tail, 5), b"89abcdef");
}

#[test]
fn a_reader_overtaken_mid_read_does_not_see_bytes_twice() {
    let mut tail = Tail::<8>::new();
    tail.write(b"0123");
    let mut piece = [0u8; 2];
    assert_eq!(tail.peek(&mut piece), 2);
    tail.write(b"456789abcdef");
    // What was peeked is gone from the tail, so consuming it must not skip
    // two bytes of what replaced it.
    tail.consume(2);
    assert_eq!(drain(&mut tail, 8), b"89abcdef");
}

#[test]
fn rewinding_replays_everything_still_held() {
    let mut tail = Tail::<8>::new();
    tail.write(b"hello");
    assert_eq!(drain(&mut tail, 8), b"hello");
    tail.rewind();
    assert_eq!(drain(&mut tail, 8), b"hello");
    tail.write(b" world");
    tail.rewind();
    assert_eq!(drain(&mut tail, 8), b"lo world");
}

#[test]
fn console_text_survives_the_bus_unchanged() {
    let display = BusView::new(1);
    let frame = display
        .report_console(b"MCP up\n")
        .expect("seven bytes fit");
    assert_eq!(protocol::id_of(&frame), Some(1));
    let heard = NodeMsg::parse(&frame).expect("a node message");
    assert_eq!(heard.opcode, Opcode::Console);
    assert_eq!(heard.console(), Some(&b"MCP up\n"[..]));
    assert!(display
        .report_console(&[b'x'; CONSOLE_PER_MSG + 1])
        .is_none());
    // Any other message carries no console.
    assert_eq!(
        NodeMsg::parse(&display.hello().unwrap()).unwrap().console(),
        None
    );
}

#[test]
fn a_pull_reaches_the_node_named_and_no_other() {
    let (cluster, gauge) = (BusView::new(1), BusView::new(2));
    let pull = console_pull(1).unwrap();
    assert_eq!(console_pull_target(&pull), Some(1));
    assert!(cluster.console_pulled(&pull));
    assert!(!gauge.console_pulled(&pull));

    let all = console_pull(NODE_BROADCAST).unwrap();
    assert!(cluster.console_pulled(&all) && gauge.console_pulled(&all));

    // A node's own message is not a pull, whatever its first byte says.
    assert!(!cluster.console_pulled(&gauge.hello().unwrap()));
    // A pull is control traffic and is never taken for a gauge.
    assert!(protocol::is_reserved(protocol::CONSOLE_PULL_ID));
}
