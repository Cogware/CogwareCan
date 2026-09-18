// SPDX-License-Identifier: GPL-3.0-only
//! Global mode, the update-in-progress flag, and the node message channel.

use cogware_can::bus::*;
use cogware_can::indicators::{self, Indicator};
use cogware_can::protocol::{self, NODE_BROADCAST};
use cogware_can::xfer;
use embedded_can::Frame;

mod common;
use common::bus;

const MODE_SPORT: u8 = 1;
const MODE_TRACK: u8 = 2;

#[test]
fn a_node_speaks_on_its_own_id() {
    // The message ID *is* the node address, so two nodes can never collide on
    // one ID and the lower address wins arbitration.
    for node in 1..=protocol::NODE_MAX {
        let f = message(node, Opcode::Hello, &[]).unwrap();
        assert_eq!(protocol::id_of(&f), Some(node as u16));
        assert_eq!(NodeMsg::parse(&f).unwrap().node, node);
    }
    // The master and out-of-range addresses own no message ID.
    assert!(message(protocol::NODE_MASTER, Opcode::Hello, &[]).is_none());
    assert!(message(protocol::NODE_MAX + 1, Opcode::Hello, &[]).is_none());
    assert!(message(NODE_BROADCAST, Opcode::Hello, &[]).is_none());
    // Eight args plus the opcode would not fit a frame.
    assert!(message(1, Opcode::Error, &[0; 7]).is_some());
    assert!(message(1, Opcode::Error, &[0; 8]).is_none());
}

#[test]
fn a_node_asks_for_a_mode_and_the_master_announces_it() {
    let mut master = Bus::new();
    let mut display = BusView::new(3);
    assert_eq!(master.mode(), MODE_NORMAL);
    assert!(!display.is_synced(), "no state seen yet");

    let request = display.request_mode(MODE_SPORT).unwrap();
    let msg = master.feed(&request).unwrap();
    assert_eq!((msg.node, msg.opcode, msg.mode()), (3, Opcode::ModeRequest, Some(MODE_SPORT)));
    assert_eq!(master.mode(), MODE_SPORT);

    let state = master.announce().expect("the change is worth announcing");
    assert!(display.feed(&state));
    assert_eq!(display.mode(), MODE_SPORT);
    assert!(display.is_synced());
}

#[test]
fn the_state_is_announced_once_per_change() {
    let mut master = Bus::new();
    master.set_mode(MODE_TRACK);
    assert!(master.announce().is_some());
    assert!(master.announce().is_none(), "nothing changed since");

    // Asking for the mode it is already in is not a change.
    assert!(!master.set_mode(MODE_TRACK));
    assert!(master.announce().is_none());

    // A node that just booted asks for the state again.
    let msg = message(1, Opcode::Hello, &[]).unwrap();
    master.feed(&msg);
    assert!(master.announce().is_some());
}

#[test]
fn an_update_flips_the_flag_for_every_node_not_being_written() {
    let mut master = Bus::new();
    let mut target = BusView::new(2);
    let mut bystander = BusView::new(5);

    master.update_started(2);
    let state = master.announce().unwrap();
    for view in [&mut target, &mut bystander] {
        view.feed(&state);
        assert!(view.update_in_progress(), "every node knows why the bus is busy");
    }
    assert!(target.update_is_mine());
    assert!(!bystander.update_is_mine(), "it is not this node's image");

    master.update_finished();
    let state = master.announce().unwrap();
    target.feed(&state);
    bystander.feed(&state);
    assert!(!target.update_in_progress() && !bystander.update_in_progress());
}

#[test]
fn a_broadcast_update_is_every_nodes_business() {
    let mut master = Bus::new();
    master.update_started(NODE_BROADCAST);
    let state = master.announce().unwrap();
    let mut view = BusView::new(7);
    view.feed(&state);
    assert!(view.update_is_mine());
}

#[test]
fn the_mode_is_locked_while_an_update_runs() {
    let mut master = Bus::new();
    master.update_started(1);
    assert!(!master.set_mode(MODE_SPORT), "a scene must not swap mid-transfer");
    assert_eq!(master.mode(), MODE_NORMAL);

    // A node asking anyway is refused, but still told the mode it is in.
    master.feed(&message(4, Opcode::ModeRequest, &[MODE_TRACK]).unwrap());
    assert_eq!(master.mode(), MODE_NORMAL);
    let state = master.announce().expect("the asker is answered");
    assert_eq!(BusState::parse(&state).unwrap().mode, MODE_NORMAL);

    master.update_finished();
    assert!(master.set_mode(MODE_SPORT));
}

#[test]
fn progress_only_moves_the_state_a_whole_percent_at_a_time() {
    let mut master = Bus::new();
    master.update_started(1);
    master.announce();

    master.update_progress(0, 1000);
    assert!(master.announce().is_none(), "still zero percent");
    master.update_progress(9, 1000);
    assert!(master.announce().is_none(), "under one percent");
    master.update_progress(10, 1000);
    let f = master.announce().expect("one percent is worth a frame");
    assert_eq!(BusState::parse(&f).unwrap().progress, 1);

    master.update_progress(2000, 1000);
    assert_eq!(master.state().progress, 100, "done is clamped, never over");
}

#[test]
fn a_sender_percentage_feeds_the_announced_progress() {
    let image = [0xAAu8; 60];
    let mut store = [0u8; 64];
    let mut tx = xfer::Sender::new(1, &image[..], xfer::kind::SCENE, 64, 1);
    let mut rx = xfer::Receiver::new(1, xfer::BufferSink::new(&mut store));
    assert_eq!(tx.percent(), 0);

    // Drive the whole session; the announced percentage tracks what the node
    // has acknowledged, not what has merely been put on the wire.
    let mut bus = Bus::new();
    bus.update_started(1);
    for _ in 0..64 {
        if let Some(f) = tx.next_frame() {
            if let Some(reply) = rx.feed(&f) {
                tx.feed_reply(&reply);
            }
        }
        bus.update_progress(tx.progress().0, tx.progress().1);
        if tx.state() == xfer::TxState::Done {
            break;
        }
    }
    assert_eq!(tx.state(), xfer::TxState::Done);
    assert_eq!(tx.percent(), 100);
    assert_eq!(bus.state().progress, 100);
    assert_eq!(tx.image(), xfer::Image { size: 60, kind: xfer::kind::SCENE, slot: 0 });
}

#[test]
fn errors_and_offers_reach_the_master_intact() {
    let mut master = Bus::new();

    let err = BusView::new(6).report_error(0x42).unwrap();
    let msg = master.feed(&err).unwrap();
    assert_eq!((msg.node, msg.error()), (6, Some(0x42)));
    assert!(master.announce().is_none(), "an error changes no state");

    let offer = BusView::new(9).offer_update(xfer::kind::SCENE, 4096).unwrap();
    let msg = master.feed(&offer).unwrap();
    assert_eq!(msg.offer(), Some((xfer::kind::SCENE, 4096)));

    // An accessor only answers for its own opcode.
    assert!(msg.mode().is_none() && msg.error().is_none());
}

#[test]
fn a_state_frame_round_trips_and_junk_is_rejected() {
    let state = BusState {
        mode: 3,
        flags: FLAG_UPDATE | FLAG_ECU_LINK,
        progress: 55,
        target: 2,
        radio: 0,
        radio_fault: 0,
    };
    let f = state.to_frame().unwrap();
    // What this build emits. `STATE_LEN` is the shortest a reader accepts,
    // which is what lets the frame grow without stranding an older node.
    assert_eq!(f.dlc(), STATE_LEN_FULL);
    assert_eq!(BusState::parse(&f), Some(state));
    assert!(state.update_in_progress() && state.ecu_link() && !state.mode_locked());

    // Not a state frame, and a truncated one, are both refused.
    assert!(BusState::parse(&protocol::ack_frame(0x2D).unwrap()).is_none());
    assert!(BusState::parse(&protocol::frame(protocol::BUS_STATE_ID, &[1, 2]).unwrap()).is_none());

    // A gauge frame is not a node message, and an unknown opcode is refused.
    let mut master = Bus::new();
    assert!(master.feed(&protocol::frame(0x2D, &[1, 2]).unwrap()).is_none());
    assert!(master.feed(&protocol::frame(1, &[0xEE]).unwrap()).is_none());
}

// --- tell-tales wired to a node's own pins ---

#[test]
fn a_node_reports_an_input_and_the_master_lights_the_lamp() {
    // The whole round trip, which is the point of the opcode: the display
    // does not light its own lamp, it says so and the master decides. What
    // comes back is an ordinary gauge broadcast that every screen sees.
    let _bus = bus();
    let mut master = Bus::new();
    let display = BusView::new(3);

    assert!(!indicators::get(Indicator::TurnLeft), "nothing has said yet");
    let report = display.report_lamp(Indicator::TurnLeft, true).unwrap();
    let msg = master.feed(&report).unwrap();
    assert_eq!((msg.node, msg.opcode), (3, Opcode::Input));
    assert!(indicators::get(Indicator::TurnLeft), "the master ignored it");

    // And the bank goes out as a frame, exactly as any gauge does.
    let frame = cogware_can::frame_for(cogware_can::INDICATORS1.id).unwrap();
    assert_eq!(protocol::id_of(&frame), Some(cogware_can::INDICATORS1.id));

    master.feed(&display.report_lamp(Indicator::TurnLeft, false).unwrap());
    assert!(!indicators::get(Indicator::TurnLeft), "the stalk let go");
}

#[test]
fn a_whole_loom_fits_one_frame() {
    // A node coming back from a bus drop hands its lot over at once: a dash
    // that arrives in two halves is a dash that is briefly wrong.
    let _bus = bus();
    let mut master = Bus::new();
    let display = BusView::new(1);
    let loom = [
        (Indicator::TurnLeft, true),
        (Indicator::TurnRight, false),
        (Indicator::HighBeam, true),
        (Indicator::ParkBrake, true),
        // Across both banks, since the pack carries the bank in the byte.
        (Indicator::Fan, true),
        (Indicator::PitLimiter, false),
        (Indicator::Knock, true),
    ];
    assert_eq!(loom.len(), LAMPS_PER_MSG, "this is the full frame");

    let report = display.report_lamps(&loom).unwrap();
    master.feed(&report).expect("the master must take it");
    for (ind, on) in loom {
        assert_eq!(indicators::get(ind), on, "{}", ind.name());
    }
}

#[test]
fn more_lamps_than_a_frame_holds_is_refused_rather_than_truncated() {
    // A silent truncation would leave the lamps past the cut stuck at
    // whatever they were last time, which is the worst of both.
    let display = BusView::new(1);
    let too_many = [(Indicator::TurnLeft, true); LAMPS_PER_MSG + 1];
    assert!(display.report_lamps(&too_many).is_none());
    assert!(display.report_lamps(&[]).is_none(), "nothing to say");
    assert!(display.report_lamps(&too_many[..LAMPS_PER_MSG]).is_some());
}

#[test]
fn every_lamp_in_the_table_survives_the_pack() {
    // The byte carries the bank in one bit and the bit in five, which is
    // exactly the shape of the table. If the table ever grows a third bank
    // or a thirty-third bit, this is what says so.
    for ind in indicators::ALL_INDICATORS {
        for on in [true, false] {
            assert_eq!(
                unpack_lamp(pack_lamp(*ind, on)),
                Some((*ind, on)),
                "{} {on}",
                ind.name()
            );
        }
    }
}

#[test]
fn every_byte_the_pack_can_produce_names_a_lamp_this_build_has() {
    // Both banks are full today, so every encodable byte resolves. The day a
    // bank gains a spare bit, `unpack_lamp` answers None for it and `lamps()`
    // skips it rather than guessing -- which is what an older display does
    // when a newer master reports a lamp added after it shipped.
    for byte in 0..=u8::MAX {
        let packed = byte & !SPARE_BIT;
        assert!(unpack_lamp(packed).is_some(), "{packed:#04x}");
    }
    // And the refusals the decode rests on.
    assert!(Indicator::from_bank_bit(3, 0).is_none(), "no third bank");
    assert!(Indicator::from_bank_bit(1, 32).is_none(), "no bit past 31");
}

#[test]
fn a_byte_with_the_spare_bit_set_is_refused_rather_than_read_around_it() {
    // It cannot have come from this build's pack. If that bit is ever spent
    // -- on a third bank, most likely -- reading it as though it were not
    // would light a bank-one lamp for a bank-three one.
    let _bus = bus();
    let mut master = Bus::new();
    let known = pack_lamp(Indicator::TurnLeft, true);
    assert_eq!(unpack_lamp(known | SPARE_BIT), None);

    // And one good byte beside one bad one still lands, so a newer master's
    // extra lamp costs an older display only that lamp.
    let frame = message(1, Opcode::Input, &[known, known | SPARE_BIT]).unwrap();
    assert_eq!(NodeMsg::parse(&frame).unwrap().lamps().count(), 1);
    master.feed(&frame);
    assert!(indicators::get(Indicator::TurnLeft));
}

#[test]
fn every_other_opcode_reports_no_lamps() {
    // A host draining `lamps()` over every message it sees must not have a
    // mode request or an error decode as a dash state.
    for (opcode, args) in [
        (Opcode::ModeRequest, &[MODE_SPORT][..]),
        (Opcode::Error, &[0x81][..]),
        (Opcode::Hello, &[][..]),
        (Opcode::UpdateOffer, &[xfer::kind::CONFIG, 1, 0, 0, 0][..]),
    ] {
        let f = message(2, opcode, args).unwrap();
        let msg = NodeMsg::parse(&f).unwrap();
        assert_eq!(msg.lamps().count(), 0, "{opcode:?}");
    }
}

#[test]
fn an_input_report_does_not_disturb_the_mode_or_the_state_frame() {
    // Reporting a stalk must not make the master re-announce: the bus state
    // has not changed, and a frame per blink is a bus nobody can use.
    let _bus = bus();
    let mut master = Bus::new();
    let display = BusView::new(4);
    master.set_mode(MODE_TRACK);
    master.announce().expect("the mode change is news");

    master.feed(&display.report_lamp(Indicator::TurnLeft, true).unwrap());
    assert_eq!(master.mode(), MODE_TRACK, "the mode moved");
    assert!(master.announce().is_none(), "a blink re-announced the bus");
}

#[test]
fn a_lamp_a_node_reports_reads_back_through_the_ordinary_gauge_path() {
    // The bank is a gauge, so what the master sends and what a display feeds
    // back in are the same path every other reading takes.
    let _bus = bus();
    let mut master = Bus::new();
    let display = BusView::new(2);
    master.feed(&display.report_lamp(Indicator::Fan, true).unwrap());

    let broadcast = cogware_can::frame_for(cogware_can::INDICATORS2.id).unwrap();
    cogware_can::INDICATORS2.clear();
    assert!(!indicators::get(Indicator::Fan), "cleared for the test");
    cogware_can::feed_frame(&broadcast).expect("an ordinary gauge frame");
    assert!(indicators::get(Indicator::Fan), "it did not come back");
}

// ===================== the server's radios =====================

/// The server is the car's only aerial and the one board nobody can reach once
/// the car is together, so every one of these is about a display asking for
/// something the server alone can do.

#[test]
fn a_display_can_ask_the_server_to_turn_bluetooth_on() {
    let view = BusView::new(0x02);
    let frame = view.request_bluetooth(true).expect("a request must frame");
    let msg = NodeMsg::parse(&frame).expect("and must parse back");
    assert_eq!(msg.node, 0x02);
    assert_eq!(msg.opcode, Opcode::RadioRequest);
    assert_eq!(msg.radio(), Some((RADIO_BLUETOOTH, RADIO_ON)));
}

#[test]
fn a_display_can_ask_the_server_to_accept_a_phone() {
    let view = BusView::new(0x01);
    let frame = view.request_pairing().expect("a request must frame");
    let msg = NodeMsg::parse(&frame).expect("and must parse back");
    assert_eq!(msg.radio(), Some((RADIO_BLUETOOTH, RADIO_PAIR)));
}

#[test]
fn the_master_answers_a_radio_request_without_acting_on_it() {
    let mut bus = Bus::new();
    let _ = bus.announce();
    let view = BusView::new(0x02);
    let asked = view.request_bluetooth(true).expect("a request must frame");

    let msg = bus.feed(&asked).expect("the master must see it");
    assert_eq!(msg.opcode, Opcode::RadioRequest);
    // Not applied: the aerial is hardware and the firmware owns it. Claiming
    // it was on here would put that on every screen before it was true.
    assert_eq!(bus.radio() & RADIO_BLUETOOTH, 0);
    // But announced, so the node that asked is told what is actually so.
    assert!(
        bus.announce().is_some(),
        "a request must be answered even when nothing comes of it"
    );
}

#[test]
fn the_state_follows_the_hardware_once_the_firmware_says_so() {
    let mut bus = Bus::new();
    bus.set_radio(RADIO_BLUETOOTH, true);
    assert!(bus.state().bluetooth_on());
    let frame = bus.state_frame().expect("a state must frame");
    let seen = BusState::parse(&frame).expect("and must parse back");
    assert!(seen.bluetooth_on());
    assert!(!seen.wifi_on());
    assert!(!seen.pairing());
}

#[test]
fn a_radio_that_is_off_cannot_be_accepting_devices() {
    let mut bus = Bus::new();
    // Nothing is on, so there is nothing for a phone to arrive on.
    assert!(!bus.set_pairing(true), "pairing on a dead radio must be refused");
    assert!(!bus.state().pairing());

    bus.set_radio(RADIO_BLUETOOTH, true);
    assert!(bus.set_pairing(true));
    assert!(bus.state().pairing());

    // And switching the radio off takes the pairing notice down with it,
    // rather than leaving a bit set for something that cannot happen.
    bus.set_radio(RADIO_BLUETOOTH, false);
    assert!(!bus.state().pairing(), "pairing outlived the radio");
}

#[test]
fn turning_one_radio_off_leaves_the_other_alone() {
    let mut bus = Bus::new();
    bus.set_radio(RADIO_BLUETOOTH | RADIO_WIFI, true);
    bus.set_radio(RADIO_BLUETOOTH, false);
    assert!(!bus.state().bluetooth_on());
    assert!(bus.state().wifi_on(), "wifi was switched off with bluetooth");
}

#[test]
fn every_node_learns_the_radio_state_from_the_broadcast() {
    let mut bus = Bus::new();
    bus.set_radio(RADIO_BLUETOOTH, true);
    bus.set_pairing(true);
    let frame = bus.state_frame().expect("a state must frame");

    // The node that asked and the ones that did not both read the same thing,
    // which is the point of putting it in the broadcast at all.
    for node in [0x01, 0x02, 0x03] {
        let mut view = BusView::new(node);
        assert!(view.feed(&frame));
        assert!(view.bluetooth_on());
        assert!(view.pairing());
    }
}

#[test]
fn a_state_frame_from_a_master_that_predates_radios_reads_as_all_off() {
    // Four bytes, which is what an older master sends. It must parse, and the
    // radios must read as off rather than as whatever was next in the buffer.
    let frame = protocol::frame(0x018, &[3, 0, 0, 0]).expect("a short state must frame");
    let seen = BusState::parse(&frame).expect("a short state must still parse");
    assert_eq!(seen.mode, 3, "the bytes it did send must survive");
    assert!(!seen.bluetooth_on());
    assert!(!seen.wifi_on());
    assert!(!seen.pairing());
}

#[test]
fn a_radio_request_too_short_to_read_is_refused_rather_than_guessed() {
    // A node this build does not know could send one byte. Reading the missing
    // action as zero would be reading it as "turn everything off".
    let frame = message(0x02, Opcode::RadioRequest, &[RADIO_BLUETOOTH])
        .expect("a one-byte request must frame");
    let msg = NodeMsg::parse(&frame).expect("and must parse");
    assert_eq!(msg.radio(), None);
}

// ===================== waiting on the server =====================

/// A helper standing in for a display's clock.
fn at(ms: u64) -> u64 {
    ms
}

#[test]
fn a_request_that_is_answered_settles_on_done() {
    let mut view = BusView::new(0x02);
    let mut ask = RadioAsk::pairing(at(0));
    assert_eq!(ask.state(), RadioWait::Waiting);

    let mut bus = Bus::new();
    bus.set_radio(RADIO_BLUETOOTH, true);
    bus.set_pairing(true);
    view.feed(&bus.state_frame().expect("a state must frame"));

    assert_eq!(ask.poll(&view, at(100)), RadioWait::Done);
}

#[test]
fn a_radio_that_came_up_but_is_not_pairing_does_not_count_as_done() {
    // Someone stood at the car with a phone would be told to hold it near a
    // server that is refusing everything.
    let mut view = BusView::new(0x02);
    let mut ask = RadioAsk::pairing(at(0));

    let mut bus = Bus::new();
    bus.set_radio(RADIO_BLUETOOTH, true);
    view.feed(&bus.state_frame().expect("a state must frame"));

    assert_eq!(ask.poll(&view, at(100)), RadioWait::Waiting);
}

#[test]
fn a_server_that_refuses_says_so_rather_than_going_quiet() {
    let mut view = BusView::new(0x02);
    let mut ask = RadioAsk::bluetooth(true, at(0));

    let mut bus = Bus::new();
    bus.radio_failed(RadioFault::Denied);
    view.feed(&bus.state_frame().expect("a state must frame"));

    assert_eq!(ask.poll(&view, at(100)), RadioWait::Failed(RadioFault::Denied));
}

#[test]
fn a_radio_that_would_not_start_is_told_apart_from_a_refusal() {
    // The whole reason the fault is on the wire: a node watching a bit stay
    // clear cannot tell these apart, and they mean different things to do.
    for fault in [
        RadioFault::Denied,
        RadioFault::NoRadio,
        RadioFault::WouldNotStart,
    ] {
        let mut view = BusView::new(0x02);
        let mut ask = RadioAsk::bluetooth(true, at(0));
        let mut bus = Bus::new();
        bus.radio_failed(fault);
        view.feed(&bus.state_frame().expect("a state must frame"));
        assert_eq!(ask.poll(&view, at(10)), RadioWait::Failed(fault));
        assert!(!fault.describe().is_empty(), "every fault must have words");
    }
}

#[test]
fn a_server_that_never_answers_stops_the_throbber_rather_than_spinning_for_ever() {
    let view = BusView::new(0x02);
    let mut ask = RadioAsk::bluetooth(true, at(0));
    assert_eq!(ask.poll(&view, at(RADIO_WAIT_MS - 1)), RadioWait::Waiting);
    assert_eq!(
        ask.poll(&view, at(RADIO_WAIT_MS)),
        RadioWait::Failed(RadioFault::NoAnswer)
    );
}

#[test]
fn an_outcome_does_not_come_undone_on_a_later_broadcast() {
    // A person has been told. The bus moving on afterwards -- somebody else
    // switching the radio off, say -- must not rewrite what they were told.
    let mut view = BusView::new(0x02);
    let mut ask = RadioAsk::bluetooth(true, at(0));

    let mut bus = Bus::new();
    bus.set_radio(RADIO_BLUETOOTH, true);
    view.feed(&bus.state_frame().expect("a state must frame"));
    assert_eq!(ask.poll(&view, at(10)), RadioWait::Done);

    bus.set_radio(RADIO_BLUETOOTH, false);
    view.feed(&bus.state_frame().expect("a state must frame"));
    assert_eq!(ask.poll(&view, at(20)), RadioWait::Done, "the outcome moved");
}

#[test]
fn a_new_request_clears_the_fault_the_last_one_left() {
    let mut bus = Bus::new();
    bus.radio_failed(RadioFault::WouldNotStart);
    assert_eq!(bus.state().fault(), Some(RadioFault::WouldNotStart));

    let view = BusView::new(0x02);
    let asked = view.request_bluetooth(true).expect("a request must frame");
    bus.feed(&asked);
    assert_eq!(
        bus.state().fault(),
        None,
        "a stale fault would fail the next request before it was tried"
    );
}

#[test]
fn a_radio_that_comes_up_clears_the_fault() {
    let mut bus = Bus::new();
    bus.radio_failed(RadioFault::WouldNotStart);
    bus.set_radio(RADIO_BLUETOOTH, true);
    assert_eq!(bus.state().fault(), None);
    assert!(bus.state().bluetooth_on());
}

#[test]
fn a_fault_from_a_newer_server_is_carried_rather_than_read_as_well() {
    let mut view = BusView::new(0x02);
    let mut ask = RadioAsk::bluetooth(true, at(0));
    let mut bus = Bus::new();
    bus.radio_failed(RadioFault::Unknown(200));
    view.feed(&bus.state_frame().expect("a state must frame"));
    assert_eq!(
        ask.poll(&view, at(10)),
        RadioWait::Failed(RadioFault::Unknown(200))
    );
}

#[test]
fn a_state_frame_from_a_master_that_predates_faults_reads_as_no_fault() {
    let frame = protocol::frame(0x018, &[0, 0, 0, 0, RADIO_BLUETOOTH])
        .expect("a five-byte state must frame");
    let seen = BusState::parse(&frame).expect("and must parse");
    assert!(seen.bluetooth_on());
    assert_eq!(seen.fault(), None);
}

#[test]
fn a_health_report_survives_the_bus_unchanged() {
    // The whole point of the message is that what a node measures is what a
    // host reads, with the master in between as a post office. A field lost
    // or shifted on the way would read as a plausible wrong number, which is
    // worse than no number.
    use cogware_can::bus::{BusView, Health, NodeMsg};

    let display = BusView::new(2);
    let said = Health { fps: 23, compose_us: 4100, painted: 19_358, rects: 2 };
    let frame = display.report_health(&said).expect("fits a frame");

    let heard = NodeMsg::parse(&frame).expect("is a node message");
    assert_eq!(heard.node, 2, "the sender's own id rides in the CAN id");
    assert_eq!(heard.health(), Some(said));
}

#[test]
fn a_short_health_report_is_refused_rather_than_guessed() {
    use cogware_can::bus::Health;
    assert_eq!(Health::decode(&[23, 0x04, 0x10, 0xbe]), None);
    assert_eq!(
        Health::decode(&[23, 0x04, 0x10, 0xbe, 0x00, 0x02]),
        Some(Health { fps: 23, compose_us: 0x1004, painted: 190, rects: 2 })
    );
}
