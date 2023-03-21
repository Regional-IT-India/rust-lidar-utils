use std::time::{Duration};

use log::{debug, trace};
use mio::{Events, Poll, PollOpt, Ready, Token};
use mio_extras::channel;
use rustdds::{DomainParticipant, Keyed, QosPolicyBuilder, StatusEvented, TopicDescription, TopicKind};
use rustdds::policy::{Durability, History, Reliability};
use serde::{Deserialize, Serialize};
use ouster_lidar::{client::CommandClient, Column};

use serde_big_array::BigArray;

const READER_READY: Token = Token(1);
const READER_STATUS_READY: Token = Token(2);
const STOP_PROGRAM: Token = Token(0);


fn main() {
    let topic_name = String::from("OusterLidar");
    let type_desc = "OusterLidarMessage".to_string();
    let domain_id = 0;

    let domain_participant: DomainParticipant = DomainParticipant::new(domain_id)
        .unwrap_or_else(|e| panic!("DomainParticipant construction failed: {:?}", e));

    let qos_b = QosPolicyBuilder::new()
        .reliability(Reliability::BestEffort)
        .durability(Durability::Volatile)
        .history(History::KeepAll);

    let qos = qos_b.build();

    let loop_delay = Duration::from_millis(200);

    let topic = domain_participant
        .create_topic(
            topic_name,
            type_desc,
            &qos,
            TopicKind::WithKey,
        )
        .unwrap_or_else(|e| panic!("create_topic failed: {:?}", e));

    println!(
        "Topic name is {}. Type is {}.",
        topic.name(),
        topic.get_type().name()
    );

    // Set Ctrl-C handler
    let (stop_sender, stop_receiver) = channel::channel();
    ctrlc::set_handler(move || {
        stop_sender.send(()).unwrap_or(());
        // ignore errors, as we are quitting anyway
    })
        .expect("Error setting Ctrl-C handler");
    println!("Press Ctrl-C to quit.");

    let poll = Poll::new().unwrap();
    let mut events = Events::with_capacity(4);


    poll.register(
        &stop_receiver,
        STOP_PROGRAM,
        Ready::readable(),
        PollOpt::edge(),
    ).unwrap();


    let mut reader = {
        let subscriber = domain_participant.create_subscriber(&qos).unwrap();
        let mut reader = subscriber
            .create_datareader_cdr::<Column>(&topic, Some(qos))
            .unwrap();
        poll
            .register(&reader, READER_READY, Ready::readable(), PollOpt::edge())
            .unwrap();
        poll
            .register(
                reader.as_status_evented(),
                READER_STATUS_READY,
                Ready::readable(),
                PollOpt::edge(),
            )
            .unwrap();
        debug!("Created DataReader");
        reader
    };

    loop {
        poll.poll(&mut events, Some(loop_delay)).unwrap();
        for event in &events {
            match event.token() {
                STOP_PROGRAM => {
                    if stop_receiver.try_recv().is_ok() {
                        println!("Done.");
                        return;
                    }
                }

                READER_READY => loop {
                    trace!("DataReader triggered");
                    match reader.take_next_sample() {
                        Ok(Some(sample)) => match sample.into_value() {
                            Ok(sample) => println!(
                                "{} {:?} \n\n\n",
                                topic.name(),
                                sample
                            ),
                            Err(key) => println!("Disposed key {:?}", key),
                        },
                        Ok(None) => break, // no more data
                        Err(e) => println!("DataReader error {:?}", e),
                    } // match
                },
                READER_STATUS_READY => {
                    while let Some(status) = reader.try_recv_status() {
                        println!("DataReader status: {:?}", status);
                    }
                }
                other_token => {
                    println!("Polled event is {:?}. WTF?", other_token);
                }
            }
        }
    }
}