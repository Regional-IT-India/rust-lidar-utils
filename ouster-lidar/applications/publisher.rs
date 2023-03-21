use std::{
    fs::File,
    io::prelude::*,
    net::{Ipv4Addr, SocketAddr, UdpSocket},
    path::PathBuf,
    time::{Duration, Instant},
};

use anyhow::Result;
use log::{warn, error};
use serde::Deserialize;

use ouster_lidar::{client::CommandClient, Column, packet::Packet as OusterPacket};

use rustdds::{DomainParticipant, QosPolicyBuilder, StatusEvented, TopicDescription, TopicKind};
use rustdds::policy::{Durability, History, Reliability};
use mio::{Events, Poll, PollOpt, Ready, Token};
use mio_extras::channel;

const WRITER_STATUS_READY: Token = Token(3);
const STOP_PROGRAM: Token = Token(0);

const MAX_UDP_PACKET_SIZE: usize = 65507;

#[derive(Deserialize, Clone, Debug)]
struct OusterClientTestConfig {
    lidar_addr: Ipv4Addr,
    listen_addr: Ipv4Addr,
    timeout: u64,
}

fn main() -> Result<()> {
    println!("Running...");
    let config: OusterClientTestConfig = {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("test_files")
            .join("ouster_client_test.toml");
        let mut config_file = File::open(path)?;
        let mut config_str = String::new();
        config_file.read_to_string(&mut config_str)?;
        toml::from_str(&config_str)?
    };

    println!("config created");
    let lidar_addr = SocketAddr::new(config.lidar_addr.into(), 7501);
    let timeout = Duration::from_secs(config.timeout);
    let mut client = CommandClient::connect(lidar_addr, Some(timeout))?;

    println!("Client created");
    // get info
    let config_txt = client.get_config_txt()?;
    dbg!(&config_txt);

    let imu_intrinsics = client.get_imu_intrinsics()?;
    dbg!(&imu_intrinsics);


    // try to receive udp packets
    client.set_lidar_mode(ouster_lidar::LidarMode::Mode1024x10)?;
    client.set_timestamp_mode(ouster_lidar::TimestampMode::TimeFromInternalOsc)?;
    client.set_sync_pulse_in_polarity(ouster_lidar::Polarity::ActiveHigh)?;
    client.set_nmea_in_polarity(ouster_lidar::Polarity::ActiveHigh)?;
    client.set_udp_ip(config.listen_addr)?;
    client.set_udp_port_lidar(config_txt.udp_port_lidar)?;
    // client.reinitialize()?;

    let bind_addr = SocketAddr::from((config.listen_addr, config_txt.udp_port_lidar));
    let socket = UdpSocket::bind(bind_addr)?;
    socket.set_read_timeout(Some(timeout))?;
    let packet_size = std::mem::size_of::<OusterPacket>();
    let instant = Instant::now();

    loop {
        // receive UDP packet
        let mut buf = [0; MAX_UDP_PACKET_SIZE];
        let (read_size, peer_addr) = socket.recv_from(&mut buf)?;


        let packet_buf = &buf[..packet_size];
        match OusterPacket::from_slice(packet_buf) {
            Ok(_packet) => {
                println!(
                    "received packet: {:?}, from LIDAR in {} milliseconds",
                    _packet,
                    instant.elapsed().as_secs()
                );

                publish_message(_packet);

                break;
            }
            Err(error) => {
                warn!(
                    "packet decoding error: {:?}. Proceed to next packet.",
                    error
                );
                continue;
            }
        }
    }

    Ok(())
}

fn publish_message(packet: &OusterPacket) {
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


    let writer = {
        let publisher = domain_participant.create_publisher(&qos).unwrap();
        let mut writer = publisher
            .create_datawriter_cdr::<Column>(&topic, None)
            // None = get qos policy from publisher
            .unwrap();
        poll.register(
            writer.as_status_evented(),
            WRITER_STATUS_READY,
            Ready::readable(),
            PollOpt::edge(),
        ).unwrap();
        writer
    };

    let mut last_write = Instant::now();

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
                WRITER_STATUS_READY => {
                    while let Some(status) = writer.try_recv_status() {
                        println!("DataWriter status: {:?}", status);
                    }
                }
                other_token => {
                    println!("Polled event is {:?}. WTF?", other_token);
                }
            }
        }

        // write to DDS
        let columns = packet.columns;

        for column in columns {
            let now = Instant::now();
            if last_write + loop_delay < now {
                writer
                    .write(column, None)
                    .unwrap_or_else(|e| error!("DataWriter write failed: {:?}", e));
                last_write = now;
            }
        }
    }
}