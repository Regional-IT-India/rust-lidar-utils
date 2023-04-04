use std::{
    fs::File,
    io::prelude::*,
    net::{Ipv4Addr, SocketAddr, UdpSocket},
    path::PathBuf,
    time::{Duration, Instant},
};

use anyhow::Result;
use log::{error, warn};
use mio::{Events, Poll, PollOpt, Ready, Token};
use mio_extras::{channel, channel::Receiver};
use rustdds::{
    DomainParticipant,
    Keyed, policy::{Durability, History, Reliability}, QosPolicyBuilder, StatusEvented, TopicDescription, TopicKind,
};
use serde::{Deserialize, Serialize};

use LidarMode::Mode1024x10;
use ouster_lidar::{client::CommandClient, Column, Config, Frame, frame_converter, FrameConverter, LidarMode, Packet, packet::Packet as OusterPacket, PacketMetaData, Pixel, PointCloudConverter};

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
    client.set_lidar_mode(Mode1024x10)?;
    client.set_timestamp_mode(ouster_lidar::TimestampMode::TimeFromInternalOsc)?;
    client.set_sync_pulse_in_polarity(ouster_lidar::Polarity::ActiveHigh)?;
    client.set_nmea_in_polarity(ouster_lidar::Polarity::ActiveHigh)?;
    client.set_udp_ip(config.listen_addr)?;
    client.set_udp_port_lidar(config_txt.udp_port_lidar)?;
    // client.reinitialize()?;

    println!("{:?}", client.get_beam_intrinsics().unwrap().beam_altitude_angles);
    println!("{:?}", client.get_beam_intrinsics().unwrap().beam_azimuth_angles);
    println!("{:?}", client);

    let converter_config = Config::new(client.get_beam_intrinsics().unwrap().beam_altitude_angles, client.get_beam_intrinsics().unwrap().beam_azimuth_angles, Mode1024x10);

    let mut frame_converter = FrameConverter::from_config(converter_config);

    let bind_addr = SocketAddr::from((config.listen_addr, config_txt.udp_port_lidar));
    let socket = UdpSocket::bind(bind_addr)?;
    socket.set_read_timeout(Some(timeout))?;
    let packet_size = std::mem::size_of::<OusterPacket>();

    // println!("Columns per revolution {}", frame_converter.columns_per_revolution());
    let mut frames: Vec<Frame> = vec![];
    let mut n = 1024 * 10;
    while n > 0 {
        n -= 1;
        println!("{}", n);
        // receive UDP packet
        let mut buf = [0; MAX_UDP_PACKET_SIZE];
        let (read_size, peer_addr) = socket.recv_from(&mut buf)?;

        let packet_buf = &buf[..packet_size];
        match OusterPacket::from_slice(packet_buf) {
            Ok(_packet) => {
                // print_packet(*_packet);

                frames = frame_converter.push_packet(_packet).unwrap();
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

    write_pcd_file(frame_converter);

    Ok(())
}

fn write_pcd_file(frame_converter: FrameConverter) {
    let frame = frame_converter.finish().unwrap();
    let path = std::path::Path::new("lidar_1.pcd");
    let mut file = std::fs::File::create(path).unwrap();
    writeln!(file, "# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {}
DATA ascii", frame.points.len(), frame.points.len());
    let points_data = frame.points.iter().map(|point| {
        writeln!(file, "{} {} {}", point.point[0].as_meters(), point.point[1].as_meters(), point.point[2].as_meters());
        (point.point[0].as_meters(), point.point[1].as_meters(), point.point[2].as_meters())
    }).collect::<Vec<(f64, f64, f64)>>();
}

fn print_packet(packet: Packet) {
    let instant = Instant::now();
    println!(
        "received packet: {:?}, from LIDAR in {} milliseconds",
        packet,
        instant.elapsed().as_secs()
    );
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

    let poll = Poll::new().unwrap();
    let mut events = Events::with_capacity(4);

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
                    return;
                }
                WRITER_STATUS_READY => {
                    while let Some(status) = writer.try_recv_status() {
                        println!("DataWriter status: {:?}", status);
                    }
                }
                other_token => {
                    println!("Polled event is {:?}", other_token);
                }
            }
        }

        // write to DDS

        packet.columns.map(|column| {
            let packet_metadata = PacketMetaData {
                timestamp: column.timestamp,
                measurement_id: column.measurement_id,
                frame_id: column.frame_id,
            };
            let now = Instant::now();
            if last_write + loop_delay < now {
                writer
                    .write(column, None)
                    .unwrap_or_else(|e| error!("DataWriter write failed: {:?}", e));
                last_write = now;
            }
        });
    }
}
