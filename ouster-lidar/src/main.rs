use std::{
    fs::File,
    io::prelude::*,
    net::{Ipv4Addr, SocketAddr, UdpSocket},
    path::PathBuf,
    time::{Duration, Instant},
};
use std::net::{IpAddr, Ipv6Addr};

use anyhow::Result;
use log::{debug, warn};
use serde::Deserialize;

use ouster_lidar::{client::CommandClient, Config, config, LidarMode, packet::Packet as OusterPacket};
use ouster_lidar::pcd_converter;
use ouster_lidar::consts;

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

    // let time_info    = client.get_time_info()?;
    // dbg!(&time_info);
    //
    // let beam_intrinsics = client.get_beam_intrinsics()?;
    // dbg!(&beam_intrinsics);
    //
    // let lidar_intrinsics = client.get_lidar_intrinsics()?;
    // dbg!(&lidar_intrinsics);

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

    // std::thread::sleep(Duration::from_secs(10));
    let bind_addr = SocketAddr::from((config.listen_addr, config_txt.udp_port_lidar));
    let socket = UdpSocket::bind(bind_addr)?;
    socket.set_read_timeout(Some(timeout))?;
    let packet_size = std::mem::size_of::<OusterPacket>();
    let instant = Instant::now();

    loop {
        // receive UDP packet
        let mut buf = [0; MAX_UDP_PACKET_SIZE];
        let (read_size, peer_addr) = socket.recv_from(&mut buf)?;

        // if lidar_addr.ip() != peer_addr.ip() || packet_size != read_size {
        //     println!("Lidar IP - {},  {}\n {}: {}",lidar_addr.ip() , peer_addr.ip(), packet_size,read_size );
        //     continue;
        // }

        let packet_buf = &buf[..packet_size];
        match OusterPacket::from_slice(packet_buf) {
            Ok(_packet) => {
                println!(
                    "received packet: {:?}, from LIDAR in {} milliseconds",
                    _packet,
                    instant.elapsed().as_secs()
                );

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