#[macro_use]
extern crate failure;
extern crate lidar_buffer;
extern crate pcap;
extern crate serde_json;
#[macro_use]
extern crate log;
extern crate pretty_env_logger;

use failure::Fallible;
use lidar_buffer::ouster::{
    Config, Frame, FrameConverter, Packet as OusterPacket, PointCloudConverter,
};
use pcap::Capture;

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_create_packet() -> Fallible<()> {
    let mut packets = vec![];

    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;
        packets.push(lidar_packet);
    }

    for (idx, packet) in packets.iter().enumerate() {
        let ts = packet.columns[0].timestamp;
        println!("No. {}, timestamp = {}", idx, ts);
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_pcd_converter() -> Fallible<()> {
    // Load config
    let config = Config::from_path("test_files/ouster_example.json")?;
    let pcd_converter = PointCloudConverter::from_config(config);

    // Load pcap file
    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp")?;

    let mut last_fid_opt = None;

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;

        for column in lidar_packet.columns.iter() {
            // Skip invalid columns
            if !column.valid() {
                warn!("Invalid column detected");
                continue;
            }

            // Check if frame ID rewinds
            ensure!(
                last_fid_opt
                    .map(|last_fid| last_fid <= column.frame_id)
                    .unwrap_or(true),
                "Column with inconsecutive frame id detected. Please report this bug."
            );

            // Construct point cloud
            last_fid_opt = Some(column.frame_id);
            let _column_points = pcd_converter.column_to_points(&column)?;
        }
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_frame_converter() -> Fallible<()> {
    // Load config
    let config = Config::from_path("test_files/ouster_example.json")?;
    let mut frame_converter = FrameConverter::from_config(config);

    // Load pcap file
    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp")?;

    let mut frames = vec![];

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;

        let new_frames = lidar_packet
            .columns
            .into_iter()
            .map(|column| {
                let new_frames = frame_converter.push(column)?;
                Ok(new_frames)
            })
            .collect::<Fallible<Vec<_>>>()?
            .into_iter()
            .flat_map(|frames| frames)
            .collect::<Vec<Frame>>();

        frames.extend(new_frames);
    }

    if let Some(frame) = frame_converter.finish() {
        frames.push(frame);
    }

    let mut prev_frame_id_opt = None;
    for frame in frames {
        if let Some(prev_frame_id) = prev_frame_id_opt {
            ensure!(prev_frame_id < frame.frame_id, "Frame ID is not ordered");
        }
        prev_frame_id_opt = Some(frame.frame_id);
    }

    Ok(())
}
