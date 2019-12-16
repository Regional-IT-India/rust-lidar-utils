//! Provides a set of tools to convert raw data from Velodyne sensors.

use super::{
    consts::{
        CHANNEL_PER_FIRING, COLUMNS_PER_PACKET, FIRINGS_PER_COLUMN, FIRING_PERIOD,
        LASER_RETURN_PERIOD,
    },
    packet::{Packet, ReturnMode},
};
use crate::common::{Point, PointPair, SphericalPoint, Timestamped};
use failure::{bail, ensure, Fallible};
use itertools::izip;
use log::warn;

pub type LastReturnPoint = Timestamped<PointPair>;
pub type StrongestPoint = Timestamped<PointPair>;
pub type DualPoint = Timestamped<(PointPair, PointPair)>; // (last_return, strongest)

/// Represents configuration for Velodyne sensors.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// Vertical angles per laser in degrees.
    pub vertical_degrees: [f64; CHANNEL_PER_FIRING],
    /// Vertical correction per laser in millimeters.
    pub vertical_corrections: [f64; CHANNEL_PER_FIRING],
}

impl Config {
    pub fn vlp_16_config() -> Self {
        Self {
            vertical_degrees: [
                -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0,
                -1.0, 15.0,
            ],
            vertical_corrections: [
                11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7,
                -11.2,
            ],
        }
    }

    pub fn puke_lite_config() -> Self {
        Self {
            vertical_degrees: [
                -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0,
                -1.0, 15.0,
            ],
            vertical_corrections: [
                11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7,
                -11.2,
            ],
        }
    }

    pub fn puke_hi_res_config() -> Self {
        Self {
            vertical_degrees: [
                -10.00, 0.67, -8.67, 2.00, -7.33, 3.33, -6.00, 4.67, -4.67, 6.00, -3.33, 7.33,
                -2.00, 8.67, -0.67, 10.00,
            ],
            vertical_corrections: [
                7.4, -0.9, 6.5, -1.8, 5.5, -2.7, 4.6, -3.7, 3.7, -4.6, 2.7, -5.5, 1.8, -6.5, 0.9,
                -7.4,
            ],
        }
    }
}

/// A list of points with mode.
#[derive(Debug, Clone, PartialEq)]
pub enum VelodynePointList {
    Strongest(Vec<StrongestPoint>),
    LastReturn(Vec<LastReturnPoint>),
    /// List of (last return point, strongest or 2nd strongest return point).
    DualReturn(Vec<DualPoint>),
}

impl VelodynePointList {
    pub fn new(mode: ReturnMode) -> Self {
        match mode {
            ReturnMode::LastReturn => Self::LastReturn(vec![]),
            ReturnMode::Strongest => Self::Strongest(vec![]),
            ReturnMode::DualReturn => Self::DualReturn(vec![]),
        }
    }

    pub fn mode(&self) -> ReturnMode {
        match self {
            Self::Strongest(_) => ReturnMode::Strongest,
            Self::LastReturn(_) => ReturnMode::LastReturn,
            Self::DualReturn(_) => ReturnMode::DualReturn,
        }
    }

    pub fn len(&self) -> usize {
        match self {
            Self::Strongest(points) => points.len(),
            Self::LastReturn(points) => points.len(),
            Self::DualReturn(points) => points.len(),
        }
    }

    pub fn push(&mut self, point: VelodynePoint) -> Fallible<()> {
        match self {
            VelodynePointList::Strongest(points) => {
                if let VelodynePoint::Strongest(pt) = point {
                    points.push(pt);
                } else {
                    bail!("Inconsistent point variant");
                }
            }
            VelodynePointList::LastReturn(points) => {
                if let VelodynePoint::LastReturn(pt) = point {
                    points.push(pt);
                } else {
                    bail!("Inconsistent point variant");
                }
            }
            VelodynePointList::DualReturn(points) => {
                if let VelodynePoint::DualReturn(pt) = point {
                    points.push(pt);
                } else {
                    bail!("Inconsistent point variant");
                }
            }
        }

        Ok(())
    }
}

impl IntoIterator for VelodynePointList {
    type Item = VelodynePoint;
    type IntoIter = std::vec::IntoIter<VelodynePoint>;

    fn into_iter(self) -> Self::IntoIter {
        // TODO: use O(1) impl instead
        match self {
            VelodynePointList::Strongest(points) => points
                .into_iter()
                .map(|point| VelodynePoint::Strongest(point))
                .collect::<Vec<_>>()
                .into_iter(),
            VelodynePointList::LastReturn(points) => points
                .into_iter()
                .map(|point| VelodynePoint::LastReturn(point))
                .collect::<Vec<_>>()
                .into_iter(),
            VelodynePointList::DualReturn(points) => points
                .into_iter()
                .map(|point| VelodynePoint::DualReturn(point))
                .collect::<Vec<_>>()
                .into_iter(),
        }
    }
}

/// A point with mode.
#[derive(Debug, Clone, PartialEq)]
pub enum VelodynePoint {
    Strongest(StrongestPoint),
    LastReturn(LastReturnPoint),
    DualReturn(DualPoint),
}

impl VelodynePoint {
    pub fn timestamp_ns(&self) -> u64 {
        match self {
            Self::Strongest(pt) => pt.timestamp_ns,
            Self::LastReturn(pt) => pt.timestamp_ns,
            Self::DualReturn(pt) => pt.timestamp_ns,
        }
    }

    pub fn mode(&self) -> ReturnMode {
        match self {
            Self::Strongest(_) => ReturnMode::Strongest,
            Self::LastReturn(_) => ReturnMode::LastReturn,
            Self::DualReturn(_) => ReturnMode::DualReturn,
        }
    }

    pub fn azimuth_angle(&self) -> f64 {
        match self {
            Self::Strongest(timestamped_point) => timestamped_point.value.spherical.azimuth_angle,
            Self::LastReturn(timestamped_point) => timestamped_point.value.spherical.azimuth_angle,
            Self::DualReturn(timestamped_point) => {
                let angle1 = (timestamped_point.value.0).spherical.azimuth_angle;
                let angle2 = (timestamped_point.value.1).spherical.azimuth_angle;
                debug_assert_eq!(angle1, angle2, "please report this bug");
                angle1
            }
        }
    }
}

/// A struct that converts raw packets to point cloud data.
#[derive(Debug, Clone)]
pub struct PointCloudConverter {
    config: Config,
}

impl PointCloudConverter {
    /// Construct helper from altitude degrees for each laser beam.
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    /// Compute point locations from firing data from sensor.
    pub fn packet_to_points(&self, packet: &Packet) -> Fallible<VelodynePointList> {
        use std::f64::consts::PI;

        let make_point =
            |distance: f64, vertical_angle: f64, azimuth_angle: f64, vertical_correction: f64| {
                let spherical =
                    SphericalPoint::from_altitude_angle(distance, azimuth_angle, vertical_angle);
                let mut cartesian = Point::from(spherical.clone());
                cartesian.z += vertical_correction;
                PointPair {
                    cartesian,
                    spherical,
                }
            };

        let interpolate = |lhs: f64, rhs: f64, ratio: f64| lhs * (1.0 - ratio) + rhs * ratio;

        let azimuth_angle_diffs = {
            let curr_angles = packet.columns.iter().map(|firing| firing.azimuth_angle());
            let next_angles = packet
                .columns
                .iter()
                .skip(1)
                .map(|firing| firing.azimuth_angle());
            let mut angle_offsets = curr_angles
                .zip(next_angles)
                .map(|(curr, next)| {
                    if next >= curr {
                        next - curr
                    } else {
                        next - curr + 2.0 * PI
                    }
                })
                .collect::<Vec<_>>();
            angle_offsets.push(*angle_offsets.last().unwrap());
            angle_offsets.into_iter()
        };

        let points = match packet.return_mode {
            ReturnMode::Strongest | ReturnMode::LastReturn => {
                let points = packet
                    .columns
                    .iter()
                    .zip(azimuth_angle_diffs)
                    .enumerate()
                    .flat_map(|args| {
                        // one column = 2 firings

                        let (column_idx, (column, azimuth_angle_diff)) = args;

                        // timestamp of current column
                        let base_timestamp =
                            packet.timestamp as f64 + (column_idx as f64) * 2.0 * FIRING_PERIOD;

                        // timestamp offset for first firing
                        let firing_time_offset_former = 0.0;

                        // timestamp offset for second firing
                        let firing_time_offset_latter = FIRING_PERIOD;

                        // azimuth angle of current column
                        let base_azimuth_angle = column.azimuth_angle();

                        vec![
                            (
                                base_timestamp,
                                firing_time_offset_former,
                                base_azimuth_angle,
                                azimuth_angle_diff,
                                column.firing_former,
                            ),
                            (
                                base_timestamp,
                                firing_time_offset_latter,
                                base_azimuth_angle,
                                azimuth_angle_diff,
                                column.firing_latter,
                            ),
                        ]
                    })
                    .flat_map(|args| {
                        let (
                            base_timestamp,
                            firing_time_offset,
                            base_azimuth_angle,
                            azimuth_angle_diff,
                            sequence,
                        ) = args;

                        let firing_time_offsets = (0..).map(|laser_idx| {
                            firing_time_offset + (laser_idx as f64) * LASER_RETURN_PERIOD
                        });

                        izip!(
                            sequence.iter(),
                            self.config.vertical_degrees.iter(),
                            self.config.vertical_corrections.iter(),
                            firing_time_offsets,
                        )
                        .map(|args| {
                            let (
                                laser_return,
                                vertical_degree,
                                vertical_correction,
                                laser_time_offset,
                            ) = args;
                            let interpolate_ratio = laser_time_offset / (FIRING_PERIOD * 2.0);
                            let azimuth_angle = {
                                // time interpolation
                                let angle = interpolate(
                                    base_azimuth_angle,
                                    base_azimuth_angle + azimuth_angle_diff,
                                    interpolate_ratio,
                                );

                                // change from clockwise to counter-clockwise and
                                // rebase the zero angle to match the Velodyne manual
                                let angle = std::f64::consts::FRAC_PI_2 - angle;
                                if angle < 0.0 {
                                    angle + PI * 2.0
                                } else {
                                    angle
                                };
                                angle
                            };
                            let distance = laser_return.mm_distance() as f64;
                            let vertical_angle = vertical_degree.to_radians();
                            let laser_timestamp = base_timestamp + laser_time_offset;
                            let point = make_point(
                                distance,
                                vertical_angle,
                                azimuth_angle,
                                *vertical_correction,
                            );
                            let timestamp_ns = (laser_timestamp * 1000.0) as u64;
                            Timestamped {
                                value: point,
                                timestamp_ns,
                            }
                        })
                        .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>();

                match packet.return_mode {
                    ReturnMode::Strongest => VelodynePointList::Strongest(points),
                    ReturnMode::LastReturn => VelodynePointList::LastReturn(points),
                    _ => unreachable!(),
                }
            }
            ReturnMode::DualReturn => {
                let points = packet
                    .columns
                    .iter()
                    .zip(azimuth_angle_diffs)
                    .enumerate()
                    .flat_map(|(column_idx, (column, azimuth_angle_diff))| {
                        let firing_timestamp =
                            packet.timestamp as f64 + column_idx as f64 * FIRING_PERIOD;
                        let base_azimuth_angle = column.azimuth_angle();
                        let laser_time_offsets =
                            (0..).map(|laser_idx| laser_idx as f64 * LASER_RETURN_PERIOD);

                        izip!(
                            column.firing_former.iter(),
                            column.firing_latter.iter(),
                            laser_time_offsets,
                            self.config.vertical_degrees.iter(),
                            self.config.vertical_corrections.iter(),
                        )
                        .map(|args| {
                            let (
                                last_return,
                                strongest_return,
                                laser_time_offset,
                                vertical_degree,
                                vertical_correction,
                            ) = args;
                            let last_distance = last_return.mm_distance() as f64;
                            let strongest_distance = strongest_return.mm_distance() as f64;
                            let interpolate_ratio = laser_time_offset / FIRING_PERIOD;
                            let azimuth_angle = interpolate(
                                base_azimuth_angle,
                                base_azimuth_angle + azimuth_angle_diff,
                                interpolate_ratio,
                            ) % (2.0 * PI);
                            let vertical_angle = vertical_degree.to_radians();
                            let laser_timestamp = firing_timestamp + laser_time_offset;
                            let last_return_point = {
                                make_point(
                                    last_distance,
                                    vertical_angle,
                                    azimuth_angle,
                                    *vertical_correction,
                                )
                            };
                            let strongest_signal_point = {
                                make_point(
                                    strongest_distance,
                                    vertical_angle,
                                    azimuth_angle,
                                    *vertical_correction,
                                )
                            };
                            let timestamp_ns = (laser_timestamp * 1000.0) as u64;
                            let point = Timestamped {
                                value: (last_return_point, strongest_signal_point),
                                timestamp_ns,
                            };
                            point
                        })
                        .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>();

                VelodynePointList::DualReturn(points)
            }
        };

        Ok(points)
    }
}

/// A struct that converts and distributes raw packets into frames.
#[derive(Debug, Clone)]
pub struct FrameConverter {
    pcd_converter: PointCloudConverter,
    period_per_frame_us: f64, // in milliseconds
    rpm: u64,
    check_timestamp: bool,
    state_opt: Option<FrameConverterState>,
}

impl FrameConverter {
    pub fn new(rpm: u64, check_timestamp: bool, config: Config) -> Fallible<Self> {
        ensure!(
            rpm > 0 && (rpm % 60 == 0),
            "rpm must be positive and be multiple of 60"
        );

        let pcd_converter = PointCloudConverter::new(config);
        let period_per_frame_us = (rpm as f64).recip() * 60_000_000.0;
        let converter = Self {
            pcd_converter,
            rpm,
            period_per_frame_us,
            check_timestamp,
            state_opt: None,
        };
        Ok(converter)
    }

    pub fn firings_per_rev(&self) -> f64 {
        60_000_000.0 / (self.rpm as f64 * FIRING_PERIOD)
    }

    pub fn push_packet(&mut self, packet: &Packet) -> Fallible<Vec<Frame>> {
        let mut output_frames = vec![];
        let points = self.pcd_converter.packet_to_points(packet)?;

        if let Some(state) = &self.state_opt {
            let timestamp = packet.timestamp;
            let current_ts = timestamp as u64 * 1000;
            let previous_ts = state.last_point_timestamp_ns;

            if self.check_timestamp {
                ensure!(
                    current_ts >= previous_ts,
                    "timestamp of input packet is less than that of previous packet"
                );
            } else if current_ts < previous_ts {
                warn!("timestamp of input packet is less than that of previous packet (current = {}ns, previous = {}ns)", current_ts, previous_ts);
            }
        }

        for point in points.into_iter() {
            let timestamp = point.timestamp_ns();
            let mode = point.mode();
            let azimuth_angle = {
                let angle = point.azimuth_angle();
                // convert back to lidar's rotor angle
                let angle = std::f64::consts::FRAC_PI_2 - angle;
                if angle < 0.0 {
                    angle + std::f64::consts::PI * 2.0
                } else {
                    angle
                }
            };

            match self.state_opt.take() {
                Some(state) => {
                    let FrameConverterState {
                        last_azimuth_angle,
                        last_point_timestamp_ns: _,
                        last_frame_timestamp_ns,
                        mut frame_opt,
                    } = state;

                    // determine whether to complete current frame
                    let if_complete_curr_frame = {
                        let case1 = azimuth_angle < last_azimuth_angle;
                        let timestamp_diff = timestamp - last_frame_timestamp_ns;
                        let case2 = timestamp_diff as f64 >= self.period_per_frame_us * 1000.0;
                        case1 || case2
                    };

                    if if_complete_curr_frame {
                        if let Some(frame) = frame_opt.take() {
                            output_frames.push(frame);
                        }
                    }

                    let new_state = match frame_opt.take() {
                        Some(mut frame) => {
                            frame.points.push(point)?;

                            let new_state = FrameConverterState {
                                last_azimuth_angle: azimuth_angle,
                                last_frame_timestamp_ns,
                                last_point_timestamp_ns: timestamp,
                                frame_opt: Some(frame),
                            };
                            new_state
                        }
                        None => {
                            let mut points = VelodynePointList::new(mode);
                            points.push(point)?;

                            let frame = Frame { points };
                            let new_state = FrameConverterState {
                                last_azimuth_angle: azimuth_angle,
                                last_frame_timestamp_ns: timestamp,
                                last_point_timestamp_ns: timestamp,
                                frame_opt: Some(frame),
                            };
                            new_state
                        }
                    };

                    self.state_opt = Some(new_state);
                }
                None => {
                    let mut points = VelodynePointList::new(mode);
                    points.push(point)?;

                    let frame = Frame { points };
                    let state = FrameConverterState {
                        last_azimuth_angle: azimuth_angle,
                        last_point_timestamp_ns: timestamp,
                        last_frame_timestamp_ns: timestamp,
                        frame_opt: Some(frame),
                    };
                    self.state_opt = Some(state);
                }
            }
        }

        Ok(output_frames)
    }
}

#[derive(Debug, Clone)]
struct FrameConverterState {
    pub last_azimuth_angle: f64,
    pub last_point_timestamp_ns: u64,
    pub last_frame_timestamp_ns: u64,
    pub frame_opt: Option<Frame>,
}

/// The frame of points returned by [FrameConverter].
#[derive(Debug, Clone)]
pub struct Frame {
    pub points: VelodynePointList,
}

// References
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L312
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L396