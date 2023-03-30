//! Constants used by Ouster data structures and calculations.

/// Number of azimuth _ticks_ in one revolution.
pub const ENCODER_TICKS_PER_REV: u32 = 90112;

/// Number of laser returns in one column.
pub const PIXELS_PER_COLUMN: usize = 16;

/// Number of columns in one packet, where each column represents a vertical scan.
pub const COLUMNS_PER_PACKET: usize = 16;

/// Altitude angles of OS-1.
pub const OS_1_BEAM_ALTITUDE_DEGREES: [f64; 16] = [
    17.042, 16.427, 15.872, 15.324, 14.851, 14.269, 13.733, 13.18, 12.713, 12.136, 11.599, 11.067,
    10.587, 10.046, 9.503, 8.966,
];
pub const OS_1_BEAM_AZIMUTH_DEGREE_CORRECTIONS: [f64; 16] = [
    3.073,
    0.922,
    -1.238,
    -3.386,
    3.057,
    0.915,
    -1.214,
    -3.321,
    3.06,
    0.937,
    -1.174,
    -3.284,
    3.051,
    0.953,
    -1.154,
    -3.242,
];
