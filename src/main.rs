//! Main application entry point for the Kalman filter module.
//!
//! Receives IMU and GPS UDP data packets, parses them, runs them through the Kalman filter,
//! and logs/sends back the resulting estimated position.

pub mod kalman;
pub mod parsing;
pub mod socket;
pub mod utils;

use kalman::Kalman;
use nalgebra::Vector3;
use parsing::Packet;
use socket::{create_socket, recv};
use utils::{log_position, set_output};

use std::io::{self};

use crate::parsing::parse;

/// Accelerometer sensor noise standard deviation (`m/s²`)
pub const SIGMA_ACC: f64 = 1e-3;
/// Gyroscope sensor noise standard deviation (`rad/s`)
pub const SIGMA_GYRO: f64 = 1e-2;
/// GPS position measurement noise standard deviation (`m`)
pub const SIGMA_GPS: f64 = 1e-1;
/// Fixed time step delta (`s`)
pub const DT: f64 = 0.01;

/// Destination UDP socket address for processed data
pub const DEST: &str = "127.0.0.1:4242";

fn main() -> io::Result<()> {
	let debug = std::env::args().any(|a| a == "--debug");
	let output = std::env::args().any(|a| a == "--output");

	let mut noise_scale = 1.0;
	let args: Vec<String> = std::env::args().collect();
	if let Some(idx) = args.iter().position(|a| a == "--noise") {
		if let Some(val) = args.get(idx + 1) {
			if let Ok(n) = val.parse::<f64>() && n >= 0.0 {
				noise_scale = n;
			} else {
				eprintln!("Error: Invalid noise factor.");
				std::process::exit(1);
			}
		}
	}

	let socket = create_socket(debug)?;

	let mut buf: [u8; 1024] = [0u8; 1024];
	let mut kalman: Option<Kalman> = None;

	let mut writer = set_output(output)?;

	loop {
		loop {
			let msg = recv(&socket, &mut buf[..], debug)?;
			match parse(msg) {
				Packet::Start => break,
				_ => continue,
			}
		}

		let mut block: Vec<Packet> = Vec::new();
		loop {
			let msg = recv(&socket, &mut buf[..], debug)?;
			match parse(msg) {
				Packet::End => break,
				pkt => block.push(pkt),
			}
		}

		match kalman {
			None => {
				let mut pos: Option<Vector3<f64>> = None;
				let mut spd: Option<f64> = None;
				let mut dir: Option<Vector3<f64>> = None;

				for pkt in block {
					match pkt {
						Packet::Position(p) => pos = Some(p),
						Packet::Speed(v) => spd = Some(v),
						Packet::Direction(d) => dir = Some(d),
						_ => {}
					}
				}

				if let (Some(p), Some(s), Some(d)) = (pos, spd, dir) {
					kalman = Some(Kalman::new(p, s / 3.6, d, noise_scale));
				}
			}

			Some(ref mut kf) => {
				let mut gps: Option<Vector3<f64>> = None;
				let mut acc: Option<Vector3<f64>> = None;

				for pkt in block {
					match pkt {
						Packet::Acceleration(a) => acc = Some(a),
						Packet::Position(p) => gps = Some(p),
						_ => {}
					}
				}

				if let Some(a) = acc {
					kf.predict(a);
				}

				if let Some(gps_pos) = gps {
					kf.update(gps_pos);
				}
			}
		}

		if let Some(ref kf) = kalman {
			let pos = kf.position();
			log_position(pos, debug, &socket, &mut writer)?;
		}
	}
}
