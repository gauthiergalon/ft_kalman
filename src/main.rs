mod kalman;
mod parsing;
mod socket;
mod utils;

use kalman::Kalman;
use nalgebra::Vector3;
use parsing::Packet;
use socket::{create_socket, recv};
use utils::{log_position, set_output};

use std::io::{self};

use crate::parsing::parse;

pub const SIGMA_ACC: f64 = 1e-3;
pub const SIGMA_GYRO: f64 = 1e-2;
pub const SIGMA_GPS: f64 = 1e-1;
pub const DT: f64 = 0.01;

pub const DEST: &str = "127.0.0.1:4242";

fn main() -> io::Result<()> {
	let debug = std::env::args().any(|a| a == "--debug");
	let output = std::env::args().any(|a| a == "--output");

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
					kalman = Some(Kalman::new(p, s / 3.6, d));
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
					kf.predict(a, DT);
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
