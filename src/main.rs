use nalgebra::{Matrix3, Matrix3x6, Matrix6, Matrix6x3, Vector3, Vector6};
use std::io;
use std::net::UdpSocket;
use std::time::Duration;

const SIGMA_ACC: f64 = 1e-3;
const SIGMA_GYRO: f64 = 1e-2;
const SIGMA_GPS: f64 = 1e-1;
const DT: f64 = 0.01;

#[derive(Debug)]
enum Packet {
	Start,
	End,
	Position(Vector3<f64>),
	Speed(f64),
	Acceleration(Vector3<f64>),
	Direction(Vector3<f64>),
	Ignore,
}

struct Kalman {
	x: Vector6<f64>,
	p: Matrix6<f64>,
}

impl Kalman {
	fn new(pos: Vector3<f64>, vel_mps: f64, dir: Vector3<f64>) -> Self {
		let h = euler_forward(dir);
		let x = Vector6::new(
			pos.x,
			pos.y,
			pos.z,
			h.x * vel_mps,
			h.y * vel_mps,
			h.z * vel_mps,
		);

		let vel_var = (vel_mps * SIGMA_GYRO).powi(2);

		let mut p = Matrix6::zeros();
		p[(3, 3)] = vel_var;
		p[(4, 4)] = vel_var;
		p[(5, 5)] = vel_var;

		Kalman { x, p }
	}

	fn predict(&mut self, a: Vector3<f64>, dt: f64) {
		let mut f = Matrix6::identity();
		f[(0, 3)] = dt;
		f[(1, 4)] = dt;
		f[(2, 5)] = dt;

		let bu = Vector6::new(
			0.5 * a.x * dt * dt,
			0.5 * a.y * dt * dt,
			0.5 * a.z * dt * dt,
			a.x * dt,
			a.y * dt,
			a.z * dt,
		);

		let s2 = SIGMA_ACC * SIGMA_ACC;
		let (dt2, dt3, dt4) = (dt * dt, dt.powi(3), dt.powi(4));
		let mut q = Matrix6::zeros();
		for i in 0..3 {
			q[(i, i)] = s2 * dt4 / 4.0;
			q[(i, i + 3)] = s2 * dt3 / 2.0;
			q[(i + 3, i)] = s2 * dt3 / 2.0;
			q[(i + 3, i + 3)] = s2 * dt2;
		}

		self.x = f * self.x + bu;
		self.p = f * self.p * f.transpose() + q;
	}

	fn update(&mut self, gps: Vector3<f64>) {
		let h = Matrix3x6::<f64>::new(
			1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
		);

		let r: Matrix3<f64> = Matrix3::identity() * (SIGMA_GPS * SIGMA_GPS);

		let y = gps - h * self.x;

		let s = h * self.p * h.transpose() + r;

		if let Some(s_inv) = s.try_inverse() {
			let k: Matrix6x3<f64> = self.p * h.transpose() * s_inv;

			self.x += k * y;

			let i_kh = Matrix6::identity() - k * h;
			self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
		}
	}

	fn position(&self) -> Vector3<f64> {
		Vector3::new(self.x[0], self.x[1], self.x[2])
	}
}

fn main() -> io::Result<()> {
	let debug = std::env::args().any(|a| a == "--debug");

	let socket = UdpSocket::bind("127.0.0.1:0")?;
	socket.set_read_timeout(Some(Duration::from_secs(5)))?;

	let dest = "127.0.0.1:4242";
	if debug {
		println!(">> READY");
	}
	socket.send_to(b"READY", dest)?;

	let mut buf = [0u8; 1024];
	let mut kalman: Option<Kalman> = None;

	loop {
		loop {
			match recv(&socket, &mut buf[..], debug)? {
				Packet::Start => break,
				_ => continue,
			}
		}

		let mut block: Vec<Packet> = Vec::new();
		loop {
			match recv(&socket, &mut buf[..], debug)? {
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
			let msg = format!("{} {} {}", pos.x, pos.y, pos.z);
			if debug {
				println!(">> {}", msg);
			}
			socket.send_to(msg.as_bytes(), dest)?;
			println!("pos: {:.3} {:.3} {:.3}", pos.x, pos.y, pos.z);
		}
	}
}

fn euler_forward(e: Vector3<f64>) -> Vector3<f64> {
	Vector3::new(e.y.cos() * e.z.cos(), e.y.cos() * e.z.sin(), e.y.sin())
}

fn recv(socket: &UdpSocket, buf: &mut [u8], debug: bool) -> io::Result<Packet> {
	let (size, _) = socket.recv_from(buf)?;
	let raw = String::from_utf8_lossy(&buf[..size]);
	if debug {
		println!("<< {}", raw.trim().replace('\n', "\\n"));
	}
	Ok(parse(raw.trim()))
}

fn parse(raw: &str) -> Packet {
	if raw == "GOODBYE." {
		std::process::exit(0);
	}
	if raw == "MSG_START" {
		return Packet::Start;
	}
	if raw == "MSG_END" {
		return Packet::End;
	}

	let (header, values) = match raw.split_once('\n') {
		Some(pair) => pair,
		None => return Packet::Ignore,
	};
	let (_, key) = match header.split_once(']') {
		Some(pair) => pair,
		None => return Packet::Ignore,
	};

	let vals: Vec<f64> = values
		.lines()
		.filter_map(|l| l.trim().parse().ok())
		.collect();

	match key.trim() {
		"TRUE POSITION" | "POSITION" => vec3(&vals).map(Packet::Position).unwrap_or(Packet::Ignore),
		"ACCELERATION" => vec3(&vals)
			.map(Packet::Acceleration)
			.unwrap_or(Packet::Ignore),
		"DIRECTION" => vec3(&vals).map(Packet::Direction).unwrap_or(Packet::Ignore),
		"SPEED" => vals
			.first()
			.copied()
			.map(Packet::Speed)
			.unwrap_or(Packet::Ignore),
		_ => Packet::Ignore,
	}
}

fn vec3(v: &[f64]) -> Option<Vector3<f64>> {
	if v.len() < 3 {
		return None;
	}
	Some(Vector3::new(v[0], v[1], v[2]))
}
