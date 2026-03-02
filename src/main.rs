use nalgebra::{Matrix3, Matrix3x6, Matrix6, Matrix6x3, Rotation3, Vector3, Vector6};
use std::io;
use std::net::UdpSocket;
use std::time::Duration;

// ── Noise parameters ──────────────────────────────────────────────────────────
const SIGMA_ACC: f64 = 1e-3;
const SIGMA_GPS: f64 = 1e-1;
const DT: f64 = 0.01;

// ── Types ─────────────────────────────────────────────────────────────────────

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

// ── Kalman Filter (6D state: [px, py, pz, vx, vy, vz]) ───────────────────────
//
//  Predict:  x = F·x + B·a_world,  P = F·P·Fᵀ + Q
//  Update:   y = gps - H·x,  S = H·P·Hᵀ + R,  K = P·Hᵀ·S⁻¹
//            x = x + K·y,  P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ  (Joseph form)

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
		Kalman {
			x,
			p: Matrix6::identity(),
		}
	}

	/// Predict step — called every dt = 10 ms with world-frame acceleration
	fn predict(&mut self, a: Vector3<f64>, dt: f64) {
		// Transition matrix F = [[I3, dt·I3], [0, I3]]
		let mut f = Matrix6::identity();
		f[(0, 3)] = dt;
		f[(1, 4)] = dt;
		f[(2, 5)] = dt;

		// Control input B·u = [½·a·dt², a·dt]
		let bu = Vector6::new(
			0.5 * a.x * dt * dt,
			0.5 * a.y * dt * dt,
			0.5 * a.z * dt * dt,
			a.x * dt,
			a.y * dt,
			a.z * dt,
		);

		// Process noise Q = σ² · [[dt⁴/4·I3, dt³/2·I3], [dt³/2·I3, dt²·I3]]
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

	/// Update step — called when GPS measurement arrives
	fn update(&mut self, gps: Vector3<f64>) {
		// H = [I3 | 0]  (3×6)
		let h = Matrix3x6::<f64>::new(
			1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
		);

		let r: Matrix3<f64> = Matrix3::identity() * (SIGMA_GPS * SIGMA_GPS);

		let y = gps - h * self.x;
		let s = h * self.p * h.transpose() + r;

		if let Some(s_inv) = s.try_inverse() {
			let k: Matrix6x3<f64> = self.p * h.transpose() * s_inv;
			self.x += k * y;

			// Joseph form: (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ
			// Numerically stable — keeps P symmetric and positive semi-definite
			// even after thousands of iterations, unlike the naive (I-KH)·P form.
			let i_kh = Matrix6::identity() - k * h;
			self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
		}
	}

	fn position(&self) -> Vector3<f64> {
		Vector3::new(self.x[0], self.x[1], self.x[2])
	}
}

// ── Main loop ─────────────────────────────────────────────────────────────────

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
	let mut last_direction = Vector3::zeros();

	loop {
		// 1. Wait for MSG_START
		loop {
			match recv(&socket, &mut buf[..], debug)? {
				Packet::Start => break,
				_ => continue,
			}
		}

		// 2. Collect packets until MSG_END
		let mut block: Vec<Packet> = Vec::new();
		loop {
			match recv(&socket, &mut buf[..], debug)? {
				Packet::End => break,
				pkt => block.push(pkt),
			}
		}

		// 3. Update Kalman filter
		match kalman {
			// ── First block: initialise filter ──────────────────────────────
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
					last_direction = d;
					kalman = Some(Kalman::new(p, s / 3.6, d));
				}
			}

			// ── Subsequent blocks: predict + optional GPS update ─────────────
			Some(ref mut kf) => {
				let mut gps: Option<Vector3<f64>> = None;
				let mut acc: Option<Vector3<f64>> = None;

				for pkt in block {
					match pkt {
						Packet::Acceleration(a) => acc = Some(a),
						Packet::Direction(d) => last_direction = d,
						Packet::Position(p) => gps = Some(p),
						_ => {}
					}
				}

				if let Some(a_body) = acc {
					let a_world = rotate_euler_to_world(last_direction, a_body);
					kf.predict(a_world, DT);
				}

				if let Some(gps_pos) = gps {
					kf.update(gps_pos);
				}
			}
		}

		// 4. Send estimated position
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

// ── Geometry ──────────────────────────────────────────────────────────────────

/// Forward unit vector from Euler angles (roll, pitch, yaw) in radians
fn euler_forward(e: Vector3<f64>) -> Vector3<f64> {
	Vector3::new(e.y.cos() * e.z.cos(), e.y.cos() * e.z.sin(), e.y.sin())
}

/// Rotate a vector from body frame to world frame using Z(yaw)·Y(pitch)·X(roll)
fn rotate_euler_to_world(e: Vector3<f64>, v: Vector3<f64>) -> Vector3<f64> {
	// Rotation3::from_euler_angles(roll, pitch, yaw) builds Rz·Ry·Rx,
	// which is the standard aerospace body→world rotation.
	Rotation3::from_euler_angles(e.x, e.y, e.z) * v
}

// ── Parsing ───────────────────────────────────────────────────────────────────

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
