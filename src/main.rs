use std::io;
use std::net::UdpSocket;
use std::time::Duration;

// ── Types ─────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy)]
struct Vec3 {
	x: f64,
	y: f64,
	z: f64,
}

#[derive(Debug)]
enum Packet {
	Start,
	End,
	Position(Vec3),
	Speed(f64),
	Acceleration(Vec3),
	Direction(Vec3),
	Ignore,
}

#[derive(Debug)]
struct State {
	position: Vec3,
	speed_mps: f64,
	direction: Vec3,
	acceleration: Vec3,
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

	let mut buf = [0u8; 4096];
	let mut state: Option<State> = None;

	loop {
		// 1. Wait for MSG_START
		loop {
			match recv(&socket, &mut buf, debug)? {
				Packet::Start => break,
				_ => continue,
			}
		}

		// 2. Collect packets until MSG_END
		let mut block: Vec<Packet> = Vec::new();
		loop {
			match recv(&socket, &mut buf, debug)? {
				Packet::End => break,
				pkt => block.push(pkt),
			}
		}

		// 3. Update state (Kalman filter goes here)
		state = update(state, block);

		// 4. Send estimated position
		if let Some(ref s) = state {
			let msg = format!("{} {} {}", s.position.x, s.position.y, s.position.z);
			if debug {
				println!(">> {}", msg);
			}
			socket.send_to(msg.as_bytes(), dest)?;
			println!(
				"pos: {:.3} {:.3} {:.3}",
				s.position.x, s.position.y, s.position.z
			);
		}
	}
}

// ── State update ──────────────────────────────────────────────────────────────

const DT: f64 = 0.01; // 10 ms per step

fn update(prev: Option<State>, block: Vec<Packet>) -> Option<State> {
	match prev {
		// First block: bootstrap from all available data
		None => {
			let mut pos = None;
			let mut spd = None;
			let mut acc = None;
			let mut dir = None;

			for pkt in block {
				match pkt {
					Packet::Position(p) => pos = Some(p),
					Packet::Speed(v) => spd = Some(v),
					Packet::Acceleration(a) => acc = Some(a),
					Packet::Direction(d) => dir = Some(d),
					_ => {}
				}
			}

			Some(State {
				position: pos?,
				speed_mps: spd? / 3.6, // convert km/h -> m/s
				acceleration: acc?,
				direction: dir?,
			})
		}

		// Subsequent blocks: integrate motion + optional GPS correction
		Some(mut s) => {
			for pkt in block {
				match pkt {
					Packet::Acceleration(a) => s.acceleration = a,
					Packet::Direction(d) => s.direction = d,
					// GPS arrives every ~3 s — TODO: replace with Kalman correction
					Packet::Position(p) => s.position = p,
					_ => {}
				}
			}

			// TODO: Kalman predict step
			// Interpret `direction` as Euler angles (roll, pitch, yaw).
			// Compute forward heading, rotate body-frame acceleration to world-frame,
			// then integrate speed (m/s) and position.
			let heading = euler_forward(s.direction);
			let acc_world = rotate_euler_to_world(s.direction, s.acceleration);
			let acc_along = dot(&acc_world, &heading);

			let v = s.speed_mps; // m/s
			let v_new = v + acc_along * DT;

			s.position.x += heading.x * v * DT + 0.5 * heading.x * acc_along * DT * DT;
			s.position.y += heading.y * v * DT + 0.5 * heading.y * acc_along * DT * DT;
			s.position.z += heading.z * v * DT + 0.5 * heading.z * acc_along * DT * DT;

			s.speed_mps = v_new;

			Some(s)
		}
	}
}

// ── Parsing ───────────────────────────────────────────────────────────────────

fn recv(socket: &UdpSocket, buf: &mut [u8], debug: bool) -> io::Result<Packet> {
	let (size, _) = socket.recv_from(buf)?;
	let raw = String::from_utf8_lossy(&buf[..size]);
	if debug {
		let display = raw.trim().replace('\n', "\\n");
		println!("<< {}", display);
	}
	Ok(parse(raw.trim()))
}

fn parse(raw: &str) -> Packet {
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
		"TRUE POSITION" => vec3(&vals).map(Packet::Position).unwrap_or(Packet::Ignore),
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

fn vec3(v: &[f64]) -> Option<Vec3> {
	if v.len() < 3 {
		return None;
	}
	Some(Vec3 {
		x: v[0],
		y: v[1],
		z: v[2],
	})
}

fn dot(a: &Vec3, b: &Vec3) -> f64 {
	a.x * b.x + a.y * b.y + a.z * b.z
}

// Forward vector from Euler angles (roll, pitch, yaw)
fn euler_forward(e: Vec3) -> Vec3 {
	let pitch = e.y;
	let yaw = e.z;
	let cp = pitch.cos();
	let sp = pitch.sin();
	let cy = yaw.cos();
	let sy = yaw.sin();
	Vec3 {
		x: cp * cy,
		y: cp * sy,
		z: sp,
	}
}

// Rotate a vector from body frame to world frame using Z(yaw)*Y(pitch)*X(roll)
fn rotate_euler_to_world(e: Vec3, v: Vec3) -> Vec3 {
	let (roll, pitch, yaw) = (e.x, e.y, e.z);
	let (cr, sr) = (roll.cos(), roll.sin());
	let (cp, sp) = (pitch.cos(), pitch.sin());
	let (cy, sy) = (yaw.cos(), yaw.sin());

	let x = (cy * cp) * v.x + (cy * sp * sr - sy * cr) * v.y + (cy * sp * cr + sy * sr) * v.z;
	let y = (sy * cp) * v.x + (sy * sp * sr + cy * cr) * v.y + (sy * sp * cr - cy * sr) * v.z;
	let z = (-sp) * v.x + (cp * sr) * v.y + (cp * cr) * v.z;
	Vec3 { x, y, z }
}
