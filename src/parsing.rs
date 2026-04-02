//! Packet format parsing and deserialization logic.
//!
//! Exposes the `Packet` enum and related logic to extract sensor
//! measurements and sync signals from incoming UDP payloads.

use nalgebra::Vector3;

/// Represents a distinct piece of data or signal received over UDP.
#[derive(Debug)]
pub enum Packet {
	/// Signifies the beginning of a data transmission sequence.
	Start,
	/// Signifies the end of a transmission frame.
	End,
	/// Geographic / true position coordinate `(x, y, z)`.
	Position(Vector3<f64>),
	/// Scalar magnitude of speed `m/s`.
	Speed(f64),
	/// Measured acceleration vector `(x, y, z)`.
	Acceleration(Vector3<f64>),
	/// Euler angle rotational direction.
	Direction(Vector3<f64>),
	/// Indicates unparseable or irrelevant UDP chunk.
	Ignore,
}

/// Parses a raw message string into a recognized `Packet`.
///
/// If a `GOODBYE.` message is received, standard process exit will be triggered.
pub fn parse(raw: String) -> Packet {
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

/// Helper function to create a `Vector3` from an array/slice of floats.
/// Returns `None` if the input lacks at least 3 elements.
pub fn vec3(v: &[f64]) -> Option<Vector3<f64>> {
	if v.len() < 3 {
		return None;
	}
	Some(Vector3::new(v[0], v[1], v[2]))
}
