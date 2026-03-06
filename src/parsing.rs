use nalgebra::Vector3;

#[derive(Debug)]
pub enum Packet {
	Start,
	End,
	Position(Vector3<f64>),
	Speed(f64),
	Acceleration(Vector3<f64>),
	Direction(Vector3<f64>),
	Ignore,
}

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

pub fn vec3(v: &[f64]) -> Option<Vector3<f64>> {
	if v.len() < 3 {
		return None;
	}
	Some(Vector3::new(v[0], v[1], v[2]))
}
