use crate::DEST;
use nalgebra::Vector3;
use std::fs::OpenOptions;
use std::io::{self, Write};

pub fn euler_forward(e: Vector3<f64>) -> Vector3<f64> {
	Vector3::new(e.y.cos() * e.z.cos(), e.y.cos() * e.z.sin(), -e.y.sin())
}

pub fn set_output(output: bool) -> Result<Box<dyn Write>, io::Error> {
	if output {
		let f = OpenOptions::new()
			.create(true)
			.write(true)
			.truncate(true)
			.open("positions.csv")?;
		return Ok(Box::new(f));
	} else {
		return Ok(Box::new(io::stdout()));
	};
}

pub fn log_position(
	pos: Vector3<f64>,
	debug: bool,
	socket: &std::net::UdpSocket,
	writer: &mut Box<dyn Write>,
) -> io::Result<()> {
	let msg = format!("{} {} {}", pos.x, pos.y, pos.z);

	if debug {
		println!(">> {}", msg);
	}

	socket.send_to(msg.as_bytes(), DEST)?;
	writeln!(writer, "{:.6},{:.6},{:.6}", pos.x, pos.y, pos.z)?;
	Ok(())
}
