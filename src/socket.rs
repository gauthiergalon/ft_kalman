//! UDP communication utilities.
//!
//! Provides creation and packet retrieval functions to communicate with the simulator.

use crate::DEST;
use std::io;
use std::net::UdpSocket;
use std::time::Duration;

/// Creates, binds and returns a new UDP socket to listen for incoming packets.
///
/// Sends an initial `"READY"` payload back to `DEST` when successfully bound.
pub fn create_socket(debug: bool) -> io::Result<UdpSocket> {
	let socket = UdpSocket::bind("127.0.0.1:0")?;
	socket.set_read_timeout(Some(Duration::from_secs(1)))?;

	if debug {
		println!(">> READY");
	}
	socket.send_to(b"READY", DEST)?;
	Ok(socket)
}

/// Receives an incoming packet over the specified socket and interprets it as a UTF-8 string.
pub fn recv(socket: &UdpSocket, buf: &mut [u8], debug: bool) -> io::Result<String> {
	let (size, _) = socket.recv_from(buf)?;
	let raw = String::from_utf8_lossy(&buf[..size]);
	if debug {
		println!("<< {}", raw.trim().replace('\n', "\\n"));
	}
	Ok(raw.trim().to_string())
}
