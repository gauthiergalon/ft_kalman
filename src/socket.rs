use crate::DEST;
use std::io;
use std::net::UdpSocket;
use std::time::Duration;

pub fn create_socket(debug: bool) -> io::Result<UdpSocket> {
	let socket = UdpSocket::bind("127.0.0.1:0")?;
	socket.set_read_timeout(Some(Duration::from_secs(1)))?;

	if debug {
		println!(">> READY");
	}
	socket.send_to(b"READY", DEST)?;
	Ok(socket)
}

pub fn recv(socket: &UdpSocket, buf: &mut [u8], debug: bool) -> io::Result<String> {
	let (size, _) = socket.recv_from(buf)?;
	let raw = String::from_utf8_lossy(&buf[..size]);
	if debug {
		println!("<< {}", raw.trim().replace('\n', "\\n"));
	}
	Ok(raw.trim().to_string())
}
