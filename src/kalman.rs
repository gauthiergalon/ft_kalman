//! Kalman filter for 3D motion tracking.
//!
//! This module estimates where an object is and how fast it moves by combining:
//! - IMU acceleration (high-rate, noisy)
//! - GPS position (slower, absolute reference)
//!
//! The filter state is 3D position + 3D velocity, and follows the usual
//! predict/update Kalman cycle.

use crate::DT;
use crate::SIGMA_ACC;
use crate::SIGMA_GPS;
use crate::SIGMA_GYRO;
use crate::utils::euler_forward;

use nalgebra::{Matrix3, Matrix3x6, Matrix6, Matrix6x3, Vector3, Vector6};

/// Kalman filter with a 6D state: position and velocity in 3 axes.
///
/// State vector:
/// `x = [px, py, pz, vx, vy, vz]^T` (size `6×1`)
///
/// Noise/step constants used by this implementation:
/// - `SIGMA_ACC`: accelerometer noise std dev (`m/s²`)
/// - `SIGMA_GYRO`: gyroscope noise std dev (`rad/s`)
/// - `SIGMA_GPS`: GPS position noise std dev (`m`)
/// - `DT`: nominal integration step (`s`)
pub struct Kalman {
	/// `(6×1)` — state vector `[px, py, pz, vx, vy, vz]^T`
	x: Vector6<f64>,
	/// `(6×6)` — state covariance matrix
	p: Matrix6<f64>,
	/// `(6×6)` — state transition matrix
	f: Matrix6<f64>,
	/// `(6×6)` — state transition matrix transpose
	f_t: Matrix6<f64>,
	/// `(6×6)` — process noise covariance
	q: Matrix6<f64>,
	/// Scaled GPS position noise
	sigma_gps: f64,
}

impl Kalman {
	/// Builds a Kalman filter from an initial pose and speed.
	///
	/// Inputs:
	/// - `pos`: initial position
	/// - `vel_mps`: initial speed magnitude (`m/s`)
	/// - `dir`: Euler angles used to derive the forward unit direction
	/// - `noise_scale`: global multiplier applied to sensor noise terms
	///
	/// The initial velocity components are computed as:
	/// `v0 = euler_forward(dir) * vel_mps`.
	pub fn new(pos: Vector3<f64>, vel_mps: f64, dir: Vector3<f64>, noise_scale: f64) -> Self {
		// Convert Euler orientation into a forward unit vector, then project
		// the scalar speed onto x/y/z to initialize [vx, vy, vz].
		let h = euler_forward(dir);
		let x = Vector6::new(
			pos.x,
			pos.y,
			pos.z,
			h.x * vel_mps,
			h.y * vel_mps,
			h.z * vel_mps,
		);

		// Initial covariance P:
		// - Position starts with near-zero uncertainty (trusted initial fix).
		// - Velocity uncertainty comes from motion sensors:
		//   var(v) = sigma_gyro^2 + sigma_acc^2 * dt
		let scaled_sigma_acc = SIGMA_ACC * noise_scale;
		let scaled_sigma_gps = SIGMA_GPS * noise_scale;

		let vel_var = SIGMA_GYRO.powi(2) + scaled_sigma_acc.powi(2) * DT;

		let mut p = Matrix6::zeros();
		p[(3, 3)] = vel_var;
		p[(4, 4)] = vel_var;
		p[(5, 5)] = vel_var;

		let mut f = Matrix6::identity();
		f[(0, 3)] = DT;
		f[(1, 4)] = DT;
		f[(2, 5)] = DT;
		
		let f_t = f.transpose();

		// Continuous white-noise acceleration model discretized over DT.
		// This builds the standard constant-acceleration process covariance:
		// position block ~ dt^4, cross block ~ dt^3, velocity block ~ dt^2.
		let s2 = SIGMA_ACC * SIGMA_ACC;
		let (dt2, dt3, dt4) = (DT * DT, DT.powi(3), DT.powi(4));
		let mut q = Matrix6::zeros();
		for i in 0..3 {
			q[(i, i)] = s2 * dt4 / 4.0;
			q[(i, i + 3)] = s2 * dt3 / 2.0;
			q[(i + 3, i)] = s2 * dt3 / 2.0;
			q[(i + 3, i + 3)] = s2 * dt2;
		}

		Kalman { x, p, f, f_t, q, sigma_gps: scaled_sigma_gps }
	}

	/// Prediction step using IMU acceleration.
	///
	/// Over one timestep `DT`, acceleration contributes:
	/// - `Δp = 0.5 * a * DT²`
	/// - `Δv = a * DT`
	///
	/// Then the filter applies:
	/// - state prediction: `x̂⁻ = F x̂ + B u`
	/// - covariance prediction: `P⁻ = F P Fᵀ + Q`
	pub fn predict(&mut self, a: Vector3<f64>) {
		// Build B*u explicitly from acceleration integration terms.
		let bu = Vector6::new(
			0.5 * a.x * DT * DT,
			0.5 * a.y * DT * DT,
			0.5 * a.z * DT * DT,
			a.x * DT,
			a.y * DT,
			a.z * DT,
		);

		// Predicted state x̂⁻.
		self.x = self.f * self.x + bu;
		// Predicted covariance P⁻.
		self.p = self.f * self.p * self.f_t + self.q;
	}

	/// Correction step using a GPS position sample.
	///
	/// Only position is observed by GPS, so this step:
	/// - compares predicted position vs measurement (`y = z - Hx̂⁻`)
	/// - computes Kalman gain (`K = P⁻HᵀS⁻¹`)
	/// - updates both position and velocity in the state through coupling
	/// - updates covariance with the Joseph form for better numerical stability
	pub fn update(&mut self, gps: Vector3<f64>) {
		// Observation matrix H keeps [px, py, pz] and ignores [vx, vy, vz]:
		// H = [ I_3 | 0_3 ] (shape 3x6)
		let h = Matrix3x6::<f64>::new(
			1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
		);

		// Measurement noise covariance: R = sigma_gps^2 * I_3
		let r: Matrix3<f64> = Matrix3::identity() * (self.sigma_gps * self.sigma_gps);

		// Innovation (measurement residual): y = z - H * x̂⁻
		let y = gps - h * self.x;

		// Innovation covariance: S = H * P⁻ * Hᵀ + R
		let s = h * self.p * h.transpose() + r;

		if let Some(s_inv) = s.try_inverse() {
			// Optimal Kalman gain: K = P⁻ * Hᵀ * S⁻¹
			let k: Matrix6x3<f64> = self.p * h.transpose() * s_inv;

			// Updated state estimate: x̂ = x̂⁻ + K * y
			self.x += k * y;

			// Updated covariance (Joseph form for numerical stability):
			//   P = (I - K*H) * P⁻ * (I - K*H)ᵀ + K * R * Kᵀ
			let i_kh = Matrix6::identity() - k * h;
			self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
		}
	}

	/// Current estimated position `[px, py, pz]`.
	pub fn position(&self) -> Vector3<f64> {
		Vector3::new(self.x[0], self.x[1], self.x[2])
	}
}
