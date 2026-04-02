//! Kalman filter implementation for 3D state estimation.
//!
//! This module provides the `Kalman` struct, which merges IMU and GPS measurements
//! to estimate the position and velocity of a moving object.

use crate::DT;
use crate::SIGMA_ACC;
use crate::SIGMA_GPS;
use crate::SIGMA_GYRO;
use crate::utils::euler_forward;

use nalgebra::{Matrix3, Matrix3x6, Matrix6, Matrix6x3, Vector3, Vector6};

/// A standard Kalman Filter tracking 3D position and velocity.
///
/// State vector `x` is `(6×1)`: `[px, py, pz, vx, vy, vz]^T`
///
/// Constants:
/// * `SIGMA_ACC`  — accelerometer noise std dev (`m/s²`)
/// * `SIGMA_GYRO` — gyroscope noise std dev (`rad/s`)
/// * `SIGMA_GPS`  — GPS position noise std dev (`m`)
/// * `DT`         — nominal time step (`s`)
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
}

impl Kalman {
	/// Initializes a new Kalman filter.
	///
	/// Uses the given initial position `pos`, velocity scalar `vel_mps` (in `m/s`),
	/// and Euler angles `dir` to build the initial state vector.
	pub fn new(pos: Vector3<f64>, vel_mps: f64, dir: Vector3<f64>) -> Self {
		// Convert heading direction to a unit vector using Euler angles,
		// then decompose the initial speed into its x/y/z components.
		// Initial state vector: x = [px, py, pz, vx, vy, vz]^T
		let h = euler_forward(dir);
		let x = Vector6::new(
			pos.x,
			pos.y,
			pos.z,
			h.x * vel_mps,
			h.y * vel_mps,
			h.z * vel_mps,
		);

		// Initial covariance matrix P:
		// Position variance is assumed negligible (GPS fix), velocity
		// variance is estimated from gyro and accelerometer noise:
		//   var(v) = sigma_gyro^2 + sigma_acc^2 * dt
		let vel_var = SIGMA_GYRO.powi(2) + SIGMA_ACC.powi(2) * DT;

		let mut p = Matrix6::zeros();
		p[(3, 3)] = vel_var;
		p[(4, 4)] = vel_var;
		p[(5, 5)] = vel_var;

		let mut f = Matrix6::identity();
		f[(0, 3)] = DT;
		f[(1, 4)] = DT;
		f[(2, 5)] = DT;
		
		let f_t = f.transpose();

		let s2 = SIGMA_ACC * SIGMA_ACC;
		let (dt2, dt3, dt4) = (DT * DT, DT.powi(3), DT.powi(4));
		let mut q = Matrix6::zeros();
		for i in 0..3 {
			q[(i, i)] = s2 * dt4 / 4.0;
			q[(i, i + 3)] = s2 * dt3 / 2.0;
			q[(i + 3, i)] = s2 * dt3 / 2.0;
			q[(i + 3, i + 3)] = s2 * dt2;
		}

		Kalman { x, p, f, f_t, q }
	}

	/// Predicts the next state given a measured acceleration `a` from the IMU.
	///
	/// Control input: integrate IMU acceleration over `DT`.
	/// `Δp = 0.5 * a * DT²`, `Δv = a * DT`
	pub fn predict(&mut self, a: Vector3<f64>) {
		// Control input: integrate IMU acceleration over DT.
		// Δp = 0.5 * a * DT²,  Δv = a * DT
		// bu = B*u where u = a (measured acceleration vector)
		let bu = Vector6::new(
			0.5 * a.x * DT * DT,
			0.5 * a.y * DT * DT,
			0.5 * a.z * DT * DT,
			a.x * DT,
			a.y * DT,
			a.z * DT,
		);

		// Predicted state:      x̂⁻ = F * x̂ + B*u
		self.x = self.f * self.x + bu;
		// Predicted covariance: P⁻ = F * P * Fᵀ + Q
		self.p = self.f * self.p * self.f_t + self.q;
	}

	/// Updates the state estimation with a new GPS position measurement `gps`.
	///
	/// Fuses the predicted position with the observed `gps` coordinates to produce
	/// an optimal state estimate using the optimal Kalman gain.
	pub fn update(&mut self, gps: Vector3<f64>) {
		// Observation matrix H: maps the full state [p, v] to the observed
		// position only.  H = [ I_3 | 0_3 ]  (3×6)
		let h = Matrix3x6::<f64>::new(
			1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
		);

		// Measurement noise covariance R = sigma_gps^2 * I_3
		let r: Matrix3<f64> = Matrix3::identity() * (SIGMA_GPS * SIGMA_GPS);

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

	/// Returns the current estimated position as a 3D vector.
	pub fn position(&self) -> Vector3<f64> {
		Vector3::new(self.x[0], self.x[1], self.x[2])
	}
}
