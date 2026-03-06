use crate::DT;
use crate::SIGMA_ACC;
use crate::SIGMA_GPS;
use crate::SIGMA_GYRO;
use crate::utils::euler_forward;

use nalgebra::{Matrix3, Matrix3x6, Matrix6, Matrix6x3, Vector3, Vector6};

pub struct Kalman {
	x: Vector6<f64>,
	p: Matrix6<f64>,
}

impl Kalman {
	pub fn new(pos: Vector3<f64>, vel_mps: f64, dir: Vector3<f64>) -> Self {
		let h = euler_forward(dir);
		let x = Vector6::new(
			pos.x,
			pos.y,
			pos.z,
			h.x * vel_mps,
			h.y * vel_mps,
			h.z * vel_mps,
		);

		let vel_var = SIGMA_GYRO.powi(2) + SIGMA_ACC.powi(2) * DT;

		let mut p = Matrix6::zeros();
		p[(3, 3)] = vel_var;
		p[(4, 4)] = vel_var;
		p[(5, 5)] = vel_var;

		Kalman { x, p }
	}

	pub fn predict(&mut self, a: Vector3<f64>, dt: f64) {
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

	pub fn update(&mut self, gps: Vector3<f64>) {
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

	pub fn position(&self) -> Vector3<f64> {
		Vector3::new(self.x[0], self.x[1], self.x[2])
	}
}
