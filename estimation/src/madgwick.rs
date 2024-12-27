//! Madgwick filter implementation.
//!
//! The Madgwick filter is an efficient attitude estimator, combining
//! accelerometer, gyroscope, and magnetometer readings. It shares similarities
//! with the Mahony filter, but emphasizes computational efficiency, making it
//! well-suited for real-time applications. Both filters use complementary
//! filtering to fuse sensor data, but the Madgwick filter incorporates gradient
//! descent optimization to minimize an objective function. This objective
//! function measures the difference between the measured and expected direction
//! of gravity and magnetic field vectors, ensuring accurate alignment of the
//! estimated orientation with the physical world. The gradient descent step
//! adjusts the orientation quaternion iteratively, refining the estimate.

use nalgebra::{Matrix6x4, UnitQuaternion, Vector6};

use crate::{omega, orientation_accel_mag, quat_to_vec4, vec4_to_quat, RawImuData, Vec4};

pub struct Madgwick {
    time_secs: f64,
    beta: f32,
    q: Vec4<f32>,
}

impl Madgwick {
    pub fn new(init: &RawImuData, beta: f32) -> Self {
        Self {
            beta,
            q: quat_to_vec4(&orientation_accel_mag(init)),
            time_secs: init.time.as_secs_f64(),
        }
    }

    pub fn estimate(&self) -> UnitQuaternion<f32> {
        vec4_to_quat(&self.q)
    }

    pub fn update(&mut self, raw: &RawImuData) {
        let (qw, qx, qy, qz) = (self.q[0], self.q[1], self.q[2], self.q[3]);
        let dt = (raw.time.as_secs_f64() - self.time_secs) as f32;

        let accel = raw.accel().normalize();
        let (ax, ay, az) = (accel.x, accel.y, accel.z);

        let mag = raw.mag().normalize();
        let (mx, my, mz) = (mag.x, mag.y, mag.z);

        let gyro = raw.gyro();

        // Rotate the measured magnetic field from the body frame to the global frame
        let q_hat = vec4_to_quat(&self.q);
        let h = q_hat * mag;
        let (hx, hy, hz) = (h.x, h.y, h.z);

        // B is the normalized magnetic field with inclination bias removed and
        // y-component ignored
        let bx = (hx * hx + hy * hy).sqrt();
        let bz = hz;

        // Objective function, comprised of accelerometer and magnetometer
        // components
        //
        // f_g: objective function for accelerometer
        let f1 = 2.0 * (qx * qz - qw * qy) - ax;
        let f2 = 2.0 * (qw * qx + qy * qz) - ay;
        let f3 = 2.0 * (0.5 - qx * qx - qy * qy) - az;

        // f_b: objective function for magnetometer
        let f4 = 2.0 * bx * (0.5 - qy * qy - qz * qz) + 2.0 * bz * (qx * qz - qw * qy) - mx;
        let f5 = 2.0 * bx * (qx * qy - qw * qz) + 2.0 * bz * (qw * qx + qy * qz) - my;
        let f6 = 2.0 * bx * (qw * qy + qx * qz) + 2.0 * bz * (0.5 - qx * qx - qy * qy) - mz;
        let f = Vector6::new(f1, f2, f3, f4, f5, f6);

        // Jacobian of the objective function, used in the gradient computation
        let j11 = -2.0 * qy;
        let j12 = 2.0 * qz;
        let j13 = -2.0 * qw;
        let j14 = 2.0 * qx;
        let j21 = 2.0 * qx;
        let j22 = 2.0 * qw;
        let j23 = 2.0 * qz;
        let j24 = 2.0 * qy;
        let j31 = 0.0;
        let j32 = -4.0 * qx;
        let j33 = -4.0 * qy;
        let j34 = 0.0;
        let j41 = -2.0 * bz * qy;
        let j42 = 2.0 * bz * qz;
        let j43 = -4.0 * bx * qy - 2.0 * bz * qw;
        let j44 = -4.0 * bx * qz + 2.0 * bz * qx;
        let j51 = -2.0 * bx * qz + 2.0 * bz * qx;
        let j52 = 2.0 * bx * qy + 2.0 * bz * qw;
        let j53 = 2.0 * bx * qx + 2.0 * bz * qz;
        let j54 = -2.0 * bx * qw + 2.0 * bz * qy;
        let j61 = 2.0 * bx * qy;
        let j62 = 2.0 * bx * qz - 4.0 * bz * qx;
        let j63 = 2.0 * bx * qw - 4.0 * bz * qy;
        let j64 = 2.0 * bx * qx;
        let jacobian = Matrix6x4::new(
            j11, j12, j13, j14, // Row 1
            j21, j22, j23, j24, // Row 2
            j31, j32, j33, j34, // Row 3
            j41, j42, j43, j44, // Row 4
            j51, j52, j53, j54, // Row 5
            j61, j62, j63, j64, // Row 6
        );

        // Compute the gradient "S" of the objective function
        //
        // S = J^T * f
        let s = (jacobian.transpose() * f).normalize();

        // Initial angular rate estimate based on gyro only
        let mut q_dot = 0.5 * omega(&gyro) * self.q;

        // Refine the angular rate estimate with the gradient descent step
        q_dot -= self.beta * s;

        self.q = (self.q + q_dot * dt).normalize();
        self.time_secs = raw.time.as_secs_f64();
    }
}

