#![allow(non_snake_case)]

use crate::{
    mag_model_to_ned, omega, orientation_accel_mag, quat_to_vec4, skew, vec4_to_quat, RawImuData,
    Vec4,
};
use nalgebra::{
    Matrix3, Matrix4, Matrix4x3, Matrix6, Matrix6x1, Matrix6x4, UnitQuaternion, Vector3,
};

pub struct Ekf {
    /// Current measurement timestamp
    time_secs: f64,
    /// State vector: a 4-vector quaternion representation of attitude
    q: Vec4<f32>,
    /// Estimated covariance matrix (after measurement correction)
    P: Matrix4<f32>,
    /// Sensor noise variances. Making the strong assumption here that noise is uniform
    /// across each axis and isotropic. 
    var_w: f32, 
    var_a: f32,
    var_m: f32,
    // Magnetic field reference at the desired location
    mag_ref: Vector3<f32>,
}

impl Ekf {
    pub fn new(init: &RawImuData) -> Self {
        Self {
            time_secs: init.time.as_secs_f64(),
            q: quat_to_vec4(&orientation_accel_mag(init)),
            P: Matrix4::identity(),
            var_w: 0.09,
            var_a: 0.25,
            var_m: 0.81,
            // mag_ref: mag_model_to_ned(12.981, 61.140),
            // We correct for the inclination on the embedded board during initialization.
            mag_ref: mag_model_to_ned(0., 0.), // [1, 0, 0] NED
        }
    }

    pub fn estimate(&self) -> UnitQuaternion<f32> {
        vec4_to_quat(&self.q)
    }

    pub fn update(&mut self, raw: &RawImuData) {
        // Update measurements
        let dt = (raw.time.as_secs_f64() - self.time_secs) as f32;
        // Treat gyro measurements as the control input "u"
        let gyr = raw.gyro();
        let acc = raw.accel().normalize();
        let mag = raw.mag().normalize();

        // ----- Prediction Step -----
        //
        // Here we integrate the angular rate to compute x_hat, the intermediate
        // state update prediction. We use only a first order linear update.
        // This corresponds to the nonlinear function "f()" in the typical EKF
        // description.
        let q_dot = 0.5 * omega(&gyr) * self.q;
        let q_hat = self.q + q_dot * dt;

        // Jacobian df/dq about linearized function f() at setpoint (Fundamental matrix)
        let F = self.dfdq(gyr, dt);
        // Jacobian df/dw
        let W = self.dfdw(dt);
        // Process noise covariance matrix
        let Q = self.var_w * W * W.transpose();
        // Prediction covariance matrix
        let P_hat = F * self.P * F.transpose() + Q;

        // ----- Correction step -----
        //
        // Sensor measurement model: y = h(q_hat) gives the expected
        // measurements given the predicted state. We first rotate the sensor
        // measurements from the global (measured-in) frame to the local body
        // frame. y is a 6x1 vector formed by stacking its two components: a_hat
        // and m_hat.
        let q_hat = vec4_to_quat(&q_hat);
        let acc_ref = Vector3::new(0., 0., 1.); // Gravitational accel NED
        let acc_hat = q_hat.inverse() * acc_ref;
        let mag_hat = q_hat.inverse() * self.mag_ref;

        // Compute residuals as (measurement - prediction)
        let acc_res = acc - acc_hat;
        let mag_res = mag - mag_hat;
        let v = Matrix6x1::from_iterator(acc_res.iter().chain(mag_res.iter()).cloned());

        // H(q_hat) is the sensor measurement model Jacobian, used for
        // linearization about the current operating point.
        let H = self.dhdq(&q_hat);

        // Measurement noise covariance matrix R
        let mut R = Matrix6::identity();
        let mut R_top_left = R.fixed_view_mut::<3, 3>(0, 0);
        R_top_left *= self.var_a;
        let mut R_bot_right = R.fixed_view_mut::<3, 3>(3, 3);
        R_bot_right *= self.var_m;

        // Update the measurement prediction covariance matrix S
        let S = H * P_hat * H.transpose() + R;

        // Compute Kalman gain
        let K = P_hat * H.transpose() * S.pseudo_inverse(1e-6).unwrap();

        dbg!(&v);
        dbg!(&K);

        // Final updates to state and covariance matrix
        let q = (quat_to_vec4(&q_hat) + K * v).normalize();
        let P = (Matrix4::identity() - K * H) * P_hat;

        // State book-keeping
        self.time_secs = raw.time.as_secs_f64();
        self.q = q;
        self.P = P;
    }

    /// Computes the Jacobian F(x_t-1, u_t) which is the partial derivative of
    /// the nonlinear function f (system model) wrt x (the state).
    fn dfdq(&self, w: Vector3<f32>, dt: f32) -> Matrix4<f32> {
        let z = 0.5 * dt * w;
        Matrix4::identity() + omega(&z)
    }

    /// Computes the Jacobian W(x_t-1, u_t) which is the partial derivative of
    /// the nonlinear function f (system model) wrt u (the control). Note that
    /// in this instance, the u term drops out.
    fn dfdw(&self, dt: f32) -> Matrix4x3<f32> {
        let q = self.q;
        0.5 * dt
            * Matrix4x3::new(
                -q.x, -q.y, -q.z, // Row 1
                q.w, -q.z, q.y, // Row 2
                q.z, q.w, -q.x, // Row 3
                -q.y, q.x, q.w, // Row 4
            )
    }

    /// Computes the measurement model Jacobian H(x_hat) which is the partial
    /// derivative of the nonlinear function h (measurement model) wrt x.
    fn dhdq(&self, q_hat: &UnitQuaternion<f32>) -> Matrix6x4<f32> {
        let q = q_hat;
        let qv = q.vector();
        let qw = q.w;
        let g = Vector3::new(0., 0., 1.); // accel gravity NED
        let r = self.mag_ref;

        let ug = skew(&g) * qv;
        let ur = skew(&r) * qv;

        let g24 = skew(&(&ug + qw * g)) + qv.dot(&g) * Matrix3::identity() - g * qv.transpose();
        let r24 = skew(&(&ur + qw * r)) + qv.dot(&r) * Matrix3::identity() - r * qv.transpose();

        let mut hg = Matrix6x4::zeros();
        hg.fixed_view_mut::<3, 1>(0, 0).copy_from(&ug);
        hg.fixed_view_mut::<3, 1>(3, 0).copy_from(&ur);
        hg.fixed_view_mut::<3, 3>(0, 1).copy_from(&g24);
        hg.fixed_view_mut::<3, 3>(3, 1).copy_from(&r24);

        2. * hg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mat_view() {
        let mut hg = Matrix6x4::zeros();
        let v1 = Vector3::new(1., 2., 3.);
        hg.fixed_view_mut::<3, 1>(3, 2).copy_from(&v1);
    }

    #[test]
    fn test_vec4_ordering() {
        let v1 = UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        dbg!(v1.as_vector());
        println!("w: {}, i: {}, j: {}, k: {}", v1.w, v1.i, v1.j, v1.k);
    }
}
