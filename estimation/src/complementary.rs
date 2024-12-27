use crate::{heading_from_mag, roll_pitch_from_gravity, RawImuData};
use nalgebra::{UnitQuaternion, Vector3};
use std::time::Duration;

#[derive(Clone, Debug)]
pub struct ComplementaryFilter {
    /// Filter gain, between [0, 1]. 0 uses only accel / mag measurements, and 1 uses only gyro.
    alpha: f32,

    /// Estimated attitude (radians).
    estimate: Vector3<f32>,

    /// Previous timestamp
    timestamp: Option<Duration>,
}

impl ComplementaryFilter {
    pub fn new(init: &RawImuData, alpha: f32) -> Self {
        let (roll, pitch) = roll_pitch_from_gravity(init);
        let yaw = heading_from_mag(init, roll, pitch);
        let estimate = Vector3::new(roll, pitch, yaw);
        Self {
            alpha,
            estimate,
            timestamp: None,
        }
    }
    
    pub fn estimate(&self) -> UnitQuaternion<f32> {
        let (r, p, y) = (self.estimate.x, self.estimate.y, self.estimate.z);
        UnitQuaternion::from_euler_angles(r, p, y)
        // ypr_to_quaternion(y, p, r)
    }

    pub fn update(&mut self, raw: &RawImuData) {
        // Compute first orientation estimate directly from accel / mag sensors
        let (roll, pitch) = roll_pitch_from_gravity(raw);
        
        // Estimate yaw from magnetic field, using the roll and pitch angles from the previous estimate
        let yaw = heading_from_mag(raw, self.estimate.x, self.estimate.y);
        let est_am = Vector3::new(roll, pitch, yaw);

        // Compute second orientation estimate by integrating gyro rates from previous estimate
        let dt = self
            .timestamp
            .map(|ct| raw.time.saturating_sub(ct).as_secs_f32())
            .unwrap_or(0.);
        let est_w = self.estimate + dt * raw.gyro();

        // Update current state
        self.estimate = self.alpha * est_w + (1. - self.alpha) * est_am;
        self.timestamp = Some(raw.time)
    }
}
