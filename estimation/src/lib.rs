pub mod complementary;
pub mod ekf;
pub mod madgwick;

use anyhow::Error;
use csv::WriterBuilder;
use nalgebra::{Matrix, Matrix3, Matrix4, Rotation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::path::PathBuf;
use std::time::Duration;

/// Various attitude estimates using different filtering techniques, all given
/// as unit quaternions.
#[derive(Debug, Serialize)]
pub struct FilteredEstimates {
    pub accel_mag: UnitQuaternion<f32>,
    pub complementary: UnitQuaternion<f32>,
    pub ekf: UnitQuaternion<f32>,
    pub madgwick: UnitQuaternion<f32>,
}

/// Struct holding raw imu measurements
#[derive(Clone, Debug, Default, Deserialize, Serialize, PartialEq)]
pub struct RawImuData {
    #[serde(
        serialize_with = "serialize_duration",
        deserialize_with = "deserialize_duration"
    )]
    pub time: Duration,
    // Accels in milli-gauss
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    // Angular velocities in deg/s
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    // Mag values in milli-G's
    pub mx: f32,
    pub my: f32,
    pub mz: f32,
}

fn serialize_duration<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    let secs = duration.as_secs_f64();
    serializer.serialize_f64(secs)
}

fn deserialize_duration<'de, D>(deserializer: D) -> Result<Duration, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let secs = f64::deserialize(deserializer)?;
    Ok(Duration::from_secs_f64(secs))
}

impl RawImuData {
    pub fn accel(&self) -> Vector3<f32> {
        Vector3::new(self.ax, self.ay, self.az)
    }

    /// Returns a vector of angular velocities in rads/s
    pub fn gyro(&self) -> Vector3<f32> {
        Vector3::new(
            self.gx.to_radians(),
            self.gy.to_radians(),
            self.gz.to_radians(),
        )
    }

    pub fn mag(&self) -> Vector3<f32> {
        Vector3::new(self.mx, self.my, self.mz)
    }
}

pub fn read_csv(file_path: &PathBuf) -> Result<Vec<RawImuData>, Error> {
    let file = File::open(file_path)?;
    let mut rdr = csv::Reader::from_reader(file);

    // Deserialize each record into a data struct
    let mut data = Vec::new();
    for result in rdr.deserialize() {
        let record: RawImuData = result?;
        data.push(record);
    }

    Ok(data)
}

pub fn write_csv(file_path: &PathBuf, data: &[RawImuData]) -> Result<(), Error> {
    let file = File::create(file_path)?;
    let mut writer = WriterBuilder::new().has_headers(true).from_writer(file);
    for record in data {
        writer.serialize(record)?;
    }
    writer.flush()?;
    Ok(())
}

/// Returns (roll, pitch) in radians, estimated from accelerometer measurements only.
fn roll_pitch_from_gravity(raw: &RawImuData) -> (f32, f32) {
    let acc = raw.accel().normalize();
    let roll = acc.y.atan2(acc.z);
    let pitch = -acc.x.atan2((acc.y.powi(2) + acc.z.powi(2)).sqrt());
    (roll, pitch)
}

/// Returns the yaw (heading) angle from the magnetic field, given a roll and
/// pitch value (in radians).
pub fn heading_from_mag(raw: &RawImuData, roll: f32, pitch: f32) -> f32 {
    let mag = raw.mag().normalize();
    let by = mag.y * roll.cos() - mag.z * roll.sin();
    let bx = mag.x * pitch.cos() + pitch.sin() * (mag.y * roll.sin() + mag.z * roll.cos());
    -by.atan2(bx)
}

// Returns the orientation estimate based only on accelerometer and magnetometer
// readings.
pub fn orientation_from_accel_mag_angles(raw: &RawImuData) -> UnitQuaternion<f32> {
    let (r, p) = roll_pitch_from_gravity(raw);
    let y = heading_from_mag(raw, r, p);
    UnitQuaternion::<f32>::from_euler_angles(r, p, y)
}

// A simpler way of computing the orientation estimate based on accelerometer
// and magnetometer readings.
pub fn orientation_accel_mag(raw: &RawImuData) -> UnitQuaternion<f32> {
    let acc = raw.accel().normalize();
    let mag = raw.mag().normalize();

    // Down x Mag = East
    // East x Down = North
    let y_axis = acc.cross(&mag).normalize();
    let x_axis = y_axis.cross(&acc).normalize();

    // Construct the rotation matrix and convert to quaternion
    let rot = Rotation3::from_matrix(&Matrix3::from_columns(&[x_axis, y_axis, acc]));
    UnitQuaternion::from_rotation_matrix(&rot).inverse()
}

/// Generates a 3x3 skew-symmetric matrix for a given 3D vector.
pub fn skew<T: nalgebra::Scalar + Copy + num_traits::Zero + std::ops::Neg<Output = T>>(
    v: &Vector3<T>,
) -> Matrix3<T> {
    Matrix3::new(
        T::zero(),
        -v.z,
        v.y,
        v.z,
        T::zero(),
        -v.x,
        -v.y,
        v.x,
        T::zero(),
    )
}

/// Converts magnetic declination and inclination angles to a North-East-Down
/// (NED) unit vector representing the direction of the magnetic field at a
/// particular location. Note the convetions used here for angle signs do not
/// align with the NED right-handed frame.
///
/// Ref: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
/// Declination (+E/-W) is 12° 58' 50" in SF in 2024
/// Inclination (+D/-U) is 61° 8' 25" in SF in 2024
fn mag_model_to_ned(heading_deg: f32, elevation_deg: f32) -> Vector3<f32> {
    let _az = heading_deg.to_radians();
    let el = elevation_deg.to_radians();
    // let n = el.cos() * az.cos();
    // let e = el.cos() * az.sin();
    // let d = el.sin();

    // Use AHRS lib convention where we only consider magnetic dip angle
    let n = el.cos();
    let e = 0_f32;
    let d = el.sin();

    let v = Vector3::new(n, e, d);
    v.normalize()
}

/// Type alias for a 4x1 column vector with a generic scalar type `T`, used to
/// represent a quaternion. Quaternion elements are ordered as (w, i, j, k).
///
/// Note: Using this rather than nalgebra's built-in Vector4 alias because
/// nalgebra's alias uses labels (x,y,z,w) which follow a different ordering
/// scheme than ours, so this avoids confusion.
pub type Vec4<T> =
    Matrix<T, nalgebra::Const<4>, nalgebra::Const<1>, nalgebra::ArrayStorage<T, 4, 1>>;

/// Converts a `UnitQuaternion<T>` to a `Vec4<T>` in (w, i, j, k) ordering.
pub fn quat_to_vec4<T: nalgebra::RealField + Copy>(q: &UnitQuaternion<T>) -> Vec4<T> {
    Vec4::from_vec(vec![q.w, q.i, q.j, q.k])
}

/// Converts a `Vec4<T>` in (w, i, j, k) ordering back to a `UnitQuaternion<T>`.
/// Assumes the input vector has valid values for constructing a unit quaternion.
pub fn vec4_to_quat<T: nalgebra::RealField + Copy>(v: &Vec4<T>) -> UnitQuaternion<T> {
    assert!(
        v.nrows() == 4 && v.ncols() == 1,
        "Input must be a 4x1 column vector."
    );
    let w = v[0];
    let i = v[1];
    let j = v[2];
    let k = v[3];
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, i, j, k))
}

/// Returns the 4x4 matrix needed to map angular rate w to a delta quaternion update.
/// The bottom right is a 3x3 skew-symmetric matrix.
///
/// OMEGA = | 0  -w^T   |
///         | w  -|w|_x |
pub fn omega(w: &Vector3<f32>) -> Matrix4<f32> {
    Matrix4::new(
        0., -w.x, -w.y, -w.z, // r0
        w.x, 0., w.z, -w.y, // r1
        w.y, -w.z, 0., w.x, // r2
        w.z, w.y, -w.x, 0., // r3
    )
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn test_csv_serialization() {
        let test_data = vec![
            RawImuData {
                time: Duration::from_secs_f64(1.234567),
                ax: 0.1,
                ay: 0.2,
                az: 0.3,
                gx: 1.1,
                gy: 1.2,
                gz: 1.3,
                mx: 0.01,
                my: 0.02,
                mz: 0.03,
            },
            RawImuData {
                time: Duration::from_secs_f64(2.345678),
                ax: 0.4,
                ay: 0.5,
                az: 0.6,
                gx: 2.1,
                gy: 2.2,
                gz: 2.3,
                mx: 0.04,
                my: 0.05,
                mz: 0.06,
            },
        ];

        let file_path = PathBuf::from("test_imu_data.csv");
        write_csv(&file_path, &test_data).expect("Failed to write CSV");

        let read_data = read_csv(&file_path).expect("Failed to read CSV");
        assert_eq!(test_data, read_data);

        // Cleanup
        fs::remove_file(file_path).expect("Failed to delete test file");
    }
}
