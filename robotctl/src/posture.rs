//! `robotctl posture` — the numbers a seated-or-standing detector would decide on, live.
//!
//! A measuring tool, not a detector: it decides nothing. The robot will one day have to tell, at
//! bring-up, whether it is standing or sitting and come up into that pose. Choosing how — and
//! where the thresholds sit — wants readings from a real robot in each pose first, so this prints
//! the candidate signals side by side and, with `--json --label`, records them for comparison.
//!
//! Every signal is computed from what `robot.state` already carries, so it works on a limp robot:
//! the encoders read with torque off, and the IMU does not care.
//!
//! - **Trunk height above the feet.** The feet's positions in the trunk frame, from the joint
//!   angles and the kinematic model, turned into the world by the IMU's attitude. Standing it is
//!   about the model's standing trunk height; sitting it should be well below. The most physical
//!   of the signals, and the one expected to separate the two best.
//! - **Trunk pitch and roll**, from the IMU. A seated duck leans back by some typical amount; a
//!   trunk near horizontal or upside down is a robot lying down, neither seated nor standing.
//! - **Leg deviation from the home pose**: the old seated-boot criterion (mean over all ten leg
//!   joints), and the same over the sagittal joints alone — hip pitch, knee, ankle — which are
//!   the ones a sit actually folds.

use duck_ipc_proto as proto;
use kinematics::Quat;

/// Leg joints in wire order ([`proto::JOINT_NAMES`]): left hip yaw … ankle, right hip yaw … ankle.
const LEG_JOINTS: [usize; 10] = [0, 1, 2, 3, 4, 10, 11, 12, 13, 14];

/// The sagittal subset of [`LEG_JOINTS`]: hip pitch, knee and ankle on each side.
const SAGITTAL_JOINTS: [usize; 6] = [2, 3, 4, 12, 13, 14];

/// One reading of every candidate signal.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct Posture {
    /// Trunk frame origin above the mean of the two feet, along gravity, metres. `None` without
    /// an IMU reading, since "above" needs to know which way is up.
    pub trunk_height_m: Option<f64>,
    /// The same with the trunk assumed level — the joints alone, for a robot with no IMU.
    pub trunk_height_level_m: f64,
    /// Nose up is positive, degrees. `None` without an IMU reading.
    pub pitch_deg: Option<f64>,
    /// Left side up is positive, degrees. `None` without an IMU reading.
    pub roll_deg: Option<f64>,
    /// Mean |angle − home| over the ten leg joints, radians — the old seated-boot criterion.
    pub leg_deviation_rad: f64,
    /// The same over hip pitch, knee and ankle only.
    pub sagittal_deviation_rad: f64,
}

/// What the model says a standing trunk's height is, for the readout's reference column.
pub fn standing_height_m() -> f64 {
    kinematics::Model::alpha().trunk_height_m()
}

/// Every signal for one state: `joints` in wire order, `quat` the IMU's trunk → world attitude.
pub fn measure(joints: &[f64], quat: Option<[f64; 4]>) -> Posture {
    let model = kinematics::Model::alpha();
    // The model's joint order is the MJCF's, not the wire's: gather the angles by name.
    let angles: Vec<f64> = model
        .joint_names()
        .map(|name| {
            proto::JOINT_NAMES
                .iter()
                .position(|wire| *wire == name)
                .and_then(|i| joints.get(i).copied())
                .unwrap_or(0.0)
        })
        .collect();
    let foot = |name: &str| {
        model
            .site(name)
            .map(|site| model.site_pose(site, &angles).pos)
            .unwrap_or([0.0; 3])
    };
    let (left, right) = (foot("left_foot"), foot("right_foot"));
    let feet = [
        (left[0] + right[0]) / 2.0,
        (left[1] + right[1]) / 2.0,
        (left[2] + right[2]) / 2.0,
    ];

    let attitude = quat.map(|[w, x, y, z]| Quat::new(w, x, y, z).normalized());
    let (trunk_height_m, pitch_deg, roll_deg) = match attitude {
        Some(q) => {
            let height = -q.rotate(feet)[2];
            let forward = q.rotate([1.0, 0.0, 0.0]);
            let left_axis = q.rotate([0.0, 1.0, 0.0]);
            (
                Some(height),
                Some(forward[2].clamp(-1.0, 1.0).asin().to_degrees()),
                Some(left_axis[2].clamp(-1.0, 1.0).asin().to_degrees()),
            )
        }
        None => (None, None, None),
    };

    let deviation = |indices: &[usize]| {
        indices
            .iter()
            .map(|&j| {
                let angle = joints.get(j).copied().unwrap_or(0.0);
                (angle - home(j)).abs()
            })
            .sum::<f64>()
            / indices.len() as f64
    };

    Posture {
        trunk_height_m,
        trunk_height_level_m: -feet[2],
        pitch_deg,
        roll_deg,
        leg_deviation_rad: deviation(&LEG_JOINTS),
        sagittal_deviation_rad: deviation(&SAGITTAL_JOINTS),
    }
}

/// The home pose's angle for a wire-order joint.
fn home(joint: usize) -> f64 {
    proto::DEFAULT_POSITION.get(joint).copied().unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// At the home pose, level, the trunk stands at about the model's standing height and every
    /// deviation is zero — the reference the other readings are compared against.
    #[test]
    fn the_home_pose_reads_as_standing_height() {
        let level = Some([1.0, 0.0, 0.0, 0.0]);
        let posture = measure(&proto::DEFAULT_POSITION, level);
        let height = posture.trunk_height_m.expect("an attitude was given");
        assert!(
            (height - standing_height_m()).abs() < 0.03,
            "home pose at {height:.3} m against a standing {:.3} m",
            standing_height_m()
        );
        assert_eq!(posture.trunk_height_level_m, height, "level is level");
        assert_eq!(posture.pitch_deg, Some(0.0));
        assert_eq!(posture.leg_deviation_rad, 0.0);
        assert_eq!(posture.sagittal_deviation_rad, 0.0);
    }

    /// Nose up is positive pitch, and a pitched trunk changes the height along gravity but not
    /// the level-trunk height, which only reads the joints.
    #[test]
    fn pitch_has_the_documented_sign() {
        // A positive turn about +y tips the nose *down* in a z-up frame with x forward, so a
        // nose-up trunk is −20° about +y.
        let half = (-20.0f64).to_radians() / 2.0;
        let nose_up = Some([half.cos(), 0.0, half.sin(), 0.0]);
        let posture = measure(&proto::DEFAULT_POSITION, nose_up);
        let pitch = posture.pitch_deg.unwrap();
        assert!((pitch - 20.0).abs() < 1e-6, "{pitch}");
        assert_ne!(posture.trunk_height_m, Some(posture.trunk_height_level_m));
    }

    /// Without an IMU there is no "up": the gravity-based signals say so rather than guessing.
    #[test]
    fn no_imu_means_no_gravity_signals() {
        let posture = measure(&proto::DEFAULT_POSITION, None);
        assert_eq!(posture.trunk_height_m, None);
        assert_eq!(posture.pitch_deg, None);
        assert_eq!(posture.roll_deg, None);
    }
}
