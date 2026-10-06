//! Which camera sensor this robot has, and the handful of facts about it that `mediad` cannot ask
//! the driver for.
//!
//! **Chosen by what the media graph contains, not by configuration.** The sensor names itself in
//! the topology (`m00_b_imx219 2-0010`, `m00_b_gc2093 2-0037`), so a board needs no setting to say
//! which camera it carries, and the same board fitted with another module needs no setting changed.
//! The `[board]` key is not consulted: a board is not a camera.
//!
//! Everything else about a sensor is either read from the driver at run time or is the same for
//! every sensor here — the pinned mode is 1920×1080 raw 10-bit on both. What is in a [`Sensor`] is
//! what is left: the units its controls are in, the exposure the auto-exposure loop may spend, and
//! the optics, when anyone has measured them.
//!
//! A sensor not in [`SENSORS`] is refused by name rather than driven with another sensor's numbers.
//! Exposure in the wrong units is a picture that is black or white, and a field of view borrowed
//! from another lens is a geometry that is quietly wrong — and a consumer has no way to tell.

use crate::camera::SensorMode;

/// One camera sensor `mediad` knows how to drive.
#[derive(Debug)]
pub struct Sensor {
    /// What the sensor's entity name contains in the media graph, and what the logs call it.
    pub name: &'static str,
    /// The readout mode `pipeline` pins, as a media bus format and size.
    pub mode: SensorMode,
    pub bus_format: &'static str,
    pub exposure: Exposure,
    /// Horizontal field of view across a frame in [`Sensor::mode`], degrees — what the nominal
    /// geometry rests on. `None` when nobody has measured it, and then nominal intrinsics are not
    /// published at all.
    pub hfov_deg: Option<f64>,
    /// A solve of this sensor behind its lens, shared by every robot built with that part. `None`
    /// for a part nobody has calibrated.
    pub family: Option<fn() -> robotd_params::CameraIntrinsics>,
}

/// What the auto-exposure loop may spend, in this sensor's own units.
///
/// The two shutter caps are kept the same in **time** across sensors — 11.4 ms soft, 22.9 ms hard —
/// because that is what they are about: a walking robot's picture is not smeared at the first, and
/// the frame time is not stretched at the second. Only the line count that buys that time differs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Exposure {
    /// Shutter spent before any gain, in lines.
    pub soft_lines: f64,
    /// The longest shutter asked for, in lines. Below the frame length on every sensor here, since
    /// a driver answers a longer exposure by stretching the frame rather than clamping.
    pub hard_lines: f64,
    /// The analogue gain register value that means 1x.
    pub unity_gain: u32,
    /// Analogue gain ceiling, in multiples of 1x.
    pub max_analogue: f64,
    /// Where the sensor starts, before the loop has metered anything: a boot value leaves the
    /// picture black rather than merely dark.
    pub start_lines: u32,
    pub start_analogue: f64,
}

impl Exposure {
    /// The starting analogue gain, as the register value.
    pub fn start_gain(&self) -> u32 {
        (self.start_analogue * f64::from(self.unity_gain)) as u32
    }
}

/// The alpha robots' head camera: the IMX219 behind a ~3.05 mm M12 lens. The numbers are the
/// prototype's, which ran for months; `crate::camera` has the optics.
pub const IMX219: Sensor = Sensor {
    name: "imx219",
    mode: SensorMode {
        width: 1920,
        height: 1080,
    },
    bus_format: "SRGGB10_1X10",
    exposure: Exposure {
        // One line is ~19.05 µs in the pinned mode, which is 1766 lines long.
        soft_lines: 600.0,
        hard_lines: 1200.0,
        unity_gain: 256,
        max_analogue: 11.0,
        start_lines: 600,
        start_analogue: 4.0,
    },
    hfov_deg: Some(62.0),
    family: Some(robotd_params::CameraIntrinsics::alpha),
};

/// The beta board's head camera: GalaxyCore's GC2093 on Seeed's main board.
///
/// The mode is the driver's only one: 1920×1080 raw 10-bit at 30 fps, 1125 lines a frame (1080 plus
/// a vertical blanking of 45), so a line is ~29.6 µs and the exposure control tops out at 1121.
/// Analogue gain is 64 for 1x — read off the driver, `min=64 max=8192`. The ceiling is kept at the
/// IMX219's 11x rather than the driver's, so the loop spends noise the same way on both.
///
/// No field of view and no family solve: nobody has measured this lens yet, so the geometry is
/// unknown until a robot is calibrated.
pub const GC2093: Sensor = Sensor {
    name: "gc2093",
    mode: SensorMode {
        width: 1920,
        height: 1080,
    },
    bus_format: "SRGGB10_1X10",
    exposure: Exposure {
        soft_lines: 385.0,
        hard_lines: 773.0,
        unity_gain: 64,
        max_analogue: 11.0,
        start_lines: 385,
        start_analogue: 4.0,
    },
    hfov_deg: None,
    family: None,
};

// The GC2093's exposure control stops at 1121 lines. A hard cap above it would be refused by the
// driver, or answered with a longer frame — so it does not build.
const _: () = assert!(GC2093.exposure.hard_lines < 1121.0);

/// Every sensor this daemon can drive.
pub const SENSORS: &[&Sensor] = &[&IMX219, &GC2093];

/// The sensor an entity name belongs to, if it is one of ours.
pub fn identify(entity: &str) -> Option<&'static Sensor> {
    SENSORS
        .iter()
        .copied()
        .find(|sensor| entity.contains(sensor.name))
}

/// What one media graph holds, read off `media-ctl -p`.
#[derive(Debug, Default)]
pub struct Topology {
    /// The first sensor this daemon can drive, as its entity name and profile.
    pub ours: Option<(String, &'static Sensor)>,
    /// Entities the driver calls a sensor and that are not in [`SENSORS`] — named in the error, so
    /// a board with a camera nobody wrote a profile for says which camera that is.
    pub others: Vec<String>,
}

impl Topology {
    /// Parse `media-ctl -p` output. An entity is a header line followed by its type:
    ///
    /// ```text
    /// - entity 76: m00_b_gc2093 2-0037 (1 pad, 1 link)
    ///              type V4L2 subdev subtype Sensor flags 0
    /// ```
    pub fn read(printed: &str) -> Self {
        let mut topology = Self::default();
        let mut entity: Option<&str> = None;
        for line in printed.lines() {
            let line = line.trim_start();
            if let Some(header) = line.strip_prefix("- entity") {
                entity = header
                    .split_once(": ")
                    .map(|(_, rest)| rest.split(" (").next().unwrap_or(rest).trim())
                    .filter(|name| !name.is_empty());
                if let Some(name) = entity
                    && topology.ours.is_none()
                    && let Some(sensor) = identify(name)
                {
                    topology.ours = Some((name.to_string(), sensor));
                }
                continue;
            }
            if line.starts_with("type ")
                && line.contains("subtype Sensor")
                && let Some(name) = entity.take()
                && identify(name).is_none()
            {
                topology.others.push(name.to_string());
            }
        }
        topology
    }
}

/// The names in [`SENSORS`], for a message that says what would have been accepted.
pub fn known() -> String {
    SENSORS
        .iter()
        .map(|sensor| sensor.name)
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sensor_is_found_by_its_entity_name() {
        assert_eq!(identify("m00_b_imx219 2-0010").unwrap().name, "imx219");
        assert_eq!(identify("m00_b_gc2093 2-0037").unwrap().name, "gc2093");
        assert!(
            identify("m00_b_gc2613 2-0031").is_none(),
            "not one we drive"
        );
        assert!(identify("rkisp-isp-subdev").is_none());
    }

    /// What `media-ctl -p` printed on a beta board, cut to the entities that matter.
    const BETA: &str = "\
- entity 73: rockchip-csi2-dphy0 (2 pads, 2 links)
             type V4L2 subdev subtype Unknown flags 0
- entity 76: m00_b_gc2093 2-0037 (1 pad, 1 link)
             type V4L2 subdev subtype Sensor flags 0
             device node name /dev/v4l-subdev3
";

    #[test]
    fn the_beta_boards_camera_is_found_in_its_topology() {
        let topology = Topology::read(BETA);
        let (entity, sensor) = topology.ours.expect("the gc2093");
        assert_eq!(entity, "m00_b_gc2093 2-0037");
        assert_eq!(sensor.name, "gc2093");
        assert!(topology.others.is_empty());
    }

    /// A camera with no profile is named, rather than reported as "no camera".
    #[test]
    fn a_sensor_with_no_profile_is_named() {
        let topology = Topology::read(
            "- entity 80: m00_b_ov5647 2-0036 (1 pad, 1 link)\n\
             \x20            type V4L2 subdev subtype Sensor flags 0\n",
        );
        assert!(topology.ours.is_none());
        assert_eq!(topology.others, vec!["m00_b_ov5647 2-0036"]);
    }

    /// The caps are one decision in milliseconds, written out per sensor in lines. A sensor whose
    /// line count drifts from the time it is meant to buy is a sensor that smears or stretches its
    /// frame where the other does not.
    #[test]
    fn the_shutter_caps_buy_the_same_time_on_every_sensor() {
        let line_us = |sensor: &Sensor| match sensor.name {
            "imx219" => 19.05,
            "gc2093" => 1e6 / 30.0 / 1125.0,
            other => panic!("no line time written down for {other}"),
        };
        for sensor in SENSORS {
            let soft_ms = sensor.exposure.soft_lines * line_us(sensor) / 1000.0;
            let hard_ms = sensor.exposure.hard_lines * line_us(sensor) / 1000.0;
            assert!((soft_ms - 11.4).abs() < 0.1, "{}: {soft_ms}", sensor.name);
            assert!((hard_ms - 22.9).abs() < 0.1, "{}: {hard_ms}", sensor.name);
        }
    }

    #[test]
    fn the_starting_gain_is_in_the_sensors_own_units() {
        assert_eq!(IMX219.exposure.start_gain(), 1024);
        assert_eq!(GC2093.exposure.start_gain(), 256);
    }
}
