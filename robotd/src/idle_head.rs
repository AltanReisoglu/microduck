//! The head looks around, slowly, when nothing else is happening.
//!
//! A robot standing perfectly still reads as switched off. After [`IDLE_AFTER`] with no motion
//! asked for and nobody steering the head, the head yaw sweeps gently left and right — a sinusoid
//! of [`AMPLITUDE`] over [`PERIOD`] — added on top of whatever the head was last asked to do, the
//! way the chorale's sway is.
//!
//! It fades in and out over [`FADE`] rather than switching, and starts at the centre of its
//! sweep, so it never jumps: a stick touched mid-sweep gets a head that glides back under it.
//!
//! Not the autonomous behaviour stack (`docs/ideas/autonomous_behavior.md`), whose LookAround
//! state this is a small piece of — just the one thing that makes an idle robot look alive.

use std::time::{Duration, Instant};

/// Still for this long before the head starts to move.
pub const IDLE_AFTER: Duration = Duration::from_secs(5);

/// How far the head turns each way, radians (~20°).
pub const AMPLITUDE: f64 = 0.35;

/// One full left-right-left sweep. Slow on purpose: looking around, not shaking its head.
pub const PERIOD: Duration = Duration::from_secs(10);

/// How long the sweep takes to fade in, and out.
pub const FADE: Duration = Duration::from_millis(1500);

/// The idle sweep's state, ticked once per control tick.
#[derive(Debug, Default)]
pub struct IdleHead {
    /// Since when nothing has asked the robot to move. `None` while something does.
    still_since: Option<Instant>,
    /// How much of the sweep is applied, 0..1, faded towards 1 while idle and 0 otherwise.
    gain: f64,
    /// When this sweep started, so it begins at the centre. `None` once it has faded out.
    started: Option<Instant>,
}

impl IdleHead {
    /// One tick. `still` says nothing is asking for motion right now — the caller decides what
    /// counts. Returns the offset to add to the head command, in its order: neck pitch, head
    /// pitch, head yaw, head roll.
    pub fn tick(&mut self, now: Instant, still: bool, dt: Duration) -> [f64; 4] {
        self.still_since = if still {
            Some(self.still_since.unwrap_or(now))
        } else {
            None
        };
        let idle = self
            .still_since
            .is_some_and(|since| now.duration_since(since) >= IDLE_AFTER);

        let step = (dt.as_secs_f64() / FADE.as_secs_f64()).clamp(0.0, 1.0);
        self.gain = if idle {
            (self.gain + step).min(1.0)
        } else {
            (self.gain - step).max(0.0)
        };

        if idle && self.started.is_none() {
            self.started = Some(now);
        }
        let Some(started) = self.started else {
            return [0.0; 4];
        };
        if self.gain == 0.0 {
            // Faded out: the next sweep starts afresh, at the centre.
            self.started = None;
            return [0.0; 4];
        }
        let phase = now.duration_since(started).as_secs_f64() / PERIOD.as_secs_f64();
        let yaw = self.gain * AMPLITUDE * (std::f64::consts::TAU * phase).sin();
        [0.0, 0.0, yaw, 0.0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: Duration = Duration::from_millis(20);

    /// Run `seconds` of ticks from `from`, returning the time reached and the last offset.
    fn run(head: &mut IdleHead, from: Instant, seconds: f64, still: bool) -> (Instant, [f64; 4]) {
        let mut now = from;
        let mut last = [0.0; 4];
        for _ in 0..(seconds / DT.as_secs_f64()).round() as usize {
            now += DT;
            last = head.tick(now, still, DT);
        }
        (now, last)
    }

    /// Nothing before the idle delay, then a sweep that stays inside its amplitude, in yaw only.
    #[test]
    fn the_head_sweeps_only_after_the_robot_has_been_still() {
        let mut head = IdleHead::default();
        let t0 = Instant::now();
        let (now, early) = run(&mut head, t0, 4.9, true);
        assert_eq!(early, [0.0; 4], "not yet");

        let mut peak = 0.0f64;
        let mut now = now;
        for _ in 0..(20.0 / DT.as_secs_f64()) as usize {
            now += DT;
            let offset = head.tick(now, true, DT);
            assert_eq!([offset[0], offset[1], offset[3]], [0.0; 3], "yaw only");
            assert!(offset[2].abs() <= AMPLITUDE + 1e-9);
            peak = peak.max(offset[2].abs());
        }
        assert!(peak > 0.9 * AMPLITUDE, "it really sweeps: peak {peak}");
    }

    /// It starts at the centre and fades in: no step on the first idle tick.
    #[test]
    fn the_sweep_starts_without_a_jump() {
        let mut head = IdleHead::default();
        let t0 = Instant::now();
        let (now, _) = run(&mut head, t0, 5.0, true);
        let first = head.tick(now + DT, true, DT);
        assert!(first[2].abs() < 0.01, "{first:?}");
    }

    /// Asked to move, it fades back to centre rather than snapping, and the delay starts over.
    #[test]
    fn motion_fades_the_sweep_out_and_restarts_the_wait() {
        let mut head = IdleHead::default();
        let t0 = Instant::now();
        let (now, _) = run(&mut head, t0, 12.0, true);
        let (now, faded) = run(&mut head, now, FADE.as_secs_f64() + 0.1, false);
        assert_eq!(faded, [0.0; 4], "gone after the fade");

        let (_, again) = run(&mut head, now, 4.0, true);
        assert_eq!(again, [0.0; 4], "the idle delay starts over");
    }
}
