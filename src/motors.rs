use crate::control::MotorAction;
use esp_hal::ledc::{
    channel::{Channel, ChannelIFace},
    timer::TimerSpeed,
};

/// Trait defining how the motors should be controlled.
pub trait MotorController {
    fn apply_action(&mut self, action: MotorAction);
    fn step_all(&mut self);
}

/// Represents a single BTS7960 Motor driver with an RPWM and LPWM pin.
pub struct Bts7960Motor<'a, S: TimerSpeed> {
    rpwm: Channel<'a, S>,
    lpwm: Channel<'a, S>,
    current_speed: i8,
    target_speed: i8,
    ramp_step: i8,
}

impl<'a, S: TimerSpeed> Bts7960Motor<'a, S> {
    pub fn new(rpwm: Channel<'a, S>, lpwm: Channel<'a, S>, ramp_step: i8) -> Self {
        Self {
            rpwm,
            lpwm,
            current_speed: 0,
            target_speed: 0,
            ramp_step,
        }
    }

    /// Sets the target speed and lets the `step` function ramp up/down to it.
    pub fn set_target(&mut self, speed: i8) {
        self.target_speed = speed.clamp(-100, 100);
    }

    /// Bypasses the ramp and applies the speed immediately.
    pub fn set_immediate(&mut self, speed: i8) {
        self.target_speed = speed.clamp(-100, 100);
        self.current_speed = self.target_speed;
        self.apply_speed();
    }

    /// Steps the current speed towards the target speed by the `ramp_step`.
    pub fn step(&mut self) {
        let diff = self.target_speed as i16 - self.current_speed as i16;
        if diff == 0 {
            return;
        }

        let step = if diff > 0 {
            core::cmp::min(diff, self.ramp_step as i16)
        } else {
            core::cmp::max(diff, -(self.ramp_step as i16))
        };

        self.current_speed = (self.current_speed as i16 + step) as i8;
        self.apply_speed();
    }

    /// Writes the current speed to the PWM hardware.
    fn apply_speed(&mut self) {
        let duty_pct = self.current_speed.abs() as u8;

        if self.current_speed >= 0 {
            let _ = self.rpwm.set_duty(duty_pct);
            let _ = self.lpwm.set_duty(0);
        } else {
            let _ = self.rpwm.set_duty(0);
            let _ = self.lpwm.set_duty(duty_pct);
        }
    }
}

/// A container for the robot's motors.
pub struct Motors<'a, S: TimerSpeed> {
    pub left: Bts7960Motor<'a, S>,
    pub right: Bts7960Motor<'a, S>,
    pub lift: Bts7960Motor<'a, S>,
}

// We change this block from `impl Motors` to `impl MotorController for Motors`
impl<'a, S: TimerSpeed> MotorController for Motors<'a, S> {
    /// Evaluates a `MotorAction` and delegates to the individual motors.
    fn apply_action(&mut self, action: MotorAction) {
        if action.emergency_stop {
            self.left.set_immediate(0);
            self.right.set_immediate(0);
            self.lift.set_immediate(0);
            return;
        }

        if action.stop_all {
            self.left.set_immediate(0);
            self.right.set_immediate(0);
            self.lift.set_immediate(0);
        }

        if let Some((l, r)) = action.drive {
            self.left.set_target(l);
            self.right.set_target(r);
        }

        if let Some(p) = action.lift {
            self.lift.set_target(p);
        }
    }

    /// Should be called periodically (e.g., every 10ms-20ms) inside the `control_task`
    /// to update the PWM ramps.
    fn step_all(&mut self) {
        self.left.step();
        self.right.step();
        self.lift.step();
    }
}
