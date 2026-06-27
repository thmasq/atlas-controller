use crate::config;
use embassy_time::{Duration, Instant};

const SPEED_MAX: i8 = 100;
const SPEED_MIN: i8 = -100;

const SENSOR_FL: u8 = 1 << 0;
const SENSOR_FC: u8 = 1 << 1;
const SENSOR_FR: u8 = 1 << 2;
const SENSOR_LIFT_TOP: u8 = 1 << 3;
const SENSOR_LIFT_BOT: u8 = 1 << 4;

pub struct Settings {
    pub block_drive_when_lifting: bool,
    pub block_lift_when_driving: bool,
    pub estop_latch: bool,
    pub command_timeout: Duration,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            block_drive_when_lifting: config::INTERLOCK_BLOCK_DRIVE_WHEN_LIFTING,
            block_lift_when_driving: config::INTERLOCK_BLOCK_LIFT_WHEN_DRIVING,
            estop_latch: config::ESTOP_LATCH,
            command_timeout: Duration::from_millis(config::CONTROL_CMD_TIMEOUT_MS),
        }
    }
}

#[derive(Default, Clone, Copy, Debug)]
pub struct SensorSnapshot {
    pub top_switch_activated: bool,
    pub bottom_switch_activated: bool,
    pub collision_fl: bool,
    pub collision_fc: bool,
    pub collision_fr: bool,
    pub magnetometer_x: i16,
    pub magnetometer_y: i16,
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
}

/// Represents an action the motors need to take.
#[derive(Default, Clone, Copy, Debug)]
pub struct MotorAction {
    pub drive: Option<(i8, i8)>,
    pub lift: Option<i8>,
    pub stop_all: bool,
    pub emergency_stop: bool,
}

#[derive(Default, Clone, Copy, Debug)]
pub struct SensorStatus {
    pub flags: u8,
    pub heading_deg: f32,
}

pub struct Controller {
    settings: Settings,
    last_cmd_at: Instant,
    estop_active: bool,
    stop_active: bool,
    drive_active: bool,
    lift_active: bool,
    failsafe_active: bool,
    fused_heading: f32,
    last_fusion_time: Option<Instant>,
}

impl Controller {
    pub fn new(settings: Settings, now: Instant) -> Self {
        Self {
            settings,
            last_cmd_at: now,
            estop_active: false,
            stop_active: false,
            drive_active: false,
            lift_active: false,
            failsafe_active: false,
            fused_heading: 0.0,
            last_fusion_time: None,
        }
    }

    pub fn reset(&mut self, now: Instant) {
        self.last_cmd_at = now;
        self.estop_active = false;
        self.stop_active = false;
        self.drive_active = false;
        self.lift_active = false;
        self.failsafe_active = false;
    }

    fn mark_command(&mut self, now: Instant) {
        self.last_cmd_at = now;
        self.failsafe_active = false;
    }

    pub fn handle_drive(&mut self, now: Instant, left: i8, right: i8) -> MotorAction {
        self.mark_command(now);

        let l = clamp_speed(left);
        let r = clamp_speed(right);
        let nonzero = (l != 0) || (r != 0);

        if self.stop_active && nonzero {
            self.stop_active = false;
        }

        let mut action = MotorAction::default();

        if self.estop_active
            || self.stop_active
            || (self.settings.block_drive_when_lifting && self.lift_active)
        {
            self.drive_active = false;
            action.drive = Some((0, 0));
            return action;
        }

        self.drive_active = nonzero;
        action.drive = Some((l, r));
        action
    }

    pub fn handle_stop(&mut self, now: Instant) -> MotorAction {
        self.mark_command(now);
        self.stop_active = true;
        self.drive_active = false;
        self.lift_active = false;

        if !self.settings.estop_latch {
            self.estop_active = false;
        }

        let mut action = MotorAction::default();
        action.stop_all = true;
        action
    }

    pub fn handle_estop(&mut self, now: Instant) -> MotorAction {
        self.mark_command(now);
        self.estop_active = true;
        self.stop_active = true;
        self.drive_active = false;
        self.lift_active = false;

        let mut action = MotorAction::default();
        action.emergency_stop = true;
        action
    }

    pub fn update_fusion(&mut self, now: Instant, sensor: &SensorSnapshot) {
        let mag_heading = heading_deg_from_mag(sensor.magnetometer_x, sensor.magnetometer_y);

        let dt = if let Some(last) = self.last_fusion_time {
            (now - last).as_micros() as f32 / 1_000_000.0
        } else {
            0.0
        };
        self.last_fusion_time = Some(now);

        if dt == 0.0 {
            self.fused_heading = mag_heading;
            return;
        }

        let gyro_z_rate_deg = sensor.gyro[2];

        const ALPHA: f32 = 0.98;

        let mut diff = mag_heading - self.fused_heading;
        if diff > 180.0 {
            diff -= 360.0;
        }
        if diff < -180.0 {
            diff += 360.0;
        }

        self.fused_heading += (gyro_z_rate_deg * dt) + ((1.0 - ALPHA) * diff);

        if self.fused_heading >= 360.0 {
            self.fused_heading -= 360.0;
        }
        if self.fused_heading < 0.0 {
            self.fused_heading += 360.0;
        }
    }

    pub fn handle_sensor_poll(&mut self, now: Instant, sensor: &SensorSnapshot) -> SensorStatus {
        self.mark_command(now);

        SensorStatus {
            flags: sensor_flags(sensor),
            heading_deg: self.fused_heading,
        }
    }

    pub fn handle_lift(
        &mut self,
        now: Instant,
        power: i8,
        _sensor: &SensorSnapshot,
    ) -> MotorAction {
        self.mark_command(now);

        let pwr = clamp_speed(power);
        let nonzero = pwr != 0;

        if self.stop_active && nonzero {
            self.stop_active = false;
        }

        let mut action = MotorAction::default();

        if self.estop_active
            || self.stop_active
            || (self.settings.block_lift_when_driving && self.drive_active)
        {
            self.lift_active = false;
            action.lift = Some(0);
            return action;
        }

        // if pwr > 0 && sensor.top_switch_activated { pwr = 0; }
        // if pwr < 0 && sensor.bottom_switch_activated { pwr = 0; }

        self.lift_active = nonzero;
        action.lift = Some(pwr);
        action
    }

    pub fn poll_failsafe(&mut self, now: Instant) -> MotorAction {
        let mut action = MotorAction::default();

        if self.estop_active {
            return action;
        }

        if !self.failsafe_active && (now - self.last_cmd_at) > self.settings.command_timeout {
            self.stop_active = true;
            self.drive_active = false;
            self.lift_active = false;
            self.failsafe_active = true;

            action.stop_all = true;
        }

        action
    }
}

fn clamp_speed(value: i8) -> i8 {
    value.clamp(SPEED_MIN, SPEED_MAX)
}

fn heading_deg_from_mag(x: i16, y: i16) -> f32 {
    let heading = libm::atan2f(y as f32, x as f32);
    let mut deg = heading * 57.2957795;
    if deg < 0.0 {
        deg += 360.0;
    }
    deg
}

fn sensor_flags(sensor: &SensorSnapshot) -> u8 {
    let mut flags = 0;
    if sensor.collision_fl {
        flags |= SENSOR_FL;
    }
    if sensor.collision_fc {
        flags |= SENSOR_FC;
    }
    if sensor.collision_fr {
        flags |= SENSOR_FR;
    }
    if sensor.top_switch_activated {
        flags |= SENSOR_LIFT_TOP;
    }
    if sensor.bottom_switch_activated {
        flags |= SENSOR_LIFT_BOT;
    }
    flags
}
