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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    Normal { driving: bool, lifting: bool },
    Stopped,
    Failsafe,
    EmergencyStop,
}

impl Default for RobotState {
    fn default() -> Self {
        Self::Normal {
            driving: false,
            lifting: false,
        }
    }
}

#[allow(clippy::struct_excessive_bools)]
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
    state: RobotState,
    fused_heading: f32,
    last_fusion_time: Option<Instant>,
}

impl Controller {
    #[must_use]
    pub const fn new(settings: Settings, now: Instant) -> Self {
        Self {
            settings,
            last_cmd_at: now,
            state: RobotState::Normal {
                driving: false,
                lifting: false,
            },
            fused_heading: 0.0,
            last_fusion_time: None,
        }
    }

    pub const fn reset(&mut self, now: Instant) {
        self.last_cmd_at = now;
        self.state = RobotState::Normal {
            driving: false,
            lifting: false,
        };
    }

    const fn mark_command(&mut self, now: Instant) {
        self.last_cmd_at = now;
        if matches!(self.state, RobotState::Failsafe) {
            self.state = RobotState::Stopped;
        }
    }

    pub fn handle_drive(&mut self, now: Instant, left: i8, right: i8) -> MotorAction {
        self.mark_command(now);

        let l = clamp_speed(left);
        let r = clamp_speed(right);
        let nonzero = (l != 0) || (r != 0);

        if matches!(self.state, RobotState::Stopped | RobotState::Failsafe) && nonzero {
            self.state = RobotState::Normal {
                driving: false,
                lifting: false,
            };
        }

        match self.state {
            RobotState::EmergencyStop | RobotState::Stopped | RobotState::Failsafe => MotorAction {
                drive: Some((0, 0)),
                ..Default::default()
            },
            RobotState::Normal { lifting, .. } => {
                if self.settings.block_drive_when_lifting && lifting {
                    self.state = RobotState::Normal {
                        driving: false,
                        lifting,
                    };
                    MotorAction {
                        drive: Some((0, 0)),
                        ..Default::default()
                    }
                } else {
                    self.state = RobotState::Normal {
                        driving: nonzero,
                        lifting,
                    };
                    MotorAction {
                        drive: Some((l, r)),
                        ..Default::default()
                    }
                }
            }
        }
    }

    pub fn handle_stop(&mut self, now: Instant) -> MotorAction {
        self.mark_command(now);

        if matches!(self.state, RobotState::EmergencyStop) && self.settings.estop_latch {
        } else {
            self.state = RobotState::Stopped;
        }

        MotorAction {
            stop_all: true,
            ..Default::default()
        }
    }

    pub fn handle_estop(&mut self, now: Instant) -> MotorAction {
        self.mark_command(now);
        self.state = RobotState::EmergencyStop;

        MotorAction {
            emergency_stop: true,
            ..Default::default()
        }
    }

    pub fn update_fusion(&mut self, now: Instant, sensor: &SensorSnapshot) {
        const ALPHA: f32 = 0.98;

        let mag_heading = heading_deg_from_mag(sensor.magnetometer_x, sensor.magnetometer_y);

        #[allow(clippy::cast_precision_loss)]
        let dt = self
            .last_fusion_time
            .map_or(0.0, |last| (now - last).as_micros() as f32 / 1_000_000.0);

        self.last_fusion_time = Some(now);

        if dt == 0.0 {
            self.fused_heading = mag_heading;
            return;
        }

        let gyro_z_rate_deg = sensor.gyro[2];

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

    pub const fn handle_sensor_poll(
        &mut self,
        now: Instant,
        sensor: &SensorSnapshot,
    ) -> SensorStatus {
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

        if matches!(self.state, RobotState::Stopped | RobotState::Failsafe) && nonzero {
            self.state = RobotState::Normal {
                driving: false,
                lifting: false,
            };
        }

        match self.state {
            RobotState::EmergencyStop | RobotState::Stopped | RobotState::Failsafe => MotorAction {
                lift: Some(0),
                ..Default::default()
            },
            RobotState::Normal { driving, .. } => {
                if self.settings.block_lift_when_driving && driving {
                    self.state = RobotState::Normal {
                        driving,
                        lifting: false,
                    };
                    MotorAction {
                        lift: Some(0),
                        ..Default::default()
                    }
                } else {
                    self.state = RobotState::Normal {
                        driving,
                        lifting: nonzero,
                    };
                    MotorAction {
                        lift: Some(pwr),
                        ..Default::default()
                    }
                }
            }
        }
    }

    pub fn poll_failsafe(&mut self, now: Instant) -> MotorAction {
        if matches!(self.state, RobotState::EmergencyStop) {
            return MotorAction::default();
        }

        if !matches!(self.state, RobotState::Failsafe)
            && (now - self.last_cmd_at) > self.settings.command_timeout
        {
            self.state = RobotState::Failsafe;
            MotorAction {
                stop_all: true,
                ..Default::default()
            }
        } else {
            MotorAction::default()
        }
    }
}

fn clamp_speed(value: i8) -> i8 {
    value.clamp(SPEED_MIN, SPEED_MAX)
}

fn heading_deg_from_mag(x: i16, y: i16) -> f32 {
    let heading = libm::atan2f(f32::from(y), f32::from(x));
    let mut deg = heading * 57.295_78;
    if deg < 0.0 {
        deg += 360.0;
    }
    deg
}

const fn sensor_flags(sensor: &SensorSnapshot) -> u8 {
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
