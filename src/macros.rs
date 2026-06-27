#[macro_export]
macro_rules! define_robot_config {
    (
        uart: { port: $u_port:literal, baud: $u_baud:literal, tx_pin: $u_tx:literal, rx_pin: $u_rx:literal $(,)? },
        mpu6050: { i2c_port: $mpu_port:literal, sda_pin: $mpu_sda:literal, scl_pin: $mpu_scl:literal $(,)? },
        hmc5883l: { i2c_port: $hmc_port:literal, sda_pin: $hmc_sda:literal, scl_pin: $hmc_scl:literal $(,)? },
        motors: {
            freq_hz: $m_freq:literal,
            left_rpwm: $l_rpwm:literal, left_lpwm: $l_lpwm:literal,
            right_rpwm: $r_rpwm:literal, right_lpwm: $r_lpwm:literal,
            lift_rpwm: $lift_rpwm:literal, lift_lpwm: $lift_lpwm:literal $(,)?
        },
        safety: {
            cmd_timeout_ms: $s_timeout:literal,
            task_interval_ms: $s_interval:literal,
            block_lift_when_driving: $s_block_lift:expr,
            block_drive_when_lifting: $s_block_drive:expr,
            estop_latch: $s_estop:expr $(,)?
        },
        wifi: { ssid: $wifi_ssid:literal, password: $wifi_pass:literal, ip: [$ip0:literal, $ip1:literal, $ip2:literal, $ip3:literal] $(,)? },
        udp: { port: $udp_port:literal $(,)? }
    ) => {
        pub const UART_BAUD: u32 = $u_baud;
        pub const MOTOR_FREQ_HZ: u32 = $m_freq;
        pub const CONTROL_CMD_TIMEOUT_MS: u64 = $s_timeout;
        pub const CONTROL_TASK_INTERVAL_MS: u64 = $s_interval;
        pub const INTERLOCK_BLOCK_LIFT_WHEN_DRIVING: bool = $s_block_lift;
        pub const INTERLOCK_BLOCK_DRIVE_WHEN_LIFTING: bool = $s_block_drive;
        pub const ESTOP_LATCH: bool = $s_estop;
        pub const WIFI_SSID: &str = $wifi_ssid;
        pub const WIFI_PASSWORD: &str = $wifi_pass;
        pub const WIFI_IP: [u8; 4] = [$ip0, $ip1, $ip2, $ip3];
        pub const UDP_PORT: u16 = $udp_port;

        #[macro_export]
        macro_rules! init_robot_hardware {
            ($p:expr) => {{
                use esp_hal::ledc::channel::ChannelIFace;
                use esp_hal::ledc::timer::TimerIFace;

                ::paste::paste! {
                    let uart_bus = esp_hal::uart::Uart::new(
                        $p.[<UART $u_port>],
                        esp_hal::uart::Config::default().with_baudrate($u_baud),
                    )
                    .unwrap()
                    .with_tx($p.[<GPIO $u_tx>])
                    .with_rx($p.[<GPIO $u_rx>])
                    .into_async();

                    let i2c_mpu = esp_hal::i2c::master::I2c::new(
                        $p.[<I2C $mpu_port>],
                        esp_hal::i2c::master::Config::default().with_frequency(esp_hal::time::Rate::from_khz(400)),
                    )
                    .unwrap()
                    .with_sda($p.[<GPIO $mpu_sda>])
                    .with_scl($p.[<GPIO $mpu_scl>])
                    .into_async();

                    let i2c_mag = esp_hal::i2c::master::I2c::new(
                        $p.[<I2C $hmc_port>],
                        esp_hal::i2c::master::Config::default().with_frequency(esp_hal::time::Rate::from_khz(400)),
                    )
                    .unwrap()
                    .with_sda($p.[<GPIO $hmc_sda>])
                    .with_scl($p.[<GPIO $hmc_scl>])
                    .into_async();

                    static LEDC_CELL: static_cell::StaticCell<esp_hal::ledc::Ledc<'static>> = static_cell::StaticCell::new();
                    let ledc = LEDC_CELL.init(esp_hal::ledc::Ledc::new($p.LEDC));

                    let mut t0 = ledc.timer::<esp_hal::ledc::LowSpeed>(esp_hal::ledc::timer::Number::Timer0);
                    t0.configure(esp_hal::ledc::timer::config::Config {
                        duty: esp_hal::ledc::timer::config::Duty::Duty10Bit,
                        clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
                        frequency: esp_hal::time::Rate::from_hz($m_freq),
                    }).unwrap();

                    static TIMER0_CELL: static_cell::StaticCell<esp_hal::ledc::timer::Timer<'static, esp_hal::ledc::LowSpeed>> = static_cell::StaticCell::new();
                    let timer0 = TIMER0_CELL.init(t0);

                    let mut l_rpwm = ledc.channel(esp_hal::ledc::channel::Number::Channel0, $p.[<GPIO $l_rpwm>]);
                    l_rpwm.configure(esp_hal::ledc::channel::config::Config { timer: timer0, duty_pct: 0, drive_mode: esp_hal::gpio::DriveMode::PushPull }).unwrap();
                    let mut l_lpwm = ledc.channel(esp_hal::ledc::channel::Number::Channel1, $p.[<GPIO $l_lpwm>]);
                    l_lpwm.configure(esp_hal::ledc::channel::config::Config { timer: timer0, duty_pct: 0, drive_mode: esp_hal::gpio::DriveMode::PushPull }).unwrap();

                    let mut r_rpwm = ledc.channel(esp_hal::ledc::channel::Number::Channel2, $p.[<GPIO $r_rpwm>]);
                    r_rpwm.configure(esp_hal::ledc::channel::config::Config { timer: timer0, duty_pct: 0, drive_mode: esp_hal::gpio::DriveMode::PushPull }).unwrap();
                    let mut r_lpwm = ledc.channel(esp_hal::ledc::channel::Number::Channel3, $p.[<GPIO $r_lpwm>]);
                    r_lpwm.configure(esp_hal::ledc::channel::config::Config { timer: timer0, duty_pct: 0, drive_mode: esp_hal::gpio::DriveMode::PushPull }).unwrap();

                    let mut lift_r = ledc.channel(esp_hal::ledc::channel::Number::Channel4, $p.[<GPIO $lift_rpwm>]);
                    lift_r.configure(esp_hal::ledc::channel::config::Config { timer: timer0, duty_pct: 0, drive_mode: esp_hal::gpio::DriveMode::PushPull }).unwrap();
                    let mut lift_l = ledc.channel(esp_hal::ledc::channel::Number::Channel5, $p.[<GPIO $lift_lpwm>]);
                    lift_l.configure(esp_hal::ledc::channel::config::Config { timer: timer0, duty_pct: 0, drive_mode: esp_hal::gpio::DriveMode::PushPull }).unwrap();

                    let motors = $crate::motors::Motors {
                        left: $crate::motors::Bts7960Motor::new(l_rpwm, l_lpwm, 1),
                        right: $crate::motors::Bts7960Motor::new(r_rpwm, r_lpwm, 1),
                        lift: $crate::motors::Bts7960Motor::new(lift_r, lift_l, 1),
                    };

                    (uart_bus, i2c_mpu, i2c_mag, motors)
                }
            }};
        }
    };
}
