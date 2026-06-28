crate::define_robot_config! {
    uart: { port: 1, baud: 115_200, tx_pin: 17, rx_pin: 16 },
    mpu6050: { i2c_port: 0, sda_pin: 19, scl_pin: 18 },
    hmc5883l: { i2c_port: 1, sda_pin: 21, scl_pin: 22 },
    motors: {
        freq_hz: 20000,
        left_rpwm: 25, left_lpwm: 23,
        right_rpwm: 27, right_lpwm: 26,
        lift_rpwm: 32, lift_lpwm: 33,
    },
    safety: {
        cmd_timeout_ms: 300,
        task_interval_ms: 50,
        block_lift_when_driving: true,
        block_drive_when_lifting: true,
        estop_latch: true,
        min_pwm: 15,
        max_pwm: 100
    },
    wifi: { ssid: "ATLAS_ESP32", password: "AtlasWillHoldTheWorld", ip: [192, 168, 4, 1] },
    udp: { port: 5005 }
}
