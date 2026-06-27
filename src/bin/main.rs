#![no_std]
#![no_main]

use atlas_controller::{
    control::{Controller, SensorSnapshot, Settings},
    motors::Motors,
    protocol::{Command, FrameParser, Response},
    sensors::{AsyncMpu6050, Hmc5883l, run_sensors},
};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use embassy_time::{Duration, Instant, Ticker};
use embedded_io_async::Write;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Async,
    ledc::LowSpeed,
    uart::{UartRx, UartTx},
};

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, Command, 10> = Channel::new();
static SENSOR_WATCH: Watch<CriticalSectionRawMutex, SensorSnapshot, 2> = Watch::new();

#[embassy_executor::task]
async fn uart_rx_task(mut rx: UartRx<'static, Async>) {
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 64];

    log::info!("UART RX Task Started");

    loop {
        let len = match embedded_io_async::Read::read(&mut rx, &mut buf).await {
            Ok(l) => l,
            Err(e) => {
                log::warn!("UART Read Error: {:?}", e);
                continue;
            }
        };

        for &byte in &buf[..len] {
            let Some(result) = parser.push(byte) else {
                continue;
            };

            match result {
                Ok(cmd) => {
                    let _ = COMMAND_CHANNEL.send(cmd).await;
                }
                Err(e) => log::warn!("UART Parse Error: {:?}", e),
            }
        }
    }
}

#[embassy_executor::task]
async fn control_task(mut motors: Motors<'static, LowSpeed>, mut tx: UartTx<'static, Async>) {
    let mut controller = Controller::new(Settings::default(), Instant::now());
    let mut ticker = Ticker::every(Duration::from_millis(
        atlas_controller::config::CONTROL_TASK_INTERVAL_MS,
    ));
    let mut receiver = SENSOR_WATCH.receiver().unwrap();

    log::info!("Control Task Started");

    loop {
        let now = Instant::now();
        let latest_sensors = receiver.get().await;

        while let Ok(cmd) = COMMAND_CHANNEL.try_receive() {
            let action = match cmd {
                Command::Drive { left, right } => controller.handle_drive(now, left, right),
                Command::Lift { power } => controller.handle_lift(now, power, &latest_sensors),
                Command::Stop => controller.handle_stop(now),
                Command::EmergencyStop => controller.handle_estop(now),

                Command::SensorPoll => {
                    let status = controller.handle_sensor_poll(now, &latest_sensors);
                    let response = Response::SensorStatus {
                        flags: status.flags,
                        heading_deg: status.heading_deg,
                    };

                    let mut buf = [0u8; 16];
                    if let Some(len) = response.build_frame(&mut buf) {
                        if let Err(e) = tx.write_all(&buf[..len]).await {
                            log::warn!("UART TX Error: {:?}", e);
                        }
                    }
                    continue;
                }
            };

            motors.apply_action(action);
        }

        controller.update_fusion(now, &latest_sensors);
        motors.apply_action(controller.poll_failsafe(now));
        motors.step_all();

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn sensor_task(
    i2c0: esp_hal::i2c::master::I2c<'static, Async>,
    i2c1: esp_hal::i2c::master::I2c<'static, Async>,
) {
    let mag = Hmc5883l::new(i2c1, 0x1E).await.unwrap();
    let mpu = AsyncMpu6050::new(i2c0, 0x68).await.unwrap();

    let sender = SENSOR_WATCH.sender();
    run_sensors(mag, mpu, sender).await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()));

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 72 * 1024);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let (uart_bus, i2c_mpu, i2c_mag, motors) = atlas_controller::init_robot_hardware!(peripherals);
    let (rx, tx) = uart_bus.split();

    spawner.spawn(uart_rx_task(rx).unwrap());
    spawner.spawn(sensor_task(i2c_mpu, i2c_mag).unwrap());
    spawner.spawn(control_task(motors, tx).unwrap());

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}
