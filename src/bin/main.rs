#![no_std]
#![no_main]
#![allow(clippy::future_not_send, clippy::large_stack_frames)]

extern crate alloc;

use alloc::string::ToString;
use atlas_controller::motors::MotorController;
use atlas_controller::protocol::FrameParser;
use atlas_controller::{
    control::{Controller, SensorSnapshot, Settings},
    motors::Motors,
    protocol::{Command, Response},
    sensors::{AsyncMpu6050, Hmc5883l, run_sensors},
};
use embassy_executor::Spawner;
use embassy_net::{
    Config, Ipv4Address, Ipv4Cidr, Stack as NetStack, StackResources, StaticConfigV4,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::rng::Rng;
use esp_hal::system::Stack as CpuStack;
use esp_hal::uart::UartRx;
use esp_hal::{Async, ledc::LowSpeed, uart::UartTx};
use esp_radio::wifi::{Config as WifiDriverConfig, ap::AccessPointConfig};
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, Command, 10> = Channel::new();
static RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, Response, 10> = Channel::new();
static SENSOR_WATCH: Watch<CriticalSectionRawMutex, SensorSnapshot, 2> = Watch::new();

static CORE1_STACK: StaticCell<CpuStack<8192>> = StaticCell::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.init($val);
        x
    }};
}

async fn uart_rx_task_impl(mut rx: impl embedded_io_async::Read) {
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 64];

    log::info!("UART RX Task Started");

    loop {
        let Ok(len) = embedded_io_async::Read::read(&mut rx, &mut buf).await else {
            log::warn!("UART Read Error");
            continue;
        };

        for &byte in &buf[..len] {
            let Some(result) = parser.push(byte) else {
                continue;
            };

            match result {
                Ok(cmd) => {
                    let () = COMMAND_CHANNEL.send(cmd).await;
                }
                Err(e) => log::warn!("UART Parse Error: {e:?}"),
            }
        }
    }
}

#[embassy_executor::task]
async fn uart_rx_task(rx: UartRx<'static, Async>) {
    uart_rx_task_impl(rx).await;
}

async fn uart_tx_task_impl(mut tx: impl embedded_io_async::Write) {
    loop {
        let response = RESPONSE_CHANNEL.receive().await;
        let mut buf = [0u8; 16];
        if let Some(len) = response.build_frame(&mut buf)
            && let Err(_) = embedded_io_async::Write::write_all(&mut tx, &buf[..len]).await
        {
            log::warn!("UART TX Error");
        }
    }
}

#[embassy_executor::task]
async fn uart_tx_task(tx: UartTx<'static, Async>) {
    uart_tx_task_impl(tx).await;
}

async fn control_task_impl(mut motors: impl MotorController) {
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

                    let _ = RESPONSE_CHANNEL.try_send(response);
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
async fn control_task(motors: Motors<'static, LowSpeed>) {
    control_task_impl(motors).await;
}

async fn sensor_task_impl(
    i2c0: impl embedded_hal_async::i2c::I2c,
    i2c1: impl embedded_hal_async::i2c::I2c,
) {
    let mag = Hmc5883l::new(i2c1, 0x1E).await.unwrap();
    let mpu = AsyncMpu6050::new(i2c0, 0x68).await.unwrap();

    let sender = SENSOR_WATCH.sender();
    run_sensors(mag, mpu, sender).await;
}

#[embassy_executor::task]
async fn sensor_task(
    i2c0: esp_hal::i2c::master::I2c<'static, Async>,
    i2c1: esp_hal::i2c::master::I2c<'static, Async>,
) {
    sensor_task_impl(i2c0, i2c1).await;
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, esp_radio::wifi::Interface<'static>>) {
    runner.run().await;
}

#[embassy_executor::task]
async fn udp_listener_task(stack: NetStack<'static>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; 1024];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; 1024];

    loop {
        if !stack.is_link_up() {
            Timer::after(Duration::from_millis(500)).await;
            continue;
        }

        let mut socket = UdpSocket::new(
            stack,
            &mut rx_meta,
            &mut rx_buffer,
            &mut tx_meta,
            &mut tx_buffer,
        );

        if let Err(e) = socket.bind(atlas_controller::config::UDP_PORT) {
            log::warn!("UDP bind error: {e:?}");
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        log::info!(
            "UDP Listener bound to {}.{}.{}.{}:{} on {}",
            atlas_controller::config::WIFI_IP[0],
            atlas_controller::config::WIFI_IP[1],
            atlas_controller::config::WIFI_IP[2],
            atlas_controller::config::WIFI_IP[3],
            atlas_controller::config::UDP_PORT,
            atlas_controller::config::WIFI_SSID
        );

        loop {
            let mut buf = [0u8; 8];
            match socket.recv_from(&mut buf).await {
                Ok((size, _remote_endpoint)) => {
                    if size == 8 {
                        let packet_type = buf[0];
                        let left = buf[1] as i8;
                        let right = buf[2] as i8;
                        let _flags = buf[3];
                        let lift = buf[4] as i8;

                        match packet_type {
                            0x01 => {
                                let _ = COMMAND_CHANNEL.try_send(Command::Drive { left, right });
                                let _ = COMMAND_CHANNEL.try_send(Command::Lift { power: lift });
                            }
                            0x02 => {
                                log::info!("Received Auto Mode UDP request. Ignoring standalone.");
                            }
                            _ => log::warn!("Unknown UDP packet type: {packet_type:#04X}"),
                        }
                    } else {
                        log::warn!("Received malformed UDP packet of size {size}");
                    }
                }
                Err(e) => {
                    log::warn!("UDP recv error: {e:?}");
                    break;
                }
            }
        }
    }
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

    let uart_bus = atlas_controller::init_core0_hardware!(peripherals);
    let (rx, tx) = uart_bus.split();

    let rng = Rng::new();
    let seed = u64::from(rng.random());

    let ap_config = WifiDriverConfig::AccessPoint(
        AccessPointConfig::default()
            .with_ssid(atlas_controller::config::WIFI_SSID)
            .with_password(atlas_controller::config::WIFI_PASSWORD.to_string())
            .with_auth_method(esp_radio::wifi::AuthenticationMethod::Wpa2Personal)
            .with_channel(1),
    );

    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        esp_radio::wifi::ControllerConfig::default().with_initial_config(ap_config),
    )
    .unwrap();

    let wifi_interface = interfaces.access_point;

    let ip = atlas_controller::config::WIFI_IP;
    let controller_ip = Ipv4Address::new(ip[0], ip[1], ip[2], ip[3]);

    let net_config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(controller_ip, 24),
        gateway: None,
        dns_servers: Default::default(),
    });

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    let _wifi_controller = controller;

    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(udp_listener_task(stack).unwrap());
    spawner.spawn(uart_rx_task(rx).unwrap());
    spawner.spawn(uart_tx_task(tx).unwrap());

    let cpu1_fn = move || {
        let executor = CORE1_EXECUTOR.init(Executor::new());

        executor.run(|spawner| {
            let (i2c_mpu, i2c_mag, motors) = atlas_controller::init_core1_hardware!(peripherals);

            spawner.spawn(sensor_task(i2c_mpu, i2c_mag).unwrap());
            spawner.spawn(control_task(motors).unwrap());
        });
    };

    let core1_stack = CORE1_STACK.init(CpuStack::new());
    let core1_sw_int = sw_interrupt.software_interrupt1;

    esp_rtos::start_second_core(peripherals.CPU_CTRL, core1_sw_int, core1_stack, cpu1_fn);

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}
