use crate::control::SensorSnapshot;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c;
use esp_hal::gpio::{Input, Output};

/// no_std, async driver for the HMC5883L Magnetometer
pub struct Hmc5883l<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Hmc5883l<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new driver and configures the sensor.
    pub async fn new(mut i2c: I2C, address: u8) -> Result<Self, E> {
        i2c.write(address, &[0x01, 0x20]).await?;
        i2c.write(address, &[0x02, 0x00]).await?;

        Timer::after_millis(100).await;

        Ok(Self { i2c, address })
    }

    /// Reads the magnetometer axes. Returns (X, Y, Z).
    pub async fn read(&mut self) -> Result<(i16, i16, i16), E> {
        let mut buf = [0u8; 6];

        self.i2c.write_read(self.address, &[0x03], &mut buf).await?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let z = i16::from_be_bytes([buf[2], buf[3]]);
        let y = i16::from_be_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }
}

/// A tiny, pure-async MPU6050 driver
pub struct AsyncMpu6050<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> AsyncMpu6050<I2C>
where
    I2C: I2c<Error = E>,
{
    pub async fn new(mut i2c: I2C, address: u8) -> Result<Self, E> {
        i2c.write(address, &[0x6B, 0x00]).await?;
        Ok(Self { i2c, address })
    }

    /// Returns ([Accel X, Y, Z in g], [Gyro X, Y, Z in deg/s])
    pub async fn read(&mut self) -> Result<([f32; 3], [f32; 3]), E> {
        let mut buf = [0u8; 14];
        self.i2c.write_read(self.address, &[0x3B], &mut buf).await?;

        let ax = i16::from_be_bytes([buf[0], buf[1]]) as f32 / 16384.0;
        let ay = i16::from_be_bytes([buf[2], buf[3]]) as f32 / 16384.0;
        let az = i16::from_be_bytes([buf[4], buf[5]]) as f32 / 16384.0;

        let gx = i16::from_be_bytes([buf[8], buf[9]]) as f32 / 131.0;
        let gy = i16::from_be_bytes([buf[10], buf[11]]) as f32 / 131.0;
        let gz = i16::from_be_bytes([buf[12], buf[13]]) as f32 / 131.0;

        Ok(([ax, ay, az], [gx, gy, gz]))
    }
}

/// A non-blocking async Ultrasonic driver
pub struct Ultrasonic<'d> {
    trig: Output<'d>,
    echo: Input<'d>,
}

impl<'d> Ultrasonic<'d> {
    pub fn new(trig: Output<'d>, echo: Input<'d>) -> Self {
        Self { trig, echo }
    }

    /// Triggers the sensor and awaits the echo asynchronously.
    /// Returns the distance in CM, or `None` if it timed out (e.g. disconnected).
    pub async fn measure(&mut self) -> Option<u32> {
        // Send a 10us pulse
        self.trig.set_low();
        Timer::after_micros(2).await;
        self.trig.set_high();
        Timer::after_micros(10).await;
        self.trig.set_low();

        if embassy_time::with_timeout(Duration::from_millis(10), self.echo.wait_for_high())
            .await
            .is_err()
        {
            return None;
        }

        let start = Instant::now();

        if embassy_time::with_timeout(Duration::from_millis(50), self.echo.wait_for_low())
            .await
            .is_err()
        {
            return None;
        }

        let duration_us = start.elapsed().as_micros();
        Some((duration_us / 58) as u32)
    }
}

pub async fn run_sensors<I2cMag, I2cMpu>(
    mut mag: Hmc5883l<I2cMag>,
    mut mpu: AsyncMpu6050<I2cMpu>,
    snapshot_sender: embassy_sync::watch::Sender<
        'static,
        CriticalSectionRawMutex,
        SensorSnapshot,
        2,
    >,
) where
    I2cMag: embedded_hal_async::i2c::I2c,
    I2cMpu: embedded_hal_async::i2c::I2c,
{
    loop {
        let mut snapshot = SensorSnapshot::default();

        if let Ok((x, y, _z)) = mag.read().await {
            snapshot.magnetometer_x = x;
            snapshot.magnetometer_y = y;
        }

        if let Ok((accel, gyro)) = mpu.read().await {
            snapshot.accel = accel;
            snapshot.gyro = gyro;
        }

        snapshot.top_switch_activated = false;
        snapshot.bottom_switch_activated = false;
        snapshot.collision_fl = false;
        snapshot.collision_fc = false;
        snapshot.collision_fr = false;

        snapshot_sender.send(snapshot);

        Timer::after_millis(20).await;
    }
}
