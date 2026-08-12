use crate::{
    AccelerometerPowerMode, BitFlags, Bmi160, Error, GyroscopePowerMode, MagnetometerPowerMode,
    Register, SensorPowerMode, SlaveAddr, Status,
    interface::{I2cInterface, ReadData, SpiInterface, WriteData},
    types::{AccelerometerRange, GyroscopeRange},
};

impl<I2C> Bmi160<I2cInterface<I2C>> {
    /// Create new instance of the BMI160 device communicating through I2C.
    pub fn new_with_i2c(i2c: I2C, address: SlaveAddr) -> Self {
        Bmi160 {
            iface: I2cInterface {
                i2c,
                address: address.addr(),
            },
            accel_range: AccelerometerRange::default(),
            gyro_range: GyroscopeRange::default(),
        }
    }

    /// Destroy driver instance, return I2C bus.
    pub fn destroy(self) -> I2C {
        self.iface.i2c
    }
}

impl<SPI> Bmi160<SpiInterface<SPI>> {
    /// Create new instance of the BMI160 device communicating through SPI.
    pub fn new_with_spi(spi: SPI) -> Self {
        Bmi160 {
            iface: SpiInterface { spi },
            accel_range: AccelerometerRange::default(),
            gyro_range: GyroscopeRange::default(),
        }
    }

    /// Destroy driver instance, return SPI device instance.
    pub fn destroy(self) -> SPI {
        self.iface.spi
    }
}

impl<DI, CommE> Bmi160<DI>
where
    DI: ReadData<Error = Error<CommE>> + WriteData<Error = Error<CommE>>,
{
    /// Get chip ID
    pub fn chip_id(&mut self) -> Result<u8, Error<CommE>> {
        self.iface.read_register(Register::CHIPID)
    }

    /// Get sensor power mode
    pub fn power_mode(&mut self) -> Result<SensorPowerMode, Error<CommE>> {
        let status = self.iface.read_register(Register::PMU_STATUS)?;
        let accel = match status & (0b11 << 4) {
            0 => AccelerometerPowerMode::Suspend,
            0b10_0000 => AccelerometerPowerMode::LowPower,
            _ => AccelerometerPowerMode::Normal,
        };
        let magnet = match status & 0b11 {
            0 => MagnetometerPowerMode::Suspend,
            2 => MagnetometerPowerMode::LowPower,
            _ => MagnetometerPowerMode::Normal,
        };
        let gyro = match status & (0b11 << 2) {
            0 => GyroscopePowerMode::Suspend,
            0b1100 => GyroscopePowerMode::FastStartUp,
            _ => GyroscopePowerMode::Normal,
        };
        Ok(SensorPowerMode {
            accel,
            gyro,
            magnet,
        })
    }

    /// Get sensor status
    pub fn status(&mut self) -> Result<Status, Error<CommE>> {
        let status = self.iface.read_register(Register::STATUS)?;
        Ok(Status {
            accel_data_ready: (status & BitFlags::DRDY_ACC) != 0,
            gyro_data_ready: (status & BitFlags::DRDY_GYR) != 0,
            magnet_data_ready: (status & BitFlags::DRDY_MAG) != 0,
            nvm_ready: (status & BitFlags::NVM_RDY) != 0,
            foc_ready: (status & BitFlags::FOC_RDY) != 0,
            magnet_manual_op: (status & BitFlags::MAG_MAN_OP) != 0,
            gyro_self_test_ok: (status & BitFlags::GYR_SELF_TEST_OK) != 0,
        })
    }

    /// Configure accelerometer power mode
    pub fn set_accel_power_mode(
        &mut self,
        mode: AccelerometerPowerMode,
    ) -> Result<(), Error<CommE>> {
        let cmd = match mode {
            AccelerometerPowerMode::Suspend => 0b0001_0000,
            AccelerometerPowerMode::Normal => 0b0001_0001,
            AccelerometerPowerMode::LowPower => 0b0001_0010,
        };
        self.iface.write_register(Register::CMD, cmd)
    }

    /// Configure gyroscope power mode
    pub fn set_gyro_power_mode(&mut self, mode: GyroscopePowerMode) -> Result<(), Error<CommE>> {
        let cmd = match mode {
            GyroscopePowerMode::Suspend => 0b0001_0100,
            GyroscopePowerMode::Normal => 0b0001_0101,
            GyroscopePowerMode::FastStartUp => 0b0001_0111,
        };
        self.iface.write_register(Register::CMD, cmd)
    }

    /// Configure magnetometer power mode
    pub fn set_magnet_power_mode(
        &mut self,
        mode: MagnetometerPowerMode,
    ) -> Result<(), Error<CommE>> {
        let cmd = match mode {
            MagnetometerPowerMode::Suspend => 0b0001_1000,
            MagnetometerPowerMode::Normal => 0b0001_1001,
            MagnetometerPowerMode::LowPower => 0b0001_1010,
        };
        self.iface.write_register(Register::CMD, cmd)
    }

    /// Set the accelerometer range
    pub fn set_accel_range(&mut self, range: AccelerometerRange) -> Result<(), Error<CommE>> {
        self.iface
            .write_register(Register::ACC_RANGE, range as u8)?;
        self.accel_range = range;
        Ok(())
    }

    /// Set the gyro range
    pub fn set_gyro_range(&mut self, range: GyroscopeRange) -> Result<(), Error<CommE>> {
        self.iface
            .write_register(Register::GYR_RANGE, range as u8)?;
        self.gyro_range = range;
        Ok(())
    }

    /// Configure FIFO to collect Gyroscope and Accelerometer data, Sensortime, and use Header mode.
    ///
    /// Note: By enabling both Gyro and Accel in the FIFO configuration (`0xD2`), the BMI160 will
    /// dynamically construct frames based on which sensors are actually powered on.
    /// - If only Gyro is powered on: outputs `0x84` frames (Gyro only)
    /// - If only Accel is powered on: outputs `0x88` frames (Accel only)
    /// - If both are powered on: outputs `0x8C` frames (Combined Gyro + Accel)
    pub fn config_fifo(&mut self) -> Result<(), Error<CommE>> {
        // FIFO_CONFIG_1 (0x47): Gyro (0x80) | Accel (0x40) | Header (0x10) | Time (0x02) = 0xD2
        self.iface.write_register(0x47, 0xD2)?;
        // Flush FIFO (CMD = 0xB0)
        self.iface.write_register(Register::CMD, 0xB0)?;
        Ok(())
    }

    /// Read raw bytes from the FIFO into the provided buffer
    pub fn read_fifo(&mut self, buffer: &mut [u8]) -> Result<usize, Error<CommE>> {
        // Read 11-bit FIFO length
        let len_lsb = self.iface.read_register(0x22)?;
        let len_msb = self.iface.read_register(0x23)?;
        let length = (((len_msb & 0x07) as u16) << 8) | (len_lsb as u16);

        let length = length as usize;
        if length == 0 {
            return Ok(0);
        }

        // The Sensortime frame (header 0x44 + 3 bytes time) is additionally appended at the end of the FIFO
        // but it is NOT included in the FIFO length register. We must read length + 4 bytes to get it.
        let to_read = core::cmp::min(length + 4, buffer.len() - 1); // leave 1 byte for register address

        // Burst read from 0x24 (FIFO_DATA)
        buffer[0] = 0x24; // Register address
        self.iface.read_data(&mut buffer[0..=to_read])?;

        Ok(to_read)
    }
}
