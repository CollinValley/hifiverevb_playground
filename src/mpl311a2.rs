use e310x::I2C0;
use e310x_hal::gpio::{gpio0, NoInvert, IOF0};
use e310x_hal::prelude::*;
use hifive1::hal::delay::Delay;
use hifive1::hal::i2c::I2c;
use hifive1::sprintln;

pub type I2cBaro = I2c<I2C0, (gpio0::Pin12<IOF0<NoInvert>>, gpio0::Pin13<IOF0<NoInvert>>)>;

/// May need to take a reference to something that writes to i2c, but uses locks rather than
/// this if we want to ever share the bus with other writers. Maybe not if we are single threaded.
pub struct Mpl311a2 {
    mode: Mode,
    i2c: I2cBaro,
    i2c_address: u8,
}

pub struct Mpl311a2Data {
    pub pressure: PressureData,
    pub temperature: f32,
}

/// Pressure data will be returned in either the form of altitude or pressure
#[derive(Debug)]
pub enum PressureData {
    /// Altitude in Meters 
    Altitiude(f32),
    /// Pressure in Pascals
    Pressure(f32),
}

/// Mpl311as can be used in either barometer or altimeter mode.
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum Mode {
    Barometer,
    Altimeter,
}

/// Custom errors for Mpl311a2 chip
#[derive(Debug)]
pub enum Error {
    // I2C error
    I2c(e310x_hal::i2c::Error),
    // Timeout while waiting for some value to be set.
    Timeout,
    // Whoami at i2c address is unknown.
    UnknownWhoAmI(u8),
}

impl Mpl311a2 {
    pub fn new(mode: Mode, oversample: Oversample, i2c: I2cBaro, i2c_address: u8) -> Result<Mpl311a2, Error> {
        let mut dev = Mpl311a2 { mode, i2c, i2c_address};

        match dev.read_reg(Register::WhoAmI) {
            Ok(0xC4) => {}
            Ok(c) => {
                sprintln!("Error whoami value {:x} is no 0xC4", c);
                return Err(Error::UnknownWhoAmI(c));
            }
            Err(e) => {
                return Err(e);
            }
        };

        dev.reset_device(1000)?;

        let control_value: u8;
        match mode {
            Mode::Altimeter => control_value = oversample as u8 | ControlOneMasks::Altitude as u8,
            Mode::Barometer => control_value = oversample as u8,
        }
        dev.write_reg(Register::ControlReg1, control_value)?;
        Delay.delay_ms(10u32);

        // Enable pressure and temperature event data flags.
        let pt_data_cfg_val = PtDataMasks::PtDataCfgTdefe as u8
            | PtDataMasks::PtDataCfgPdefe as u8
            | PtDataMasks::PtDataCfgDrem as u8;

        dev.write_reg(Register::PtDataConfig, pt_data_cfg_val)?;

        Ok(dev)
    }

    /// Use control register 1 to reset the device.
    fn reset_device(&mut self, timeout: u32) -> Result<(), Error> {
        // Issue software reset
        match self.i2c.write(self.i2c_address, &[Register::ControlReg1 as u8, ControlOneMasks::Reset as u8]) {
            Ok(_) | Err(e310x_hal::i2c::Error::NoAck) => {/* Stuff a NoAck error, or Ok, since the device is resetting*/},
            Err(e) => return Err(Error::I2c(e)), 
        };

        // Wait device to finish reset.
        for _ in 0..timeout / 10 {
            Delay.delay_ms(10u32);
            if let Ok(c) = self.read_reg(Register::ControlReg1) {
                // Poll Control register until software reset bit is set low again, indicating reset is complete.
                if (c & ControlOneMasks::Reset as u8) == 0 {
                    sprintln!("Device reset");
                    return Ok(());
                }
            }
        }

        sprintln! {"Timeout waiting for control register to reset"};
        return Err(Error::Timeout);
    }

    pub fn get_data(&mut self) -> Result<Mpl311a2Data, Error> {
        // Wait for oneshot bit to be 0
        self.wait_for_oneshot(1000)?;

        // Get current control register value.
        let mut control_value = self.read_reg(Register::ControlReg1)?;
        
        // Set OneShot bit to get measurement.
        control_value |= ControlOneMasks::OneShot as u8;
        self.write_reg(Register::ControlReg1, control_value)?;

        self.wait_for_data(StatusMasks::TempPressureDataReady, 1000)?;

        let pressure_data: PressureData = match self.mode {
            Mode::Altimeter => self.read_altitude()?,
            Mode::Barometer => self.read_pressure()?,
        };

        let temperature_data = self.read_temperature()?;

        Ok(
            Mpl311a2Data{
                pressure: pressure_data,
                temperature: temperature_data,
            }
        )
    }

    fn read_pressure(&mut self) -> Result<PressureData, Error> {
        // Get data measurement, we request the MSB first and the fields are auto incrementing
        let mut pressure_buf = [0; 0x3];
        if let Err(e) = self.i2c.write_read(
            self.i2c_address,
            &[Register::PressureMSB as u8],
            &mut pressure_buf,
        ) {
            sprintln! {"Error reading pressure data: {:?}", e};
            return Err(Error::I2c(e));
        }

        // Sensor data in Q18.2 format in Pascals
        // Bits 2..=19 form integer part
        // Bits 0..=1 form fracational part, hence the weird divide by 4.
        let pressure_sensor_data =
            // Bits 12..=19
            (pressure_buf[0] as u32) << 12
            // Bits 4..=11 
            | (pressure_buf[1] as u32) << 4
            // Bits 0..=3 
            | (pressure_buf[2] as u32) >> 4;
        let pressure: f32 = (pressure_sensor_data as f32) / 4.0;

        Ok(PressureData::Pressure(pressure))
    }

    /// Read pressure data from 
    pub fn read_altitude(&mut self) -> Result<PressureData, Error> {
        // Get data measurement, we request the MSB first and the fields are auto incrementing
        let mut pressure_buf = [0; 0x3];
        if let Err(e) = self.i2c.write_read(
            self.i2c_address,
            &[Register::PressureMSB as u8],
            &mut pressure_buf,
        ) {
            sprintln! {"Error reading pressure data: {:?}", e};
            return Err(Error::I2c(e));
        }

        // Sensor data in Q16.4 format in Meters
        // Bits 4..=19 form integer part
        // Bits 0..=3 form fractional part, hence the weird divide by 16.
        let pressure_sensor_data =
            // Bits 12..=19
            (pressure_buf[0] as u32) << 12
            // Bits 4..=11 
            | (pressure_buf[1] as u32) << 4
            // Bits 0..=3 
            | (pressure_buf[2] as u32) >> 4;
        let altitude: f32 = (pressure_sensor_data as f32) / 16.0;

        Ok(PressureData::Altitiude(altitude))
    }

    /// Read temperature data from temp data registers. 
    fn read_temperature(&mut self) -> Result<f32, Error> {
        // Get data measurement, we request the MSB first and the fields are auto incrementing
        let mut temperature_buf = [0; 0x2];
        if let Err(e) =
            self.i2c
                .write_read(self.i2c_address, &[Register::TempMSB as u8], &mut temperature_buf)
        {
            sprintln! {"Error reading temperature data: {:?}", e};
            return Err(Error::I2c(e));
        }

        // Sensor data in Q8.4 format in Celsius
        // Bits 4..=12 form integer part
        // Bits 0..=3 form fractional part, hence the weird divide by 16.
        let temperature_sensor_data =
            // Bits 4..=11 
            (temperature_buf[0] as u32) << 4
            // Bits 0..=3 
            | (temperature_buf[1] as u32) >> 4;

        let temperature: f32 = (temperature_sensor_data as f32) / 16.0;

        Ok(temperature)
    }

    /// Poll status register on a loop until status data specifed by mask is ready.
    /// Timeout in MS, should be in incrememnts of 10.
    fn wait_for_data(&mut self, mask: StatusMasks, timeout: u32) -> Result<(), Error> {
        for _ in 0..timeout / 10 {
            if let Ok(c) = self.read_reg(Register::Status) {
                if (c & mask as u8) != 0 {
                    return Ok(());
                }
            }
            Delay.delay_ms(10u32);
        }

        return Err(Error::Timeout);
    }

    /// Wait for oneshot bit to be clear
    fn wait_for_oneshot(&mut self, timeout: u32) -> Result<(), Error> {
        for _ in 0..=timeout / 10 {
            if let Ok(c) = self.read_reg(Register::ControlReg1) {
                if (c & (ControlOneMasks::OneShot as u8)) == 0 {
                    return Ok(());
                }
            }
            Delay.delay_ms(10u32);
        }

        sprintln! {"Timeout waiting for oneshot to be clear"};
        return Err(Error::Timeout);
    }

    /// Read a byte from a register.
    fn read_reg(&mut self, reg_addr: Register) -> Result<u8, Error> {
        let mut recv_buf = [0, 0x1];
        if let Err(e) = self
            .i2c
            .write_read(self.i2c_address, &[reg_addr as u8], &mut recv_buf)
        {
            sprintln! {"Error reading register {:?}: {:?}", reg_addr, e};
            return Err(Error::I2c(e));
        }
        Ok(recv_buf[0])
    }

    /// Write a byte to a register.
    fn write_reg(&mut self, reg_addr: Register, byte: u8) -> Result<(), Error> {
        if let Err(e) = self.i2c.write(self.i2c_address, &[reg_addr as u8, byte]) {
            sprintln!(
                "Error writing register {:?} with {}, {:?}",
                reg_addr,
                byte,
                e
            );
            return Err(Error::I2c(e));
        }
        Ok(())
    }
}

/// Constants and enums for the mpl311a2 altimeter.
/// Register Addresses
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub enum Register {
    Status = 0x00,
    PressureMSB = 0x01,
    PressureCSB = 0x02,
    PressureLSB = 0x03,
    TempMSB = 0x04,
    TempLSB = 0x05,
    DrStatus = 0x06,
    OutPressureDeltaMSB = 0x07,
    OutPressureDeltaCSB = 0x08,
    OutPressureDeltaLSB = 0x09,
    OutTempDeltaMSB = 0x0A,
    OutTempDeltaLSB = 0x0B,
    WhoAmI = 0x0C,
    PtDataConfig = 0x13,
    InputBarMSB = 0x14,
    InputBarLSB = 0x15,
    ControlReg1 = 0x26,
    ControlReg2 = 0x27,
    ControlReg3 = 0x28,
    ControlReg4 = 0x29,
    ControlReg5 = 0x2A,
}

/// Masks for status resgister bits, for data being ready to read.
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub enum StatusMasks {
    TempDataReady = 0x02,
    PressureDataReady = 0x04,
    TempPressureDataReady = 0x08,
}

/// Masks for pressure and temperature data register bits
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub enum PtDataMasks {
    PtDataCfgTdefe = 0x01,
    PtDataCfgPdefe = 0x02,
    PtDataCfgDrem = 0x04,
}

/// Masks for control register one bits
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub enum ControlOneMasks {
    Standby = 0x01,
    OneShot = 0x02,
    Reset = 0x04,
    Raw = 0x40,
    Altitude = 0x80,
    Bar = 0x00,
}

#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub enum Oversample {
    Oversample1 = 0x00,
    Oversample2 = 0x08,
    Oversample4 = 0x10,
    Oversample8 = 0x18,
    Oversample16 = 0x20,
    Oversample32 = 0x28,
    Oversample64 = 0x30,
    Oversample128 = 0x38,
}