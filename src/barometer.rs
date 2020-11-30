use crate::mpl311a2::{
    ControlOneMasks, I2CAddress, Oversample, PtDataMasks, Register, StatusMasks,
};
use e310x::I2C0;
use e310x_hal::gpio::{gpio0, NoInvert, IOF0};
use e310x_hal::prelude::*;
use hifive1::hal::delay::Delay;
use hifive1::hal::i2c::I2c;
use hifive1::sprintln;

pub type I2cBaro = I2c<I2C0, (gpio0::Pin12<IOF0<NoInvert>>, gpio0::Pin13<IOF0<NoInvert>>)>;

/// May need to take a reference to something that writes to i2c, but uses locks rather than
/// this if we want to ever share the bus with other writers. Maybe not if we are single threaded.
pub struct Barometer {
    mode: Mode,
    i2c: I2cBaro,
}

#[allow(dead_code)]
pub enum Mode {
    Poll,
    Interrupt,
    Fifo,
}

impl Barometer {
    pub fn new(mode: Mode, i2c: I2cBaro) -> Barometer {
        Barometer { mode, i2c }
    }

    /// Initialize barometer for communication. Works based
    /// on the mode set.
    // TODO: Return real errors.
    pub fn initialize(&mut self) -> Result<(), ()> {
        // Check for correct whoami
        match self.read_reg(Register::WhoAmI) {
            Some(0xC4) => {}
            Some(c) => {
                sprintln!("Error whoami value {:x} is no 0xC4", c);
                return Err(());
            }
            None => {
                return Err(());
            }
        };

        self.reset_device(1000)?;

        // Set to altimeter mode with OSR of 128.
        let ctrl_reg_one_value = Oversample::Oversample128 as u8 | ControlOneMasks::Altitude as u8;
        self.write_reg(Register::ControlReg1, ctrl_reg_one_value)?;
        Delay.delay_ms(10u32);

        // Enable data flags.
        let pt_data_cfg_val = PtDataMasks::PtDataCfgTdefe as u8
            | PtDataMasks::PtDataCfgPdefe as u8
            | PtDataMasks::PtDataCfgDrem as u8;

        self.write_reg(Register::PtDataConfig, pt_data_cfg_val)?;

        match self.mode {
            Mode::Poll => self.initialize_poll(),
            Mode::Interrupt => unimplemented!(),
            Mode::Fifo => unimplemented!(),
        }
    }

    /// Use control register 1 to reset the device.
    fn reset_device(&mut self, timeout: u32) -> Result<(), ()> {
        // Issue software reset
        self.write_reg(Register::ControlReg1, ControlOneMasks::Reset as u8)?;
        Delay.delay_ms(10u32);

        // Wait device to finish reset.
        for _ in 0..timeout / 10 {
            if let Some(c) = self.read_reg(Register::ControlReg1) {
                // Poll Control register until software reset bit is set low again, indicating reset is complete.
                if (c & ControlOneMasks::Reset as u8) == 0 {
                    sprintln!("Device reset");
                    return Ok(());
                }
            }
            Delay.delay_ms(10u32);
        }

        sprintln! {"Timeout waiting for control register to reset"};
        return Err(());
    }

    fn initialize_poll(&mut self) -> Result<(), ()> {
        Ok(())
        // Could set the standby bit of control register one here,
        // but the adafruit library prefers the oneshot bit instead.
        // Do nothing for now.
    }

    pub fn get_pressure(&mut self) -> Result<f32, ()> {
        // Wait for oneshot bit to be 0
        self.wait_for_oneshot(100)?;

        // Set control register, barometer mode.
        let mut control_value = Oversample::Oversample4 as u8;
        self.write_reg(Register::ControlReg1, control_value)?;

        // Set OneShot bit to get measurement.
        control_value |= ControlOneMasks::OneShot as u8;
        self.write_reg(Register::ControlReg1, control_value)?;

        self.wait_for_data(StatusMasks::PressureDataReady, 100)?;

        // Get data measurement, we request the MSB first and the fields are auto incrementing
        let mut pressure_buf = [0; 0x3];
        if let Err(e) = self.i2c.write_read(
            I2CAddress,
            &[Register::PressureMSB as u8],
            &mut pressure_buf,
        ) {
            sprintln! {"Error reading pressure data: {:?}", e};
            return Err(());
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

        Ok(pressure)
    }

    pub fn get_altitude(&mut self) -> Result<f32, ()> {
        // Wait for oneshot bit to be 0
        self.wait_for_oneshot(100)?;

        // Set control register, altimeter mode.
        let mut control_value = Oversample::Oversample4 as u8 | ControlOneMasks::Altitude as u8;
        self.write_reg(Register::ControlReg1, control_value)?;

        // Set OneShot bit to get measurement.
        control_value |= ControlOneMasks::OneShot as u8;
        self.write_reg(Register::ControlReg1, control_value)?;

        self.wait_for_data(StatusMasks::PressureDataReady, 100)?;

        // Get data measurement, we request the MSB first and the fields are auto incrementing
        let mut pressure_buf = [0; 0x3];
        if let Err(e) = self.i2c.write_read(
            I2CAddress,
            &[Register::PressureMSB as u8],
            &mut pressure_buf,
        ) {
            sprintln! {"Error reading pressure data: {:?}", e};
            return Err(());
        }

        // Sensor data in Q16.4 format in Meters
        // Bits 4..=19 form integer part
        // Bits 0..=3 form fracational part, hence the weird divide by 16.
        let pressure_sensor_data =
            // Bits 12..=19
            (pressure_buf[0] as u32) << 12
            // Bits 4..=11 
            | (pressure_buf[1] as u32) << 4
            // Bits 0..=3 
            | (pressure_buf[2] as u32) >> 4;
        let altitude: f32 = (pressure_sensor_data as f32) / 16.0;

        Ok(altitude)
    }

    /// Poll status register on a loop until status data specifed by mask is ready.
    /// Timeout in MS, should be in incrememnts of 10.
    fn wait_for_data(&mut self, mask: StatusMasks, timeout: u32) -> Result<(), ()> {
        for _ in 0..timeout / 10 {
            if let Some(c) = self.read_reg(Register::Status) {
                if (c & mask as u8) != 0 {
                    return Ok(());
                }
            }
            Delay.delay_ms(10u32);
        }

        sprintln! {"Timeout waiting for status with mask {:?}", mask};
        return Err(());
    }

    /// Wait for oneshot bit to be clear
    fn wait_for_oneshot(&mut self, timeout: u32) -> Result<(), ()> {
        for _ in 0..=timeout / 10 {
            if let Some(c) = self.read_reg(Register::ControlReg1) {
                if (c & (ControlOneMasks::OneShot as u8)) == 0 {
                    return Ok(());
                }
            }
            Delay.delay_ms(10u32);
        }

        sprintln! {"Timeout waiting for oneshot to be clear"};
        return Err(());
    }

    /// Read a byte from a register.
    fn read_reg(&mut self, reg_addr: Register) -> Option<u8> {
        let mut recv_buf = [0, 0x1];
        if let Err(e) = self
            .i2c
            .write_read(I2CAddress, &[reg_addr as u8], &mut recv_buf)
        {
            sprintln! {"Error reading register {:?}: {:?}", reg_addr, e};
            return None;
        }
        Some(recv_buf[0])
    }

    /// Write a byte to a register.
    fn write_reg(&mut self, reg_addr: Register, byte: u8) -> Result<(), ()> {
        if let Err(e) = self.i2c.write(I2CAddress, &[reg_addr as u8, byte]) {
            sprintln!(
                "Error writing register {:?} with {}, {:?}",
                reg_addr,
                byte,
                e
            );
            return Ok(());
        }
        Ok(())
    }
}
