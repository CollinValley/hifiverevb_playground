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

        self.reset_device()?;

        // Set to altimeter with OSR of 128.
        let ctrl_reg_one_value = Oversample::Oversample128 as u8 | ControlOneMasks::Altitude as u8;

        self.write_reg(Register::ControlReg1, ctrl_reg_one_value)?;

        // Enable data flags.
        let pt_data_cfg_val = PtDataMasks::PtDataCfgTdefe as u8
            | PtDataMasks::PtDataCfgPdefe as u8
            | PtDataMasks::PtDataCfgDrem as u8;

        self.write_reg(Register::ControlReg1, pt_data_cfg_val)?;

        match self.mode {
            Mode::Poll => self.initialize_poll(),
            Mode::Interrupt => unimplemented!(),
            Mode::Fifo => unimplemented!(),
        }
    }

    /// Use control register 1 to reset the device.
    fn reset_device(&mut self) -> Result<(), ()> {
        // Issue software reset
        if let Err(e) = self.write_reg(Register::ControlReg1, ControlOneMasks::Reset as u8) {
            sprintln!("Error reseting device on startup: {:?}", e);
            return Err(());
        }
        Delay.delay_ms(10u32);

        // Wait device to finish reset.
        for i in 0..=100 {
            if i == 100 {
                sprintln! {"Timeout waiting for control register to reset"};
                return Err(());
            }
            if let Some(c) = self.read_reg(Register::ControlReg1) {
                // Poll Control register until software reset bit is set low again, indicating reset is complete.
                if (c & ControlOneMasks::Reset as u8) == 0 {
                    break;
                }
            }
            Delay.delay_ms(10u32);
        }
        Ok(())
    }

    fn initialize_poll(&mut self) -> Result<(), ()> {
        Ok(())
        // Could set the standby bit of control register one here,
        // but the adafruit library prefers the oneshot bit instead.
        // Do nothing for now.
    }

    pub fn get_pressure(&mut self) -> Result<u32, ()> {
        // Wait for oneshot bit to be 0
        for i in 0..=100 {
            if i == 100 {
                sprintln! {"GetPressure: Timeout waiting OneShot == 0"};
                return Err(());
            }
            if let Some(c) = self.read_reg(Register::ControlReg1) {
                if (c & ControlOneMasks::OneShot as u8) == 0 {
                    break;
                }
            }
            Delay.delay_ms(10u32);
        }

        // Set control register, without altitude measurement, disabling alititude.
        let mut control_value = Oversample::Oversample128 as u8;
        self.write_reg(Register::ControlReg1, control_value)?;

        // Set OneShot bit to get measurement.
        control_value |= ControlOneMasks::OneShot as u8;
        self.write_reg(Register::ControlReg1, control_value)?;

        // Wait for status to show pressure data available
        for i in 0..=100 {
            if i == 100 {
                sprintln! {"GetPressure: Timeout waiting OneShot == 0"};
                return Err(());
            }
            if let Some(c) = self.read_reg(Register::Status) {
                if (c & StatusMasks::PressureDataReady as u8) == 1 {
                    break;
                }
            }
            Delay.delay_ms(10u32);
        }

        //Get data measurement, we request the MSB first and the fields are auto incrementing
        let mut pressure_buf = [0; 0x3];
        if let Err(e) = self.i2c.write_read(
            I2CAddress,
            &[Register::PressureMSB as u8],
            &mut pressure_buf,
        ) {
            sprintln! {"Error reading pressure data: {:?}", e};
            return Err(());
        }
        let pressure =
            (pressure_buf[0] as u32) << 16 | (pressure_buf[1] as u32) << 8 | pressure_buf[2] as u32;

        Ok(pressure)
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
                "Error writing register {:?}, with {}, {:?}",
                reg_addr,
                byte,
                e
            );
            return Err(());
        }
        Ok(())
    }
}
