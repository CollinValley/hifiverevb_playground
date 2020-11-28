use hifive1::hal::i2c::I2c;
use hifive1::sprintln;
use e310x::I2C0;
use e310x_hal::gpio::{gpio0, IOF0, NoInvert};
use e310x_hal::prelude::*;

pub type I2cBaro = I2c<I2C0, (gpio0::Pin12<IOF0<NoInvert>>, gpio0::Pin13<IOF0<NoInvert>>)>;

/// May need to take a reference to something that writes to i2c, but uses locks rather than 
/// this if we want to ever share the bus with other writers. Maybe not if we are single threaded.
pub struct Barometer {
    address: u8,
    mode: Mode,
    i2c: I2cBaro,
}

pub enum Mode {
    Poll,
    Interrupt,
    Fifo,
}

pub struct Data {
    pub pressure: Option<u32>,
    pub temperature: Option<u32>,
}

impl Barometer {
    pub fn new(address: u8, mode: Mode, i2c: I2cBaro) -> Barometer {
        Barometer {
            address,
            mode,
            i2c,
        }
    }

    /// Initialize barometer for communication. Works based
    /// on the mode set.
    // TODO: Return real errors.
    pub fn initialize(&mut self) -> Result<(), ()> {
        // TODO: Check whoami

        // Basic initialization
        match self.i2c.write(self.address, &[0x26, 0xB8, 0x13, 0x07]) {
            Ok(_) => {},
            Err(e) => { 
                sprintln!("Error during startup: {:?}", e);
                return Err(()); 
            },
        };

        match self.mode {
            Mode::Poll => self.initialize_poll(),
            Mode::Interrupt => unimplemented!(),
            Mode::Fifo => unimplemented!(),
        }
    }

    fn initialize_poll(&mut self) -> Result<(), ()> {
        match self.i2c.write(self.address, &[0x26, 0xB9]) {
            Ok(_) => Ok(()),
            Err(e) => {
                sprintln!("Error in polling setup: {:?}", e);
                Err(())
            }
        }
    }
    
    /// Return true if data from barometer is ready to read
    pub fn ready(&mut self) -> bool {
        let mut recv_buff = [0; 0x10];
        if let Err(e) = self.i2c.write_read(self.address, &[0x00], &mut recv_buff) {
            sprintln!("Error reading status register: {:?}", e);
            return false;
        }
        // Check and return status boolean.
        sprintln!{"Got status: {}", recv_buff[0]};
        recv_buff[0] & 0x08 != 0
    }

    /// Make a data request to barometer, return Nones for values that are 0.
    pub fn get_data(&mut self) -> Data {
        let mut data_buf = [0; 0x05];

        // Read data registers 0x1..=0x4; Auto incrementing.
        if let Err(e) = self.i2c.write_read(self.address, &[0x01], &mut data_buf){
            sprintln!("Error reading barometer data: {:?}", e);
        }

        let pressure = (data_buf[0] as u32) << 16 & (data_buf[1] as u32) << 8 & (data_buf[2] as u32); 
        let temperature = (data_buf[3] as u32) << 8 & (data_buf[4] as u32);

        match (pressure, temperature) {
            (0, 0) => Data{ pressure: None, temperature: None },
            (0, t) => Data { pressure: None, temperature: Some(t)},
            (p, 0) => Data { pressure: Some(p), temperature: None },
            (p, t) => Data { pressure: Some(p), temperature: Some(t)},
        }
    }
}