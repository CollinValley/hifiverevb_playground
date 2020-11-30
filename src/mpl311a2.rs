
/// Constants and enums for the mpl311a2 altimeter.

/// I2C address of the barometer.
#[allow(non_upper_case_globals)]
#[allow(dead_code)]
pub const I2CAddress: u8 = 0x60;

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
    PtDataCfg = 0x13,
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
