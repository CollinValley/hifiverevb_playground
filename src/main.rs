#![no_std]
#![no_main]

extern crate panic_halt;

mod console;
mod mpl311a2;

use hifive1::hal::i2c::{I2c, Speed};
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::{pin, sprintln};
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 100.mhz().into());

    // Configure UART for stdout
    let mut rx = hifive1::stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    // Configure I2C
    let sda = pin!(pins, i2c0_sda).into_iof0();
    let scl = pin!(pins, i2c0_scl).into_iof0();
    let i2c = I2c::new(p.I2C0, sda, scl, Speed::Normal, clocks);

    sprintln!("Welcome to RiscV!......");
    sprintln!("_______________________");

    // Create a new console.
    let mut console = console::Console::new();
    let mut barometer = mpl311a2::Mpl311a2::new(
        mpl311a2::Mode::Barometer,
        mpl311a2::Oversample::Oversample128,
        i2c,
        0x60,
    ).unwrap();

    sprintln!("Barometer started");

    loop {
        // Handle console input
        /*
        if let Ok(c) = rx.read() {
            console.handle_character(c);
        }
        */

        match barometer.get_data() {
            Ok(data) => {
                sprintln! {"Got data: {:?}, Temperature {} ", data.pressure, data.temperature};
            }
            Err(_) => sprintln!("Error getting data"),
        };
    }
}
