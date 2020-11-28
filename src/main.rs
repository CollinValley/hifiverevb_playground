#![no_std]
#![no_main]

extern crate panic_halt;

mod console;
mod barometer;

use riscv_rt::entry;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::{sprintln, pin};
use hifive1::hal::i2c::{Speed, I2c};


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
        clocks
    );

    // Configure I2C
    let sda = pin!(pins, i2c0_sda).into_iof0();
    let scl = pin!(pins, i2c0_scl).into_iof0();
    let mut i2c = I2c::new(p.I2C0, sda, scl, Speed::Normal, clocks);

    sprintln!("Welcome to RiscV!......");
    sprintln!("_______________________");


    // Create a new console.
    let mut console = console::Console::new();
    let mut barometer = barometer::Barometer::new(
        0x60, 
        barometer::Mode::Poll, 
        i2c
    );
    
    if let Err(_) = barometer.initialize() {
        sprintln!("Error starting barometer");
    }

    loop {
        // Handle console input
        if let Ok(c) = rx.read(){
            console.handle_character(c);
        }

        // Handle barometer
        if !barometer.ready() {
            let data = barometer.get_data();
            if data.pressure.is_some() {
                sprintln!("Got pressure: {}", data.pressure.unwrap());
            }
            if data.temperature.is_some() {
                sprintln!("Got temperature: {}", data.temperature.unwrap());
            }
        }
    }
}

