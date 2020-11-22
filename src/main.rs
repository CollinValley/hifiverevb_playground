#![no_std]
#![no_main]

extern crate panic_halt;

mod console;

use riscv_rt::entry;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::{sprintln, pin};
// use hifive1::hal::serial::Rx;
// use e310x::UART0;


#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure UART for stdout
    let mut rx = hifive1::stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(), 
        clocks
    );

    sprintln!("Welcome to RiscV!......");
    sprintln!("_______________________");


    // Create a new console.
    let mut console = console::Console::new();
    loop {
        let read_character = rx.read();
        match read_character {
            Ok(c) => console.handle_character(c),
            Err(_) => {},
        }
    }
}

