# `Collin's HiFive Playground`

> I copied this from the riscv-rust-quickstart template, highly recommmend the template. 


## Dependencies

To build embedded programs using this template you'll need:

- Rust 1.36 or a newer toolchain. e.g. `rustup default stable`

- The `cargo generate` subcommand. [Installation
  instructions](https://github.com/ashleygwilliams/cargo-generate#installation).

- `rust-std` components (pre-compiled `core` crate) for the RISC-V target. Run:

``` console
$ rustup target add riscv32imac-unknown-none-elf
```

- [RISC-V toolchain for SiFive boards](https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.1.0-2019.01.0-x86_64-linux-ubuntu14.tar.gz)

- Programmer software
  * HiFive1 Rev B: [Segger JLink software & documentation pack for Linux](https://www.segger.com/downloads/jlink/)
  * HiFive1: [OpenOCD from SiFive](https://static.dev.sifive.com/dev-tools/riscv-openocd-0.10.0-2019.02.0-x86_64-linux-ubuntu14.tar.gz) 

## Using this template

**NOTE**: This is the very short version that only covers building programs. For
the long version, which additionally covers flashing, running and debugging
programs, check [the embedded Rust book](https://rust-embedded.github.io/book).


**NOTE**: If you don't have `cargo generate` installed, use

    cargo install cargo-generate

to install it. 


1. Instantiate the template.

``` console
$ cargo generate --git https://github.com/riscv-rust/riscv-rust-quickstart
 Project Name: app
 Creating project called `app`...
 Done! New project created /tmp/app

$ cd app
```

2. If you have an old HiFive1 board, edit `Cargo.toml`:
replace `board-hifive1-revb` with `board-hifive1`.

3. Run the programmer software.
  * HiFive1 Rev B:
```sh
/path/to/JLinkGDBServer -device FE310 -if JTAG -speed 4000 -port 3333 -nogui
```
  * HiFive1:
```sh
/path/to/openocd -f board/sifive-hifive1.cfg
```

4. Build the template application or one of the examples.

``` console
$ cargo build
or
$ cargo build --example leds_blink
```

5. Run the template application or one of the examples.

``` console
$ cargo run
or
$ cargo run --example leds_blink
```

## Troubleshooting

Rust versions 1.45.0..1.45.2 constain [a bug](https://github.com/icebreaker-fpga/icebreaker-litex-examples/issues/6#issuecomment-667601893) that marks some of the ELF sections as PROGBITS.
This may result in huge binaries as well as overwrite the HiFive1 bootloader with zeros. It's recommended to use `beta` or `nightly` toolchain before Rust `1.46.0` comes out.

If your bootloader is damaged, you will not see the green led blinking when you reset the board. In this case, you can restore the bootloader with the procedure described in the [`hifive1-recover` repo](https://github.com/riscv-rust/hifive1-recover/).


## License (Collin Valley)
Copyright 2020 Collin Valley

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## License (riscv-rust-quickstart)
Copyright 2017-2019 [RISC-V team][team]

Permission to use, copy, modify, and/or distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright notice
and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
THIS SOFTWARE.

[team]: https://github.com/rust-embedded/wg#the-risc-v-team
