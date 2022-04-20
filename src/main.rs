#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::{pixelcolor::*, prelude::*, primitives::*};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use embedded_time::rate::*;

use ssd1306::mode::DisplayConfig;

use xiao_rp2040 as bsp;

use bsp::hal;
use bsp::hal::gpio::DynPin;
use bsp::hal::prelude::*;
use bsp::pac;

#[bsp::entry]
fn main() -> ! {
    // start
    info!("Program start.");

    // initialize variables
    let mut v = [0u8; 16];
    let mut i = 0u16;
    let mut pc = 0x200usize;
    let mut st = 0u8;
    let mut dt = 0u8;
    let mut ram = [0u8; 0x1000];
    let mut vram = [[false; 64]; 32];
    let mut stack = [0u16; 16];
    let mut sp = 0usize;
    let mut keystatus = [false; 16];

    // load fonts
    let fonts = &[
        0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
        0x20, 0x60, 0x20, 0x20, 0x70, // 1
        0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
        0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
        0x90, 0x90, 0xF0, 0x10, 0x10, // 4
        0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
        0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
        0xF0, 0x10, 0x20, 0x40, 0x40, // 7
        0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
        0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
        0xF0, 0x90, 0xF0, 0x90, 0x90, // A
        0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
        0xF0, 0x80, 0x80, 0x80, 0xF0, // C
        0xE0, 0x90, 0x90, 0x90, 0xE0, // D
        0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
        0xF0, 0x80, 0xF0, 0x80, 0x80, // F
    ];
    ram[..fonts.len()].copy_from_slice(fonts);

    // load rom
    let rom = include_bytes!("../roms/BLINKY");
    ram[0x200..(0x200 + rom.len())].copy_from_slice(rom);

    // setup rp2040
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let sio = hal::Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // setup display
    let sda_pin = pins.gpio6.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio7.into_mode::<hal::gpio::FunctionI2C>();

    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        1.MHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );

    let interface = ssd1306::I2CDisplayInterface::new(i2c);

    let mut display = ssd1306::Ssd1306::new(
        interface,
        ssd1306::size::DisplaySize128x64,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().unwrap();
    display.clear();
    display.flush().unwrap();

    // setup buzzer
    let pwm_slice = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slice.pwm6;
    pwm.default_config();
    pwm.set_div_int(5);
    pwm.enable();

    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio29);
    channel.set_duty(0x0);

    // keyboard
    let mut rows: [&mut DynPin; 4] = [
        &mut pins.gpio3.into(),
        &mut pins.gpio4.into(),
        &mut pins.gpio2.into(),
        &mut pins.gpio1.into(),
    ];
    for pin in rows.iter_mut() {
        pin.into_push_pull_output();
        pin.set_high().unwrap();
    }

    let mut cols: [&mut DynPin; 4] = [
        &mut pins.gpio26.into(),
        &mut pins.gpio27.into(),
        &mut pins.gpio28.into(),
        &mut pins.gpio0.into(),
    ];
    for pin in cols.iter_mut() {
        pin.into_pull_up_input();
    }

    const KEYMAP: [usize; 16] = [
        0x1, 0x2, 0x3, 0xC, //
        0x4, 0x5, 0x6, 0xD, //
        0x7, 0x8, 0x9, 0xE, //
        0xA, 0x0, 0xB, 0xF, //
    ];

    let random_u8 = || -> u8 {
        let mut x = 0u8;
        for b in 0..8 {
            let bit = pac.ROSC.randombit.read().randombit().bits();
            x |= if bit { 1 << b } else { 0 };
        }
        x
    };

    // main loop
    loop {
        // reset vram flag
        let mut vram_changed = false;

        // delay timer
        if dt > 0 {
            dt -= 1;
        };

        // sound timer
        if st > 0 {
            st -= 1;
        };

        if st > 0 {
            channel.set_duty(0x0100);
        } else {
            channel.set_duty(0x0);
        }

        // scan keyboard
        for (r, row) in rows.iter_mut().enumerate() {
            row.set_low().unwrap();
            delay.delay_us(1);
            for (c, col) in cols.iter().enumerate() {
                keystatus[KEYMAP[r * 4 + c]] = col.is_low().unwrap()
            }
            row.set_high().unwrap();
        }

        // go to usb boot mode
        if keystatus[1] && keystatus[2] && keystatus[3] && keystatus[0xc] {
            display.clear();
            display.flush().unwrap();
            hal::rom_data::reset_to_usb_boot(0, 0);
            loop {
                cortex_m::asm::wfe();
            }
        }

        // fetch opecode
        let nibbles = (
            ram[pc] >> 4,
            ram[pc] & 0x0f,
            ram[pc + 1] >> 4,
            ram[pc + 1] & 0x0f,
        );
        let nnn = ((nibbles.1 as usize) << 8) | ((nibbles.2 as usize) << 4) | (nibbles.3 as usize);
        let kk = (nibbles.2 << 4) | nibbles.3;
        let x = nibbles.1 as usize;
        let y = nibbles.2 as usize;

        // run opecode
        match nibbles {
            // 0nnn: SYS addr
            (0x0, 0x0, 0x0, 0x0) => {
                pc = nnn;
            }
            // 00E0: CLS
            (0x0, 0x0, 0xE, 0x0) => {
                for y in vram.as_mut() {
                    for x in y {
                        *x = false;
                    }
                }
                pc += 2;
            }
            // 00EE: RET
            (0x0, 0x0, 0xE, 0xE) => {
                sp -= 1;
                pc = stack[sp] as usize;
            }
            // 1nnn: JP addr
            (0x1, _, _, _) => {
                pc = nnn;
            }
            // 2nnn: CALL addr
            (0x2, _, _, _) => {
                stack[sp] = pc as u16 + 2;
                sp += 1;
                pc = nnn;
            }
            // 3xkk: SE Vx, byte
            (0x3, _, _, _) => {
                if v[x] == kk {
                    pc += 2
                };
                pc += 2;
            }
            // 4xkk: SNE Vx, byte
            (0x4, _, _, _) => {
                if v[x] != kk {
                    pc += 2
                };
                pc += 2;
            }
            // 5xy0: SE Vx, Vy
            (0x5, _, _, _) => {
                if v[x] == v[y] {
                    pc += 2
                };
                pc += 2;
            }
            // 6xkk: LD Vx, byte
            (0x6, _, _, _) => {
                v[x] = kk;
                pc += 2
            }
            // 7xkk: ADD Vx, byte
            (0x7, _, _, _) => {
                let (a, _) = v[x].overflowing_add(kk);
                v[x] = a;
                pc += 2;
            }
            // 8xy0: LD Vx, Vy
            (0x8, _, _, 0x0) => {
                v[x] = v[y];
                pc += 2;
            }
            // 8xy1: OR Vx, Vy
            (0x8, _, _, 0x1) => {
                v[x] |= v[y];
                pc += 2;
            }
            // 8xy2: AND Vx, Vy
            (0x8, _, _, 0x2) => {
                v[x] &= v[y];
                pc += 2;
            }
            // 8xy3: XOR Vx, Vy
            (0x8, _, _, 0x3) => {
                v[x] ^= v[y];
                pc += 2;
            }
            // 8xy4: ADD Vx, Vy
            (0x8, _, _, 0x4) => {
                let (a, o) = v[x].overflowing_add(v[y]);
                v[x] = a;
                v[0xF] = if o { 1 } else { 0 };
                pc += 2;
            }
            // 8xy5: SUB Vx, Vy
            (0x8, _, _, 0x5) => {
                let (a, o) = v[x].overflowing_sub(v[y]);
                v[x] = a;
                v[0xF] = if !o { 1 } else { 0 };
                pc += 2;
            }
            // 8xy6: SHR Vx{, Vy}
            (0x8, _, _, 0x6) => {
                v[0xF] = v[x] & 0x1;
                v[x] >>= 1;
                pc += 2;
            }
            // 8xy7: SUBN Vx, Vy
            (0x8, _, _, 0x7) => {
                let (a, o) = v[y].overflowing_sub(v[x]);
                v[x] = a;
                v[0xF] = if !o { 1 } else { 0 };
                pc += 2;
            }
            // 8xyE: SHL Vx{, Vy}
            (0x8, _, _, 0xE) => {
                let (a, o) = v[x].overflowing_mul(2);
                v[x] = a;
                v[0xF] = if o { 1 } else { 0 };
                pc += 2;
            }
            // 9xy0: SNE Vx, Vy
            (0x9, _, _, 0) => {
                if v[x] != v[y] {
                    pc += 2
                };
                pc += 2;
            }
            // Annn: LD I, addr
            (0xA, _, _, _) => {
                i = nnn as u16;
                pc += 2;
            }
            // Bnnn JP Vo, addr
            (0xB, _, _, _) => {
                pc = i as usize + v[0] as usize;
            }
            // Cxkk: RND Vx, byte
            (0xC, _, _, _) => {
                v[x] = random_u8() & kk;
                pc += 2;
            }
            // Dxyn: DRW Vx, Vy, nibble
            (0xD, _, _, _) => {
                vram_changed = true;
                let mut flipped = false;
                for line in 0..nibbles.3 as usize {
                    let byte = ram[i as usize + line as usize];
                    for bit in 0..8_usize {
                        let vram_x = (v[x] as usize + bit) % 64;
                        let vram_y = (v[y] as usize + line) % 32;
                        let pixel = (byte >> (7 - bit)) & 0x1 > 0;
                        flipped |= vram[vram_y][vram_x] & pixel;
                        vram[vram_y][vram_x] ^= pixel;
                    }
                }
                v[0xF] = if flipped { 1 } else { 0 };
                pc += 2;
            }
            // Ex9E: SKP Vx
            (0xE, _, 0x9, 0xE) => {
                if keystatus[v[x] as usize] {
                    pc += 2
                };
                pc += 2;
            }
            // ExA1: SKNP Vx
            (0xE, _, 0xA, 0x1) => {
                if !keystatus[v[x] as usize] {
                    pc += 2
                };
                pc += 2
            }
            // Fx07: LD Vx, DT
            (0xF, _, 0x0, 0x7) => {
                v[x] = dt;
                pc += 2;
            }
            // Fx0A: KD Vx, K
            (0xF, _, 0x0, 0xA) => {
                let mut pressed = false;
                for (key, status) in keystatus.iter().enumerate() {
                    if *status {
                        pressed |= true;
                        v[x] = key as u8;
                    }
                }

                if pressed {
                    pc += 2
                }
            }
            // Fx15: LD DT, Vx
            (0xF, _, 0x1, 0x5) => {
                dt = v[x];
                pc += 2;
            }
            // Fx18: KD ST, Vx
            (0xF, _, 0x1, 0x8) => {
                st = v[x];
                pc += 2;
            }
            // Fx1E: ADD I, Vx
            (0xF, _, 0x1, 0xE) => {
                let (v, _) = i.overflowing_add(v[x] as u16);
                i = v;
                pc += 2;
            }
            // Fx29: LD F, Vx
            (0xF, _, 0x2, 0x9) => {
                i = v[x] as u16 * 5;
                pc += 2;
            }
            // Fx33: LD B, Vx
            (0xF, _, 0x3, 0x3) => {
                ram[i as usize] = v[x] / 100;
                ram[i as usize + 1] = v[x] % 100 / 10;
                ram[i as usize + 2] = v[x] % 10;
                pc += 2;
            }
            // Fx55: LD [I], Vx
            (0xF, _, 0x5, 0x5) => {
                for pos in 0..=x as usize {
                    ram[i as usize + pos] = v[pos];
                }
                pc += 2;
            }
            // Fx65: LD Vx, [I]
            (0xF, _, 0x6, 0x5) => {
                for pos in 0..=x as usize {
                    v[pos] = ram[i as usize + pos];
                }
                pc += 2;
            }
            _ => {}
        }

        // render
        if vram_changed {
            display.clear();
            for (y, line) in vram.iter().enumerate() {
                for (x, pixel) in line.iter().enumerate() {
                    if *pixel {
                        let point = Point::new(x as i32 * 2, y as i32 * 2);
                        let size = Size::new(2, 2);
                        let style = PrimitiveStyle::with_fill(BinaryColor::On);
                        let _ = Rectangle::new(point, size)
                            .into_styled(style)
                            .draw(&mut display);
                    }
                }
            }
            display.flush().unwrap();
        }
    }
}
