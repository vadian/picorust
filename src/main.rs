//! # Pico Morse Code Example
//!
//! Accepts UART input and outputs Morse code via GP25, the on-board LED.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;
use core::fmt::Write;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use cortex_m::delay::Delay;
use rp2040_hal::gpio::*;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_9600_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.write_full_blocking(b"\nMorse Controller\n");
    uart.write_full_blocking(b"Press 'r' to reboot into flash mode\n");
    uart.write_full_blocking(b"Press 's' to send SOS\n");
    uart.write_full_blocking(b"Type 'm <message>' to send morse message\n");

    loop {        
        let mut buffer: [char; 1024] = [0 as char; 1024];
        buffer[1023] = '\n';
        let mut pos = 0;

        write!(uart, "Please enter a command:").unwrap();
        led_pin.set_high().unwrap();

        'reader: loop {
            let mut temp_buff: [u8; 1024] = [0; 1024];
            delay.delay_ms(500);
            let bytes = match uart.read_raw(&mut temp_buff) {
                Ok(bytes) =>  bytes,
                Err(_e) => {
                    continue;
                }
            };

            for i in 0..bytes {
                let x = temp_buff[i] as char;
                
                buffer[pos] = temp_buff[i] as char;
                write!(uart, "{}", buffer[pos]).unwrap();

                pos += 1;
                    
                if x == '\n' {
                    break 'reader;
                }
            }
        }
        led_pin.set_low().unwrap();

        write!(uart, "recieved {}:", pos-1).unwrap();
        for x in buffer{
            if x == 0 as char {
                break;
            }

            led_pin.set_high().unwrap();
            write!(uart, "{}", x).unwrap();
            led_pin.set_low().unwrap();
        }
        
        if buffer[0] == 'r' {
            // Reboot back into USB mode (no activity, both interfaces enabled)
            writeln!(uart, "Rebooting").unwrap();
            delay.delay_ms(1000);
            rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
        }

        if buffer[0] == 's' {
            led_pin.set_low().unwrap();
            delay.delay_ms(INTRA_CHARACTER);
            s(&mut delay, &mut led_pin, &mut uart);
            delay.delay_ms(INTER_CHARACTER);
            o(&mut delay, &mut led_pin, &mut uart);
            delay.delay_ms(INTER_CHARACTER);
            s(&mut delay, &mut led_pin, &mut uart);
        }
        if buffer[0] == 'm' {
            morse(&buffer[2..pos], &mut delay, &mut led_pin, &mut uart);
        }

        delay.delay_ms(1000);

    }
}

const DIT: u32 = 250;
const DAH: u32 = 3 * DIT;

const DOT: u32 = DIT;
const DASH: u32 = DAH;
const INTRA_CHARACTER: u32 = DIT;
const INTER_CHARACTER: u32 = DAH * 3;
const WORD_SPACE: u32 = 7 * DIT;

fn s(delay: &mut Delay, pin: &mut Pin<bank0::Gpio25, Output<PushPull>>, uart: &mut dyn Write ) {
    for i in 0..3 {
        write!(uart, "High S {}", i).unwrap();
        pin.set_high().unwrap();
        delay.delay_ms(DOT);
        writeln!(uart, " --> Low S {}", i).unwrap();
        pin.set_low().unwrap();
        if i != 2 {
            delay.delay_ms(INTRA_CHARACTER);
        }
    }
}

fn o(delay: &mut Delay, pin: &mut Pin<bank0::Gpio25, Output<PushPull>>, uart: &mut dyn Write) {
    for i in 0..3 {
        write!(uart, "High O {}", i).unwrap();
        pin.set_high().unwrap();
        delay.delay_ms(DASH);
        writeln!(uart, " --> LOW O {}", i).unwrap();
        pin.set_low().unwrap();
        if i != 2 {
            delay.delay_ms(INTRA_CHARACTER);
        }
    }
}

struct MorseCharacter {
    character: char,
    length: usize,
    code: [u32; 5]
}

fn morse(buffer: &[char], delay: &mut Delay, pin: &mut Pin<bank0::Gpio25, Output<PushPull>>, uart: &mut dyn Write ) {
    let characters:  [MorseCharacter; 2] = [
        MorseCharacter { character: 's', code: [DIT, DIT, DIT, 0, 0], length: 3},
        MorseCharacter { character: 'o', code: [DAH, DAH, DAH, 0, 0], length: 3},
        ];

    for i in 0..buffer.len() {
        for this_char in &characters {
            if this_char.character == buffer[i] {
                for symbol in 0..this_char.length {
                    pin.set_high().unwrap();
                    write!(uart, "High {} {}", this_char.character, this_char.code[symbol]).unwrap();
                    delay.delay_ms(this_char.code[symbol]);
                    pin.set_low().unwrap();
                    writeln!(uart, "--> Low {} {}", this_char.character, this_char.code[symbol]).unwrap();
                    if symbol+1 != this_char.length {
                        delay.delay_ms(INTRA_CHARACTER)
                    }
                }
                delay.delay_ms(INTER_CHARACTER);
            }
        }
        pin.set_high().unwrap();
        write!(uart, "High {}", buffer[i]).unwrap();
        pin.set_high().unwrap();
        delay.delay_ms(DOT);
        writeln!(uart, " --> Low S {}", i).unwrap();
        pin.set_low().unwrap();
        if i != 2 {
            delay.delay_ms(INTRA_CHARACTER);
        }
    }
}
