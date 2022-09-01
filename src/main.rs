//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

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
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
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

    // Blink the LED at 1 Hz
    loop {
        s(&mut delay, &mut led_pin);
        delay.delay_ms(INTER_CHARACTER);
        o(&mut delay, &mut led_pin);
        delay.delay_ms(INTER_CHARACTER);
        s(&mut delay, &mut led_pin);
        delay.delay_ms(WORD_SPACE);        
    }
}

const DIT: u32 = 250;
const DAH: u32 = 6 * DIT;

const DOT: u32 = DIT;
const DASH: u32 = DAH;
const INTRA_CHARACTER: u32 = DIT;
const INTER_CHARACTER: u32 = DAH * 3;
const WORD_SPACE: u32 = 7 * DIT;

fn s(delay: &mut Delay, pin: &mut Pin<bank0::Gpio25, Output<PushPull>>) {
    for i in 0..3 {
        pin.set_high().unwrap();
        delay.delay_ms(DOT);
        pin.set_low().unwrap();
        if i == 2 {
            delay.delay_ms(INTRA_CHARACTER);
        }
    }
}

fn o(delay: &mut Delay, pin: &mut Pin<bank0::Gpio25, Output<PushPull>>) {
    for i in 0..3 {
        pin.set_high().unwrap();
        delay.delay_ms(DASH);
        pin.set_low().unwrap();
        if i == 2 {
            delay.delay_ms(INTRA_CHARACTER);
        }
    }
}

// End of file
