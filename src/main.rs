//! UART and multicore-fifo and timer
//! 
#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;


// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_time::fixed_point::FixedPoint;
use hal::clocks::Clock;
use hal::multicore::{Multicore, Stack};
use hal::sio::Sio;
use embedded_hal::digital::v2::ToggleableOutputPin;

// Alias for our HAL crate
use rp2040_hal as hal;

/// 
const CORE1_TASK_COMPLETE: u32 = 0xEE;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// stak for core 1
/// core1.spqwnにより任意のサイズのスタックを確保する。
/// (core0はデフォルトで確保される)
static mut CORE1_STACK: Stack<4096> = Stack::new();
fn core1_task() -> !{
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    let mut sio1 = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio1.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    // core1とcore0で鑑賞するのでdelayに使うsystickはfifoを経由する。
    let sys_freq = sio1.fifo.read_blocking();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);
    loop {
        let fifoinput = sio1.fifo.read();
        // word 
        // 
        if let Some(word) = fifoinput {
            delay.delay_ms(word);
            led_pin.toggle().unwrap();
            sio1.fifo.write_blocking(CORE1_TASK_COMPLETE);
        }
    }
    

}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut sio0 = hal::sio::Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio0);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(core1_task, unsafe { &mut CORE1_STACK.mem });

    let sys_freq = clocks.system_clock.freq().integer();

    // sys_freqを送信する
    sio0.fifo.write_blocking(sys_freq);
    const LED_PERIOD_INCREMENT: i32 = 2;
    const LED_PERIOD_MIN: i32 = 0;
    const LED_PERIOD_MAX: i32 = 100;
    let mut led_period: i32 = LED_PERIOD_MIN;

    let mut count_up = true;
    loop {
        if count_up {
            // toggle 
            led_period += LED_PERIOD_INCREMENT;
            if led_period > LED_PERIOD_MAX {
                led_period = LED_PERIOD_MAX;
                count_up = false;
            }
        } 
        else{
            led_period -= LED_PERIOD_INCREMENT;
            if led_period < LED_PERIOD_MIN{
                led_period = LED_PERIOD_MIN;
                count_up = true;
            }
        }

        // this if statement will not execute.
        if led_period < 0 {
            led_period = 0;
        }

        sio0.fifo.write(led_period as u32);
        let ack = sio0.fifo.read_blocking();
        if ack != CORE1_TASK_COMPLETE {
            
        }

    }
}

// End of file