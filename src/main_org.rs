//! Raspberry Pi Pico でLチカ (タイマーを利用)
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_time::duration::*;
use pac::interrupt;
use panic_probe as _;
use rp_pico::hal::gpio::{bank0::Gpio25, PushPullOutput};

use rp_pico as bsp;

use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};

// 割り込みハンドラからハードウェア制御できるように、static変数にする
// Mutex<RefCell<Option<共有変数>>> = Mutex::new(RefCell::new(None));
static G_LED: Mutex<RefCell<Option<bsp::hal::gpio::Pin<Gpio25, PushPullOutput>>>> =
    Mutex::new(RefCell::new(None));
static G_TIMER: Mutex<RefCell<Option<bsp::hal::timer::Timer>>> = Mutex::new(RefCell::new(None));
static G_ALARM0: Mutex<RefCell<Option<bsp::hal::timer::Alarm0>>> = Mutex::new(RefCell::new(None));

// タイマー割り込みハンドラ
#[interrupt]
fn TIMER_IRQ_0() {
    // クリティカルセクション内で操作
    free(|cs| {
        // 割り込みのクリアは必須 (クリアしないと割り込みハンドラが呼ばれ続ける!)
        // 必要であれば、次のアラームを設定する
        if let Some(ref mut a) = G_ALARM0.borrow(cs).borrow_mut().deref_mut() {
            let _ = a.schedule(100_000.microseconds()); // 0.1秒
            if let Some(ref mut t) = G_TIMER.borrow(cs).borrow_mut().deref_mut() {
                a.clear_interrupt(t);
            }
        }
        // LEDをトグルする
        if let Some(ref mut led) = G_LED.borrow(cs).borrow_mut().deref_mut() {
            let _ = led.toggle();
        }
    });
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let _clock = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LEDピンの初期化
    let mut led_pin = pins.led.into_push_pull_output();
    // Highに設定して、初回アラーム発生までLEDを点灯させる
    led_pin.set_high().unwrap();

    // タイマーの初期化
    // 初回アラーム 1秒後
    let mut timer = bsp::hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = timer.alarm_0().unwrap();
    let _ = alarm0.schedule(1_000_000.microseconds()); // 1秒
    alarm0.enable_interrupt(&mut timer);

    // 所有権をstatic変数に移す
    // 操作はクリティカルセクション内で行う
    free(|cs| {
        G_LED.borrow(cs).replace(Some(led_pin));
        G_TIMER.borrow(cs).replace(Some(timer));
        G_ALARM0.borrow(cs).replace(Some(alarm0));
    });

    // NVICでの割り込みマスク解除
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    };

    // メインループでは何もしない
    loop {}
}
