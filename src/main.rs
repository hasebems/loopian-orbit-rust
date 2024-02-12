//  Created by Hasebe Masahiko on 2023/08/22.
//  Copyright (c) 2023 Hasebe Masahiko.
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php
//
#![no_std]
#![no_main]

mod i2c_device;
mod lpn_chore;

//*******************************************************************
//          USE
//*******************************************************************
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::exception; // SysTick割り込み

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_probe as _;
use rp_pico as bsp;

use bsp::hal;
use bsp::hal::{
    adc::Adc, adc::AdcPin, clocks::init_clocks_and_plls, gpio::*, pac, pac::interrupt, sio::Sio,
    watchdog::Watchdog,
};
use cortex_m::prelude::_embedded_hal_adc_OneShot;

// for USB MIDI
use usb_device::{class_prelude::*, prelude::*};
use usbd_midi::data::midi::message::Message;
use usbd_midi::data::usb_midi::{
    cable_number::CableNumber, usb_midi_event_packet::UsbMidiEventPacket,
};
use usbd_midi::{
    //    data::byte::from_traits::FromClamped,
    data::usb::constants::USB_CLASS_NONE,
    midi_device::MidiClass,
};

use i2c_device::{Ada88, Mbr3110, Pca9544, Pca9685};
use lpn_chore::{
    DetectPosition, LoopClock, PositionLed, SwitchEvent, MAX_DEVICE_MBR3110, MAX_TOUCH_EV,
};
//use rp2040_hal::gpio::pin,

//*******************************************************************
//          Global Variable/DEF
//*******************************************************************
// 割り込みハンドラからハードウェア制御できるように、static変数にする
// Mutex<RefCell<Option<共有変数>>> = Mutex::new(RefCell::new(None));
static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut MIDI: Option<MidiClass<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

static mut PIN_LED: Option<hal::gpio::Pin<crate::bank0::Gpio25, FunctionSio<SioOutput>, PullDown>> =
    None;
static mut PIN_J44: Option<hal::gpio::Pin<crate::bank0::Gpio13, FunctionSio<SioInput>, PullDown>> =
    None;
static mut PIN_JSTK: Option<hal::gpio::Pin<crate::bank0::Gpio14, FunctionSio<SioInput>, PullDown>> =
    None;
static mut PIN_WHITELED_EN: Option<
    hal::gpio::Pin<crate::bank0::Gpio15, FunctionSio<SioOutput>, PullDown>,
> = None;
static mut PIN_LED_ERR: Option<
    hal::gpio::Pin<crate::bank0::Gpio16, FunctionSio<SioOutput>, PullDown>,
> = None;
static mut PIN_LED_1: Option<
    hal::gpio::Pin<crate::bank0::Gpio17, FunctionSio<SioOutput>, PullDown>,
> = None;
static mut PIN_LED_2: Option<
    hal::gpio::Pin<crate::bank0::Gpio22, FunctionSio<SioOutput>, PullDown>,
> = None;

fn lederr_off() {
    unsafe {
        if let Some(pin_lederr) = &mut PIN_LED_ERR {
            pin_lederr.set_low().unwrap(); // 消灯
        }
    }
}
fn lederr_on() {
    unsafe {
        if let Some(pin_lederr) = &mut PIN_LED_ERR {
            pin_lederr.set_high().unwrap();
        }
    }
}
fn en_whiteled() {
    unsafe {
        if let Some(pin_whiteled_en) = &mut PIN_WHITELED_EN {
            pin_whiteled_en.set_high().unwrap();
        }
    }
}
fn dis_whiteled() {
    unsafe {
        if let Some(pin_whiteled_en) = &mut PIN_WHITELED_EN {
            pin_whiteled_en.set_low().unwrap(); // Touch部白色LEDの Mute
        }
    }
}
fn led1_off() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_1 {
            pin_led.set_low().unwrap(); // 消灯
        }
    }
}
fn led1_on() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_1 {
            pin_led.set_high().unwrap();
        }
    }
}
fn led2_off() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_2 {
            pin_led.set_low().unwrap(); // 消灯
        }
    }
}
fn led2_on() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_2 {
            pin_led.set_high().unwrap();
        }
    }
}
fn ledboard_on() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED {
            pin_led.set_high().unwrap();
        }
    }
}
fn ledboard_off() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED {
            pin_led.set_low().unwrap();
        }
    }
}
//*******************************************************************
//          interrupt/exception
//*******************************************************************
// SysTickハンドラ
#[exception]
fn SysTick() {
    // クリティカルセクション内で操作
    free(|cs| {
        *COUNTER.borrow(cs).borrow_mut().deref_mut() += 1;
    });
}
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    if let Some(usb_dev) = USB_DEVICE.as_mut() {
        if let Some(midi) = MIDI.as_mut() {
            usb_dev.poll(&mut [midi]);
        }
    }
}
//*******************************************************************
//          main
//*******************************************************************
#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
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

    // GPIO
    let mut setup_mode: bool = false;
    let mut ledchk_mode: bool = false;
    unsafe {
        PIN_LED = Some(pins.led.into_push_pull_output());
        PIN_J44 = Some(pins.gpio13.into_pull_down_input());
        PIN_JSTK = Some(pins.gpio14.into_pull_down_input());
        PIN_WHITELED_EN = Some(pins.gpio15.into_push_pull_output());
        PIN_LED_ERR = Some(pins.gpio16.into_push_pull_output());
        PIN_LED_1 = Some(pins.gpio17.into_push_pull_output());
        PIN_LED_2 = Some(pins.gpio22.into_push_pull_output());
        if let Some(pin_swj44) = &mut PIN_J44 {
            ledchk_mode = pin_swj44.is_low().unwrap();
        }
        if let Some(pin_joysticksw) = &mut PIN_JSTK {
            setup_mode = pin_joysticksw.is_low().unwrap();
        }
    }
    dis_whiteled();
    lederr_off();
    led1_off();
    led2_off();
    ledboard_on();

    // Enable ADC
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = AdcPin::new(pins.gpio26);
    let mut adc_pin_1 = AdcPin::new(pins.gpio27);

    // SysTickの設定
    // 自前でSysTickを制御するときは cortex_m::delay::Delay が使えないので注意
    core.SYST.disable_counter();
    core.SYST.clear_current();
    // set_reloadで設定する値は、(割り込み周期のクロック数 - 1)
    // Raspberry Pi Picoでは、1クロック=1マイクロ秒。
    core.SYST.set_reload(1_000 - 1); // 1msec周期
    core.SYST.enable_interrupt();
    core.SYST.enable_counter();

    // I2C
    i2c_device::i2c0_init(
        pac.I2C0,
        pins.gpio20.into_function(),
        pins.gpio21.into_function(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
//    i2c_device::i2c1_init(
//        pac.I2C1,
//        pins.gpio18.into_function(),
//        pins.gpio19.into_function(),
//        &mut pac.RESETS,
//        &clocks.system_clock,
//    );
    Ada88::init();
    Ada88::write_letter(1);
    for i in 0..MAX_DEVICE_MBR3110 {
        Pca9544::change_i2cbus(0, 3, i);
        Pca9685::init(0, i + 16);
        Pca9544::change_i2cbus(0, 1, i);
    }

    // USB MIDI
    unsafe {
        USB_BUS = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));
    }
    setup_midi();

    // Application Setup mode
    let mut available_each_device = [true; MAX_DEVICE_MBR3110];
    if setup_mode {
        check_and_setup_board(ledchk_mode);
        // 戻ってこない
    } else {
        // Normal Mode
        let mut exist_err = 0;
        for i in 0..MAX_DEVICE_MBR3110 {
            Pca9544::change_i2cbus(0, 0, i);
            let err = Mbr3110::init(i);
            led1_on();
            if err != 0 {
                available_each_device[i] = false;
                exist_err = err;
            }
        }
        let mut disp_num: i32;
        if exist_err != 0 {
            lederr_on();
            disp_num = 20 + exist_err; // Error: 19:き, 18:ま,
            if disp_num >= 23 {
                disp_num = 23;
            }
            // Er
            else if disp_num < 0 {
                disp_num = 0;
            }
        } else {
            disp_num = 22; // OK
        }
        Ada88::write_letter(disp_num as usize);
        delay_msec(3000);
    }

    // Initialize Variables
    let mut lpclk = LoopClock::init();
    let mut dtct = DetectPosition::init();
    let mut pled = PositionLed::init();
    let mut swevt: [SwitchEvent; MAX_DEVICE_MBR3110] = Default::default();

    // Touch部白色LEDの Mute 解除
    en_whiteled();
    led1_off(); //test

    let mut err_number: i16;
    let mut ad_velocity: u8 = 100;
    loop {
        free(|cs| {
            lpclk.set_clock(*COUNTER.borrow(cs).borrow());
        });
        let ev1s = lpclk.event_1s();
        let ev10ms = lpclk.event_10ms();
        let tm = lpclk.get_ms();

        if ev10ms {
            let mut touch_someone = false;
            for i in 0..MAX_DEVICE_MBR3110 {
                if available_each_device[i] {
                    Pca9544::change_i2cbus(0, 0, i);
                    match Mbr3110::read_touch_sw(0, i) {
                        Ok(sw) => {
                            touch_someone |= swevt[i].update_sw_event(sw, tm);
                        }
                        Err(_err) => {
                            info!("Error!");
                            lederr_on();
                            err_number = (i as i16) * 100 + (_err as i16);
                            Ada88::write_number(err_number as i16);
                        }
                    }
                }
            }
            if touch_someone {
                // switch が押されている
                led2_on();
            } else {
                led2_off();
            }
            let finger = dtct.update_touch_position(&swevt, ad_velocity);
            if finger >= 2 {
                //  指を２本検出している
                led1_on();
            } else {
                led1_off();
            }
        }

        //  update touch location
        let tchev: [i32; MAX_TOUCH_EV] = dtct.interporate_location(tm);
        pled.gen_lighting_in_loop(tm, &tchev);

        //  display location
        let position = dtct.get_1st_position();
        if position != -1 {
            Ada88::write_number((position / 10) as i16);
        } else {
            Ada88::write_letter(0);
        }

        // ADC
        let pin_adx_value: u16 = adc.read(&mut adc_pin_0).unwrap();
        let pin_ady_value: u16 = adc.read(&mut adc_pin_1).unwrap();
        let ad_vel_temp = if pin_adx_value > pin_ady_value {
            pin_adx_value
        } else {
            pin_ady_value
        };
        let vel_temp = DetectPosition::get_velocity_from_adc(ad_vel_temp);
        if vel_temp != ad_velocity {
            ad_velocity = vel_temp;
            Ada88::write_number(vel_temp as i16);
        }

        // Heart beat
        if ev1s {
            if (lpclk.get_ms() / 1000) % 2 == 0 {
                ledboard_on();
            } else {
                ledboard_off();
            }
        }
    }
}
//*******************************************************************
//          System Functions
//*******************************************************************
fn check_and_setup_board(ledchk_mode: bool) {
    let mut sup_ok: bool = false;
    if ledchk_mode {
        const MAX_EACH_LIGHT: usize = 16;
        Ada88::write_letter(2); //"B"
        delay_msec(200);
        PositionLed::clear_all();
        en_whiteled();
        loop {
            for l in 0..2 {
                for e in 0..MAX_EACH_LIGHT {
                    let bright: u16 = if (e % 2) == 0 { l } else { (l + 1) % 2 };
                    PositionLed::light_led_each(e, 0, bright * 200);
                }
                delay_msec(200);
            }
        } //  無限ループ
    } else {
        // CapSense Setup Mode
        Ada88::write_letter(21); // SU
        for i in 0..MAX_DEVICE_MBR3110 {
            Pca9544::change_i2cbus(0, 0, i);
            let err = Mbr3110::setup_device(i);
            if err == 0 {
                Ada88::write_letter(22); // Ok
                sup_ok = true;
                for _ in 0..3 {
                    // when finished, flash 3times.
                    lederr_on();
                    delay_msec(100);
                    lederr_off();
                    delay_msec(100);
                }
                delay_msec(100);
                break;
            }
        }
    }

    if !sup_ok {
        Ada88::write_letter(23); // Er
                                 //lederr_on(); // Err LED on
    }

    loop {}
}
fn delay_msec(time: u32) {
    let mut first: Option<u32> = None;
    let mut diff = 0;
    let mut realtime = 0;
    loop {
        if diff > time {
            break;
        }
        free(|cs| {
            realtime = *COUNTER.borrow(cs).borrow();
        });
        match first {
            Some(f) => diff = realtime - f,
            None => first = Some(realtime),
        }
    }
}
//*******************************************************************
//          MIDI Out
//*******************************************************************
fn setup_midi() {
    unsafe {
        if let Some(usb_bus_ref) = USB_BUS.as_ref() {
            MIDI = Some(MidiClass::new(usb_bus_ref));
            USB_DEVICE = Some(
                UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x2e8a, 0x1248))
                    .product("Loopian-ORBIT")
                    .device_class(USB_CLASS_NONE)
                    .build(),
            );

            // Enable the USB interrupt
            pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        }
    }
}
fn output_midi_msg(message: Message) -> bool {
    // Send MIDI message
    let mut state: bool = false;
    unsafe {
        if let Some(midi) = MIDI.as_mut() {
            match midi.send_message(UsbMidiEventPacket {
                cable_number: CableNumber::Cable0,
                message,
            }) {
                Ok(_) => state = true,
                Err(_) => state = false,
            }
        }
    }
    delay_msec(1);
    state
}
// End of file
