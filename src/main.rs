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
    DetectPosition, LoopClock, PositionLed, SwitchEvent, MAX_KAMABOKO_NUM, MAX_TOUCH_EV,
};
//use rp2040_hal::gpio::pin,

//*******************************************************************
//          Global Variable/DEF
//*******************************************************************
// 割り込みハンドラからハードウェア制御できるように、static変数にする
// Mutex<RefCell<Option<共有変数>>> = Mutex::new(RefCell::new(None));
static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0)); //49日で一回り
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut MIDI: Option<MidiClass<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut ADC: Option<Adc> = None;

static mut PIN_LED: Option<hal::gpio::Pin<crate::bank0::Gpio25, FunctionSio<SioOutput>, PullDown>> =
    None;
static mut PIN_J44: Option<hal::gpio::Pin<crate::bank0::Gpio13, FunctionSio<SioInput>, PullDown>> =
    None;
static mut PIN_JSTK: Option<hal::gpio::Pin<crate::bank0::Gpio14, FunctionSio<SioInput>, PullDown>> =
    None;
static mut ADC_PIN_1: Option<
    AdcPin<
        rp_pico::hal::gpio::Pin<crate::bank0::Gpio26, FunctionNull, rp_pico::hal::gpio::PullDown>,
    >,
> = None;
static mut ADC_PIN_2: Option<
    AdcPin<
        rp_pico::hal::gpio::Pin<crate::bank0::Gpio27, FunctionNull, rp_pico::hal::gpio::PullDown>,
    >,
> = None;
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

pub fn lederr_off() {
    unsafe {
        if let Some(pin_lederr) = &mut PIN_LED_ERR {
            pin_lederr.set_low().unwrap(); // 消灯
        }
    }
}
pub fn lederr_on() {
    unsafe {
        if let Some(pin_lederr) = &mut PIN_LED_ERR {
            pin_lederr.set_high().unwrap();
        }
    }
}
pub fn en_whiteled() {
    unsafe {
        if let Some(pin_whiteled_en) = &mut PIN_WHITELED_EN {
            pin_whiteled_en.set_high().unwrap();
        }
    }
}
pub fn dis_whiteled() {
    unsafe {
        if let Some(pin_whiteled_en) = &mut PIN_WHITELED_EN {
            pin_whiteled_en.set_low().unwrap(); // Touch部白色LEDの Mute
        }
    }
}
pub fn led1_off() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_1 {
            pin_led.set_low().unwrap(); // 消灯
        }
    }
}
pub fn led1_on() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_1 {
            pin_led.set_high().unwrap();
        }
    }
}
pub fn led2_off() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_2 {
            pin_led.set_low().unwrap(); // 消灯
        }
    }
}
pub fn led2_on() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED_2 {
            pin_led.set_high().unwrap();
        }
    }
}
pub fn ledboard_on() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED {
            pin_led.set_high().unwrap();
        }
    }
}
pub fn ledboard_off() {
    unsafe {
        if let Some(pin_led) = &mut PIN_LED {
            pin_led.set_low().unwrap();
        }
    }
}
pub fn get_joystick_position_x() -> u16 {
    // 0-4095
    let mut pin_adx_value: u16 = 0;
    unsafe {
        if let Some(adc) = &mut ADC {
            if let Some(adc1) = &mut ADC_PIN_1 {
                pin_adx_value = adc.read(adc1).unwrap();
            }
        }
    }
    pin_adx_value
}
pub fn get_joystick_position_y() -> u16 {
    // 0-4095
    let mut pin_ady_value: u16 = 0;
    unsafe {
        if let Some(adc) = &mut ADC {
            if let Some(adc2) = &mut ADC_PIN_2 {
                pin_ady_value = adc.read(adc2).unwrap();
            }
        }
    }
    pin_ady_value
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
    //let mut ledchk_mode: bool = false;
    unsafe {
        PIN_LED = Some(pins.led.into_push_pull_output());
        PIN_J44 = Some(pins.gpio13.into_pull_down_input());
        PIN_JSTK = Some(pins.gpio14.into_pull_down_input());
        PIN_WHITELED_EN = Some(pins.gpio15.into_push_pull_output());
        PIN_LED_ERR = Some(pins.gpio16.into_push_pull_output());
        PIN_LED_1 = Some(pins.gpio17.into_push_pull_output());
        PIN_LED_2 = Some(pins.gpio22.into_push_pull_output());
        //if let Some(pin_swj44) = &mut PIN_J44 {
        //    ledchk_mode = pin_swj44.is_low().unwrap();
        //}
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
    unsafe {
        ADC = Some(Adc::new(pac.ADC, &mut pac.RESETS));
        // Configure GPIO26 as an ADC input
        ADC_PIN_1 = Some(AdcPin::new(pins.gpio26));
        ADC_PIN_2 = Some(AdcPin::new(pins.gpio27));
    }

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

    for i in 0..MAX_KAMABOKO_NUM {
        Pca9544::change_i2cbus(0, 3, i);
        Pca9685::init(0, i + 16); // delay_msec() の後に置くと、LEDが光らない
        Pca9544::change_i2cbus(0, 1, i);
    }

    // Opening
    for i in 0..4 {
        Ada88::write_letter(26 + i);
        delay_msec(300);
    }
    for i in 0..3 {
        Ada88::write_letter(28 - i);
        delay_msec(300);
    }
    Ada88::write_number(59); // version No. 一の位は9固定
    delay_msec(500);

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
    let mut available_each_device = [false; 16];
    if setup_mode {
        check_and_setup_board();
        // 戻ってこない
    } else {
        // Normal Mode
        let mut exist_err = 0;
        for i in 0..MAX_KAMABOKO_NUM {
            Pca9544::change_i2cbus(0, 0, i);
            match Mbr3110::init(i) {
                Ok(_) => {
                    available_each_device[i] = true;
                    led1_on();
                    Ada88::write_number((i as i16) * 10);
                }
                Err(err) => {
                    available_each_device[i] = false;
                    exist_err = err;
                    led1_off();
                    Ada88::write_letter(25); //--
                }
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
        delay_msec(2000);
        Ada88::write_org_bit(available_each_device);
        delay_msec(2000);
    }

    // Initialize Variables
    let mut lpclk = LoopClock::init();
    let mut dtct = DetectPosition::init();
    let mut pled = PositionLed::init();
    let mut swevt: [SwitchEvent; MAX_KAMABOKO_NUM] = Default::default();

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
            for i in 0..MAX_KAMABOKO_NUM {
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
        let _pin_adx_value: u16 = get_joystick_position_x();
        let pin_ady_value: u16 = get_joystick_position_y();
        let vel_temp = DetectPosition::get_velocity_from_adc(pin_ady_value);
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
fn check_and_setup_board() {
    let mut selmode: i32 = 0;
    let mut incdec_old = 0;
    const SELMODE_MAX: i32 = 13;

    Ada88::write_letter(21); // SU
    delay_msec(5000);
    loop {
        let adval: u16 = get_joystick_position_x();
        let mut incdec = 0;
        if adval > 3000 {
            incdec = 1;
        } else if adval < 1000 {
            incdec = -1;
        }
        if incdec != incdec_old {
            if incdec > 0 {
                selmode += 1;
                if selmode >= SELMODE_MAX {
                    selmode = 0;
                }
            } else if incdec < 0 {
                selmode -= 1;
                if selmode < 0 {
                    selmode = SELMODE_MAX - 1;
                }
            }
            incdec_old = incdec;
        }
        let cnt = get_msec_counter();
        display_setup(selmode, cnt);
        let mut swon: bool = false;
        unsafe {
            if let Some(pin_joysticksw) = &mut PIN_JSTK {
                swon = pin_joysticksw.is_low().unwrap();
            }
        }
        if swon {
            break;
        }
    }

    if selmode == 12 {
        const MAX_EACH_LIGHT: usize = 16;
        Ada88::write_letter(24); // "LE"
        delay_msec(200);
        PositionLed::clear_all();
        en_whiteled();
        loop {
            for l in 0..2 {
                for k in 0..MAX_KAMABOKO_NUM {
                    for e in 0..MAX_EACH_LIGHT {
                        let bright: u16 = if (e % 2) == 0 { l } else { (l + 1) % 2 };
                        PositionLed::light_led_each(e, k, bright * 200);
                    }
                }
                delay_msec(200);
            }
        } //  無限ループ
    } else {
        // CapSense Setup Mode
        Ada88::write_number((selmode * 10) as i16);
        let success = setup_mbr(selmode as usize);
        if success == 0 {
            Ada88::write_letter(22); // Ok
        } else if success == 1 {
            Ada88::write_letter(25); // --
        } else {
            Ada88::write_letter(23); // Er
            lederr_on(); // Err LED on
        }
    }

    loop {}
}
fn display_setup(sup: i32, counter: u32) {
    if (counter % 400) > 200 {
        if sup < 12 {
            Ada88::write_number((sup * 10) as i16);
        } else {
            Ada88::write_letter(24); // "LE"
        }
    } else {
        Ada88::write_letter(0);
    }
}
fn setup_mbr(num: usize) -> i32 {
    match Mbr3110::setup_device(num) {
        Ok(v) => {
            if v == 0 {
                // 書き込みしてOKだった場合
                for _ in 0..3 {
                    // when finished, flash 3times.
                    lederr_on();
                    delay_msec(100);
                    lederr_off();
                    delay_msec(100);
                }
                delay_msec(500);
                0
            } else {
                1
            }
        }
        Err(_) => -1,
    }
}
pub fn delay_msec(time: u32) {
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
pub fn get_msec_counter() -> u32 {
    let mut realtime = 0;
    free(|cs| {
        realtime = *COUNTER.borrow(cs).borrow();
    });
    realtime
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
