//  Created by Hasebe Masahiko on 2023/08/22.
//  Copyright (c) 2023 Hasebe Masahiko.
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php
//
#![no_std]
#![no_main]

mod i2c_device;

//*******************************************************************
//          USE
//*******************************************************************
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
//use crate::hal::gpio::bank0::Gpio25;
//use crate::hal::gpio::PushPullOutput;
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::exception; // SysTick割り込み

use rp_pico as bsp;
use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_probe as _;

use bsp::hal;
use bsp::hal::{
    clocks::init_clocks_and_plls,
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
    adc::Adc,
    adc::AdcPin,
};
use cortex_m::prelude::_embedded_hal_adc_OneShot;

//use fugit::RateExtU32;
use i2c_device::{Ada88, I2cEnv, Mbr3110, Pca9544};

// for USB MIDI
use usb_device::{class_prelude::*, prelude::*};
use usbd_midi::data::midi::{channel::Channel, message::Message, notes::Note};
use usbd_midi::data::usb_midi::{
    cable_number::CableNumber, usb_midi_event_packet::UsbMidiEventPacket,
};
use usbd_midi::{
    data::byte::from_traits::FromClamped, data::usb::constants::USB_CLASS_NONE,
    midi_device::MidiClass,
};
//*******************************************************************
//          Global Variable/DEF
//*******************************************************************
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut MIDI: Option<MidiClass<hal::usb::UsbBus>> = None;

const MAX_DEVICE_MBR3110: usize = 6;
// 割り込みハンドラからハードウェア制御できるように、static変数にする
// Mutex<RefCell<Option<共有変数>>> = Mutex::new(RefCell::new(None));
static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

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
    let mut led_pin = pins.led.into_push_pull_output();
    let mut exledsw_pin = pins.gpio15.into_push_pull_output();
    let mut exled_err_pin = pins.gpio16.into_push_pull_output();
    let mut exled_1_pin = pins.gpio17.into_push_pull_output();
    let sw1_pin = pins.gpio14.into_pull_down_input();
    let setup_mode = sw1_pin.is_low().unwrap();

    // Enable ADC
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = AdcPin::new(pins.gpio26);

    // SysTickの設定
    // 自前でSysTickを制御するときは cortex_m::delay::Delay が使えないので注意
    core.SYST.disable_counter();
    core.SYST.clear_current();
    // set_reloadで設定する値は、(割り込み周期のクロック数 - 1)
    // Raspberry Pi Picoでは、1クロック=1マイクロ秒。
    core.SYST.set_reload(1_000 - 1); // 1m秒周期
    core.SYST.enable_interrupt();
    core.SYST.enable_counter();

    // I2C
    let mut i2c = I2cEnv::set_i2cenv(
        pac.I2C0,
        pins.gpio20.into_function(),
        pins.gpio21.into_function(),
        &mut pac.RESETS,
        clocks.system_clock,
    );
    Ada88::init(&mut i2c);
    Ada88::write_letter(&mut i2c, 0);

    // USB MIDI
    unsafe {
        USB_BUS = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));
        if let Some(usb_bus_ref) = USB_BUS.as_ref() {
            MIDI = Some(MidiClass::new(usb_bus_ref));
            USB_DEVICE = Some(
                UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x2e8a, 0x0000))
                    .product("Loopian-ORBIT")
                    .device_class(USB_CLASS_NONE)
                    .build(),
            );
            // Enable the USB interrupt
            pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        }
    };

    // Application Setup mode
    let mut available_each_device = [true; MAX_DEVICE_MBR3110];
    if setup_mode {
        Ada88::write_letter(&mut i2c, 21);
        exledsw_pin.set_high().unwrap();
        check_and_setup_board();
        // 戻ってこない
    } else {
        // Normal Mode
        let mut exist_err = false;
        for i in 0..MAX_DEVICE_MBR3110 {
            Pca9544::change_i2cbus(&mut i2c, 0, i);
            let err = Mbr3110::init(&mut i2c, i);
            if err != 0 {
                available_each_device[i] = false;
                exist_err = true;
            }
        }
        if exist_err {
            // Error
            exled_err_pin.set_high().unwrap();
            Ada88::write_letter(&mut i2c, 23);
        } else {
            // OK
            Ada88::write_letter(&mut i2c, 22);
        }
        delay_msec(3000);
    }
    exledsw_pin.set_low().unwrap();

    let mut count: u32 = 0;
    let mut count_old: u32 = 0;
    let mut time: i16 = 0;

    loop {
        free(|cs| {
            count = *COUNTER.borrow(cs).borrow();
        });
        let ev = count - count_old >= 1000;
        if ev {
            count_old = count;
            time += 1;
        }

        let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();        
        Ada88::write_number(&mut i2c, pin_adc_counts as i16);
        if ev {
            if time % 2 == 0 {
                info!("on!");
                output_midi_msg(Message::NoteOn(
                    Channel::Channel1,
                    Note::C3,
                    FromClamped::from_clamped(100),
                ));
                //count += 1;
                led_pin.set_high().unwrap();
                exled_1_pin.set_high().unwrap();
            } else {
                info!("off!");
                output_midi_msg(Message::NoteOff(
                    Channel::Channel1,
                    Note::C3,
                    FromClamped::from_clamped(64),
                ));
                led_pin.set_low().unwrap();
                exled_1_pin.set_low().unwrap();
            }
        }
    }
}
//*******************************************************************
//          System Functions
//*******************************************************************
fn check_and_setup_board() {
    loop {}
}
fn delay_msec(time: u32) {
    let mut count = 0;
    loop {
        free(|cs| {
            count = *COUNTER.borrow(cs).borrow();
        });
        if count > time {
            break;
        }
    }
}
//*******************************************************************
//          MIDI Out
//*******************************************************************
fn output_midi_msg(message: Message) {
    // Send MIDI message
    let midi = unsafe { MIDI.as_mut().unwrap() };
    match midi.send_message(UsbMidiEventPacket {
        cable_number: CableNumber::Cable0,
        message,
    }) {
        Ok(_) => (),
        Err(_) => (),
    }
}
// End of file
