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
use rp_pico as bsp;

use bsp::entry;
use bsp::hal;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    i2c::I2C,
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};

use fugit::RateExtU32;
use i2c_device::{Ada88, I2cEnv};

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
//          Global Variable
//*******************************************************************
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut MIDI: Option<MidiClass<hal::usb::UsbBus>> = None;

//*******************************************************************
//          main
//*******************************************************************
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    if let Some(usb_dev) = USB_DEVICE.as_mut() {
        if let Some(midi) = MIDI.as_mut() {
            usb_dev.poll(&mut [midi]);
        }
    }
}
#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // I2C
    let mut i2c = I2cEnv::set_i2cenv(I2C::i2c0(
        pac.I2C0,
        pins.gpio20.into_mode(), // sda
        pins.gpio21.into_mode(), // scl
        400_u32.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    ));
    let mut ada = Ada88::init(&mut i2c);

    // GPIO
    let mut led_pin = pins.led.into_push_pull_output();

    // USB
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

    ada.write_letter(&mut i2c, 1);
    delay.delay_ms(2000);
    let mut count = 0;

    loop {
        info!("on!");
        output_midi_msg(Message::NoteOn(
            Channel::Channel1,
            Note::C3,
            FromClamped::from_clamped(100),
        ));
        ada.write_number(&mut i2c, count);
        count += 1;
        led_pin.set_high().unwrap();
        delay.delay_ms(500);

        info!("off!");
        output_midi_msg(Message::NoteOff(
            Channel::Channel1,
            Note::C3,
            FromClamped::from_clamped(64),
        ));
        //ada.write_letter(&mut i2c, 2);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
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
