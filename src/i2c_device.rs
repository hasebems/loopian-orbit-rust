//  Created by Hasebe Masahiko on 2023/08/22.
//  Copyright (c) 2023 Hasebe Masahiko.
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php
//
use defmt::*;
use defmt_rtt as _;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use panic_probe as _;

use crate::delay_msec;
use bsp::hal::{
    clocks::SystemClock,
    gpio::{bank0::*, FunctionI2C, Pin, PullDown},
    i2c::I2C,
    pac::{I2C0, RESETS},
};
use fugit::RateExtU32;
use rp_pico as bsp;

use crate::MAX_DEVICE_MBR3110;

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};

static I2C_CONCLETE: Mutex<RefCell<Option<I2cEnv>>> = Mutex::new(RefCell::new(None));

//*******************************************************************
//          I2C Environment
//*******************************************************************
type SDAPin = Gpio20;
type SCLPin = Gpio21;

pub struct I2cEnv {
    i2c_env: I2C<
        I2C0,
        (
            Pin<SDAPin, FunctionI2C, PullDown>,
            Pin<SCLPin, FunctionI2C, PullDown>,
        ),
    >,
}
impl I2cEnv {
    pub fn set_i2cenv(
        i2c: I2C0,
        sda: Pin<SDAPin, FunctionI2C, PullDown>,
        scl: Pin<SCLPin, FunctionI2C, PullDown>,
        resets: &mut RESETS,
        sys_clocks: SystemClock,
    ) -> Self {
        let i2c_env = I2C::i2c0(i2c, sda, scl, 400_u32.kHz(), resets, &sys_clocks);
        Self { i2c_env }
    }
    pub fn write_dt(&mut self, adrs: u8, dt: &[u8]) {
        match self.i2c_env.write(adrs, dt) {
            Err(_err) => info!("I2C Wrong!"),
            _ => (),
        }
    }
    pub fn read_dt<const T: usize>(&mut self, adrs: u8, dt: &[u8]) -> Option<[u8; T]> {
        let mut readbuf: [u8; T] = [0; T];
        match self.i2c_env.write_read(adrs, dt, &mut readbuf) {
            Err(_err) => {
                info!("I2C Wrong!");
                return None;
            },
            _ => return Some(readbuf),
        }        
    }
}

pub fn i2c_init(
    i2cn: I2C0,
    sda: Pin<SDAPin, FunctionI2C, PullDown>,
    scl: Pin<SCLPin, FunctionI2C, PullDown>,
    resets: &mut RESETS,
    sys_clocks: SystemClock,    
) {
    free(|cs| {
        let mut i2cc = I2cEnv::set_i2cenv(
            i2cn,
            sda,
            scl,
            resets,
            sys_clocks,
        );
        *I2C_CONCLETE.borrow(cs).borrow_mut() = Some(i2cc);
    });
}
//*******************************************************************
//          I2C device Access IF
//*******************************************************************
pub trait I2cDev {
    fn read(&self, env: &mut I2cEnv);
    fn write(&mut self, env: &mut I2cEnv);
}
//*******************************************************************
//          CY8CMBR3110 Touch Sensor
//*******************************************************************
//const MAX_DEVICE_MBR3110: usize = 6;
const MAX_TOUCH_EV: usize = 8;
const MAX_EACH_SENS: usize = 8;

const CONFIG_DATA_OFFSET: u8 = 0;
const CONFIG_DATA_SZ: usize = 128;

const SENSOR_EN: u8 = 0x00; //	Register Address
const SENSITIVITY0: u8 = 0x08; //	Register Address
const SENSITIVITY1: u8 = 0x09; //	Register Address
const SENSITIVITY2: u8 = 0x0a; //	Register Address
const I2C_ADDR: u8 = 0x51; //	Register Address
const CONFIG_CRC: u8 = 0x7e; //	Register Address

const CTRL_CMD: u8 = 0x86; //	Register Address
const POWER_ON_AND_FINISHED: u8 = 0x00;
const SAVE_CHECK_CRC: u8 = 0x02;
const DEVICE_RESET: u8 = 0xff;
const CTRL_CMD_ERR: u8 = 0x89; //	Register Address

const FAMILY_ID_ADRS: u8 = 0x8f; //	Register Address
const FAMILY_ID: u8 = 0x9a;
const DEVICE_ID_ADRS: u8 = 0x90; //	Register Address
const DEVICE_ID_LOW: u8 = 0x02;
const DEVICE_ID_HIGH: u8 = 0x0a;

const TOTAL_WORKING_SNS: u8 = 0x97; //	Register Address
const SNS_VDD_SHORT: u8 = 0x9a; //	Register Address
const SNS_GND_SHORT: u8 = 0x9c; //	Register Address
const BUTTON_STAT: u8 = 0xaa; //	Register Address

const CAP_SENSE_ADDRESS_ORG: u8 = 0x37; //  Factory-Set
const MBR_I2C_ADDRESS: [u8; 12] = [
    0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
];

/*----------------------------------------------------------------------------*/
//
//      Write CY8CMBR3110 Config Data
//
/*----------------------------------------------------------------------------*/
// wide range small resolution
const CY8CMBR3110_CONFIG_DATA: [[u8; CONFIG_DATA_SZ]; 12] = [
    [
        /* Project: C:\Users\hasebems\Documents\Cypress Projects\Design0602\Design0602.cprj
         * Generated: 2019/06/02 6:52:53 +09:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x38, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB7, 0xCA,
    ],
    [
        /* Project: C:\Users\hasebems\Documents\Cypress Projects\Design0602\Design0602-2.cprj
         * Generated: 2019/06/02 7:07:15 +09:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x39, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x1E,
    ],
    [
        /* Projet: C:\sers\hsebemsDocumets\Cypess Prjects\esign0602\Design210124-3.cprj
         * Generted: 221/01/4 22:5:50 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x3A, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x72,
    ],
    [
        /* Projet: C:\sers\hsebemsDocumets\Cypess Prjects\esign0602\Design210124-4.cprj
         * Generted: 221/01/4 22:5:37 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x3B, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3, 0xA6,
    ],
    [
        //5
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:23 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x3C, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0xAA,
    ],
    [
        //6
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:41 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x3D, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x7E,
    ],
    [
        //7
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:06 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x3E, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x12,
    ],
    [
        //8
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:35 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x3F, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0xC6,
    ],
    [
        //9
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:51 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x40, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD9, 0xC1,
    ],
    [
        //10
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:07 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x41, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x15,
    ],
    [
        //11
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:18 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x42, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x79,
    ],
    [
        //12
        /* Projet: C:\sers\MsahikoHASEBEOneDrie - YAAHA Grup\prv\cy8cmbr3110\TouchKeyboard\TouchKeyboard.cprj
         * Generted: 222/06/8 10:2:30 +0:00 */
        0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0x00, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x02, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x1E, 0x1E, 0x00,
        0x00, 0x00, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x11, 0x02, 0x01, 0x08, 0x00, 0x43, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xAD,
    ],
];
//-------------------------------------------------------------------------
#[cfg(feature = "Mbr3110")]
pub struct Mbr3110;
#[cfg(feature = "Mbr3110")]
impl Mbr3110 {
    //-------------------------------------------------------------------------
    fn reset(i2c_adrs: u8) {
        let i2c_data: [u8; 2] = [CTRL_CMD, POWER_ON_AND_FINISHED];
        delay_msec(15);

        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                i2c.write_dt(i2c_adrs, &i2c_data);
            }
        });

        delay_msec(900);
    }
    //-------------------------------------------------------------------------
    pub fn init(number: usize) -> i32 {
        if number >= MAX_DEVICE_MBR3110 {
            return -1;
        }

        let config_data: &[u8; CONFIG_DATA_SZ] = &CY8CMBR3110_CONFIG_DATA[number];
        let i2c_adrs: u8 = MBR_I2C_ADDRESS[number];

        let check_sum_l: u8 = config_data[126];
        let check_sum_h: u8 = config_data[127];
        let mut err = Self::check_write_config(check_sum_l, check_sum_h, i2c_adrs);
        if err != 0 {
            return err;
        }

        let result = Self::self_test(number);
        match result {
            Ok(self_check_result) => {
                if (self_check_result & 0x80) != 0 {
                    err = (self_check_result & 0x1f) as i32; //  SENSOR_COUNT
                }
            }
            Err(er) => return er,
        }
        err
    }
    //-------------------------------------------------------------------------
    pub fn _setup_device(number: usize) -> i32 {
        let i2c_adrs: u8 = MBR_I2C_ADDRESS[number];
        if number >= MAX_DEVICE_MBR3110 {
            return -1;
        }

        Self::reset(i2c_adrs);

        let config_data = &CY8CMBR3110_CONFIG_DATA[number];
        let check_sum_l = config_data[126];
        let check_sum_h = config_data[127];
        if Self::check_write_config(check_sum_l, check_sum_h, i2c_adrs) == 0 {
            //  checksum got correct.
            //  it doesn't need to write config.
            return 0;
        }

        //  check if factory preset device
        let mut err = Self::write_config(number, CAP_SENSE_ADDRESS_ORG);
        if err != 0 {
            //  if err then change I2C Address
            //    and rewrite config to current device
            err = Self::write_config(number, i2c_adrs);
            if err != 0 {
                return err;
            }
        }

        //  After writing, Check again.
        Self::reset(i2c_adrs);
        err = Self::check_write_config(check_sum_l, check_sum_h, i2c_adrs);
        if err != 0 {
            //  checksum error
            return err;
        }

        //let result = Mbr3110::self_test(env, number);
        match Self::self_test(number) {
            Err(err) => return err,
            Ok(check_result) => {
                if (check_result & 0x80) != 0 {
                    return (check_result & 0x1f_u8) as i32;
                }
            }
        }
        return err;
    }
    //-------------------------------------------------------------------------
    fn self_test(number: usize) -> Result<u8, i32> {
        let wt_dt: [u8; 1] = [TOTAL_WORKING_SNS];
        let _ = free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                if let Some(rd_dt) = i2c.read_dt::<1>(MBR_I2C_ADDRESS[number], &wt_dt) {
                    Ok(rd_dt[0])
                }
                else {Err(-1)}
            }
            else {Err(-1)}
        });
        Err(-1)
    }
    //-------------------------------------------------------------------------
    pub fn _change_sensitivity(data: u8, number: usize) {
        //	data : 0-3
        let mut reg_data2 = data & 0x03;
        reg_data2 |= reg_data2 << 2;
        let reg_data4 = reg_data2 | (reg_data2 << 4);
        let i2c_adrs: u8 = MBR_I2C_ADDRESS[number];

        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                let mut i2c_data: [u8; 2] = [SENSITIVITY0, reg_data2];
                i2c.write_dt(i2c_adrs, &i2c_data);
                i2c_data = [SENSITIVITY1, reg_data4];
                i2c.write_dt(i2c_adrs, &i2c_data);
                i2c_data = [SENSITIVITY2, reg_data2];
                i2c.write_dt(i2c_adrs, &i2c_data);
            }
        });
    }
    //-------------------------------------------------------------------------
    pub fn read_touch_sw(number: usize) -> Result<[u8; 2], i32> {
        let mut count = 0;
        loop {
            let _ = free(|cs| {
                if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {            
                    if let Some(rd_dt) = i2c.read_dt::<2>(MBR_I2C_ADDRESS[number], &[BUTTON_STAT]) {
                        return Ok(rd_dt)
                    }
                    else {Err(-1)}
                }
                else {Err(-1)}
            });
            count += 1;
            if count > 10 {return Err(-1)}
            //delay_msec(1);
        }
    }
    //-------------------------------------------------------------------------
    fn check_write_config(
        check_sum_l: u8,
        check_sum_h: u8,
        crnt_adrs: u8,
    ) -> i32 {
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                if let Some(rd_dt) = i2c.read_dt::<2>(crnt_adrs, &[CONFIG_CRC]) {
                    if (rd_dt[0] == check_sum_l) && (rd_dt[1] == check_sum_h) {
                        return 0;
                    }
                    else {return -2;} //  check sum didn't match
                }
                else {return -1;}
            }
            else {return -1;}
        });
        -1
    }
    //-------------------------------------------------------------------------
    fn collate_1byte(i2c_adrs: u8, dt: &[u8;1], cdt: u8) -> i32 {
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                if let Some(data) = i2c.read_dt::<1>(i2c_adrs, dt) {
                    if data[0] == cdt {return 0}
                    else {return -2}
                }
                else {return -1}
            }
            else {return -1}
        });
        0
    }
    fn collate_2bytes(i2c_adrs: u8, dt: &[u8;1], cdt1: u8, cdt2: u8) -> i32 {
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                if let Some(data) = i2c.read_dt::<2>(i2c_adrs, dt) {
                    if data[0] == cdt1 && data[1] == cdt2 {return 0}
                    else {return -2}
                }
                else {return -1}
            }
            else {return -1}
        });
        0
    }
    fn check_finished(i2c_adrs: u8) -> i32 {
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                if let Some(data) = i2c.read_dt::<1>(i2c_adrs, &[CTRL_CMD_ERR]) {
                    if data[0] == 0xfe      {return -4;}    //  bad check sum
                    else if data[0] == 0xff {return -5;}    //  invalid command
                    else if data[0] == 0xfd {return -6;}    //  failed to write flash
                    else {return 0}
                }
                else {return -1}
            }
            else {return -1}
        });
        0
    }
    fn write_config(number: usize, i2c_adrs: u8) -> i32 {
        let config_data = &CY8CMBR3110_CONFIG_DATA[number];

        // ** Prepare **
        Mbr3110::reset(i2c_adrs);

        // ** Step 1 **
        //	Check Power On

        //	Check I2C Address
        let err = Self::collate_1byte(i2c_adrs, &[I2C_ADDR], i2c_adrs);
        if err != 0 {
            return err;
        }
//        free(|cs| {
//            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
//                if let Some(data) = i2c.read_dt::<1>(i2c_adrs, &[I2C_ADDR]) {
//                    if data[0] == i2c_adrs {/* OK */}
//                    else {return -2}
//                }
//                else {return -1}
//            }
//            else {return -1}
//        });

        // ** Step 2 **
        let err = Self::collate_2bytes(i2c_adrs, &[DEVICE_ID_ADRS], DEVICE_ID_LOW, DEVICE_ID_HIGH);
        if err != 0 {
            return err;
        }       
//        free(|cs| {
//            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
//                if let Some(data) = i2c.read_dt::<2>(i2c_adrs, &[DEVICE_ID_ADRS]) {
//                    if (data[0] == DEVICE_ID_LOW) && (data[1] == DEVICE_ID_HIGH) {}/* OK */
//                    else {return -3}
//                }
//                else {return -1}
//            }
//            else {return -1}
//        });

        let err = Self::collate_1byte(i2c_adrs, &[FAMILY_ID_ADRS], FAMILY_ID);
        if err != 0 {
            return err;
        }
//          free(|cs| {
//            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {                
//                if let Some(data) = i2c.read_dt::<1>(i2c_adrs, &[FAMILY_ID_ADRS]) {
//                    if data[0] == FAMILY_ID { /* OK */}
//                    else {return -4}
//                }
//                else {return -1}
//            }
//            else {-1}
//        });

        // ** Step 3 **
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                //	send Config Data
                let mut data: [u8; 2];
                for i in 0..CONFIG_DATA_SZ {
                    data = [CONFIG_DATA_OFFSET + (i as u8), config_data[i]];
                    i2c.write_dt(i2c_adrs, &data);
                }

                //	Write to flash
                let data = [CTRL_CMD, SAVE_CHECK_CRC];
                i2c.write_dt(i2c_adrs, &data);
            }
        });

        //	220msec Wait
        delay_msec(300);

        //	Check to finish writing
        let err = Self::check_finished(i2c_adrs);
        if err != 0 {
            return err;
        }
//        free(|cs| {
//            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
//                let wrt_dt = [CTRL_CMD_ERR];
//                if let Some(data) = i2c.read_dt::<1>(i2c_adrs, &wrt_dt) {
//                    if data[0] == 0xfe      {return -4;}    //  bad check sum
//                    else if data[0] == 0xff {return -5;}    //  invalid command
//                    else if data[0] == 0xfd {return -6;}    //  failed to write flash
//                    else { /* OK */}
//                }
//                else {return -1}
//            else {return -1}
//        });

        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {            
                //	Reset
                let data: [u8; 2] = [CTRL_CMD, DEVICE_RESET];
                i2c.write_dt(i2c_adrs, &data);
            }
        });

        //	100msec Wait
        delay_msec(100);

        // ** Step 4 **
        //	Get Config Data
        let wrt_dt = [CONFIG_DATA_OFFSET];
        let i2c_adrs_for_num = MBR_I2C_ADDRESS[number];

        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                if let Some(rd_dt) = i2c.read_dt::<CONFIG_DATA_SZ>(i2c_adrs_for_num, &wrt_dt) {
                //	Compare both Data
                    for i in 0..CONFIG_DATA_SZ {
                        if config_data[i] != rd_dt[i] {
                            return i as i32;
                        }
                    }
                    0   /* OK */
                }
                else {return -1}
            }
            else {return -1}
        });
        0
    }
}
//*******************************************************************
//          Ada fruit 8*8 LED matrix
//*******************************************************************
#[cfg(feature = "ada88")]
pub struct Ada88;
#[cfg(feature = "ada88")]
impl Ada88 {
    const I2C_ADRS: u8 = 0x70;
    pub fn init() -> i32 {
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                i2c.write_dt(Self::I2C_ADRS, &[0x21, 0]);
                i2c.write_dt(Self::I2C_ADRS, &[0x81, 0]);
                i2c.write_dt(Self::I2C_ADRS, &[0xef, 0]);
            }
        });
        0
    }
    pub fn write_letter(ltr: usize) {
        let mut wrdt: [u8; 17] = [0; 17];
        const LETTER_DOT: [[u8; 8]; 24] = [
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], //	0:nothing
            [0x02, 0x05, 0x88, 0x88, 0x8f, 0x88, 0x88, 0x88], //	1:A
            [0x87, 0x88, 0x88, 0x87, 0x88, 0x88, 0x88, 0x87], //	2:B
            [0x07, 0x88, 0x88, 0x80, 0x80, 0x88, 0x88, 0x07], //	3:C
            [0x87, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x87], //	4:D
            [0x87, 0x80, 0x80, 0x87, 0x80, 0x80, 0x80, 0x8f], //	5:E
            [0x87, 0x80, 0x80, 0x87, 0x80, 0x80, 0x80, 0x80], //	6:F
            [0x07, 0x88, 0x80, 0x80, 0x8e, 0x88, 0x88, 0x07], //	7:G
            [0x88, 0x88, 0x88, 0x8f, 0x88, 0x88, 0x88, 0x88], //	8:H
            [0x02, 0x05, 0x88, 0xe8, 0xaf, 0xe8, 0xa8, 0xa8], //	9:AF
            [0x87, 0x88, 0x88, 0xe7, 0xa8, 0xe8, 0xa8, 0xa7], //	10:BF
            [0x07, 0x88, 0x88, 0xe0, 0xa0, 0xe8, 0xa8, 0x27], //	11:CF
            [0x87, 0x88, 0x88, 0xe8, 0xa8, 0xe8, 0xa8, 0xa7], //	12:DF
            [0x87, 0x80, 0x80, 0xe7, 0xa0, 0xe0, 0xa0, 0xaf], //	13:EF
            [0x87, 0x80, 0x80, 0xe7, 0xa0, 0xe0, 0xa0, 0xa0], //	14:FF
            [0x07, 0x88, 0x80, 0xe0, 0xae, 0xe8, 0xa8, 0x27], //	15:GF
            [0x97, 0x90, 0x90, 0x97, 0x90, 0x90, 0x90, 0xd0], //	16:Fl.
            [0x13, 0x94, 0x94, 0xf4, 0xd4, 0xd4, 0xb4, 0x53], //	17:Ob.
            [0x04, 0x1f, 0x04, 0x1f, 0x04, 0x0f, 0x15, 0x22], //	18:MA
            [0x04, 0x0f, 0x04, 0x1e, 0x08, 0x12, 0x01, 0x0e], //	19:KI
            [0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa], //	20:
            [0x03, 0x80, 0x80, 0x49, 0x4a, 0x4a, 0xca, 0x79], //	21:SU(setup)
            [0x09, 0x8a, 0xca, 0xaa, 0x9a, 0xaa, 0xaa, 0x49], //	22:Ok
            [0x83, 0x80, 0x80, 0xab, 0xd8, 0x88, 0x88, 0x8b], //	23:Er
        ];
        for (i, dt) in LETTER_DOT[ltr].iter().enumerate() {
            wrdt[2 * i + 1] = *dt;
        }
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                i2c.write_dt(Self::I2C_ADRS, &wrdt);
            }
        });
    }
    pub fn write_number(num: i16) {
        //	num 1999 .. -1999
        let mut cnvt_num: i16 = num;
        const NUM_LETTER: [[u8; 5]; 10] = [
            [0x07, 0x05, 0x05, 0x05, 0x07],
            [0x04, 0x04, 0x04, 0x04, 0x04],
            [0x07, 0x04, 0x07, 0x01, 0x07],
            [0x07, 0x04, 0x07, 0x04, 0x07],
            [0x05, 0x05, 0x07, 0x04, 0x04],
            [0x07, 0x01, 0x07, 0x04, 0x07],
            [0x07, 0x01, 0x07, 0x05, 0x07],
            [0x07, 0x04, 0x04, 0x04, 0x04],
            [0x07, 0x05, 0x07, 0x05, 0x07],
            [0x07, 0x05, 0x07, 0x04, 0x07],
        ];
        const GRAPH: [[u8; 2]; 10] = [
            [0x00, 0x00],
            [0x00, 0x40],
            [0x40, 0x60],
            [0x60, 0x70],
            [0x70, 0x78],
            [0x78, 0x7c],
            [0x7c, 0x7e],
            [0x7e, 0x7f],
            [0x7f, 0xff],
            [0xff, 0xff],
        ];

        if num > 1999 {
            cnvt_num = 1999;
        } else if num < -1999 {
            cnvt_num = -1999;
        }

        let mut led_ptn: [u8; 8] = [0; 8];
        //	+/-, over 1000 or not
        if cnvt_num / 1000 != 0 {
            led_ptn[5] |= 0x80;
        }
        if cnvt_num < 0 {
            led_ptn[2] |= 0x80;
            cnvt_num = -cnvt_num;
        }

        let num3digits = cnvt_num % 1000;
        let hundred = (num3digits / 100) as usize;
        let num2degits = (num3digits % 100) as usize;
        let deci = num2degits / 10;
        let z2n = num2degits % 10;

        for i in 0..5 {
            led_ptn[i] |= NUM_LETTER[hundred][i];
            led_ptn[i] |= NUM_LETTER[deci][i] << 4;
        }
        for i in 0..2 {
            led_ptn[i + 6] |= GRAPH[z2n][i];
        }

        let mut wrdt: [u8; 17] = [0; 17];
        for i in 0..8 {
            wrdt[i * 2 + 1] = led_ptn[i];
            wrdt[i * 2 + 2] = 0;
        }
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                i2c.write_dt(Self::I2C_ADRS, &wrdt);
            }
        });
    }
}
//-------------------------------------------------------------------------
//			PCA9544A ( I2C Multiplexer : I2c Device)
//-------------------------------------------------------------------------
#[cfg(feature = "Pca9544")]
pub struct Pca9544;
#[cfg(feature = "Pca9544")]
impl Pca9544 {
    const PCA9544A_I2C_ADRS: u8 = 0x70;
    pub fn change_i2cbus(i2c_num: u8, dev_num: usize) {
        let i2c_buf = [0x04 | (i2c_num & 0x0003)];
        let i2c_adrs = Self::PCA9544A_I2C_ADRS + dev_num as u8;
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
                i2c.write_dt(i2c_adrs, &i2c_buf);
            }
        });
    }
}

//-------------------------------------------------------------------------
//			PCA9685 (LED Driver : I2c Device)
//-------------------------------------------------------------------------
#[cfg(feature = "Pca9685")]
pub struct Pca9685;
#[cfg(feature = "Pca9685")] //	for LED Driver
impl Pca9685 {
    const PCA9685_ADDRESS: u8 = 0x40;
    pub fn write(chip_number: usize, cmd1: u8, cmd2: u8) {
        let i2c_buf: [u8; 2] = [cmd1, cmd2];
        free(|cs| {
            if let Some(i2c) = &mut *I2C_CONCLETE.borrow(cs).borrow_mut() {
               i2c.write_dt(Self::PCA9685_ADDRESS + chip_number as u8, &i2c_buf);
            }
        });
    }
    //-------------------------------------------------------------------------
    //		Initialize
    //-------------------------------------------------------------------------
    pub fn init(chip_number: usize) {
        //	Init Parameter
        Self::write(chip_number, 0x00, 0x00);
        Self::write(chip_number, 0x01, 0x12); //	Invert, OE=high-impedance
    }
    //-------------------------------------------------------------------------
    //		rNum, gNum, bNum : 0 - 4094  bigger, brighter
    //-------------------------------------------------------------------------
    pub fn _set_fullcolor_led(
        chip_number: usize,
        mut led_num: u8,
        color: &[u16; 3],
    ) {
        while led_num > 4 {
            led_num -= 4;
        }
        for i in 0..3 {
            //	figure out PWM counter
            let mut color_cnt: u16 = color[i];
            color_cnt = 4095 - color_cnt;
            if color_cnt <= 0 {
                color_cnt = 1;
            }

            //	Set PWM On Timing
            let color_ofs: u8 = (i as u8) * 4 + led_num * 16;
            let cmd2: u8 = (color_cnt & 0x00ff) as u8;
            Self::write(chip_number, 0x06 + color_ofs, cmd2);
            let cmd2: u8 = ((color_cnt & 0xff00) >> 8) as u8;
            Self::write(chip_number, 0x07 + color_ofs, cmd2);
            Self::write(chip_number, 0x08 + color_ofs, 0);
            Self::write(chip_number, 0x09 + color_ofs, 0);
        }
    }
}
