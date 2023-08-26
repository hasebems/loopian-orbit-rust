//  Created by Hasebe Masahiko on 2023/08/22.
//  Copyright (c) 2023 Hasebe Masahiko.
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php
//

use defmt_rtt as _;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use panic_probe as _;

use bsp::hal::{
    gpio::pin::{bank0::*, FunctionI2C, Pin},
    i2c::I2C,
    pac::I2C0,
};
use rp_pico as bsp;

//*******************************************************************
//          I2C Environment
//*******************************************************************
type SDAPin = Gpio20;
type SCLPin = Gpio21;

pub struct I2cEnv {
    i2c_env: I2C<I2C0, (Pin<SDAPin, FunctionI2C>, Pin<SCLPin, FunctionI2C>)>,
}
impl I2cEnv {
    pub fn set_i2cenv(
        i2c_env: I2C<I2C0, (Pin<SDAPin, FunctionI2C>, Pin<SCLPin, FunctionI2C>)>,
    ) -> Self {
        Self { i2c_env }
    }
    pub fn write_dt(&mut self, adrs: u8, dt: &[u8]) {
        self.i2c_env.write(adrs, dt).unwrap();
    }
}
//*******************************************************************
//          I2C device Access IF
//*******************************************************************
pub trait I2cDev {
    fn read(&self, env: &mut I2cEnv);
    fn write(&mut self, env: &mut I2cEnv);
}
//*******************************************************************
//          Ada fruit 8*8 LED matrix
//*******************************************************************
pub struct Ada88 {}
impl Ada88 {
    const I2C_ADRS: u8 = 0x70;
    pub fn init(env: &mut I2cEnv) -> Self {
        env.write_dt(Self::I2C_ADRS, &[0x21, 0]);
        env.write_dt(Self::I2C_ADRS, &[0x81, 0]);
        env.write_dt(Self::I2C_ADRS, &[0xef, 0]);
        Self {}
    }
    pub fn write_letter(&mut self, env: &mut I2cEnv, ltr: usize) {
        let mut wrdt: [u8; 17] = [0; 17];
        const LETTER_DOT: [[u8; 8]; 21] = [
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
        ];
        for (i, dt) in LETTER_DOT[ltr].iter().enumerate() {
            wrdt[2 * i + 1] = *dt;
        }
        env.write_dt(Self::I2C_ADRS, &wrdt);
    }
}
impl I2cDev for Ada88 {
    fn read(&self, _env: &mut I2cEnv) {}
    fn write(&mut self, _env: &mut I2cEnv) {}
}
