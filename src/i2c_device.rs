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
//          CY8CMBR3110 Touch Sensor
//*******************************************************************


//*******************************************************************
//          Ada fruit 8*8 LED matrix
//*******************************************************************
#[cfg(feature = "ada88")]
pub struct Ada88();
#[cfg(feature = "ada88")]
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
    pub fn write_number(&mut self, env: &mut I2cEnv, num: i16) { //	num 1999 .. -1999
        let mut cnvt_num: i16 = num;
        const NUM_LETTER: [[u8; 5];10] = [
            [ 0x07, 0x05, 0x05, 0x05, 0x07 ],
            [ 0x04, 0x04, 0x04, 0x04, 0x04 ],
            [ 0x07, 0x04, 0x07, 0x01, 0x07 ],
            [ 0x07, 0x04, 0x07, 0x04, 0x07 ],
            [ 0x05, 0x05, 0x07, 0x04, 0x04 ],
            [ 0x07, 0x01, 0x07, 0x04, 0x07 ],
            [ 0x07, 0x01, 0x07, 0x05, 0x07 ],
            [ 0x07, 0x04, 0x04, 0x04, 0x04 ],
            [ 0x07, 0x05, 0x07, 0x05, 0x07 ],
            [ 0x07, 0x05, 0x07, 0x04, 0x07 ]
        ];
        const GRAPH: [[u8; 2]; 10] = [
            [ 0x00, 0x00 ],
            [ 0x00, 0x40 ],
            [ 0x40, 0x60 ],
            [ 0x60, 0x70 ],
            [ 0x70, 0x78 ],
            [ 0x78, 0x7c ],
            [ 0x7c, 0x7e ],
            [ 0x7e, 0x7f ],
            [ 0x7f, 0xff ],
            [ 0xff, 0xff ],
        ];

        if num > 1999 { cnvt_num = 1999; }
        else if num < -1999 { cnvt_num = -1999;}

        let mut led_ptn: [u8; 8] = [0; 8];
        //	+/-, over 1000 or not
        if cnvt_num/1000 != 0 { led_ptn[5] |= 0x80; }
        if cnvt_num < 0 {
            led_ptn[2] |= 0x80;
            cnvt_num = -cnvt_num;
        }

        let num3digits = cnvt_num%1000;
        let hundred = (num3digits/100) as usize;
        let num2degits = (num3digits%100) as usize;
        let deci = num2degits/10;
        let z2n = num2degits%10;
    
        for i in 0..5 {
            led_ptn[i] |= NUM_LETTER[hundred][i];
            led_ptn[i] |= NUM_LETTER[deci][i] << 4;
        }
        for i in 0..2 {
            led_ptn[i+6] |= GRAPH[z2n][i];
        }
    
        let mut wrdt: [u8; 17] = [0; 17];
        for i in 0..8 {
            wrdt[i*2+1] = led_ptn[i];
            wrdt[i*2+2] = 0;            
        }
        env.write_dt(Self::I2C_ADRS, &wrdt);
    }
}
