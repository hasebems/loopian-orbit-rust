//  Created by Hasebe Masahiko on 2023/08/22.
//  Copyright (c) 2023 Hasebe Masahiko.
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php
//
use crate::i2c_device::{Ada88, I2cEnv, Mbr3110, Pca9544, Pca9685};
//use crate::I2C_CONCLETE;
use cortex_m::interrupt::{free, Mutex};

pub const MAX_DEVICE_MBR3110: usize = 6;
pub const MAX_ELECTRODE_PER_DEV: usize = 8;
pub const MAX_EACH_LIGHT: usize = 16;

//use crate::delay_msec;

//*******************************************************************
//          Loop Clock
//*******************************************************************
pub struct LoopClock {
    count_1ms: u32,
    count_10ms_old: u32,
    count_100ms_old: u32,
    count_1s_old: u32,
    event_10ms: bool,
    event_100ms: bool,
    event_1s: bool,
}
impl LoopClock {
    pub fn init() -> Self {
        Self {
            count_1ms: 0,
            count_10ms_old: 0,
            count_100ms_old: 0,
            count_1s_old: 0,
            event_10ms: false,
            event_100ms: false,
            event_1s: false,
        }
    }
    pub fn set_clock(&mut self, clk: u32) {
        //clk は 1msec ごとに増える値
        self.event_10ms = false;
        self.event_100ms = false;
        self.event_1s = false;
        if clk - self.count_10ms_old >= 10 {
            self.event_10ms = true;
            self.count_10ms_old = clk;
        }
        if clk - self.count_100ms_old >= 100 {
            self.event_100ms = true;
            self.count_100ms_old = clk;
        }
        if clk - self.count_1s_old >= 1000 {
            self.event_1s = true;
            self.count_1s_old = clk;
        }
        self.count_1ms = clk;
    }
    pub fn event_10ms(&self) -> bool {
        self.event_10ms
    }
    pub fn event_100ms(&self) -> bool {
        self.event_100ms
    }
    pub fn event_1s(&self) -> bool {
        self.event_1s
    }
    pub fn get_ms(&self) -> u32 {
        self.count_1ms
    }
}
//*******************************************************************
//          Switch Event
//*******************************************************************
#[derive(Default)]
pub struct SwitchEvent {
    sw: [u32; MAX_ELECTRODE_PER_DEV],
}
impl SwitchEvent {
    pub fn _init() -> Self {
        Self {
            sw: [0; MAX_ELECTRODE_PER_DEV],
        }
    }
    fn clear_event(&mut self, time: u32, ele: usize) {
        if self.sw[ele] == 0 {
            return;
        }
        if self.sw[ele] + 5 < time {
            self.sw[ele] = 0;
        }
    }
    pub fn update_sw_event(&mut self, sw: [u8;2], tm: u32) -> bool {
        if tm == 0 {return false;}
        let mut light_someone = false;
        let bptn: u16 = (sw[0] as u16) << 8 + (sw[1] as u16);
        for j in 0..MAX_ELECTRODE_PER_DEV {
            if (bptn & (0x0001 << j)) != 0 {
                self.sw[j] = tm;
                light_someone = true;
            } else {
                self.clear_event(tm, j);
            }
        }
        light_someone
    }
}
//*******************************************************************
//          Position LED
//*******************************************************************
pub struct PositionLed {
    total_time: u32,
}
impl PositionLed {
    pub fn init() -> Self {
        Self::clear_all();
        Self {
            total_time: 0,
        }
    }
    pub fn clear_all() {
        for i in 0..MAX_DEVICE_MBR3110 {
            for j in 0..MAX_EACH_LIGHT {
                Self::light_led_each(j, i, 0);
            }
        }
    }
    pub fn gen_lighting_in_loop(&mut self, tm: u32) {
        self.total_time = tm;
        for j in 0..MAX_DEVICE_MBR3110 {
            self.one_kamaboco(j);
        }
        //if (_fade_counter == 0){_fade_counter = 1;}  
    }
    /*
    int WhiteLed::gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV])
    {
      _total_time += difftm;
      _fade_counter += difftm;
      if (_fade_counter > FADE_RATE){_fade_counter = 0;}

      //for (int x=0; x<MAX_EACH_LIGHT*MAX_DEVICE_MBR3110; x++){_light_lvl[x]=0;}
      memset(&_light_lvl[0], 0, sizeof(int)*MAX_EACH_LIGHT*MAX_DEVICE_MBR3110);

      // tchev : 0-1599 + 1600*kamanum で絶対位置が表現され、イベントごとにその数値が入力される
      int max_ev = 0;
      for (int i=0; i<MAX_TOUCH_EV; i++){
        if (tchev[i] == -1){break;}
        int frac = tchev[i]%100;
        int pos = tchev[i]/100;
        for (int j=0; j<2; j++){
          //  触った箇所の前後二つのLEDが点灯する
          _light_lvl[pos+1+j] += (frac+100)>j*100? (frac+100)-j*100: 0;    // 199 - 0
          if (pos>=j){
            _light_lvl[pos-j] += (199-frac)>j*100? (199-frac)-j*100: 0;
          }
        }
        max_ev += 1;
      }

      for (int j=0; j<MAX_DEVICE_MBR3110; j++){one_kamaboco(j);}
      if (_fade_counter == 0){_fade_counter = 1;}

      return max_ev;
    }*/
    fn one_kamaboco(&mut self, dev_num: usize) {
        const MIN_DIFF: u16 = 4;
        const MAX_STR: u16 = (MAX_EACH_LIGHT as u16)*MIN_DIFF; // 64
        const HALF_STR: u16 = MAX_STR/2; // 32
        const BG_SPEED: u32 = 10;   // smaller, faster

        let divieded_time = ((self.total_time/BG_SPEED)%(MAX_STR as u32)) as u16;
        for i in 0..MAX_EACH_LIGHT {
          // 背景で薄く光っている
          let mut ptn: u16 = (divieded_time+(MIN_DIFF*(i as u16)))%MAX_STR;
          if ptn >= HALF_STR {ptn = MAX_STR-ptn;}
          Self::light_led_each(i, dev_num, ptn);
        }
    }
    /*{
      uint16_t time = static_cast<uint16_t>(_total_time/5);
      const int offset_num = kamanum*MAX_EACH_LIGHT;
      pca9544_changeI2cBus(3,kamanum);

      for (int i=0; i<MAX_EACH_LIGHT; i++){
        int x = i+offset_num;
        if ((_light_lvl[x] > 0) || (_light_lvl_itp[x] > 0)){
          if (_light_lvl[x]>_light_lvl_itp[x]){_light_lvl_itp[x] = _light_lvl[x];}
          else if (_fade_counter == 0){
            // だんだん暗くなるとき
            _light_lvl_itp[x] = (_light_lvl_itp[x]-_light_lvl[x])*3/4 + _light_lvl[x];
          }
          light_led_each(i, _light_lvl_itp[x]*20);
        }
        else {
          // 背景で薄く光っている
          int ptn = (time+(4*i))%64;
          ptn = ptn<32? ptn:64-ptn;
          light_led_each(i, ptn);
        }
      }
      pca9544_changeI2cBus(1,kamanum); // 別のI2Cバスに変えないと、他のkamanumのときに上書きされてしまう
    }*/
    fn light_led_each(num: usize, dev_num: usize, mut strength: u16) {
        // strength=0-4095
        let adrs = (num as u8)* 4 + 0x06;
        if strength > 4000 {
            strength = 4000;
        }
        Pca9544::change_i2cbus(3, dev_num);
        Pca9685::write(0, adrs, 0); // ONはtime=0
        Pca9685::write(0, adrs + 1, 0); // ONはtime=0
        Pca9685::write(0, adrs + 2, (strength & 0x00ff) as u8); // OFF 0-4095 (0-0x0fff) の下位8bit
        Pca9685::write(0, adrs + 3, (strength >> 8) as u8); // OFF 上位4bit
        // 別のI2Cバスに変えないと、他のkamanumのときに上書きされてしまう
        Pca9544::change_i2cbus(1, dev_num);
    }
}
//*******************************************************************
//          Detect Position
//*******************************************************************
pub struct DetectPosition {}
impl DetectPosition {
    pub fn init() -> Self {
        Self {}
    }
    pub fn update_touch_position(&mut self) {}
}
