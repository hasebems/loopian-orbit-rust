//  Created by Hasebe Masahiko on 2023/08/22.
//  Copyright (c) 2023 Hasebe Masahiko.
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php
//
use crate::i2c_device::{/*Ada88,*/ Pca9544, Pca9685};

pub const MAX_DEVICE_MBR3110: usize = 6;
pub const MAX_ELECTRODE_PER_DEV: usize = 8;
pub const MAX_EACH_LIGHT: usize = 16;
pub const MAX_NOTE: usize = MAX_DEVICE_MBR3110*16;

pub const MAX_TOUCH_EV: usize = 8;
pub const MAX_EACH_SENS: usize = 8;

use usbd_midi::data::midi::{channel::Channel, message::Message, notes::Note};
use usbd_midi::data::byte::from_traits::FromClamped;

use crate::output_midi_msg;

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
    pub fn _event_100ms(&self) -> bool {
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
    const OFF: u32 = 0;
    const CHATTERING_TIME: u32 = 50; //msec
    pub fn _init() -> Self {
        Self {
            sw: [Self::OFF; MAX_ELECTRODE_PER_DEV],
        }
    }
    fn clear_event(&mut self, time: u32, ele: usize) {
        if self.sw[ele] == Self::OFF {
            return;
        }
        if self.sw[ele] + Self::CHATTERING_TIME < time {
            self.sw[ele] = Self::OFF;
        }
    }
    pub fn update_sw_event(&mut self, sw: [u8;2], time: u32) -> bool {
        if time == 0 {return false;}
        let mut light_someone = false;
        let bptn: u16 = ((sw[1] as u16) << 8) + (sw[0] as u16);
        for j in 0..MAX_ELECTRODE_PER_DEV {
            if (bptn & (0x0001 << j)) != 0 {
                if self.sw[j] == Self::OFF {self.sw[j] = time;}
                light_someone = true;
            } else {
                self.clear_event(time, j);
            }
        }
        light_someone
    }
}
//*******************************************************************
//          Touch Event
//*******************************************************************
#[derive(Clone, Copy)]
pub struct TouchEvent {
    locate_current: i32,  // -1, 0 - 9599 (16*6*100 - 1)
    locate_target: i32,   // -1, 0 - 9599
    mintch_locate: i32,   // -1, 0 - 47 (8*6 - 1)
    maxtch_locate: i32,   // -1, 0 - 47
    last_midi: usize,       // 0 - 95 (locate/100)
    time: i32,
}
impl TouchEvent {
    const NOTHING: i32 = -1;
    const COLLATED: i32 = -2;
    fn move_from(&mut self, old: &mut TouchEvent) {
        self.locate_current = old.locate_current;
        self.time = old.time;
        old.locate_target = Self::COLLATED;
    }
    fn set_midi_note(&mut self) {
        self.last_midi = ((self.locate_target+50)/100) as usize;
    }
    fn midi_note(&self) -> usize {self.last_midi}
    const U8_TO_NOTE: [Note; 128] = [
    Note::C1m,Note::Cs1m,Note::D1m,Note::Ds1m,Note::E1m,Note::F1m,Note::Fs1m,Note::G1m,
    Note::Gs1m,Note::A1m,Note::As1m,Note::B1m,Note::C0,Note::Cs0,Note::D0,Note::Ds0,
    Note::E0,Note::F0,Note::Fs0,Note::G0,Note::Gs0,Note::A0,Note::As0,Note::B0,
    Note::C1,Note::Cs1,Note::D1,Note::Ds1,Note::E1,Note::F1,Note::Fs1,Note::G1,
    Note::Gs1,Note::A1,Note::As1,Note::B1,Note::C2,Note::Cs2,Note::D2,Note::Ds2,
    Note::E2,Note::F2,Note::Fs2,Note::G2,Note::Gs2,Note::A2,Note::As2,Note::B2,
    Note::C3,Note::Cs3,Note::D3,Note::Ds3,Note::E3,Note::F3,Note::Fs3,Note::G3,
    Note::Gs3,Note::A3,Note::As3,Note::B3,Note::C4,Note::Cs4,Note::D4,Note::Ds4,
    Note::E4,Note::F4,Note::Fs4,Note::G4,Note::Gs4,Note::A4,Note::As4,Note::B4,
    Note::C5,Note::Cs5,Note::D5,Note::Ds5,Note::E5,Note::F5,Note::Fs5,Note::G5,
    Note::Gs5,Note::A5,Note::As5,Note::B5,Note::C6,Note::Cs6,Note::D6,Note::Ds6,
    Note::E6,Note::F6,Note::Fs6,Note::G6,Note::Gs6,Note::A6,Note::As6,Note::B6,
    Note::C7,Note::Cs7,Note::D7,Note::Ds7,Note::E7,Note::F7,Note::Fs7,Note::G7,
    Note::Gs7,Note::A7,Note::As7,Note::B7,Note::C8,Note::Cs8,Note::D8,Note::Ds8,
    Note::E8,Note::F8,Note::Fs8,Note::G8,Note::Gs8,Note::A8,Note::As8,Note::B8,
    Note::C9,Note::Cs9,Note::D9,Note::Ds9,Note::E9,Note::F9,Note::Fs9,Note::G9,
    //Note::Gs9,
    ];
}
impl Default for TouchEvent {
    fn default() -> Self {
        Self {
            locate_current: Self::NOTHING,
            locate_target:  Self::NOTHING,
            mintch_locate:  Self::NOTHING,
            maxtch_locate:  Self::NOTHING,
            last_midi:      60,
            time:           Self::NOTHING,
        }
    }
}
//*******************************************************************
//          Detect Position
//*******************************************************************
pub struct DetectPosition {
    ev: [TouchEvent; MAX_TOUCH_EV],
    note_status: [bool; MAX_NOTE],
    notes_all: bool,
}
impl DetectPosition {
    const SAME_FINGER: i32 = 450;   // 10msec あたりの一つの指とみなす最大動作量(1接点で200)
    const LED_CHASE_SPEED: i32 = 10;
    const OFFSET_NOTE: usize = 0;

    pub fn init() -> Self {
        Self {
            ev: [TouchEvent::default(); MAX_TOUCH_EV],
            note_status: [false; MAX_NOTE],
            notes_all: false,
        }
    }
    pub fn get_velocity_from_adc(adc: u16) -> u8 {  // adc: 0-4095
        let ret;
        if adc > 2100 {ret = (adc/70)+69}
        else if adc < 1900 {ret = (adc/25)+20}
        else {ret=98}
        return ret as u8;
    }
    pub fn get_1st_position(&self) -> i32 {self.ev[0].locate_target}
    pub fn update_touch_position(&mut self, swdev: &[SwitchEvent; MAX_DEVICE_MBR3110], vel: u8) -> u8 {
        let mut new_ev = self.detect_touch(swdev);
        for x in new_ev.iter_mut() {
            if x.locate_target == TouchEvent::NOTHING {break}
            let mut found: bool = false;
            for y in self.ev.iter_mut() {
                if y.locate_target == TouchEvent::NOTHING {break}
                if y.locate_target == TouchEvent::COLLATED {continue}
                if y.locate_target - Self::SAME_FINGER < x.locate_target && 
                  x.locate_target < y.locate_target + Self::SAME_FINGER {
                    found = true;
                    x.set_midi_note();
                    if x.midi_note() != y.midi_note() {
                        let _ = Self::generate_midi(2, x.midi_note(), y.midi_note(), vel);
                        self.note_status[x.midi_note()] = true;
                        self.note_status[y.midi_note()] = false;
                        self.notes_all = true;
                    }
                    x.move_from(y);
                    break;
                }
            }
            if !found {
                x.locate_current = x.locate_target;
                x.time = 0;
                x.set_midi_note();
                let _ = Self::generate_midi(1, x.midi_note(), 0, vel);
                self.note_status[x.midi_note()] = true;
                self.notes_all = true;
            }
        }
        let mut finger: u8 = 0;
        for z in self.ev.iter_mut() {
            if z.locate_target == TouchEvent::NOTHING {}
            else if z.locate_target == TouchEvent::COLLATED {
                finger += 1;
            }
            else {
                let _ = Self::generate_midi(0, 0, z.midi_note(), vel);
                self.note_status[z.midi_note()] = false;
            }
        }
        // 最後にCopyする
        self.ev = new_ev;
        finger
    }
    fn detect_touch(&mut self, swdev: &[SwitchEvent; MAX_DEVICE_MBR3110]) -> [TouchEvent; MAX_TOUCH_EV] {
        let mut start: bool = false;
        let mut new_ev: [TouchEvent; MAX_TOUCH_EV] = [Default::default(); MAX_TOUCH_EV];
        let mut ev: usize = 0;

        for idx in 0..MAX_DEVICE_MBR3110*MAX_EACH_SENS {
            let which_dev=idx/MAX_EACH_SENS;
            let each_sw=idx%MAX_EACH_SENS;
            let mut end_tch = |last:usize| {
                let last_tch = last as i32;
                new_ev[ev].maxtch_locate = last_tch;
                new_ev[ev].locate_target = (new_ev[ev].mintch_locate + last_tch)*100; // /2 *200
            };
            if swdev[which_dev].sw[each_sw] != SwitchEvent::OFF {
                if !start { // 今On、これまでOff
                    start = true;
                    new_ev[ev].mintch_locate = idx as i32;
                }
                else if idx == MAX_DEVICE_MBR3110*MAX_EACH_SENS - 1 {
                    // 一番右まで On だったら
                    end_tch(idx);
                }
            }
            else if start { // 今Off、これまでOn
                start = false;
                end_tch(idx - 1);
                ev += 1;
                if ev >= MAX_TOUCH_EV {break;}
            }
        }
        new_ev
    }
    fn generate_midi(note_type: i32, on_note: usize, off_note: usize, vel: u8) -> bool {
        let mut sccs1 = false;
        let mut sccs2 = false;
        if note_type > 0 {
            sccs2 = output_midi_msg(Message::NoteOn(
                Channel::Channel1,
                TouchEvent::U8_TO_NOTE[on_note+Self::OFFSET_NOTE],
                FromClamped::from_clamped(vel),
            ));
        }
        if note_type != 1 {
            sccs1 = output_midi_msg(Message::NoteOff(
                Channel::Channel1,
                TouchEvent::U8_TO_NOTE[off_note+Self::OFFSET_NOTE],
                FromClamped::from_clamped(64),
            ));
        }
        if !sccs1 || !sccs2 {false}
        else {true}
    }
    pub fn interporate_location(&mut self, difftm: u32) -> [i32; MAX_TOUCH_EV] {
        for e in self.ev.iter_mut() {
            if e.locate_target == TouchEvent::NOTHING {break;}
            let mut diff = e.locate_target - e.locate_current;
            let diffcs = (difftm as i32)*Self::LED_CHASE_SPEED;
            if diff > 0 {
                diff = if diffcs > diff {diff} else {diffcs}
            }
            else {
                diff = if diffcs > -diff {diff} else {-diffcs}
            }
            e.locate_current += diff;
            e.time += difftm as i32;
        }

        let mut ret: [i32; MAX_TOUCH_EV] = [TouchEvent::NOTHING; MAX_TOUCH_EV];
        for i in 0..MAX_TOUCH_EV {
            ret[i] = self.ev[i].locate_current;
        }
        ret
    }
}
//*******************************************************************
//          Position LED
//*******************************************************************
pub struct PositionLed {
    total_time: u32,
    fade_counter: u32,
    light_lvl: [i32; MAX_EACH_LIGHT*MAX_DEVICE_MBR3110],
    light_lvl_itp: [i32; MAX_EACH_LIGHT*MAX_DEVICE_MBR3110],
}
impl PositionLed {
    const FADE_RATE: u32 = 5;
    pub fn init() -> Self {
        Self::clear_all();
        Self {
            total_time: 0,
            fade_counter: 0,
            light_lvl: [0; MAX_EACH_LIGHT*MAX_DEVICE_MBR3110],
            light_lvl_itp: [0; MAX_EACH_LIGHT*MAX_DEVICE_MBR3110],
        }
    }
    pub fn clear_all() {
        for i in 0..MAX_DEVICE_MBR3110 {
            for j in 0..MAX_EACH_LIGHT {
                Self::light_led_each(j, i, 0);
            }
        }
    }
    pub fn gen_lighting_in_loop(&mut self, abstime: u32, tchev: &[i32; MAX_TOUCH_EV]) {
        let difftm = abstime - self.total_time;
        self.total_time = abstime;
        self.fade_counter += difftm;
        if self.fade_counter > Self::FADE_RATE {self.fade_counter = 0;}

        // Touch 位置を光らせる
        self.light_lvl = [0; MAX_EACH_LIGHT*MAX_DEVICE_MBR3110];
        let mut _max_ev: i32 = 0;
        for i in 0..MAX_TOUCH_EV {
            if tchev[i] == TouchEvent::NOTHING {break;}
            let frac = tchev[i]%100;
            let pos = (tchev[i]/100) as usize;
            for j in 0..2 { // タッチ位置とその両側
                let tm100 = 100*(j as i32);
                self.light_lvl[pos+1+j] += if (frac+100)>tm100 {(frac+100)-tm100} else {0};
                if pos >= j {
                    self.light_lvl[pos-j] += if (199-frac)>tm100 {(199-frac)-tm100} else {0};
                }
            }
            _max_ev += 1;
        }

        // 各かまぼこのLED点灯処理
        for k in 0..MAX_DEVICE_MBR3110 {
            self.one_kamaboco(k);
        }
        if self.fade_counter == 0 {self.fade_counter = 1;}

        //return max_ev;
    }
    fn one_kamaboco(&mut self, kamanum: usize) {
        const MIN_DIFF: u32 = 4;    // 隣り合う LED の明るさの差
        const MAX_STRENGTH: u32 = (MAX_EACH_LIGHT as u32)*(MIN_DIFF as u32); // 64
        const HALF_STRENGTH: u32 = MAX_STRENGTH/2; // 32

        let divided_time = self.total_time/5;
        let offset_num = kamanum*MAX_EACH_LIGHT;

        for i in 0..MAX_EACH_LIGHT {
            let x = i+offset_num;
            if self.light_lvl[x] > 0 || self.light_lvl_itp[x] > 0 {
                if self.light_lvl[x]>self.light_lvl_itp[x] {
                    self.light_lvl_itp[x] = self.light_lvl[x];
                }
                else if self.fade_counter == 0 {
                    self.light_lvl_itp[x] = (self.light_lvl_itp[x]-self.light_lvl[x])*3/4 + self.light_lvl[x];
                }
                Self::light_led_each(i, kamanum, (self.light_lvl_itp[x]*20) as u16);
            }
            else {
                // 背景で薄く光っている
                let mut strength = (divided_time + (MIN_DIFF*(i as u32)))%MAX_STRENGTH;
                if strength >= HALF_STRENGTH {strength = MAX_STRENGTH - strength;}
                Self::light_led_each(i, kamanum, strength as u16);
            }
        }
    }
    pub fn light_led_each(num: usize, dev_num: usize, mut strength: u16) {
        // strength=0-4095
        let adrs = (num as u8)* 4 + 0x06;
        if strength > 4095 {
            strength = 4095;
        }
        Pca9544::change_i2cbus(0, 3, dev_num);
        Pca9685::write(0, 0, adrs, 0); // ONはtime=0
        Pca9685::write(0, 0, adrs + 1, 0); // ONはtime=0
        Pca9685::write(0, 0, adrs + 2, (strength & 0x00ff) as u8); // OFF 0-4095 (0-0x0fff) の下位8bit
        Pca9685::write(0, 0, adrs + 3, (strength >> 8) as u8); // OFF 上位4bit
        // 別のI2Cバスに変えないと、他のkamanumのときに上書きされてしまう
        Pca9544::change_i2cbus(0, 1, dev_num);
    }
}