//  Created by Hasebe Masahiko on 2023/08/22.
//  Copyright (c) 2023 Hasebe Masahiko.
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php
//

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
    pub fn event_10ms(&self) -> bool {self.event_10ms}
    pub fn event_100ms(&self) -> bool {self.event_100ms}
    pub fn event_1s(&self) -> bool {self.event_1s}
    pub fn get_ms(&self) -> u32 {self.count_1ms}
}