#![no_std]
#![no_main]
#![feature(asm)]

#[macro_use] extern crate stm32f103xx;
#[macro_use] extern crate cortex_m_rt_macros;
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate panic_halt;

use stm32f103xx::{Interrupt, Peripherals, CorePeripherals, gpioa};
use core::cell::{Cell, UnsafeCell};
use core::mem::uninitialized;
use cortex_m_rt::entry;

mod mutex;
mod i2c;
mod ds3231;
mod at24c;
mod tim;

#[inline(always)]
fn get_gs() -> &'static mut GlobalState<'static> {
    unsafe {GS.as_mut().unwrap()}
}

#[inline(always)]
#[exception]
fn SysTick() {
    GlobalState::systick_handler();
}

#[inline]
fn digits2bcds(digs: &[u8]) -> u32 {
    let mut res: u32 = 0;
    for d in digs.iter().rev() {
        res = (res << 4) | (*d as u32);
    }
    res
}

fn inc_rotate(n: &mut u8, reset: u8, limit: u8) {
    *n += 1;
    if *n == limit {
        *n = reset;
    }
}

#[inline]
fn countdown(n: u32) -> u32 {
    if n > 0 { n - 1 } else { n }
}

fn render1(buff: &mut [u8; 6], mut n: u32) {
    for i in 0..buff.len() {
        buff[i] = (n % 10) as u8;
        n /= 10;
    }
}

fn render3(buff: &mut [u8; 6], n1: u8, n2: u8, n3: u8) {
    buff[1] = n3 / 10; buff[0] = n3 - buff[1] * 10;
    buff[3] = n2 / 10; buff[2] = n2 - buff[3] * 10;
    buff[5] = n1 / 10; buff[4] = n1 - buff[5] * 10;
}

struct ShiftRegister<'a> {
    gpioa: &'a gpioa::RegisterBlock,
    width: u8,
}

#[derive(Clone, Copy)]
struct Time {
    sec: u8,
    min: u8,
    hr: u8,
}

#[derive(Clone, Copy)]
enum DispState {
    On,
    Off,
    TempOn
}

struct GlobalState<'a> {
    core_perip: CorePeripherals,
    perip: Peripherals,
    disp: ShiftRegister<'a>,
    btn1: Button<'a>,
    btn2: Button<'a>,
    events: EventTimer<'a>,
    i2c: i2c::I2C<'a>,
    sync_cnt: Cell<u8>,
    buff: [u8; 6],
    disp_state: Cell<DispState>,
    tempon_cycle: Cell<u16>,
    tempon_peroid: Cell<u16>,
    tempon_cnt: Cell<u16>,
    blinky: [bool; 6],
    blink_state: Cell<bool>,
    blinky_enabled: Cell<Option<u8>>,
    pidx: usize,
    time_panel: TimePanel,
    date_panel: DatePanel,
    temp_panel: TempPanel,
    cd_panel: CountdownPanel,
    cu_panel: CountupPanel,
    set_panel: SettingPanel,
    panels: [&'a mut Panel<'a>; 6],
}

#[derive(Clone, Copy)]
enum ButtonState {
    Idle,
    PressedLock,
    PressedUnlock,
    ReleaseLock,
    Continous
}

#[derive(Clone, Copy)]
enum ButtonMode {
    Release,
    Immediate,
    Continous
}

struct Button<'a> {
    state: Cell<ButtonState>,
    long: Cell<bool>,
    events: &'a EventTimer<'a>,
    ev_id: Cell<u8>,
    mode: Cell<ButtonMode>
}

enum ButtonResult {
    FalseAlarm,
    ShortPress,
    LongPress
}

trait Panel<'a> {
    fn btn1_press(&'a mut self) -> bool {
        get_gs().btn1.press(ButtonMode::Release);
        true
    }

    fn btn1_release_extra(&mut self) {}

    fn btn1_release(&'a mut self) -> bool {
        self.btn1_release_extra();
        let gs = get_gs();
        let btn1 = &gs.btn1;
        match btn1.release() {
            ButtonResult::FalseAlarm => true,
            ButtonResult::ShortPress => self.btn1_short(),
            ButtonResult::LongPress => {
                gs.disp_state.set(DispState::Off);
                gs.tempon_cnt.set(gs.tempon_cycle.get());
                true
            }
        }
    }

    fn btn1_short(&mut self) -> bool {false}
    fn btn1_long(&mut self) -> bool {false}
    fn btn2_short(&mut self) -> bool {false}
    fn btn2_long(&mut self) -> bool {false}
    fn update_output(&self);
}

trait Timeoutable<'a> {
    fn timeout(&'a self);
}

#[derive(Clone, Copy)]
enum TimePanelState {
    Inactive,
    View,
    EditHr,
    EditMin,
    EditSec,
}

struct TimePanel {
    state: Cell<TimePanelState>,
    time: Time,
    tmp: Time,
    blink_enabled: Cell<bool>
}

#[derive(Clone, Copy)]
struct Date {
    yr: u8,
    mon: u8,
    day: u8
}

#[derive(Clone, Copy)]
enum DatePanelState {
    Inactive,
    View,
    EditYr,
    EditMon,
    EditDay
}

struct DatePanel {
    state: Cell<DatePanelState>,
    date: Date,
    tmp: Date,
    blink_enabled: Cell<bool>
}

#[derive(Clone, Copy)]
enum TempPanelState {
    Inactive,
    View
}

struct TempPanel {
    state: Cell<TempPanelState>,
    temp: Cell<ds3231::Temp>
}

#[derive(Clone, Copy)]
enum CountdownPanelState {
    Inactive,
    View,
    EditWhole,
    Edit3,
    Edit2,
    Edit1,
    OnGoing,
    OnGoingPaused,
    TimeUp
}

struct CountdownPanel {
    state: Cell<CountdownPanelState>,
    presets: [(u8, u8, u8); 4],
    counter: Cell<u32>,
    didx: Cell<u8>,
    blink_enabled: Cell<bool>
}

#[derive(Clone, Copy)]
enum CountupPanelState {
    Inactive,
    View,
    OnGoing,
    OnGoingPaused
}

struct CountupPanel {
    state: Cell<CountupPanelState>,
    counter: Cell<u32>
}

#[derive(Clone, Copy)]
enum SettingPanelState {
    Inactive,
    View,
    SettingChoice,
    Edit3,
    Edit2,
    Edit1,
    Edit0
}

#[derive(Clone, Copy)]
enum SettingIdx {
    TempOnCycle,
    TempOnPeroid
}

struct SettingPanel {
    state: Cell<SettingPanelState>,
    tmp: [u8; 6],
    idx: Cell<SettingIdx>
}

#[derive(Clone, Copy)]
struct AlarmEvent<'a> {
    cb: &'a Timeoutable<'a>,
    cnt: u32,
    free: bool
}

struct EventTimer<'a> {
    events: UnsafeCell<[AlarmEvent<'a>; 8]>
}

impl<'a> ShiftRegister<'a> {
    fn new(gpioa: &'a gpioa::RegisterBlock,
           width: u8) -> Self {
        ShiftRegister{gpioa, width}
    }

    fn output_bits(&self, bits: u32) {
        let bsrr = &self.gpioa.bsrr;
        for i in (0..self.width).rev() {
            bsrr.write(|w| w.br1().reset());
            /* feed the ser */
            match (bits >> i) & 1 {
                0 => bsrr.write(|w| w.br0().reset()),
                1 => bsrr.write(|w| w.bs0().set()),
                _ => panic!()
            }
            /* shift (trigger the sclk) */
            bsrr.write(|w| w.bs1().set());
        }
        /* latch on (trigger the clk) */
        bsrr.write(|w| w.br2().reset());
        bsrr.write(|w| w.bs2().set());
    }
}

impl Time {
    fn tick(&mut self) {
        self.sec += 1;
        if self.sec == 60 {
            self.min += 1;
            self.sec = 0;
        }

        if self.min == 60 {
            self.hr += 1;
            self.min = 0;
        }

        if self.hr == 24 {
            self.hr = 0;
        }
    }
}

impl<'a> Timeoutable<'a> for Button<'a> {
    fn timeout(&'a self) {
        use ButtonState::*;
        self.state.set(match self.state.get() {
            PressedLock => {
                match self.mode.get() {
                    ButtonMode::Immediate => (),
                    _ => self.ev_id.set(self.events.add(self, BUTTON_LONGPRESS_THRES))
                }
                PressedUnlock
            },
            PressedUnlock => {
                match self.mode.get() {
                    ButtonMode::Continous => {
                        self.ev_id.set(self.events.add(self, BUTTON_CONT_CYCLE));
                        Continous
                    },
                    ButtonMode::Release => {
                        self.long.set(true);
                        PressedUnlock
                    },
                    ButtonMode::Immediate => PressedUnlock
                }
            },
            ReleaseLock => Idle,
            Continous => {
                let gs = get_gs();
                gs.panels[gs.pidx].btn1_short();
                self.ev_id.set(self.events.add(self, BUTTON_CONT_CYCLE));
                Continous
            },
            s => s
        });
    }
}

impl<'a> Button<'a> {
    fn new(events: &'a EventTimer<'a>) -> Self {
        Button {state: Cell::new(ButtonState::Idle),
                long: Cell::new(false),
                ev_id: Cell::new(0),
                mode: Cell::new(ButtonMode::Release),
                events}
    }

    fn press(&'a self, mode: ButtonMode) {
        if let ButtonState::Idle = self.state.get() {
            self.mode.set(mode);
            self.state.set(ButtonState::PressedLock);
            self.long.set(false);
            self.ev_id.set(self.events.add(self, BUTTON_PRESSLOCK_PEROID));
        }
    }

    fn release(&'a self) -> ButtonResult {
        match self.state.get() {
            ButtonState::PressedUnlock => {
                let mut r = ButtonResult::ShortPress;
                if let ButtonMode::Immediate = self.mode.get() {
                    r = ButtonResult::FalseAlarm;
                } else {
                    if self.long.get() { /* ev_id already dropped */
                        r = ButtonResult::LongPress;
                    } else {
                        self.events.drop(self.ev_id.get());
                    }
                }
                self.state.set(ButtonState::ReleaseLock);
                self.ev_id.set(self.events.add(self, BUTTON_PRESSLOCK_PEROID));
                r
            },
            ButtonState::Continous => {
                self.events.drop(self.ev_id.get());
                self.state.set(ButtonState::ReleaseLock);
                self.ev_id.set(self.events.add(self, BUTTON_PRESSLOCK_PEROID));
                ButtonResult::FalseAlarm
            },
            _ => ButtonResult::FalseAlarm
        }
    }
}

impl<'a> Panel<'a> for TimePanel {
    fn btn1_press(&'a mut self) -> bool {
        use TimePanelState::*;
        get_gs().btn1.press(match self.state.get() {
            EditHr | EditMin | EditSec => {
                self.blink_enabled.set(false);
                self.update_output();
                ButtonMode::Continous
            },
            _ => ButtonMode::Release
        });
        true
    }

    fn btn1_release_extra(&mut self) {
        use TimePanelState::*;
        match self.state.get() {
            EditHr | EditMin | EditSec => {
                self.blink_enabled.set(true);
                self.update_output();
            },
            _ => ()
        }
    }

    fn btn1_short(&mut self) -> bool {
        use TimePanelState::*;
        {
            let tmp = &mut self.tmp;
            match self.state.get() {
                View => {
                    self.state.set(Inactive);
                    return false;
                }, /* yield to the next panel */
                EditHr => inc_rotate(&mut tmp.hr, 0, 24),
                EditMin => inc_rotate(&mut tmp.min, 0, 60),
                EditSec => inc_rotate(&mut tmp.sec, 0, 60),
                Inactive => self.state.set(View)
            };
        }
        self.update_output();
        true
    }

    fn btn2_short(&mut self) -> bool {
        use TimePanelState::*;
        let s = match self.state.get() {
            View => {
                let tmp = &mut self.tmp;
                let time = &mut self.time;
                tmp.hr = time.hr;
                tmp.min = time.min;
                tmp.sec = time.sec;
                EditHr
            },
            EditHr => EditMin,
            EditMin => EditSec,
            EditSec => {
                let tmp = &self.tmp;
                ds3231::DS3231(&get_gs().i2c)
                                .write_time(tmp.hr, tmp.min, tmp.sec);
                self.time = *tmp;
                View
            },
            s => s
        };
        self.state.set(s);
        self.update_output();
        true
    }

    fn update_output(&self) {
        use TimePanelState::*;
        let s = self.state.get();
        get_gs().update_blinky(
            if self.blink_enabled.get() {
                match s {
                    EditHr => 0b110000,
                    EditMin => 0b001100,
                    EditSec => 0b000011,
                    _ => 0x0,
                }
            } else {
                0x0
            });
        let gs = get_gs();
        let time = &self.time;
        let tmp = &self.tmp;
        match s {
            View => gs.render3(time.hr, time.min, time.sec),
            _ => gs.render3(tmp.hr, tmp.min, tmp.sec)
        }
        gs.display();
    }
}

impl TimePanel {
    fn update_clock(&mut self, clk: Option<Time>) {
        let time = &mut self.time;
        let gs = get_gs();
        match clk {
            Some(clk) => *time = clk,
            None => time.tick()
        }
        if let TimePanelState::View = self.state.get() {
            gs.render3(time.hr, time.min, time.sec);
            gs.display();
        }
    }
}

impl<'a> Panel<'a> for DatePanel {
    fn btn1_press(&'a mut self) -> bool {
        use DatePanelState::*;
        get_gs().btn1.press(match self.state.get() {
            EditYr | EditMon | EditDay => {
                self.blink_enabled.set(false);
                self.update_output();
                ButtonMode::Continous
            },
            _ => ButtonMode::Release
        });
        true
    }

    fn btn1_release_extra(&mut self) {
        use DatePanelState::*;
        match self.state.get() {
            EditYr | EditMon | EditDay => {
                self.blink_enabled.set(true);
                self.update_output();
            },
            _ => ()
        }
    }

    fn btn1_short(&mut self) -> bool {
        use DatePanelState::*;
        {
            let tmp = &mut self.tmp;
            match self.state.get() {
                View => {
                    self.state.set(Inactive);
                    return false;
                }, /* yield to the next panel */
                EditYr => inc_rotate(&mut tmp.yr, 0, 100),
                EditMon => inc_rotate(&mut tmp.mon, 1, 13),
                EditDay => inc_rotate(&mut tmp.day, 1, 32),
                Inactive => self.state.set(View)
            };
        }
        self.update_output();
        true
    }

    fn btn2_short(&mut self) -> bool {
        use DatePanelState::*;
        let s = match self.state.get() {
            View => {
                let date = &self.date;
                let tmp = &mut self.tmp;
                tmp.yr = date.yr;
                tmp.mon = date.mon;
                tmp.day = date.day;
                EditYr
            },
            EditYr => EditMon,
            EditMon => EditDay,
            EditDay => {
                let tmp = &self.tmp;
                ds3231::DS3231(&get_gs().i2c)
                                .write_date(tmp.yr, tmp.mon, tmp.day);
                self.date = *tmp;
                View
            },
            s => s
        };
        self.state.set(s);
        self.update_output();
        true
    }

    fn update_output(&self) {
        use DatePanelState::*;
        let s = self.state.get();
        get_gs().update_blinky(
            if self.blink_enabled.get() {
                match s {
                    EditYr => 0b110000,
                    EditMon => 0b001100,
                    EditDay => 0b000011,
                    _ => 0x0,
                }
            } else {
                0x0
            });
        let gs = get_gs();
        let date = &self.date;
        let tmp = &self.tmp;
        match s {
            View => gs.render3(date.yr, date.mon, date.day),
            _ => gs.render3(tmp.yr, tmp.mon, tmp.day)
        };
        gs.display();
    }
}

impl DatePanel {
    fn update_clock(&mut self, d: Option<Date>) {
        let date = &mut self.date;
        match d {
            Some(d) => *date = d,
            None => ()
        }
        if let DatePanelState::View = self.state.get() {
            let gs = get_gs();
            gs.render3(date.yr, date.mon, date.day);
            gs.display();
        }
    }
}

impl<'a> Panel<'a> for TempPanel {
    fn btn1_short(&mut self) -> bool {
        use TempPanelState::*;
        match self.state.get() {
            View => {
                self.state.set(Inactive);
                return false;
            },
            Inactive => {
                self.state.set(View);
            }
        }
        self.update_output();
        true
    }

    fn update_output(&self) {
        let mut buff: [u8; 6] = [0xf; 6];
        let temp = self.temp.get();
        let q = temp.quarter * 25;
        let zeros: [u8; 4] = [0; 4];
        let mut c = if temp.cels > 0 { temp.cels } else {
            buff[2..6].clone_from_slice(&zeros);
            -temp.cels
        } as u8;
        let mut i = 2;
        while c > 0 {
            buff[i] = c % 10;
            c /= 10;
            i += 1;
        }
        buff[1] = q / 10;
        buff[0] = q - 10 * buff[1];
        let gs = get_gs();
        gs.buff = buff;
        gs.display();
    }
}

impl TempPanel {
    fn update_clock(&mut self, temp: Option<ds3231::Temp>) {
        match temp {
            Some(temp) => self.temp.set(temp),
            None => ()
        }
        if let TempPanelState::View = self.state.get() {
            self.update_output();
        }
    }
}

impl<'a> Panel<'a> for CountdownPanel {
    fn btn1_press(&'a mut self) -> bool {
        use CountdownPanelState::*;
        get_gs().btn1.press(match self.state.get() {
            Edit3 | Edit2 | Edit1 => {
                self.blink_enabled.set(false);
                self.update_output();
                ButtonMode::Continous
            },
            OnGoing | OnGoingPaused => {
                self.btn1_short();
                ButtonMode::Immediate
            },
            _ => ButtonMode::Release
        });
        true
    }

    fn btn1_release_extra(&mut self) {
        use CountdownPanelState::*;
        match self.state.get() {
            Edit3 | Edit2 | Edit1 => {
                self.blink_enabled.set(true);
                self.update_output();
            },
            _ => ()
        }
    }

    fn btn1_short(&mut self) -> bool {
        use CountdownPanelState::*;
        let timer = tim::Timer(&get_gs().perip.TIM3);
        {
            let didx = self.didx.get();
            let presets = &mut self.presets;
            let len = presets.len() as u8;
            let p = &mut presets[didx as usize];
            match self.state.get() {
                View => {
                    self.state.set(Inactive);
                    return false;
                },
                EditWhole => {
                    let mut nidx = didx;
                    inc_rotate(&mut nidx, 0, len);
                    self.didx.set(nidx);
                },
                Edit3 => inc_rotate(&mut p.0, 0, 100),
                Edit2 => inc_rotate(&mut p.1, 0, 100),
                Edit1 => inc_rotate(&mut p.2, 0, 100),
                OnGoing => {
                    timer.stop();
                    self.state.set(OnGoingPaused);
                },
                OnGoingPaused => {
                    timer.go();
                    self.state.set(OnGoing);
                },
                TimeUp => self.state.set(View),
                Inactive => self.state.set(View)
            }
        }
        self.update_output();
        true
    }

    fn btn2_long(&mut self) -> bool {
        use CountdownPanelState::*;
        self.state.set(match self.state.get() {
            OnGoing => OnGoing,
            OnGoingPaused => OnGoingPaused,
            TimeUp => TimeUp,
            _ => {
                self.go();
                OnGoing
            },
        });
        self.update_output();
        true
    }

    fn btn2_short(&mut self) -> bool {
        use CountdownPanelState::*;
        self.state.set(match self.state.get() {
            View => EditWhole,
            EditWhole => Edit3,
            Edit3 => Edit2,
            Edit2 => Edit1,
            Edit1 => {
                self.go();
                OnGoing
            },
            OnGoingPaused => View,
            s => s
        });
        self.update_output();
        true
    }

    fn update_output(&self) {
        use CountdownPanelState::*;
        let s = self.state.get();
        get_gs().update_blinky(
            if self.blink_enabled.get() {
                match s {
                    EditWhole | TimeUp => 0b111111,
                    Edit3 => 0b110000,
                    Edit2 => 0b001100,
                    Edit1 => 0b000011,
                    _ => 0x0
                }
            } else {
                0x0
            });
        let gs = get_gs();
        match s {
            OnGoing | OnGoingPaused => gs.render1(self.counter.get()),
            _ => {
                let preset = &self.presets[self.didx.get() as usize];
                gs.render3(preset.0, preset.1, preset.2);
            }
        }
        gs.display();
    }
}

impl CountdownPanel {
    fn go(&self) {
        let timer = tim::Timer(&get_gs().perip.TIM3);
        let (p0, p1, p2) = self.presets[self.didx.get() as usize];
        let x = (p0 as u32) * 10000 + (p1 as u32) * 100 + (p2 as u32);
        self.counter.set(x);
        timer.reset();
        timer.go();
    }

    fn update_clock(&mut self) {
        if let CountdownPanelState::Inactive = self.state.get() {
            return
        }
        let x = countdown(self.counter.get());
        if x == 0 {
            let timer = tim::Timer(&get_gs().perip.TIM3);
            timer.stop();
            self.state.set(CountdownPanelState::TimeUp);
        } else {
            self.counter.set(x);
        }
        self.update_output();
    }
}

impl<'a> Panel<'a> for CountupPanel {
    fn btn1_short(&mut self) -> bool {
        use CountupPanelState::*;
        let timer = tim::Timer(&get_gs().perip.TIM3);
        match self.state.get() {
            View => {
                self.state.set(Inactive);
                return false;
            },
            OnGoing => {
                timer.stop();
                self.state.set(OnGoingPaused);
            },
            OnGoingPaused => {
                timer.go();
                self.state.set(OnGoing);
            },
            Inactive => self.state.set(View)
        }
        self.update_output();
        true
    }

    fn btn2_short(&mut self) -> bool {
        use CountupPanelState::*;
        let timer = tim::Timer(&get_gs().perip.TIM3);
        self.state.set(match self.state.get() {
            View => {
                self.counter.set(0);
                timer.reset();
                timer.go();
                OnGoing
            },
            OnGoingPaused => View,
            s => s
        });
        self.update_output();
        true
    }

    fn update_output(&self) {
        use CountupPanelState::*;
        get_gs().update_blinky(match self.state.get() {
            View => 0b111111,
            _ => 0x0
        });
        let gs = get_gs();
        gs.render1(self.counter.get());
        gs.display();
    }
}

impl CountupPanel {
    fn update_clock(&mut self) {
        if let CountupPanelState::Inactive = self.state.get(){
            return
        }
        let x = self.counter.get() + 1;
        self.counter.set(if x > 999999 {0} else {x});
        self.update_output();
    }
}

impl SettingPanel {
    fn render_idx(&mut self) {
        let gs = get_gs();
        render1(&mut self.tmp, match self.idx.get() {
            SettingIdx::TempOnCycle => gs.tempon_cycle.get(),
            SettingIdx::TempOnPeroid => gs.tempon_peroid.get()
        } as u32);
    }

    fn set_idx(&self) {
        let mut x: u32 = 0;
        let gs = get_gs();
        for v in self.tmp.iter().rev() {
            x *= 10;
            x += *v as u32;
        }
        match self.idx.get() {
            SettingIdx::TempOnCycle => gs.tempon_cycle.set(x as u16),
            SettingIdx::TempOnPeroid => gs.tempon_peroid.set(x as u16)
        }
    }
}

impl<'a> Panel<'a> for SettingPanel {
    fn btn1_short(&mut self) -> bool {
        use SettingPanelState::*;
        {
            match self.state.get() {
                View => {
                    self.state.set(Inactive);
                    return false;
                },
                SettingChoice => {
                    self.idx.set(match self.idx.get() {
                        SettingIdx::TempOnCycle => SettingIdx::TempOnPeroid,
                        SettingIdx::TempOnPeroid => SettingIdx::TempOnCycle
                    });
                    self.render_idx();
                },
                Edit3 => inc_rotate(&mut self.tmp[3], 0, 10),
                Edit2 => inc_rotate(&mut self.tmp[2], 0, 10),
                Edit1 => inc_rotate(&mut self.tmp[1], 0, 10),
                Edit0 => inc_rotate(&mut self.tmp[0], 0, 10),
                Inactive => {
                    self.render_idx();
                    self.state.set(View);
                }
            }
        }
        self.update_output();
        true
    }

    fn btn2_short(&mut self) -> bool {
        use SettingPanelState::*;
        self.state.set(match self.state.get() {
            View => {
                SettingChoice
            },
            SettingChoice => Edit3,
            Edit3 => Edit2,
            Edit2 => Edit1,
            Edit1 => Edit0,
            Edit0 => { self.set_idx(); View },
            s => s
        });
        self.update_output();
        true
    }

    fn btn2_long(&mut self) -> bool {
        use SettingPanelState::*;
        self.state.set(match self.state.get() {
            View => View,
            SettingChoice => View,
            _ => { self.set_idx(); View },
        });
        self.update_output();
        true
    }

    fn update_output(&self) {
        use SettingPanelState::*;
        let s = self.state.get();
        get_gs().update_blinky(match s {
            SettingChoice => 0b111111,
            Edit3 => 0b001000,
            Edit2 => 0b000100,
            Edit1 => 0b000010,
            Edit0 => 0b000001,
            _ => 0x0
        });
        let gs = get_gs();
        gs.render(&self.tmp);
        gs.display();
    }
}

impl<'a> EventTimer<'a> {
    fn new() -> Self {
        unsafe {
            EventTimer{events: UnsafeCell::new([
                    AlarmEvent{cb: uninitialized(),
                               cnt: 0,
                               free: true}; 8])}
        }
    }

    fn add(&self, f: &'a Timeoutable<'a>, timeout: u32) -> u8 {
        unsafe {
            for (i, v) in (*self.events.get()).iter_mut().enumerate() {
                if v.free {
                    v.cb = f;
                    v.cnt = timeout;
                    v.free = false;
                    return i as u8;
                }
            }
            panic!("event manager full");
        }
    }

    fn drop(&self, id: u8) {
        unsafe {
            (*self.events.get())[id as usize].free = true;
        }
    }

    fn tick(&self) {
        unsafe {
            for v in &mut *self.events.get() {
                if v.free {continue;}
                v.cnt = countdown(v.cnt);
                if v.cnt == 0 {
                    v.cb.timeout();
                    v.free = true;
                }
            }
        }
    }
}

impl<'a> GlobalState<'a> {

    fn render(&mut self, nbuff: &[u8; 6]) {
        self.buff.copy_from_slice(nbuff);
    }
    fn render1(&mut self, n: u32) {
        render1(&mut self.buff, n);
    }

    fn render3(&mut self, n1: u8, n2: u8, n3: u8) {
        render3(&mut self.buff, n1, n2, n3);
    }

    fn display(&self) {
        let mut buff = self.buff;
        let b = self.blink_state.get();
        let bs = self.blinky;
        match self.disp_state.get() {
            DispState::On | DispState::TempOn =>
                for (i, v) in buff.iter_mut().enumerate() {
                    if b && bs[i] { *v = 0xf; }
                },
            DispState::Off => for i in buff.iter_mut() { *i = 0xf; }
        }
        self.disp.output_bits(digits2bcds(&buff[..]));
    }

    fn update_blinky(&'a mut self, nb: u8) {
        let flag = nb == 0; /* if nothing is blinking */
        let en = self.blinky_enabled.get();
        for (i, v) in self.blinky.iter_mut().enumerate() {
            *v = ((nb >> i) & 1) == 1;
        }
        let ev = &self.events;
        if flag {
            if let Some(ev_id) = en {
                self.blink_state.set(false);
                self.blinky_enabled.set(None);
                ev.drop(ev_id);
            }
        } else {
            if en.is_none() {
                self.blink_state.set(false);
                self.blinky_enabled.set(Some(ev.add(self, BLINK_PERIOD)));
            }
        }
    }

    fn digits_countup(&self) {
        self.display();
        let mut buff = self.buff;
        let mut i = 0;
        let mut carry = 1;
        while carry > 0 && i < buff.len() {
            buff[i] += carry;
            carry = if buff[i] > 9 {buff[i] = 0; 1} else {0};
            i += 1;
        }
    }

    fn systick_handler() {
        let gs = get_gs();
        let mut clk = None;
        let mut d = None;
        let mut temp = None;
        if gs.sync_cnt.get() == 0 {
            let rtc = ds3231::DS3231(&gs.i2c);
            let ds3231::Date{second: sec,
                            minute: min,
                            hour: hr,
                            date: day,
                            month: mon,
                            year: yr, ..} = rtc.read_fulldate();
            gs.sync_cnt.set(SYNC_PERIOD);
            clk = Some(Time{sec, min, hr});
            d = Some(Date{yr, mon, day});
            temp = Some(rtc.read_temperature());
        } else {
            gs.sync_cnt.set(gs.sync_cnt.get() - 1);
        }
        let tcnt = countdown(gs.tempon_cnt.get() as u32) as u16;
        match gs.disp_state.get() {
            DispState::Off => {
                if tcnt == 0 {
                    gs.disp_state.set(DispState::TempOn);
                    gs.tempon_cnt.set(gs.tempon_peroid.get());
                } else {
                    gs.tempon_cnt.set(tcnt);
                }
            },
            DispState::TempOn => {
                if tcnt == 0 {
                    gs.disp_state.set(DispState::Off);
                    gs.tempon_cnt.set(gs.tempon_cycle.get());
                } else {
                    gs.tempon_cnt.set(tcnt);
                }
            },
            _ => ()
        }
        gs.time_panel.update_clock(clk);
        gs.date_panel.update_clock(d);
        gs.temp_panel.update_clock(temp);
    }

    #[inline(always)]
    fn tim4_callback() {
        let gs = get_gs();
        {
            let p = &gs.perip;
            p.TIM4.sr.modify(|_, w| w.uif().clear());
        }
        gs.events.tick();
    }

    #[inline(always)]
    fn tim3_callback() {
        let gs = get_gs();
        {
            let p = &gs.perip;
            p.TIM3.sr.modify(|_, w| w.uif().clear());
        }
        gs.cd_panel.update_clock();
        gs.cu_panel.update_clock();
    }


    #[inline(always)]
    fn exti3_handler() {
        let gs = get_gs();
        let p = &gs.perip;
        p.EXTI.pr.write(|w| w.pr3().set_bit());
        let x = p.GPIOA.idr.read().idr3().bit();
        match gs.disp_state.get() {
            DispState::Off => {gs.disp_state.set(DispState::On); return},
            DispState::TempOn => gs.disp_state.set(DispState::On),
            _ => ()
        }
        if !(match x {
                false => gs.panels[gs.pidx].btn1_press(),
                true => gs.panels[gs.pidx].btn1_release()})
        {
            let gs = get_gs();
            /* swtich the sub state machine */
            gs.pidx += 1;
            if gs.pidx == gs.panels.len() {
                gs.pidx = 0;
            }
            gs.panels[gs.pidx].btn1_short();
            gs.display();
        }
    }

    #[inline(always)]
    fn exti4_handler() {
        let gs = get_gs();
        let btn2 = &gs.btn2;
        let p = &gs.perip;
        p.EXTI.pr.write(|w| w.pr4().set_bit());
        let x = p.GPIOA.idr.read().idr4().bit();
        if !x {
            btn2.press(ButtonMode::Release);
        } else {
            let gs = get_gs();
            match btn2.release() {
                ButtonResult::FalseAlarm => (),
                ButtonResult::ShortPress => {gs.panels[gs.pidx].btn2_short();},
                ButtonResult::LongPress => {gs.panels[gs.pidx].btn2_long();}
            }
            gs.display();
        }
    }
}

impl<'a> Timeoutable<'a> for GlobalState<'a> {
    fn timeout(&'a self) {
        let ev = &self.events;
        self.blink_state.set(!self.blink_state.get());
        self.display();
        if self.blinky_enabled.get().is_some() {
            self.blinky_enabled.set(Some(ev.add(self, BLINK_PERIOD)));
        }
    }
}

//exception!(SYS_TICK, GlobalState::systick_handler);
interrupt!(EXTI4, GlobalState::exti4_handler);
interrupt!(EXTI3, GlobalState::exti3_handler);
interrupt!(TIM4, GlobalState::tim4_callback);
interrupt!(TIM3, GlobalState::tim3_callback);

const SYNC_PERIOD: u8 = 10;
const BLINK_PERIOD: u32 = 500;
const BUTTON_PRESSLOCK_PEROID: u32 = 50;
const BUTTON_LONGPRESS_THRES: u32 = 500;
const BUTTON_CONT_CYCLE: u32 = 100;
const SYSTICK_CYCLE: u32 = 8_000_000;

static mut GS: Option<GlobalState> = None;

fn init_hardware() -> (CorePeripherals, Peripherals) {
    let mut cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    cp.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    cp.SYST.set_reload(SYSTICK_CYCLE);

    /* RCC */
    /* enable clock for GPIOA, GPIOB and AFIO */
    p.RCC.apb2enr.modify(|_, w| w.iopaen().enabled()    /* GPIOA */
                                .iopben().enabled()     /* GPIOB */
                                .afioen().enabled());   /* I2C AFIO */
    /* enable timers */
    p.RCC.apb1enr.modify(|_, w| w.tim4en().enabled()
                                 .tim3en().enabled());
    /* enable and reset I2C1 */
    p.RCC.apb1enr.modify(|_, w| w.i2c1en().enabled());
    p.RCC.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
    p.RCC.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());

    /* GPIO */
    /* set button operating mode to pull down */
    p.GPIOA.odr.modify(|_, w| w.odr3().set_bit()
                               .odr4().set_bit());
    /* enable PA0-2 for manipulating shift register */
    p.GPIOA.crl.modify(|_, w|
        w.mode0().output().cnf0().push()        /* disp bit */
         .mode1().output().cnf1().push()        /* disp shift */
         .mode2().output().cnf2().push()        /* disp latch */
         .mode3().input().cnf3().bits(0b10)     /* button 1 */
         .mode4().input().cnf4().bits(0b10));   /* button 2 */
    /* enable PB6 and PB7 for I2C1 */
    p.GPIOB.crl.modify(|_, w|
        w.mode6().output50().cnf6().alt_open()
         .mode7().output50().cnf7().alt_open());

    /* EXTI */
    p.AFIO.exticr1.write(|w| unsafe { w.exti3().bits(0b0000) });
    p.AFIO.exticr2.write(|w| unsafe { w.exti4().bits(0b0000) });
    p.EXTI.imr.write(|w| w.mr3().set_bit()
                          .mr4().set_bit());
    p.EXTI.rtsr.write(|w| w.tr3().set_bit()
                           .tr4().set_bit());
    p.EXTI.ftsr.write(|w| w.tr3().set_bit()
                           .tr4().set_bit());

    /* NVIC */
    cp.NVIC.enable(Interrupt::EXTI3);
    cp.NVIC.enable(Interrupt::EXTI4);
    cp.NVIC.enable(Interrupt::TIM3);
    cp.NVIC.enable(Interrupt::TIM4);
    return (cp, p)
}

fn init(cp: CorePeripherals, p: Peripherals) {
    unsafe {
        GS = Some(GlobalState{
            core_perip: cp,
            perip: p,
            btn1: uninitialized(),
            btn2: uninitialized(),
            events: EventTimer::new(),
            i2c: uninitialized(),
            disp: uninitialized(),
            sync_cnt: Cell::new(0),
            buff: [0; 6],
            disp_state: Cell::new(DispState::On),
            tempon_cycle: Cell::new(20),
            tempon_peroid: Cell::new(5),
            tempon_cnt: Cell::new(0),
            blinky: [false; 6],
            blink_state: Cell::new(false),
            blinky_enabled: Cell::new(None),
            pidx: 0,
            time_panel: TimePanel {
                state: Cell::new(TimePanelState::View),
                tmp: Time{sec: 0, min: 0, hr: 0},
                time: Time{sec: 0, min: 0, hr: 0},
                blink_enabled: Cell::new(true)
            },
            date_panel: DatePanel {
                state: Cell::new(DatePanelState::Inactive),
                tmp: Date{yr: 0, mon: 1, day: 1},
                date: Date{yr: 0, mon: 1, day: 1},
                blink_enabled: Cell::new(true)
            },
            temp_panel: TempPanel{
                state: Cell::new(TempPanelState::Inactive),
                temp: Cell::new(ds3231::Temp{cels: 0, quarter: 0})
            },
            cd_panel: CountdownPanel {
                state: Cell::new(CountdownPanelState::Inactive),
                presets: [(0, 0, 0); 4],
                counter: Cell::new(0),
                didx: Cell::new(0),
                blink_enabled: Cell::new(true),
            },
            cu_panel: CountupPanel{
                state: Cell::new(CountupPanelState::Inactive),
                counter: Cell::new(0)
            },
            set_panel: SettingPanel{
                state: Cell::new(SettingPanelState::Inactive),
                tmp: [9; 6],
                idx: Cell::new(SettingIdx::TempOnCycle),
            },
            panels: uninitialized()});
    }

    let gs = get_gs();

    gs.i2c = i2c::I2C(&gs.perip.I2C1);
    gs.disp = ShiftRegister::new(&gs.perip.GPIOA, 24);
    gs.btn1 = Button::new(&gs.events);
    gs.btn2 = Button::new(&gs.events);
    gs.panels = [&mut gs.time_panel,
                 &mut gs.date_panel,
                 &mut gs.temp_panel,
                 &mut gs.cd_panel,
                 &mut gs.cu_panel,
                 &mut gs.set_panel];

    /* configure I2C */
    gs.i2c.init(&gs.perip.RCC, 0x01, 400_000, i2c::DutyType::DUTY1, true);
    //gs.i2c.init(0x01, 100_000, i2c::DutyType::DUTY1, false);
    /* display zeros */
    gs.disp.output_bits(0);
    /* initialize 10ms couting clock */
    tim::Timer(&gs.perip.TIM3).init(10 * (8_000_000 / 1000));
    /* initialize and start 1ms-precision event clock */
    let tim4 = tim::Timer(&gs.perip.TIM4);
    tim4.init(1 * (8_000_000 / 1000));
    tim4.reset();
    tim4.go();
    /* start systick */
    gs.core_perip.SYST.enable_interrupt();
    gs.core_perip.SYST.enable_counter();
    GlobalState::systick_handler(); /* initialize internal time */
}

#[entry]
fn main() -> ! {
    loop {
        let (cp, p) = init_hardware();
        init(cp, p);
    }
}
