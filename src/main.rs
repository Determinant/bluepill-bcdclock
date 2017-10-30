#![no_std]
#![feature(asm)]
#![feature(const_cell_new)]
#![feature(const_refcell_new)]

#[macro_use] extern crate stm32f103xx;
extern crate cortex_m;

use stm32f103xx::{Interrupt, Peripherals, gpioa};
use core::cell::{Cell, RefCell, UnsafeCell};

mod mutex;
mod i2c;
mod ds3231;
mod at24c;
mod tim;

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

fn countdown(n: u32) -> u32 {
    if n > 0 { n - 1 } else { n }
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

#[derive(PartialEq, Clone, Copy)]
enum DispState {
    On,
    Off,
    TempOn
}

struct GlobalState<'a> {
    perip: Option<Peripherals<'a>>,
    disp: Option<ShiftRegister<'a>>,
    btn1: Option<Button<'a>>,
    btn2: Option<Button<'a>>,
    events: Option<AlarmEventManager<'a>>,
    i2c: Option<i2c::I2C<'a>>,
    i2c_inited: bool,
    sync_cnt: Cell<u8>,
    buff: RefCell<[u8; 6]>,
    disp_state: Cell<DispState>,
    tempon_cycle: Cell<u16>,
    tempon_peroid: Cell<u16>,
    tempon_cnt: Cell<u16>,
    blinky: RefCell<[bool; 6]>,
    blink_state: Cell<bool>,
    blinky_enabled: Cell<Option<u8>>,
    pidx: usize,
    panels: [&'a Panel; 5],
}

#[derive(PartialEq, Clone, Copy)]
enum ButtonState {
    Idle,
    PressedLock,
    PressedUnlock,
    ReleaseLock
}

struct Button<'a> {
    state: Cell<ButtonState>,
    long: Cell<bool>,
    events: &'a AlarmEventManager<'a>,
    ev_id: Cell<u8>
}

enum ButtonResult {
    FalseAlarm,
    ShortPress,
    LongPress
}

trait Panel {
    fn btn1_short(&self) -> bool {false}
    fn btn1_long(&self) -> bool {false}
    fn btn2_short(&self) -> bool {false}
    fn btn2_long(&self) -> bool {false}
    fn update_output(&self);
}

trait Timeoutable<'a> {
    fn timeout(&'a self);
}

#[derive(PartialEq, Clone, Copy)]
enum TimePanelState {
    Inactive,
    View,
    EditHr,
    EditMin,
    EditSec,
}

struct TimePanel<'a> {
    gs: &'a GlobalState<'a>,
    state: Cell<TimePanelState>, 
    time: RefCell<Time>,
    tmp: RefCell<Time>
}

#[derive(Clone, Copy)]
struct Date {
    yr: u8,
    mon: u8,
    day: u8
}

#[derive(PartialEq, Clone, Copy)]
enum DatePanelState {
    Inactive,
    View,
    EditYr,
    EditMon,
    EditDay
}

struct DatePanel<'a> {
    gs: &'a GlobalState<'a>,
    state: Cell<DatePanelState>,
    date: RefCell<Date>,
    tmp: RefCell<Date>
}

#[derive(PartialEq, Clone, Copy)]
enum TempPanelState {
    Inactive,
    View
}

struct TempPanel<'a> {
    state: Cell<TempPanelState>,
    temp: Cell<ds3231::Temp>,
    gs: &'a GlobalState<'a>,
}

#[derive(PartialEq, Clone, Copy)]
enum CountdownPanelState {
    Inactive,
    View,
    EditWhole,
    Edit3,
    Edit2,
    Edit1,
    Edit0,
    Edit2m,
    Edit1m,
    OnGoing,
    OnGoingPaused,
    TimeUp
}

struct CountdownPanel<'a> {
    state: Cell<CountdownPanelState>,
    presets: RefCell<[[u8; 6]; 2]>,
    counter: Cell<u32>,
    didx: Cell<u8>,
    gs: &'a GlobalState<'a>,
}

#[derive(PartialEq, Clone, Copy)]
enum CountupPanelState {
    Inactive,
    View,
    OnGoing,
    OnGoingPaused
}

struct CountupPanel<'a> {
    state: Cell<CountupPanelState>,
    counter: Cell<u32>,
    gs: &'a GlobalState<'a>,
}

#[derive(Clone, Copy)]
struct AlarmEvent<'a> {
    cb: &'a Timeoutable<'a>,
    cnt: u32,
    free: bool
}

struct AlarmEventManager<'a> {
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
        self.state.set(match self.state.get() {
            ButtonState::PressedLock => {
                self.ev_id.set(self.events.add(self, 500));
                ButtonState::PressedUnlock
            },
            ButtonState::PressedUnlock => {
                self.long.set(true);
                ButtonState::PressedUnlock
            },
            ButtonState::ReleaseLock => {
                ButtonState::Idle
            },
            s => s
        });
    }
}

impl<'a> Button<'a> {
    fn new(events: &'a AlarmEventManager<'a>) -> Self {
        Button {state: Cell::new(ButtonState::Idle),
                long: Cell::new(false),
                ev_id: Cell::new(0),
                events}
    }

    fn press(&'a self) {
        if self.state.get() == ButtonState::Idle {
            self.state.set(ButtonState::PressedLock);
            self.long.set(false);
            self.ev_id.set(self.events.add(self, 50));
        }
    }

    fn release(&'a self) -> ButtonResult {
        if self.state.get() == ButtonState::PressedUnlock {
            self.events.drop(self.ev_id.get());
            self.state.set(ButtonState::ReleaseLock);
            self.ev_id.set(self.events.add(self, 50));
            if self.long.get() { ButtonResult::LongPress }
            else { ButtonResult::ShortPress }
        } else { ButtonResult::FalseAlarm }
    }
}

impl<'a> Panel for TimePanel<'a> {
    fn btn1_short(&self) -> bool {
        use TimePanelState::*;
        {
            let mut tmp = self.tmp.borrow_mut();
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

    fn btn2_short(&self) -> bool {
        use TimePanelState::*;
        let s = match self.state.get() {
            View => {
                let mut tmp = self.tmp.borrow_mut();
                let time = self.time.borrow();
                tmp.hr = time.hr;
                tmp.min = time.min;
                tmp.sec = time.sec;
                EditHr
            },
            EditHr => EditMin,
            EditMin => EditSec,
            EditSec => {
                let tmp = self.tmp.borrow();
                ds3231::DS3231(self.gs.i2c.as_ref().unwrap())
                                .write_time(tmp.hr, tmp.min, tmp.sec);
                *self.time.borrow_mut() = *tmp;
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
        self.gs.update_blinky(match s {
            EditHr => 0b110000,
            EditMin => 0b001100,
            EditSec => 0b000011,
            _ => 0x0,
        });
        let time = self.time.borrow();
        let tmp = self.tmp.borrow();
        match s {
            View => self.gs.render3(time.hr, time.min, time.sec),
            _ => self.gs.render3(tmp.hr, tmp.min, tmp.sec)
        };
        self.gs.display();
    }
}

impl<'a> TimePanel<'a> {
    fn update_clock(&self, clk: Option<Time>) {
        let mut time = self.time.borrow_mut();
        let gs = self.gs;
        match clk {
            Some(clk) => *time = clk,
            None => time.tick()
        }
        if self.state.get() == TimePanelState::View {
            gs.render3(time.hr, time.min, time.sec);
            gs.display();
        }
    }
}

impl<'a> Panel for DatePanel<'a> {
    fn btn1_short(&self) -> bool {
        use DatePanelState::*;
        {
            let mut tmp = self.tmp.borrow_mut();
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

    fn btn2_short(&self) -> bool {
        use DatePanelState::*;
        let s = match self.state.get() {
            View => {
                let date = self.date.borrow();
                let mut tmp = self.tmp.borrow_mut();
                tmp.yr = date.yr;
                tmp.mon = date.mon;
                tmp.day = date.day;
                EditYr
            },
            EditYr => EditMon,
            EditMon => EditDay,
            EditDay => {
                let tmp = self.tmp.borrow();
                ds3231::DS3231(self.gs.i2c.as_ref().unwrap())
                                .write_date(tmp.yr, tmp.mon, tmp.day);
                *self.date.borrow_mut() = *tmp;
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
        self.gs.update_blinky(match s{
            EditYr => 0b110000,
            EditMon => 0b001100,
            EditDay => 0b000011,
            _ => 0x0,
        });
        let date = self.date.borrow();
        let tmp = self.tmp.borrow();
        match s {
            View => self.gs.render3(date.yr, date.mon, date.day),
            _ => self.gs.render3(tmp.yr, tmp.mon, tmp.day)
        };
        self.gs.display();
    }
}

impl<'a> DatePanel<'a> {
    fn update_clock(&self, d: Option<Date>) {
        let mut date = self.date.borrow_mut();
        match d {
            Some(d) => *date = d,
            None => ()
        }
        if self.state.get() == DatePanelState::View {
            self.gs.render3(date.yr, date.mon, date.day);
            self.gs.display();
        }
    }
}

impl<'a> Panel for TempPanel<'a> {
    fn btn1_short(&self) -> bool {
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
        *self.gs.buff.borrow_mut() = buff;
        self.gs.display();
    }
}

impl<'a> TempPanel<'a> {
    fn update_clock(&self, temp: Option<ds3231::Temp>) {
        match temp {
            Some(temp) => self.temp.set(temp),
            None => ()
        }
        if self.state.get() == TempPanelState::View {
            self.update_output();
        }
    }
}

impl<'a> Panel for CountdownPanel<'a> {
    fn btn1_short(&self) -> bool {
        use CountdownPanelState::*;
        let tim = self.gs.perip.as_ref().unwrap().TIM3;
        let timer = tim::Timer(tim);
        {
            let didx = self.didx.get();
            let mut presets = self.presets.borrow_mut();
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
                Edit3 => inc_rotate(&mut p[5], 0, 10),
                Edit2 => inc_rotate(&mut p[4], 0, 10),
                Edit1 => inc_rotate(&mut p[3], 0, 10),
                Edit0 => inc_rotate(&mut p[2], 0, 10),
                Edit2m => inc_rotate(&mut p[1], 0, 10),
                Edit1m => inc_rotate(&mut p[0], 0, 10),
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

    fn btn2_long(&self) -> bool {
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

    fn btn2_short(&self) -> bool {
        use CountdownPanelState::*;
        self.state.set(match self.state.get() {
            View => EditWhole,
            EditWhole => Edit3,
            Edit3 => Edit2,
            Edit2 => Edit1,
            Edit1 => Edit0,
            Edit0 => Edit2m,
            Edit2m => Edit1m,
            Edit1m => {
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
        self.gs.update_blinky(match s {
            EditWhole => 0b111111,
            Edit3 => 0b100000,
            Edit2 => 0b010000,
            Edit1 => 0b001000,
            Edit0 => 0b000100,
            Edit2m => 0b000010,
            Edit1m => 0b000001,
            TimeUp => 0b111111,
            _ => 0x0
        });
        match s {
            OnGoing | OnGoingPaused => self.gs.render1(self.counter.get()),
            _ => {
                let preset = &self.presets.borrow()[self.didx.get() as usize];
                self.gs.render(preset);
            }
        }
        self.gs.display();
    }
}

impl<'a> CountdownPanel<'a> {
    fn go(&self) {
        let tim = self.gs.perip.as_ref().unwrap().TIM3;
        let timer = tim::Timer(tim);
        let p = &self.presets.borrow()[self.didx.get() as usize];
        let mut x: u32 = 0;
        for v in p.iter().rev() {
            x *= 10;
            x += *v as u32;
        }
        self.counter.set(x);
        timer.reset();
        timer.go();
    }

    fn update_clock(&self) {
        if self.state.get() == CountdownPanelState::Inactive {
            return
        }
        let x = countdown(self.counter.get());
        if x == 0 {
            let tim = self.gs.perip.as_ref().unwrap().TIM3;
            let timer = tim::Timer(tim);
            timer.stop();
            self.state.set(CountdownPanelState::TimeUp);
        } else {
            self.counter.set(x);
        }
        self.update_output();
    }
}

impl<'a> Panel for CountupPanel<'a> {
    fn btn1_short(&self) -> bool {
        use CountupPanelState::*;
        let tim = self.gs.perip.as_ref().unwrap().TIM3;
        let timer = tim::Timer(tim);
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

    fn btn2_short(&self) -> bool {
        use CountupPanelState::*;
        let tim = self.gs.perip.as_ref().unwrap().TIM3;
        let timer = tim::Timer(tim);
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
        self.gs.update_blinky(match self.state.get() {
            View => 0b111111,
            _ => 0x0
        });
        self.gs.render1(self.counter.get());
        self.gs.display();
    }
}

impl<'a> CountupPanel<'a> {
    fn update_clock(&self) {
        if self.state.get() == CountupPanelState::Inactive {
            return
        }
        let x = self.counter.get() + 1;
        self.counter.set(if x > 999999 {0} else {x});
        self.update_output();
    }
}

impl<'a> AlarmEventManager<'a> {
    fn new() -> Self {
        unsafe {
            AlarmEventManager{events: UnsafeCell::new([AlarmEvent{cb: core::mem::uninitialized(),
                                                                  cnt: 0,
                                                                  free: true}; 8]) }
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

    fn render(&self, nbuff: &[u8; 6]) {
        self.buff.borrow_mut().copy_from_slice(nbuff);
    }
    fn render1(&self, mut n: u32) {
        let mut buff = self.buff.borrow_mut();
        for i in 0..buff.len() {
            buff[i] = (n % 10) as u8;
            n /= 10;
        }
    }

    fn render3(&self, n1: u8, n2: u8, n3: u8) {
        let mut buff = self.buff.borrow_mut();
        buff[1] = n3 / 10; buff[0] = n3 - buff[1] * 10;
        buff[3] = n2 / 10; buff[2] = n2 - buff[3] * 10;
        buff[5] = n1 / 10; buff[4] = n1 - buff[5] * 10;
    }

    fn display(&self) {
        let mut buff = *self.buff.borrow();
        let b = self.blink_state.get();
        let bs = self.blinky.borrow();
        match self.disp_state.get() {
            DispState::On | DispState::TempOn =>
                for (i, v) in buff.iter_mut().enumerate() {
                    if b && bs[i] { *v = 0xf; }
                },
            DispState::Off => for i in buff.iter_mut() { *i = 0xf; }
        }
        self.disp.as_ref().unwrap().output_bits(digits2bcds(&buff[..]));
    }

    fn update_blinky(&'a self, nb: u8) {
        let flag = nb == 0; /* if nothing is blinking */
        let en = self.blinky_enabled.get();
        for (i, v) in self.blinky.borrow_mut().iter_mut().enumerate() {
            *v = ((nb >> i) & 1) == 1;
        }
        let ev = self.events.as_ref().unwrap();
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
        let mut buff = *self.buff.borrow();
        let mut i = 0;
        let mut carry = 1;
        while carry > 0 && i < buff.len() {
            buff[i] += carry;
            carry = if buff[i] > 9 {buff[i] = 0; 1} else {0};
            i += 1;
        }
    }

 
    fn update_clock(&self) {
        let mut clk = None;
        let mut d = None;
        let mut temp = None;
        if self.sync_cnt.get() == 0 {
            let rtc = ds3231::DS3231(self.i2c.as_ref().unwrap());
            let ds3231::Date{second: sec,
                            minute: min,
                            hour: hr,
                            date: day,
                            month: mon,
                            year: yr, ..} = rtc.read_fulldate();
            self.sync_cnt.set(SYNC_PERIOD);
            clk = Some(Time{sec, min, hr});
            d = Some(Date{yr, mon, day});
            temp = Some(rtc.read_temperature());
        } else {
            self.sync_cnt.set(self.sync_cnt.get() - 1);
        }
        let tcnt = countdown(self.tempon_cnt.get() as u32) as u16;
        match self.disp_state.get() {
            DispState::Off => {
                if tcnt == 0 {
                    self.disp_state.set(DispState::TempOn);
                    self.tempon_cnt.set(self.tempon_peroid.get());
                } else {
                    self.tempon_cnt.set(tcnt);
                }
            },
            DispState::TempOn => {
                if tcnt == 0 {
                    self.disp_state.set(DispState::Off);
                    self.tempon_cnt.set(self.tempon_cycle.get());
                } else {
                    self.tempon_cnt.set(tcnt);
                }
            },
            _ => ()
        }
        unsafe {
            TIME_PANEL.update_clock(clk);
            DATE_PANEL.update_clock(d);
            TEMP_PANEL.update_clock(temp);
        }
    }

    #[inline(always)]
    fn tim4_callback() {
        let gs = get_gs();
        {
            let p = gs.perip.as_ref().unwrap();
            p.TIM4.sr.modify(|_, w| w.uif().clear());
        }
        match gs.events.as_ref() {
            Some(ev) => ev.tick(),
            None => ()
        }
    }

    #[inline(always)]
    fn tim3_callback() {
        let gs = get_gs();
        {
            let p = gs.perip.as_ref().unwrap();
            p.TIM3.sr.modify(|_, w| w.uif().clear());
        }
        unsafe {
            CD_PANEL.update_clock();
            CU_PANEL.update_clock();
        }
    }
}

impl<'a> Timeoutable<'a> for GlobalState<'a> {
    fn timeout(&'a self) {
        let ev = self.events.as_ref().unwrap();
        self.blink_state.set(!self.blink_state.get());
        self.display();
        if self.blinky_enabled.get().is_some() {
            self.blinky_enabled.set(Some(ev.add(self, BLINK_PERIOD)));
        }
    }
}

#[inline(always)]
fn get_gs() -> &'static mut GlobalState<'static> {
    unsafe {&mut GS}
}

fn systick_handler() {
    // digits_countup();
    let gs = get_gs();
    if !gs.i2c_inited {return}
    gs.update_clock();
}

fn exti3_handler() {
    let gs = get_gs();
    let btn1 = gs.btn1.as_ref().unwrap();
    let p = gs.perip.as_ref().unwrap();
    p.EXTI.pr.write(|w| w.pr3().set_bit());
    let x = p.GPIOA.idr.read().idr3().bit();
    if !x {
        btn1.press();
    } else {
        let gs = get_gs();
        match btn1.release() {
            ButtonResult::FalseAlarm => (),
            ButtonResult::ShortPress => {
                match gs.disp_state.get() {
                    DispState::Off => {
                        gs.disp_state.set(DispState::On);
                        gs.display();
                        return;
                    },
                    DispState::TempOn => gs.disp_state.set(DispState::On),
                    _ => ()
                }
                if !gs.panels[gs.pidx].btn1_short() {
                    gs.pidx += 1;
                    if gs.pidx == gs.panels.len() {
                        gs.pidx = 0;
                    }
                    gs.panels[gs.pidx].btn1_short();
                }
            },
            ButtonResult::LongPress => {
                gs.disp_state.set(DispState::Off);
                gs.tempon_cnt.set(gs.tempon_cycle.get());
            }
        }
        gs.display();
    }
}

fn exti4_handler() {
    let gs = get_gs();
    let btn2 = gs.btn2.as_ref().unwrap();
    let p = gs.perip.as_ref().unwrap();
    p.EXTI.pr.write(|w| w.pr4().set_bit());
    let x = p.GPIOA.idr.read().idr4().bit();
    if !x {
        btn2.press();
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

exception!(SYS_TICK, systick_handler);
interrupt!(EXTI4, exti4_handler);
interrupt!(EXTI3, exti3_handler);
interrupt!(TIM4, GlobalState::tim4_callback);
interrupt!(TIM3, GlobalState::tim3_callback);

const SYNC_PERIOD: u8 = 10;
const BLINK_PERIOD: u32 = 500;

static mut TIME_PANEL: TimePanel = TimePanel{state: Cell::new(TimePanelState::View),
                                        tmp: RefCell::new(Time{sec: 0, min: 0, hr: 0}),
                                        time: RefCell::new(Time{sec: 0, min: 0, hr: 0}),
                                        gs: unsafe{&GS}};
static mut DATE_PANEL: DatePanel = DatePanel{state: Cell::new(DatePanelState::Inactive),
                                        tmp: RefCell::new(Date{yr: 0, mon: 1, day: 1}),
                                        date: RefCell::new(Date{yr: 0, mon: 1, day: 1}),
                                        gs: unsafe{&GS}};
static mut TEMP_PANEL: TempPanel = TempPanel{state: Cell::new(TempPanelState::Inactive),
                                        temp: Cell::new(ds3231::Temp{cels: 0, quarter: 0}),
                                        gs: unsafe{&GS}};

static mut CD_PANEL: CountdownPanel = CountdownPanel{state: Cell::new(CountdownPanelState::Inactive),
                                        presets: RefCell::new([[0; 6]; 2]),
                                        counter: Cell::new(0),
                                        didx: Cell::new(0),
                                        gs: unsafe{&GS}};
static mut CU_PANEL: CountupPanel = CountupPanel{state: Cell::new(CountupPanelState::Inactive),
                                        counter: Cell::new(0),
                                        gs: unsafe{&GS}};

static mut GS: GlobalState =
    GlobalState{perip: None,
                disp: None,
                btn1: None,
                btn2: None,
                events: None,
                i2c: None,
                i2c_inited: false,
                sync_cnt: Cell::new(0),
                buff: RefCell::new([0; 6]),
                disp_state: Cell::new(DispState::On),
                tempon_cycle: Cell::new(20),
                tempon_peroid: Cell::new(5),
                tempon_cnt: Cell::new(0),
                blinky: RefCell::new([false; 6]),
                blink_state: Cell::new(false),
                blinky_enabled: Cell::new(None),
                pidx: 0,
                panels: unsafe {[&TIME_PANEL, &DATE_PANEL, &TEMP_PANEL, &CD_PANEL, &CU_PANEL]}
    };

fn init() {
    let gs = get_gs();
    let p = {
        gs.perip = Some(unsafe {Peripherals::all()});
        gs.perip.as_ref().unwrap()
    };

    p.SYST.set_clock_source(cortex_m::peripheral::SystClkSource::Core);
    p.SYST.set_reload(8_000_000);
    p.SYST.enable_interrupt();
    p.SYST.enable_counter();

    /* enable GPIOA, GPIOB and AFIO */
    p.RCC.apb2enr.modify(|_, w| w.iopaen().enabled()
                                .iopben().enabled()
                                .afioen().enabled());

    p.RCC.apb1enr.modify(|_, w| w.tim4en().enabled()
                                 .tim3en().enabled());

    /* GPIO */
    /* enable PA0-2 for manipulating shift register */
    p.GPIOA.odr.modify(|_, w| w.odr3().set_bit()
                               .odr4().set_bit());
    p.GPIOA.crl.modify(|_, w|
        w.mode0().output().cnf0().push()
         .mode1().output().cnf1().push()
         .mode2().output().cnf2().push()
         .mode3().input().cnf3().bits(0b10)
         .mode4().input().cnf4().bits(0b10));

    /* enable PB6 and PB7 for I2C1 */
    p.GPIOB.crl.modify(|_, w|
        w.mode6().output50().cnf6().alt_open()
         .mode7().output50().cnf7().alt_open());

    /* I2C */
    /* enable and reset I2C1 */
    p.RCC.apb1enr.modify(|_, w| w.i2c1en().enabled());
    p.RCC.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
    p.RCC.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());

    /* NVIC & EXTI */
    p.AFIO.exticr1.write(|w| unsafe { w.exti3().bits(0b0000) });
    p.AFIO.exticr2.write(|w| unsafe { w.exti4().bits(0b0000) });
    p.NVIC.enable(Interrupt::EXTI3);
    p.NVIC.enable(Interrupt::EXTI4);
    p.NVIC.enable(Interrupt::TIM3);
    p.NVIC.enable(Interrupt::TIM4);
    p.EXTI.imr.write(|w| w.mr3().set_bit()
                          .mr4().set_bit());
    p.EXTI.rtsr.write(|w| w.tr3().set_bit()
                           .tr4().set_bit());
    p.EXTI.ftsr.write(|w| w.tr3().set_bit()
                           .tr4().set_bit());


    gs.i2c = Some(i2c::I2C(p.I2C1));
    gs.disp = Some(ShiftRegister::new(p.GPIOA, 24));
    gs.i2c.as_ref().unwrap().init(p.RCC, 0x01, 400_000, i2c::DutyType::DUTY1, true);
    //i2c.init(0x01, 100_000, i2c::DutyType::DUTY1, false);
    gs.disp.as_ref().unwrap().output_bits(0);
    gs.i2c_inited = true;

    /* 10ms couting clock */
    tim::Timer(p.TIM3).init(10 * (8_000_000 / 1000));
    /* 1ms-precision event clock */
    let tim4 = tim::Timer(p.TIM4);
    tim4.init(1 * (8_000_000 / 1000));
    gs.update_clock(); /* initialize internal time */
    gs.events = Some(AlarmEventManager::new());
    let ev = gs.events.as_ref().unwrap();
    gs.btn1 = Some(Button::new(ev));
    gs.btn2 = Some(Button::new(ev));
    tim4.reset();
    tim4.go();
}

fn main() {
    init();
}
