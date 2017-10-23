#![no_std]
#![feature(asm)]
#![feature(const_fn)]

#[macro_use] extern crate stm32f103xx;
extern crate cortex_m;

use stm32f103xx::{GPIOA, GPIOB, RCC, SYST, I2C1, EXTI, NVIC, Interrupt, AFIO, Peripherals};
use stm32f103xx::{gpioa};
use cortex_m::peripheral::SystClkSource;
use core::cell::{Cell, RefCell};

mod mutex;
mod i2c;
mod ds3231;
mod at24c;
mod tim;

const SYNC_PERIOD: u8 = 10;
const BLINK_PERIOD: u32 = 500;

fn digits2bcds(digs: &[u8]) -> u32 {
    let mut res: u32 = 0;
    for d in digs.iter().rev() {
        res = (res << 4) | (*d as u32);
    }
    res
}

struct ShiftRegister<'a> {
    gpioa: &'a gpioa::RegisterBlock,
    width: u8,
}

#[derive(Copy, Clone)]
struct Time {
    sec: u8,
    min: u8,
    hr: u8,
}

struct GlobalState {
    disp: Option<ShiftRegister<'static>>,
    i2c: Option<i2c::I2C<'static>>,
    btn1: Option<Button<'static>>,
    //tim: Option<tim::Timer<'static>>,
    i2c_inited: bool,
    sync_cnt: u8,
    buff: RefCell<[u8; 6]>,
    blinky: RefCell<[bool; 6]>,
    blink_state: Cell<bool>,
    perip: Option<Peripherals<'static>>,
    disp_on: bool,
    pidx: usize,
    panels: [&'static Panel; 3],
}

struct Button<'a> {
    state: Cell<bool>,
    long: Cell<bool>,
    timer: tim::Timer<'a>
}

enum ButtonResult {
    FalseAlarm,
    ShortPress,
    LongPress
}

impl<'a> Button<'a> {
    fn new(timer: tim::Timer<'a>, thres: u32) -> Self {
        /* in milliseconds */
        timer.init(thres * (8_000_000 / 1000));
        Button {state: Cell::new(false),
                long: Cell::new(false),
                timer}
    }

    fn press(&self) {
        if !self.state.get() {
            self.state.set(true);
            self.long.set(false);
            self.timer.reset();
            self.timer.go();
        }
    }

    fn release(&self) -> ButtonResult {
        if self.state.get() {
            self.timer.stop();
            self.state.set(false);
            if self.long.get() { ButtonResult::LongPress }
            else { ButtonResult::ShortPress }
        } else { ButtonResult::FalseAlarm }
    }

    fn timeout(&self) {
        self.timer.stop();
        self.long.set(true);
    }
}

trait Panel {
    fn btn1_short(&self) -> bool {false}
    fn btn1_long(&self) -> bool {false}
    fn btn2_short(&self) -> bool {false}
    fn btn2_long(&self) -> bool {false}
    fn update_output(&self);
}

#[derive(PartialEq, Clone, Copy)]
enum TimePanelState {
    INACTIVE,
    VIEW,
    EditHr,
    EditMin,
    EditSec,
}

struct TimePanel<'a> {
    gs: &'a GlobalState,
    state: Cell<TimePanelState>, 
    time: RefCell<Time>,
    tmp: RefCell<Time>
}

impl<'a> Panel for TimePanel<'a> {
    fn btn1_short(&self) -> bool {
        use TimePanelState::*;
        {
            let mut tmp = self.tmp.borrow_mut();
            match self.state.get() {
                VIEW => {
                    self.state.set(INACTIVE);
                    return false;
                }, /* yield to the next panel */
                EditHr => {
                    tmp.hr += 1;
                    if tmp.hr == 24 { tmp.hr = 0 };
                },
                EditMin => {
                    tmp.min += 1;
                    if tmp.min == 60 { tmp.min = 0 };
                },
                EditSec => {
                    tmp.sec += 1;
                    if tmp.sec == 60 { tmp.sec = 0 };
                },
                INACTIVE => {
                    self.state.set(VIEW);
                }
            };
        }
        self.update_output();
        true
    }

    fn btn2_short(&self) -> bool {
        use TimePanelState::*;
        let s = match self.state.get() {
            VIEW => {
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
                                .write_time(&ds3231::Date{second: tmp.sec,
                                                         minute: tmp.min,
                                                         hour: tmp.hr,
                                                         day: 0,
                                                         date: 0,
                                                         month: 0,
                                                         year: 0,
                                                         am: false,
                                                         am_enabled: false});
                *self.time.borrow_mut() = *tmp;
                VIEW
            },
            s => s
        };
        self.state.set(s);
        self.update_output();
        self.state.get() == VIEW
    }

    fn update_output(&self) {
        use TimePanelState::*;
        let s = self.state.get();
        self.gs.update_blinky(match s {
            EditHr => [false, false, false, false, true, true],
            EditMin => [false, false, true, true, false, false],
            EditSec => [true, true, false, false, false, false],
            _ => [false; 6],
        });
        let time = self.time.borrow();
        let tmp = self.tmp.borrow();
        match s {
            VIEW => self.gs.render3(time.hr, time.min, time.sec),
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
        if self.state.get() == TimePanelState::VIEW {
            gs.render3(time.hr, time.min, time.sec);
            gs.display();
        }
    }
}

#[derive(Clone, Copy)]
struct Date {
    yr: u8,
    mon: u8,
    day: u8
}

#[derive(PartialEq, Clone, Copy)]
enum DatePanelState {
    INACTIVE,
    VIEW,
    EditYr,
    EditMon,
    EditDay
}

struct DatePanel<'a> {
    gs: &'a GlobalState,
    state: Cell<DatePanelState>, 
    date: RefCell<Date>,
    tmp: RefCell<Date>
}

impl<'a> Panel for DatePanel<'a> {
    fn btn1_short(&self) -> bool {
        use DatePanelState::*;
        {
            let mut tmp = self.tmp.borrow_mut();
            match self.state.get() {
                VIEW => {
                    self.state.set(INACTIVE);
                    return false;
                }, /* yield to the next panel */
                EditYr => {
                    if tmp.yr == 255 { tmp.yr = 0 }
                    else { tmp.yr += 1 };
                },
                EditMon => {
                    tmp.mon += 1;
                    if tmp.mon == 13 { tmp.mon = 1 };
                },
                EditDay => {
                    tmp.day += 1;
                    if tmp.day == 32 { tmp.day = 1 };
                },
                INACTIVE => {
                    self.state.set(VIEW);
                }
            };
        }
        self.update_output();
        true
    }

    fn btn2_short(&self) -> bool {
        use DatePanelState::*;
        let s = match self.state.get() {
            VIEW => {
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
                                .write_date(&ds3231::Date{second: 0,
                                                         minute: 0,
                                                         hour: 0,
                                                         day: 0,
                                                         date: tmp.day,
                                                         month: tmp.mon,
                                                         year: tmp.yr,
                                                         am: false,
                                                         am_enabled: false});
                *self.date.borrow_mut() = *tmp;
                VIEW
            },
            s => s
        };
        self.state.set(s);
        self.update_output();
        self.state.get() == VIEW
    }

    fn update_output(&self) {
        use DatePanelState::*;
        let s = self.state.get();
        self.gs.update_blinky(match s{
            EditYr => [false, false, false, false, true, true],
            EditMon => [false, false, true, true, false, false],
            EditDay => [true, true, false, false, false, false],
            _ => [false; 6],
        });
        let date = self.date.borrow();
        let tmp = self.tmp.borrow();
        match s {
            VIEW => self.gs.render3(date.yr, date.mon, date.day),
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
        if self.state.get() == DatePanelState::VIEW {
            self.gs.render3(date.yr, date.mon, date.day);
            self.gs.display();
        }
    }
}

#[derive(PartialEq, Copy, Clone)]
enum TempPanelState {
    INACTIVE,
    VIEW
}

struct TempPanel<'a> {
    state: Cell<TempPanelState>,
    temp: Cell<ds3231::Temp>,
    gs: &'a GlobalState,
}

impl<'a> Panel for TempPanel<'a> {
    fn btn1_short(&self) -> bool {
        use TempPanelState::*;
        match self.state.get() {
            VIEW => {
                self.state.set(INACTIVE);
                return false;
            },
            INACTIVE =>  {
                self.state.set(VIEW);
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
        if self.state.get() == TempPanelState::VIEW {
            self.update_output();
        }
    }
}

static mut TPANEL: TimePanel = TimePanel{state: Cell::new(TimePanelState::VIEW),
                                        tmp: RefCell::new(Time{sec: 0, min: 0, hr: 0}),
                                        time: RefCell::new(Time{sec: 0, min: 0, hr: 0}),
                                        gs: unsafe{&GS}};
static mut DPANEL: DatePanel = DatePanel{state: Cell::new(DatePanelState::INACTIVE),
                                        tmp: RefCell::new(Date{yr: 0, mon: 1, day: 1}),
                                        date: RefCell::new(Date{yr: 0, mon: 1, day: 1}),
                                        gs: unsafe{&GS}};
static mut TEMP_PANEL: TempPanel = TempPanel{state: Cell::new(TempPanelState::INACTIVE),
                                        temp: Cell::new(ds3231::Temp{cels: 0, quarter: 0}),
                                        gs: unsafe{&GS}};

static mut GS: GlobalState =
    GlobalState{disp: None,
                i2c: None,
                sync_cnt: 0,
                btn1: None,
                i2c_inited: false,
                buff: RefCell::new([0; 6]),
                perip: None,
                disp_on: true,
                blinky: RefCell::new([false; 6]),
                blink_state: Cell::new(false),
                pidx: 0,
                panels: unsafe {[&TPANEL, &DPANEL, &TEMP_PANEL]}
    };

fn get_gs() -> &'static mut GlobalState {
    unsafe {&mut GS}
}

impl GlobalState {

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
        if self.disp_on {
            for (i, v) in buff.iter_mut().enumerate() {
                if b && bs[i] { *v = 0xf; }
            }
        } else {
            for i in buff.iter_mut() { *i = 0xf; }
        }
        self.disp.as_ref().unwrap().output_bits(digits2bcds(&buff[..]));
    }

    fn update_blinky(&self, ns: [bool; 6]) {
        let tim4 = self.perip.as_ref().unwrap().TIM4;
        let timer = tim::Timer(tim4);
        let en = timer.is_enabled();
        let flag = ns.iter().all(|x| !x); /* if nothing is blinking */
        *self.blinky.borrow_mut() = ns;
        if en && flag {
            self.blink_state.set(false);
            timer.stop();
        } else if !en && !flag {
            self.blink_state.set(false);
            timer.reset();
            timer.go();
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

 
    fn update_clock(&mut self) {
        let mut clk = None;
        let mut d = None;
        let mut temp = None;
        if self.sync_cnt == 0 {
            let rtc = ds3231::DS3231(self.i2c.as_ref().unwrap());
            let ds3231::Date{second: sec,
                            minute: min,
                            hour: hr,
                            date: day,
                            month: mon,
                            year: yr, ..} = rtc.read_fulldate();
            self.sync_cnt = SYNC_PERIOD;
            clk = Some(Time{sec, min, hr});
            d = Some(Date{yr, mon, day});
            temp = Some(rtc.read_temperature());
        } else {
            self.sync_cnt -= 1;
        }
        unsafe {
            TPANEL.update_clock(clk);
            DPANEL.update_clock(d);
            TEMP_PANEL.update_clock(temp);
        }
    }
        /*
        let rtc = ds3231::DS3231(self.i2c.as_ref().unwrap());
        rtc.write_fulldate(&ds3231::Date{second: 30,
                                minute: 23,
                                hour: 18,
                                day: 2,
                                date: 10,
                                month: 10,
                                year: 17,
                                am: false,
                                am_enabled: false});
        */
        /*
        let rom = ROM.as_ref().unwrap();
        let mut buf: [u8; 64] = [23; 64];
        rom.write(23, 64, &buf);
        let mut buf2: [u8; 80] = [0; 80];
        rom.read(20, 80, &mut buf2);
        */
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
                if !gs.panels[gs.pidx].btn1_short() {
                    gs.pidx += 1;
                    if gs.pidx == gs.panels.len() {
                        gs.pidx = 0;
                    }
                    gs.panels[gs.pidx].btn1_short();
                }
            },
            ButtonResult::LongPress => { gs.panels[gs.pidx].btn2_short(); }
        }
        gs.display();
    }
}

fn tim2_handler() {
    let gs = get_gs();
    let p = gs.perip.as_ref().unwrap();
    p.TIM2.sr.modify(|_, w| w.uif().clear());
    gs.btn1.as_ref().unwrap().timeout();
}

fn tim4_handler() {
    let gs = get_gs();
    {
        let p = gs.perip.as_ref().unwrap();
        p.TIM4.sr.modify(|_, w| w.uif().clear());
    }
    gs.blink_state.set(!gs.blink_state.get());
    gs.display();
}

exception!(SYS_TICK, systick_handler);
interrupt!(EXTI3, exti3_handler);
interrupt!(TIM2, tim2_handler);
interrupt!(TIM4, tim4_handler);

impl<'a> ShiftRegister<'a> {
    fn new(gpioa: &'a gpioa::RegisterBlock,
           width: u8) -> Self {
        let this = ShiftRegister{gpioa, width};
        this
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

fn init() {
    let gs = get_gs();
    let p = {
        gs.perip = Some(unsafe {Peripherals::all()});
        gs.perip.as_ref().unwrap()
    };

    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(8_000_000);
    p.SYST.enable_interrupt();
    p.SYST.enable_counter();

    /* enable GPIOA, GPIOB and AFIO */
    p.RCC.apb2enr.modify(|_, w| w.iopaen().enabled()
                                .iopben().enabled()
                                .afioen().enabled());
    p.RCC.apb1enr.modify(|_, w| w.tim2en().enabled()
                                 .tim4en().enabled());

    /* GPIO */
    /* enable PA0-2 for manipulating shift register */
    p.GPIOA.odr.modify(|_, w| w.odr3().set_bit());
    p.GPIOA.crl.modify(|_, w|
        w.mode0().output().cnf0().push()
         .mode1().output().cnf1().push()
         .mode2().output().cnf2().push()
         .mode3().input().cnf3().bits(0b10));

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
    p.NVIC.enable(Interrupt::EXTI3);
    p.NVIC.enable(Interrupt::TIM2);
    p.NVIC.enable(Interrupt::TIM4);
    p.EXTI.imr.write(|w| w.mr3().set_bit());
    p.EXTI.rtsr.write(|w| w.tr3().set_bit());
    p.EXTI.ftsr.write(|w| w.tr3().set_bit());


    gs.i2c = Some(i2c::I2C(p.I2C1));
    gs.disp = Some(ShiftRegister::new(p.GPIOA, 24));
    gs.i2c.as_ref().unwrap().init(p.RCC, 0x01, 400_000, i2c::DutyType::DUTY1, true);
    //i2c.init(0x01, 100_000, i2c::DutyType::DUTY1, false);
    gs.disp.as_ref().unwrap().output_bits(0);
    gs.i2c_inited = true;

    gs.btn1 = Some(Button::new(tim::Timer(p.TIM2), 300));
    tim::Timer(p.TIM4).init(BLINK_PERIOD * (8_000_000 / 1000));
}

fn main() {
    init();
    get_gs().update_clock();
    //set_clock();
    /*
    let x = mutex::Mutex::new(42);
    {
        let y = x.lock();
        let z = *y + 1;
        let w = z;
    }
    */
}
