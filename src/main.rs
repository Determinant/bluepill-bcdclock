#![no_std]
#![feature(asm)]

#[macro_use] extern crate stm32f103xx;
extern crate cortex_m;

use stm32f103xx::{GPIOA, GPIOB, RCC, SYST, I2C1, EXTI, NVIC, Interrupt, AFIO, Peripherals};
use stm32f103xx::{gpioa};
use cortex_m::peripheral::SystClkSource;

mod mutex;
mod i2c;
mod ds3231;
mod at24c;
mod tim;

const SYNC_PERIOD: u8 = 10;

struct ShiftRegister<'a> {
    gpioa: &'a gpioa::RegisterBlock,
    width: u8,
}

struct Clock {
    sec: u8,
    min: u8,
    hr: u8,
    reset: u8
}

struct GlobalState {
    disp: Option<ShiftRegister<'static>>,
    i2c: Option<i2c::I2C<'static>>,
    tim: Option<tim::Timer<'static>>,
    i2c_inited: bool,
    buff: [u8; 6],
    time: Clock,
    perip: Option<Peripherals<'static>>,
    bs: bool
}

static mut GS: GlobalState =
    GlobalState{disp: None,
                i2c: None,
                tim: None,
                i2c_inited: false,
                buff: [0; 6],
                time: Clock{sec: 0, min: 0, hr: 0, reset: 0},
                perip: None,
                bs: false};

fn get_gs() -> &'static mut GlobalState {
    unsafe {&mut GS}
}

fn digits2bcds(digs: &[u8]) -> u32 {
    let mut res: u32 = 0;
    for d in digs.iter().rev() {
        res = (res << 4) | (*d as u32);
    }
    res
}

fn display() {
    let gs = get_gs();
    gs.disp.as_ref().unwrap().output_bits(digits2bcds(&gs.buff[..]));
}

fn digits_countup() {
    let gs = get_gs();
    let mut digits = &mut gs.buff;
    gs.disp.as_ref().unwrap().output_bits(digits2bcds(&digits[..]));
    let mut i = 0;
    let mut carry = 1;
    while carry > 0 && i < digits.len() {
        digits[i] += carry;
        carry = if digits[i] > 9 {digits[i] = 0; 1} else {0};
        i += 1;
    }
}

fn render_clock() {
    let gs = get_gs();
    let mut digits = &mut gs.buff;
    let time = &gs.time;
    if gs.bs {
        digits[1] = time.sec / 10; digits[0] = time.sec - digits[1] * 10;
        digits[3] = time.min / 10; digits[2] = time.min - digits[3] * 10;
        digits[5] = time.hr / 10; digits[4] = time.hr - digits[5] * 10;
    } else {
        for i in digits {
            *i = 0xf;
        }
    }
}

fn update_clock() {
    let gs = get_gs();
    let p = gs.perip.as_ref().unwrap();
    let rtc = ds3231::DS3231(gs.i2c.as_ref().unwrap());
    if !gs.i2c_inited {return}
    if !gs.time.tick() {
        let ds3231::Date{second: sec,
                        minute: min,
                        hour: hr, ..} = rtc.read_fulldate();
        gs.time = Clock{sec, min, hr,
                        reset: SYNC_PERIOD};
    }
    render_clock();
    display();
}

fn systick_handler() {
    // digits_countup();
    update_clock();
}

fn exti3_handler() {
    let gs = get_gs();
    let p = gs.perip.as_ref().unwrap();
    p.EXTI.pr.write(|w| w.pr3().set_bit());
    let x = p.GPIOA.idr.read().idr3().bit();
    if !x && !gs.bs {
        gs.bs = true;
    } else if x && gs.bs {
        gs.bs = false;
    }
    render_clock();
    display();
}

fn tim2_handler() {
    let gs = get_gs();
    let p = gs.perip.as_ref().unwrap();
    p.TIM2.sr.modify(|_, w| w.uif().clear());
    gs.bs = !gs.bs;
    render_clock();
    display();
}

exception!(SYS_TICK, systick_handler);
interrupt!(EXTI3, exti3_handler);
interrupt!(TIM2, tim2_handler);

impl<'a> ShiftRegister<'a> {
    fn new(gpioa: &'a gpioa::RegisterBlock,
           width: u8) -> ShiftRegister<'a> {
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

impl Clock {
    fn tick(&mut self) -> bool {
        if self.reset == 0 {
            return false;
        }

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
        self.reset -= 1;
        true
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
    p.RCC.apb1enr.modify(|_, w| w.tim2en().enabled());

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
    p.EXTI.imr.write(|w| w.mr3().set_bit());
    p.EXTI.rtsr.write(|w| w.tr3().set_bit());
    p.EXTI.ftsr.write(|w| w.tr3().set_bit());


    gs.i2c = Some(i2c::I2C(p.I2C1));
    gs.disp = Some(ShiftRegister::new(p.GPIOA, 24));
    gs.tim = Some(tim::Timer(p.TIM2));
    gs.i2c.as_ref().unwrap().init(p.RCC, 0x01, 400_000, i2c::DutyType::DUTY1, true);
    gs.tim.as_ref().unwrap().init(16_000_000);
    gs.tim.as_ref().unwrap().go();
    //i2c.init(0x01, 100_000, i2c::DutyType::DUTY1, false);
    gs.disp.as_ref().unwrap().output_bits(0);
    gs.i2c_inited = true;
}

fn set_clock() {
    let gs = get_gs();
    let p = gs.perip.as_ref().unwrap();
    let rtc = ds3231::DS3231(gs.i2c.as_ref().unwrap());
    rtc.write_fulldate(&ds3231::Date{second: 30,
                            minute: 23,
                            hour: 18,
                            day: 2,
                            date: 10,
                            month: 10,
                            year: 17,
                            am: false,
                            am_enabled: false});
    /*
    let rom = ROM.as_ref().unwrap();
    let mut buf: [u8; 64] = [23; 64];
    rom.write(23, 64, &buf);
    let mut buf2: [u8; 80] = [0; 80];
    rom.read(20, 80, &mut buf2);
    */
}

fn main() {
    init();
    //set_clock();
    /*
    let x = mutex::Mutex::new(42);
    {
        let y = x.lock();
        let z = *y + 1;
        let w = z;
    }
    */
    update_clock();
}
