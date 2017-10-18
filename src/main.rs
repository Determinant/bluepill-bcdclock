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

const RESET_PERIOD: u8 = 10;
static mut SR: Option<ShiftRegister> = None;
static mut I2C: Option<i2c::I2C> = None;
static mut RTC: Option<ds3231::DS3231> = None;
static mut ROM: Option<at24c::AT24C> = None;
static mut DIGITS: [u8; 6] = [0; 6];
static mut TIME: Clock = Clock{sec: 0, min: 0, hr: 0, reset: 0};
static mut PERIP: Option<Peripherals> = None;

fn digits2bcds(digs: &[u8]) -> u32 {
    let mut res: u32 = 0;
    for d in digs.iter().rev() {
        res = (res << 4) | (*d as u32);
    }
    res
}

fn digits_countup() {
    unsafe {
        SR.as_ref().unwrap().output_bits(digits2bcds(&DIGITS[..]));
        let mut i = 0;
        let mut carry = 1;
        while carry > 0 && i < DIGITS.len() {
            DIGITS[i] += carry;
            carry = if DIGITS[i] > 9 {DIGITS[i] = 0; 1} else {0};
            i += 1;
        }
    }
}

fn refresh_clock() {
    unsafe {
        SR.as_ref().unwrap().output_bits(digits2bcds(&DIGITS[..]));
    }
}

fn render_clock() {
    unsafe {
        if bs {
            DIGITS[1] = TIME.sec / 10; DIGITS[0] = TIME.sec - DIGITS[1] * 10;
            DIGITS[3] = TIME.min / 10; DIGITS[2] = TIME.min - DIGITS[3] * 10;
            DIGITS[5] = TIME.hr / 10; DIGITS[4] = TIME.hr - DIGITS[5] * 10;
        } else {
            for i in &mut DIGITS {
                *i = 0xf;
            }
        }
    }
}

fn update_clock() {
    unsafe {
        if RTC.is_none() {return}
        if !TIME.tick() {
            let ds3231::Date{second: sec,
                            minute: min,
                            hour: hr, ..} = RTC.as_ref().unwrap()
                                                .read_fulldate();
            TIME = Clock{sec, min, hr,
                        reset: RESET_PERIOD};
        }
    }
    render_clock();
    refresh_clock();
}

fn systick_handler() {
    // digits_countup();
    update_clock();
}

static mut bs: bool = false;

fn exti3_handler() {
    let p = unsafe {PERIP.as_ref().unwrap()};
    p.EXTI.pr.write(|w| w.pr3().set_bit());
    let x = p.GPIOA.idr.read().idr3().bit();
    unsafe {
        if !x && !bs {
            bs = true;
        } else if x && bs {
            bs = false;
        }
    }
    render_clock();
    refresh_clock();
}

exception!(SYS_TICK, systick_handler);
interrupt!(EXTI3, exti3_handler);

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
    let p = unsafe {
        PERIP = Some(Peripherals::all());
        PERIP.as_ref().unwrap()
    };

    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(8_000_000);
    p.SYST.enable_interrupt();
    p.SYST.enable_counter();

    /* enable GPIOA, GPIOB and AFIO */
    p.RCC.apb2enr.modify(|_, w| w.iopaen().enabled()
                                .iopben().enabled()
                                .afioen().enabled());

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
    p.EXTI.imr.write(|w| w.mr3().set_bit());
    p.EXTI.rtsr.write(|w| w.tr3().set_bit());
    p.EXTI.ftsr.write(|w| w.tr3().set_bit());


    unsafe {
        I2C = Some(i2c::I2C::new(p.I2C1));
        let i2c = I2C.as_ref().unwrap();
        RTC = Some(ds3231::DS3231::new(i2c));
        ROM = Some(at24c::AT24C::new(i2c));
        SR = Some(ShiftRegister::new(p.GPIOA, 24));

        i2c.init(p.RCC, 0x01, 400_000, i2c::DutyType::DUTY1, true);
        //i2c.init(0x01, 100_000, i2c::DutyType::DUTY1, false);
        SR.as_ref().unwrap().output_bits(0);
    }
}

fn set_clock() {
    let rtc = unsafe {RTC.as_ref().unwrap()};
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
