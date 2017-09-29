#![no_std]

#[macro_use] extern crate stm32f103xx;
extern crate cortex_m;
use stm32f103xx::{GPIOA, GPIOB, RCC, SYST, I2C1};
use cortex_m::peripheral::SystClkSource;
mod i2c;
mod ds3231;
mod at24c;

struct ShiftRegister<'a> {
    gpioa: &'a stm32f103xx::gpioa::RegisterBlock,
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

fn digits2bcds(digs: &[u8]) -> u32 {
    let mut res: u32 = 0;
    for d in digs.iter().rev() {
        res = (res << 4) | (*d as u32);
    }
    res
}

fn digits_countup() {
    unsafe {
        SR.as_mut().unwrap().output_bits(digits2bcds(&DIGITS[..]));
        let mut i = 0;
        let mut carry = 1;
        while carry > 0 && i < DIGITS.len() {
            DIGITS[i] += carry;
            carry = if DIGITS[i] > 9 {DIGITS[i] = 0; 1} else {0};
            i += 1;
        }
    }
}

fn update_clock() {
    unsafe {
        if RTC.is_none() {return}
        if !TIME.tick() {
            let ds3231::Date{second: sec,
                            minute: min,
                            hour: hr, ..} = RTC.as_mut().unwrap()
                                                .read_fulldate();
            TIME = Clock{sec, min, hr,
                        reset: RESET_PERIOD};
        }

        DIGITS[4] = TIME.sec / 10; DIGITS[5] = TIME.sec - DIGITS[4] * 10;
        DIGITS[2] = TIME.min / 10; DIGITS[3] = TIME.min - DIGITS[2] * 10;
        DIGITS[0] = TIME.hr / 10; DIGITS[1] = TIME.hr - DIGITS[0] * 10;
        SR.as_mut().unwrap().output_bits(digits2bcds(&DIGITS[..]));
    }
}

fn systick_handler() {
    // digits_countup();
    update_clock();
}

exception!(SYS_TICK, systick_handler);

impl<'a> ShiftRegister<'a> {
    fn new(gpioa: &'a stm32f103xx::gpioa::RegisterBlock,
           width: u8) -> ShiftRegister<'a> {
        let this = ShiftRegister{gpioa, width};
        this
    }

    fn output_bits(&mut self, bits: u32) {
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

fn main() {

    let gpioa: &stm32f103xx::gpioa::RegisterBlock = unsafe { &*GPIOA.get() };
    let gpiob: &stm32f103xx::gpioa::RegisterBlock = unsafe { &*GPIOB.get() };
    let rcc: &stm32f103xx::rcc::RegisterBlock = unsafe { &*RCC.get() };
    let i2c: &stm32f103xx::i2c1::RegisterBlock = unsafe { &*I2C1.get() };
    let syst: &cortex_m::peripheral::SYST = unsafe { &*SYST.get() };

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(8_000_000);
    syst.enable_interrupt();
    syst.enable_counter();

    /* enable GPIOA, GPIOB and AFIO */
    rcc.apb2enr.modify(|_, w| w.iopaen().enabled()
                                .iopben().enabled()
                                .afioen().enabled());

    /* GPIO */
    /* enable PA0-2 for manipulating shift register */
    gpioa.crl.modify(|_, w|
        w.mode0().output().cnf0().push()
         .mode1().output().cnf1().push()
         .mode2().output().cnf2().push());

    /* enable PB6 and PB7 for I2C1 */
    gpiob.crl.modify(|_, w|
        w.mode6().output50().cnf6().alt_open()
         .mode7().output50().cnf7().alt_open());

    /* I2C */
    /* enable and reset I2C1 */
    rcc.apb1enr.modify(|_, w| w.i2c1en().enabled());
    rcc.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());

    unsafe {
        I2C = Some(i2c::I2C::new(i2c, rcc));
        let i2c = I2C.as_mut().unwrap();
        let rtc = ds3231::DS3231::new(i2c);
        ROM = Some(at24c::AT24C::new(i2c));
        SR = Some(ShiftRegister::new(gpioa, 24));

        i2c.init(0x01, 400_000, i2c::DutyType::DUTY1, true);
        //i2c.init(0x01, 100_000, i2c::DutyType::DUTY1, false);
        SR.as_mut().unwrap().output_bits(0);

        /* initialize the ds3231 */
        /*
        rtc.write_fulldate(&ds3231::Date{second: 30,
                                minute: 48,
                                hour: 21,
                                day: 4,
                                date: 21,
                                month: 9,
                                year: 17,
                                am: false,
                                am_enabled: false});
        */
        /*
        let rom = ROM.as_mut().unwrap();
        let mut buf: [u8; 64] = [23; 64];
        rom.write(23, 64, &buf);
        let mut buf2: [u8; 80] = [0; 80];
        rom.read(20, 80, &mut buf2);
        */
        RTC = Some(rtc);
    }

    update_clock();
}
