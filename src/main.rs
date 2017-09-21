#![no_std]

#[macro_use] extern crate stm32f103xx;
extern crate cortex_m;
use stm32f103xx::{GPIOA, RCC, SYST, I2C1};
use cortex_m::peripheral::SystClkSource;
mod i2c;
mod ds3231;

struct ShiftRegister<'a> {
    gpioa: &'a stm32f103xx::gpioa::RegisterBlock,
    width: u8,
}

static mut SR: Option<ShiftRegister> = None;
static mut RTC: Option<ds3231::DS3231> = None;
static mut N: u16 = 0;
static mut DIGITS: [u8; 6] = [0; 6];

fn systick_handler() {
    unsafe {
        /*
        SR.as_mut().unwrap().output_bits(N as u32);
        N += 1;
        */
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

//exception!(SYS_TICK, systick_handler);

impl<'a> ShiftRegister<'a> {
    fn new(g: &'a stm32f103xx::gpioa::RegisterBlock,
           width: u8) -> ShiftRegister<'a> {
        let this = ShiftRegister{gpioa: g, width: width};
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

fn digits2bcds(digs: &[u8]) -> u32 {
    let mut res: u32 = 0;
    for d in digs.iter().rev() {
        res = (res << 4) | (*d as u32);
    }
    res
}

fn main() {

    let gpioa: &stm32f103xx::gpioa::RegisterBlock = unsafe { &*GPIOA.get() };
    let rcc: &stm32f103xx::rcc::RegisterBlock = unsafe { &*RCC.get() };
    let i2c: &stm32f103xx::i2c1::RegisterBlock = unsafe { &*I2C1.get() };
    let syst: &cortex_m::peripheral::SYST = unsafe { &*SYST.get() };

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(100_000);
    syst.enable_interrupt();
    syst.enable_counter();
    rcc.apb2enr.modify(|_, w| w.iopaen().enabled());
    gpioa.crl.modify(|_, w|
        w.mode0().output().cnf0().push()
         .mode1().output().cnf1().push()
         .mode2().output().cnf2().push());

    unsafe {
        RTC = Some(ds3231::DS3231::new(i2c));
        SR = Some(ShiftRegister::new(gpioa, 24));
        SR.as_mut().unwrap().output_bits(0);
        let t = RTC.as_mut().unwrap().read_fulldate();
    }
}
