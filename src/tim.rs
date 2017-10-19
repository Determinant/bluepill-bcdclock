use stm32f103xx::{tim2, TIM2};

pub struct Timer<'a> (pub &'a tim2::RegisterBlock);

impl<'a> Timer<'a> {
    pub fn init(&self, timeout: u32) {
        let tim = self.0;
        self.set_timeout(timeout);
        tim.cr1.write(|w| unsafe {
                          w.opm().continuous()
                           .cms().bits(0b00)
                           .dir().up()});
        tim.dier.modify(|_, w| w.uie().set_bit());
    }

    pub fn set_timeout(&self, timeout: u32) {
        let psc: u16 = (timeout / (1 << 16)) as u16;
        let arr: u16 = (timeout / (psc + 1) as u32) as u16;
        self.0.psc.write(|w| w.psc().bits(psc));
        self.0.arr.write(|w| w.arr().bits(arr));
    }

    pub fn reset(&self) {
        self.0.cnt.write(|w| w.cnt().bits(0));
    }

    pub fn go(&self) {
        self.0.cr1.modify(|_, w| w.cen().enabled());
    }

    pub fn stop(&self) {
        self.0.cr1.modify(|_, w| w.cen().disabled());
    }
}
