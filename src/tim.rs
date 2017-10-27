use stm32f103xx::{tim1, tim2, TIM2};

pub struct Timer<'a> (pub &'a tim2::RegisterBlock);

impl<'a> Timer<'a> {
    pub fn init(&self, timeout: u32) {
        let tim = self.0;
        tim.dier.modify(|_, w| w.uie().clear_bit());
        self.set_timeout(timeout);
        tim.cr1.write(|w| w.opm().continuous());
        /* UEV to reload the psc and arr (without an interrupt) */
        tim.egr.write(|w| w.ug().set_bit());
        /* clear the interrupt flag caused by UG */
        tim.sr.modify(|_, w| w.uif().clear());
        /* finally enable the interrupt trigger */
        tim.dier.modify(|_, w| w.uie().set_bit());
    }

    pub fn set_timeout(&self, timeout: u32) {
        let psc: u16 = (timeout / (1 << 16)) as u16;
        let arr: u16 = (timeout / (psc + 1) as u32) as u16;
        self.0.psc.write(|w| w.psc().bits(psc));
        self.0.arr.write(|w| w.arr().bits(arr));
    }

    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.0.cr1.read().cen() == tim1::cr1::CENR::ENABLED
    }

    #[inline]
    pub fn reset(&self) {
        self.0.cnt.write(|w| w.cnt().bits(0));
    }

    #[inline]
    pub fn go(&self) {
        self.0.cr1.modify(|_, w| w.cen().enabled());
    }

    #[inline]
    pub fn stop(&self) {
        self.0.cr1.modify(|_, w| w.cen().disabled());
    }
}
