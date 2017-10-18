use core::ops::{Deref, DerefMut};
use core::cell::UnsafeCell;

pub struct Mutex<T> {
    res: UnsafeCell<T>,
    locked: u8
}

pub struct MutexGuard<'a, T: 'a> {
    lock: &'a Mutex<T>
}

impl<'a, T> Drop for MutexGuard<'a, T> {
    fn drop(&mut self) {
        unsafe {
            asm!("ldr r2, $0
                  mov r1, $1
                  dmb
                  str r1, [r2]"
                :
                : "m"(&self.lock.locked), "i"(0));
        }
    }
}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        unsafe { &*self.lock.res.get() }
    }
}

impl<'a, T> DerefMut for MutexGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *self.lock.res.get() }
    }
}

impl<T> Mutex<T> {
    pub fn new(t: T) -> Mutex<T> {
        Mutex{res: UnsafeCell::new(t), locked: 0}
    }
    pub fn lock(&self) -> MutexGuard<T> {
        unsafe {
            asm!("ldr r2, $0
                  mov r1, $1
                  spin_lock:
                  ldrex r0, [r2]
                  cmp r0, $1
                  itt ne
                  strexbne r0, r1, [r2]
                  cmpne r0, #1
                  beq spin_lock
                  dmb"
              :
              : "m"(&self.locked), "i"(1)
              : "r2", "r1", "r0");
        }
        MutexGuard{lock: self}
    }
}
