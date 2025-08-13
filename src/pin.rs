mod memory;
use memory::*;
pub trait GpioOutput {
    fn is_low(&self) -> bool;
    fn set_low(&mut self);

    fn is_high(&self) -> bool;
    fn set_high(&mut self);

}

pub struct OutputPin {
    port: *mut u32,
    num: u8,
}

impl OutputPin {
    pub fn new(port: Port, num: u8, speed: u8, pull_up: bool) -> Self {
        debug_assert!(num <= 15);
        if port == Port::I {
            debug_assert!(num <= 11);
        }
        let addr: *mut u32 = match port {
            Port::A => GPIO_A_ADDR,
            Port::B => GPIO_B_ADDR,
            Port::C => GPIO_C_ADDR,
            Port::D => GPIO_D_ADDR,
            Port::E => GPIO_E_ADDR,
            Port::F => GPIO_F_ADDR,
            Port::G => GPIO_G_ADDR,
            Port::H => GPIO_H_ADDR,
            Port::I => GPIO_I_ADDR,
        };
        unsafe {
            set_mode(1, num, addr);
            set_output_type(false, num, addr);
            set_speed(speed, num, addr);
            set_pull(if pull_up { 1 } else { 2 }, num, addr);
        }
        Self {
            port: addr,
            num
        }
    }
}

impl GpioOutput for OutputPin {
    fn is_low(&self) -> bool {
        let v = unsafe {read_odr(
            self.num,
            self.port,
        )};
        if v >= 1 { true } else { false }
    }
    fn set_low(&mut self) {
        unsafe {
            set_bsrr(self.num, true, self.port);
        }
    }

    fn is_high(&self) -> bool {
        let v = unsafe {read_odr(
            self.num,
            self.port,
        )};
        if v >= 1 { false } else { true }
    }
    fn set_high(&mut self) {
        unsafe {
            set_bsrr(self.num, false, self.port);
        }
    }
}
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Port {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I
}