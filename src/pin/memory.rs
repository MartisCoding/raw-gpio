
pub(super) const GPIO_A_ADDR: *mut u32 = 0x4002_0000 as _;
pub(super) const GPIO_B_ADDR: *mut u32 = 0x4002_0400 as _;
pub(super) const GPIO_C_ADDR: *mut u32 = 0x4002_0800 as _;
pub(super) const GPIO_D_ADDR: *mut u32 = 0x4002_0C00 as _;
pub(super) const GPIO_E_ADDR: *mut u32 = 0x4002_1000 as _;
pub(super) const GPIO_F_ADDR: *mut u32 = 0x4002_1400 as _;
pub(super) const GPIO_G_ADDR: *mut u32 = 0x4002_1800 as _;
pub(super) const GPIO_H_ADDR: *mut u32 = 0x4002_1C00 as _;
pub(super) const GPIO_I_ADDR: *mut u32 = 0x4002_2000 as _;
pub(super) const GPIO_J_ADDR: *mut u32 = 0x4002_2400 as _;
pub(super) const GPIO_K_ADDR: *mut u32 = 0x4002_2800 as _;






const GPIOX_MODER: u8 = 0x00;
const GPIOX_OTYPER: u8 = 0x04;
const GPIOX_OSPEEDR: u8 = 0x08;
const GPIOX_PUPDR: u8 = 0x0C;
const GPIOX_IDR: u8 = 0x10;
const GPIOX_ODR: u8 = 0x14;
const GPIOX_BSRR: u8 = 0x18;
const GPIOX_LCKR: u8 = 0x1C;
const GPIOX_AFRL: u8 = 0x20;
const GPIOX_AFRH: u8 = 0x22;


/// Sets a mode for Gpio pin.
/// Default is 0 - input, 1 - gpio output, 2 - alternate function, 3 - analog.
/// Safety: ensure that 1) Pin implements adc if needed. 2) Port address is correct.
#[inline(always)]
pub(super) unsafe fn set_mode(
    mode: u8,
    pin_num: u8,
    port_addr: *mut u32
) {
    let moder_reg = port_addr.byte_offset(GPIOX_MODER as isize);
    set_2_bits_for_32_bit_reg(
        pin_num,
        mode as u32,
        moder_reg,
    )
}


/// Set the output type of specific pin. 
/// If not open-drain. Then push-pull
#[inline(always)]
pub(super) unsafe fn set_output_type(
    open_drain: bool,
    pin_num: u8,
    port_addr: *mut u32
) {

    let otyper_addr = port_addr.byte_offset(GPIOX_OTYPER as isize);
    set_1_bit_for_32_bit_reg_halved(
        pin_num,
        open_drain as u32,
        otyper_addr,
    )
}
#[inline(always)]
pub(super) unsafe fn set_speed(
    speed: u8,
    pin_num: u8,
    port_addr: *mut u32
) {
    let ospeed_addr = port_addr.byte_offset(GPIOX_OSPEEDR as isize);
    set_1_bit_for_32_bit_reg_halved(
        pin_num,
        speed as u32,
        ospeed_addr,
    )
}
#[inline(always)]
pub(super) unsafe fn set_pull(
    pupd: u8,
    pin_num: u8,
    port_addr: *mut u32
) {
    let pupd_addr = port_addr.byte_offset(GPIOX_PUPDR as isize);
    set_2_bits_for_32_bit_reg(
        pin_num,
        pupd as u32,
        pupd_addr,
    )
}
#[inline(always)]
pub(super) unsafe fn read_from_idr(
    pin_num: u8,
    port_addr: *mut u32
) -> u8 {
    debug_assert!(pin_num <= 15);
    let idr = port_addr.byte_offset(GPIOX_IDR as isize);
    let current = core::ptr::read_volatile(idr);
    let mask = 1u32 << pin_num;
    let val = (current & mask) >> pin_num;
    val as u8
}
#[inline(always)]
pub(super) unsafe fn set_odr(
    pin_num: u8,
    high: bool,
    port_addr: *mut u32,
) {
    debug_assert!(pin_num <= 15);
    let odr = port_addr.byte_offset(GPIOX_ODR as isize);
    set_1_bit_for_32_bit_reg_halved(
        pin_num,
        high as u32,
        odr,
    )
}
#[inline(always)]
pub(super) unsafe fn read_odr(
    pin_num: u8,
    port_addr: *mut u32
) -> u8 {
    debug_assert!(pin_num <= 15);
    let odr = port_addr.byte_offset(GPIOX_ODR as isize);
    let current = core::ptr::read_volatile(odr);
    let mask = 1u32 << pin_num;
    let val = (current & mask) >> pin_num;
    val as u8
}
#[inline(always)]
pub(super) unsafe fn set_bsrr(
    pin_num: u8,
    reset: bool,
    port_addr: *mut u32
) {
    debug_assert!(pin_num <= 15);
    let bsrr = port_addr.byte_offset(GPIOX_BSRR as isize);
    let value: u32 = if reset {
        1 << pin_num
    } else { 
        1 << (pin_num + 16) 
    };
    core::ptr::write_volatile(bsrr, value)
}

/// Performs a set of Alternate fucntion to pin. Values are from 0 to 15.
/// It is up to a user to ensure, that the alternate function needed is implemented for pin.
pub(super) unsafe fn set_afr(
    pin_num: u8,
    af: u8,
    port_addr: *mut u32
) {
    assert!(pin_num <= 15);
    assert!(af <= 15);
    
    if pin_num <= 7 {
        let afr = port_addr.byte_offset(GPIOX_AFRL as isize);
        let val = core::ptr::read_volatile(afr);
        let mask = 0b1111 << (pin_num * 4);
        let new_val = (val & !mask) | ((af as u32) << (pin_num * 4));
        core::ptr::write_volatile(afr, new_val);
    } else {
        let afr = port_addr.byte_offset(GPIOX_AFRH as isize);
        let pin_num = pin_num - 8;
        let mask = 0b1111 << (pin_num * 4);
        let new_val = (af as u32 & !mask) | ((af as u32) >> (pin_num * 4));
        core::ptr::write_volatile(afr, new_val);
    }
}











// Internal convinience methods. NO EXTERNAL USAGE IS ALLOWED.
#[inline(always)]
unsafe fn set_2_bits_for_32_bit_reg(
    pin_num: u8,
    bits: u32,
    reg_addr: *mut u32
) {
    debug_assert!(pin_num <= 15);
    debug_assert!(bits <= 3);
    debug_assert_eq!((reg_addr as usize) % 4, 0);
    let current = core::ptr::read_volatile(reg_addr);
    let mask = 0b11 << (pin_num * 2);
    let new_value = (current & !mask) | ((bits) << (pin_num * 2));
    core::ptr::write_volatile(reg_addr, new_value);
}

#[inline(always)]
unsafe fn set_1_bit_for_32_bit_reg_halved(
    pin_num: u8,
    bit: u32,
    reg_addr: *mut u32
) {
    debug_assert!(pin_num <= 15);
    debug_assert!(bit <= 1);
    debug_assert_eq!((reg_addr as usize) % 4, 0);
    let current = core::ptr::read_volatile(reg_addr);
    let mask = 0b1 << pin_num;
    let new_value = (current & !mask) | ((bit) << pin_num);
    core::ptr::write_volatile(reg_addr, new_value);
}

