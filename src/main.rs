#![no_std]
#![no_main]

mod fmt;
mod pin;

use defmt::info;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_time::Timer;
use crate::pin::{GpioOutput, Port};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    
    let mut led = pin::OutputPin::new(Port::B, 14, 2, false);
    
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        info!("Tick");
    }
}
