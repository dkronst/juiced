
use linux_embedded_hal::{Spidev, Pin};
use spidev::{SpidevOptions, SpidevTransfer};
use rust_gpiozero::OutputDevice;
use std::thread::sleep;
use std::time::Duration;

// Configure SPI
fn configure_spi() -> Spidev {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)
        .mode(spidev::SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();
    spi
}

fn read_adc(spi: &mut Spidev, chan: u8) -> u16 {
    let mut buf = [0u8; 3];
    let tx_buf = [1, 0x80 | (chan << 4), 0xff];
    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut buf);
    spi.transfer(&mut transfer).unwrap();
    // let val = ((buf[1] & 0x3) << 8) | buf[2] would overflow
    let val = ((buf[1] as u16 & 0x3) << 8) | buf[2] as u16;
    val as u16
}

// Test the pilot voltage
fn test_pilot(spi: &mut Spidev) {
    for _ in 0..500 {
        let val = read_adc(spi, 0);
        if val < 184 || val > 932 {
            println!("Error: Unexpected pilot voltage. ADC reading is {}", val);
            return;
        }
    }
    println!("Pilot voltage is within expected range.");
}

fn main() {
    let mut spi = configure_spi();
    test_pilot(&mut spi);
}
