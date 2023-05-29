use std::fs::File;
use std::io::{Error, Write};
use linux_embedded_hal::{Spidev, Pin};
use spidev::{SpidevOptions, SpidevTransfer};
use rust_gpiozero::OutputDevice;

pub struct ADC {
    spi: Spidev,
}

pub struct Pilot {
    period_file: File,
    duty_cycle_file: File,
    enable_file: File,
}

impl Pilot {
    pub fn new() -> Result<Self, Error> {
        let period_file = File::create("/sys/class/pwm/pwmchip0/pwm0/period")?;
        let duty_cycle_file = File::create("/sys/class/pwm/pwmchip0/pwm0/duty_cycle")?;
        let enable_file = File::create("/sys/class/pwm/pwmchip0/pwm0/enable")?;

        Ok(Self {
            period_file,
            duty_cycle_file,
            enable_file,
        })
    }

    pub fn activate_pilot(&mut self) -> Result<(), Error> {
        self.period_file.write_all(b"1000000\n")?;
        self.duty_cycle_file.write_all(b"100000\n")?;
        self.enable_file.write_all(b"1\n")?;

        Ok(())
    }

    pub fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<(), Error> {
        let duty_cycle_ns = (duty_cycle * 10000.0) as u32;
        self.duty_cycle_file.write_all(format!("{}\n", duty_cycle_ns).as_bytes())?;

        Ok(())
    }

    pub fn override_output(&mut self, override_output: bool) -> Result<(), Error> {
        let duty_cycle_ns = if override_output {
            1010000
        } else {
            100000
        };
        self.duty_cycle_file.write_all(format!("{}\n", duty_cycle_ns).as_bytes())?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let evse = Pilot::new().unwrap();
        assert!(evse.period_file.metadata().unwrap().len() > 0);
        assert!(evse.duty_cycle_file.metadata().unwrap().len() > 0);
        assert!(evse.enable_file.metadata().unwrap().len() > 0);
    }

    #[test]
    fn test_activate_pilot() {
        let mut evse = Pilot::new().unwrap();
        evse.activate_pilot().unwrap();
        let duty_cycle = std::fs::read_to_string("/sys/class/pwm/pwmchip0/pwm0/duty_cycle").unwrap();
        assert_eq!(duty_cycle.trim(), "100000");
        let enable = std::fs::read_to_string("/sys/class/pwm/pwmchip0/pwm0/enable").unwrap();
        assert_eq!(enable.trim(), "1");
    }

    #[test]
    fn test_set_duty_cycle() {
        let mut evse = Pilot::new().unwrap();
        evse.set_duty_cycle(10.0).unwrap();
        let duty_cycle = std::fs::read_to_string("/sys/class/pwm/pwmchip0/pwm0/duty_cycle").unwrap();
        assert_eq!(duty_cycle.trim(), "1000000");
    }

    #[test]
    fn test_override_output() {
        let mut evse = Pilot::new().unwrap();
        evse.override_output(true).unwrap();
        let duty_cycle = std::fs::read_to_string("/sys/class/pwm/pwmchip0/pwm0/duty_cycle").unwrap();
        assert_eq!(duty_cycle.trim(), "1010000");
        evse.override_output(false).unwrap();
        let duty_cycle = std::fs::read_to_string("/sys/class/pwm/pwmchip0/pwm0/duty_cycle").unwrap();
        assert_eq!(duty_cycle.trim(), "100000");
    }
}