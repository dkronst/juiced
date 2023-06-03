use std::time::Duration;
use rppal::pwm::{Pwm, Error as PwmError, Channel};

pub struct Pilot {
    pwm: Pwm,
}

impl Pilot {
    pub fn new() -> Result<Self, PwmError> {
        let pwm = Pwm::new(Channel::Pwm0)?;
        pwm.set_period(Duration::from_millis(1))?;
        pwm.enable()?;

        Ok(Self {
            pwm,
        })
    }

    pub fn set_to_waiting_for_vehicle(&mut self) -> Result<(), PwmError> {
        // Setting the dc to 1.0 will cause the pilot to go to +12V constant
        // which is the waiting for vehicle state.
        self.pwm.set_duty_cycle(1.01 as f64)?;

        Ok(())
    }

    pub fn set_duty_cycle(&mut self, duty_cycle: f64) -> Result<(), PwmError> {
        self.pwm.set_duty_cycle(duty_cycle)?;

        Ok(())
    }

    pub fn set_to_error(&mut self) -> Result<(), PwmError> {
        // Setting the dc to 0 will cause the pilot to go to -12V which is
        // the error state.
        self.pwm.set_duty_cycle(0 as f64)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_to_waiting_for_vehicle() -> Result<(), PwmError> {
        let mut pilot = Pilot::new()?;
        pilot.set_to_waiting_for_vehicle()?;
        assert_eq!(pilot.pwm.duty_cycle().unwrap(), 1.0);
        // TODO: Measure the PWM using an oscilloscope
        panic!("TODO: Measure the PWM using an oscilloscope");
        Ok(())
    }

    #[test]
    fn test_set_duty_cycle() -> Result<(), PwmError> {
        let mut pilot = Pilot::new()?;
        pilot.set_duty_cycle(0.5)?;
        assert_eq!(pilot.pwm.duty_cycle().unwrap(), 0.5);
        Ok(())
    }

    #[test]
    fn test_set_to_error() -> Result<(), PwmError> {
        let mut pilot = Pilot::new()?;
        pilot.set_to_error()?;
        assert_eq!(pilot.pwm.duty_cycle().unwrap(), 0.0);
        Ok(())
    }
}