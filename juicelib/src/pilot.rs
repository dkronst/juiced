use std::{time::Duration, fmt::{Display, Formatter, self}};
use error_stack::{ResultExt, Context, Result};
use rppal::{pwm::{Pwm, Channel}, gpio::{Gpio, Level}};

use log::{info, warn, error, debug, trace};

pub struct Pilot {
    pwm: Pwm,
}

#[derive(Debug)]
pub enum PilotError {
    PwmError,
}

impl Context for PilotError {}

impl Display for PilotError {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            PilotError::PwmError => write!(f, "PwmError"),
        }
    }
}

impl Pilot {
    pub fn new() -> Result<Self, PilotError> {
        let pwm = Pwm::new(Channel::Pwm0).change_context(PilotError::PwmError)?;
        pwm.set_period(Duration::from_millis(1)).change_context(PilotError::PwmError)?; // 1KHz 
        pwm.enable().change_context(PilotError::PwmError)?;
        pwm.set_duty_cycle(1 as f64).change_context(PilotError::PwmError)?;

        Ok(Self {
            pwm,
        })
    }

    pub fn set_to_waiting_for_vehicle(&mut self) -> Result<(), PilotError> {
        // Setting the dc to 1.0 will cause the pilot to go to +12V constant
        // which is the waiting for vehicle state.
        self.pwm.set_duty_cycle(1.01 as f64).change_context(PilotError::PwmError)?;

        Ok(())
    }

    pub fn set_duty_cycle(&mut self, duty_cycle: f64) -> Result<(), PilotError> {

        self.pwm.set_duty_cycle(duty_cycle).change_context(PilotError::PwmError)?;

        Ok(())
    }

    pub fn set_to_error(&mut self) -> Result<(), PilotError> {
        // Setting the dc to 0 will cause the pilot to go to -12V which is
        // the error state.
        self.set_duty_cycle(0 as f64)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_to_waiting_for_vehicle() -> Result<(), PilotError> {
        let mut pilot = Pilot::new()?;
        pilot.set_to_waiting_for_vehicle()?;
        assert_eq!(pilot.pwm.duty_cycle().unwrap(), 1.0);
        // TODO: Measure the PWM using an oscilloscope
        // panic!("TODO: Measure the PWM using an oscilloscope");
        Ok(())
    }

    #[test]
    fn test_set_duty_cycle() -> Result<(), PilotError> {
        let mut pilot = Pilot::new()?;
        pilot.set_duty_cycle(0.5)?;
        assert_eq!(pilot.pwm.duty_cycle().unwrap(), 0.5);
        Ok(())
    }

    #[test]
    fn test_set_to_error() -> Result<(), PilotError> {
        let mut pilot = Pilot::new()?;
        pilot.set_to_error()?;
        assert_eq!(pilot.pwm.duty_cycle().unwrap(), 0.0);
        Ok(())
    }

    #[test]
    fn test_measure_pwm_update() {
        // tests the time it takes to change the duty cycle
        let mut pilot = Pilot::new().unwrap();
        let start = std::time::Instant::now();
        let mut count = 0;
        while start.elapsed() < Duration::from_millis(10) {
            let rand_dc = rand::random::<f64>();
            pilot.set_duty_cycle(rand_dc).unwrap();
            count += 1;
        }
        println!("count: {}", count);
        assert!(count > 25);
        assert!(start.elapsed() < Duration::from_millis(11));
    }
}