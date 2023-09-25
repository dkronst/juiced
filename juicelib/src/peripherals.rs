use std::{sync::{atomic::AtomicBool, Arc, Mutex}, thread, time::Duration, fmt::{Display, Formatter, self}};

use error_stack::{Context, ResultExt, Result, Report};
///
/// Implementation of the peripherals module.
///

// use gpio's hal
use rppal::gpio::{Gpio, OutputPin, InputPin, Level};
use crate::{pilot::Pilot, adc::Adc};

use log::{info, warn, error, debug, trace};

#[derive(Debug)]
pub struct PeripheralsError;

impl Display for PeripheralsError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "PeripheralsError")
    }
}

impl Context for PeripheralsError {}

/// ## GPIO Pin Configuration
/// Please note the following descriptions refer to GPIO numbers, not actual pin numbers on the Raspberry Pi GPIO connector:

/// - `4` - Power Watchdog Pin: Toggle this pin at 1-10 kHz whenever the relay is powered on.
/// - `17` - Power Pin: Set this pin high to turn on the vehicle power. Ensure the power watchdog pin is toggling, else a synthetic GFI event will occur.
/// - `18` - Pilot Pin: Set this pin high for +12, low for -12. Further details on using this pin are discussed below.
/// - `22` - GFI Status Pin: This input pin goes high when the GFI is set, preventing the power from being turned on.
/// - `23` - Relay Test Pin: This input pin provides the status of the HV relay / GCM test. It should go high and remain that way within 100 ms of turning the power on. Similarly, it should go low and remain so within 100 ms of turning the power off.
/// - `24` - GFI Test Pin: This output pin is connected to a wire taking two loops through the GFI CT, then connecting to the ground. Toggle this pin at 60 Hz to simulate a ground fault as part of the GFI test procedure.
/// - `27` - GFI Reset Pin: Pulse this pin high to clear the GFI. This action cannot be performed while vehicle power is on.

// define the GPIO pins

const POWER_WATCHDOG_PIN: u8 = 4;
const CONTACTOR_PIN: u8 = 17;
const GFI_STATUS_PIN: u8 = 22;
const RELAY_TEST_PIN: u8 = 23;
const GFI_TEST_PIN: u8 = 24;
const GFI_RESET_PIN: u8 = 27;

pub struct Pins {
    gpio: Gpio,
    pub contactor_pin: OutputPin,
    pub gfi_status_pin: InputPin,
    pub relay_test_pin: InputPin,
    pub gfi_test_pin: OutputPin,
    pub gfi_reset_pin: OutputPin,
}

pub struct GpioPeripherals {
    pilot: Pilot,
    adc: Arc<Mutex<Adc>>,
    power_watchdog: Arc<AtomicBool>,
    pins: Arc<Mutex<Pins>>,
}

impl GpioPeripherals {
    #[must_use]
    pub fn new() -> Self {
        // TODO: unwraps
        let gpio = Gpio::new().unwrap();

        let pilot = Pilot::new().unwrap();
        let mut power_watchdog_pin = gpio.get(POWER_WATCHDOG_PIN).unwrap().into_output();
        let mut contactor_pin = gpio.get(CONTACTOR_PIN).unwrap().into_output();
        let gfi_status_pin = gpio.get(GFI_STATUS_PIN).unwrap().into_input();
        let relay_test_pin = gpio.get(RELAY_TEST_PIN).unwrap().into_input();
        let mut gfi_test_pin = gpio.get(GFI_TEST_PIN).unwrap().into_output();
        let mut gfi_reset_pin = gpio.get(GFI_RESET_PIN).unwrap().into_output();

        power_watchdog_pin.set_low();
        contactor_pin.set_low();
        gfi_test_pin.set_low();
        gfi_reset_pin.set_low();


        let power_watchdog = Arc::new(AtomicBool::new(false));

        Self::power_pin_thread(power_watchdog_pin, Arc::clone(&power_watchdog));
        let adc = Arc::new(Mutex::new(Adc::new().unwrap()));

        let pins = Arc::new(Mutex::new(Pins {
            gpio: gpio,
            contactor_pin: contactor_pin,
            gfi_status_pin: gfi_status_pin,
            relay_test_pin: relay_test_pin,
            gfi_test_pin: gfi_test_pin,
            gfi_reset_pin: gfi_reset_pin,
        }));

        Self {
            adc: adc,
            power_watchdog: power_watchdog,
            pilot: pilot,
            pins: pins,
        }
    }

    pub fn get_adc(&mut self) -> Arc<Mutex<Adc>> {
        Arc::clone(&self.adc)
    }

    pub fn set_power_watchdog(&mut self, level: Level) {
        let state = match level {
            Level::Low => false,
            Level::High => true,
        };
        self.power_watchdog.store(state, std::sync::atomic::Ordering::Relaxed);
    }

    pub fn set_pilot_ampere(&mut self, ampere: f32) -> Result<(), PeripheralsError> {
        let duty_cycle = ampere / 0.6; // Based on: https://www.fveaa.org/fb/J1772_386.pdf
        self.pilot.set_duty_cycle(duty_cycle as f64).change_context(PeripheralsError)
    }

    pub fn set_waiting_for_vehicle(&mut self) -> Result<(), PeripheralsError> {
        self.pilot.set_to_waiting_for_vehicle().change_context(PeripheralsError)
    }

    fn power_pin_thread(mut wdp: OutputPin, watch_dog: Arc<AtomicBool>) {
        let mut state = Level::High;
        
        wdp.set_low();
        thread::spawn(move || {
            let mut i = 0;
            let mut state_changes = 0;
            let mut last = std::time::Instant::now();
            loop {
                let togle = watch_dog.load(std::sync::atomic::Ordering::Relaxed);
                thread::sleep(Duration::from_micros(900));
                if togle {
                    state = match state {
                        Level::Low => Level::High,
                        Level::High => Level::Low,
                    };
                    wdp.write(state);
                    state_changes += 1;
                } else if state == Level::High {
                    wdp.write(Level::Low);
                }

                if i % 1000 == 0 {
                    let now = std::time::Instant::now();
                    if now - last > std::time::Duration::from_millis(2000) && togle {
                        debug!("power watchdog: toggles per second: {} (Hz)", state_changes as f32 / (now - last).as_secs_f32());
                        state_changes = 0;
                        last = now;
                    }
                    // TODO: Make sure the watchdog oscilates at around 1 kHz, otherwise raise an error
                    i = 0;
                }
            }
        });
        thread::sleep(std::time::Duration::from_millis(500));
    }

    pub fn set_contactor_pin(&mut self, level: Level) {
        let mut pins = self.pins.lock().unwrap();
        pins.contactor_pin.write(level);
    }

    pub fn reset_gfi_status_pin(&mut self) -> Result<(), PeripheralsError> {
        let mut pins = self.pins.lock().unwrap();
        pins.gfi_status_pin.clear_interrupt().change_context(PeripheralsError)?;
        pins.gfi_status_pin.clear_async_interrupt().change_context(PeripheralsError)?;
        match pins.gfi_status_pin.read() {
            Level::High => Err(Report::new(PeripheralsError)),
            Level::Low => Ok(()),
        }
    }

    pub fn read_gfi_status_pin(&self) -> Level {
        let pins = self.pins.lock().unwrap();
        pins.gfi_status_pin.read()
    }

    pub fn get_pins(&mut self) -> Arc<Mutex<Pins>> {
        // TODO: Get rid of the unwrap
        Arc::clone(&self.pins)
    }

    pub fn read_relay_test_pin(&self) -> Level {
        let pins = self.pins.lock().unwrap();
        pins.relay_test_pin.read()
    }

    pub fn set_gfi_test_pin(&mut self, level: Level) {
        let mut pins = self.pins.lock().unwrap();
        pins.gfi_test_pin.write(level);
    }

    pub fn gfi_reset(&mut self) -> Result<(), PeripheralsError> {
        debug!("Resetting GFI");
        let mut pins = self.pins.lock().unwrap();
        pins.gfi_reset_pin.set_high();
        thread::sleep(std::time::Duration::from_millis(100));
        pins.gfi_reset_pin.set_low();
        thread::sleep(std::time::Duration::from_millis(100));
        Ok(())
    }
}

#[cfg(test)]
mod testgpio {
    use std::time::Duration;
    use lazy_static::lazy_static;
    use std::sync::Mutex;

    use super::*;
    lazy_static! {
        static ref GPIO: Mutex<GpioPeripherals> = Mutex::new(GpioPeripherals::new());
    }

    #[test]
    fn test_set_power_watchdog() {
        let mut gpio = GPIO.lock().unwrap();
        gpio.set_power_watchdog(Level::High);
        thread::sleep(Duration::from_millis(100));
        gpio.set_power_watchdog(Level::Low);
    }

    #[test]
    fn test_set_contactor_pin() {
        let mut gpio = GPIO.lock().unwrap();
        gpio.set_contactor_pin(Level::High);
    }

    #[test]
    fn test_get_gfi_status_pin() {
        let gpio = GPIO.lock().unwrap();
        assert_eq!(gpio.read_gfi_status_pin(), Level::Low);
    }

    #[test]
    fn test_get_relay_test_pin() {
        let gpio = GPIO.lock().unwrap();
        assert_eq!(gpio.read_relay_test_pin(), Level::Low);
    }

    #[test]
    fn test_set_gfi_test_pin() {
        let mut gpio = GPIO.lock().unwrap();
        gpio.set_gfi_test_pin(Level::High);
    }

    #[test]
    fn test_set_gfi_reset_pin() {
        let mut gpio = GPIO.lock().unwrap();
        gpio.gfi_reset();
    }

    #[test]
    fn test_set_pilot_ampere() {
        let mut gpio = GPIO.lock().unwrap();
        gpio.set_pilot_ampere(32.0).unwrap();
    }

}