use std::{sync::{atomic::AtomicBool, Arc}, thread};

///
/// Implementation of the peripherals module.
///

// use gpio's hal
use rppal::gpio::{Gpio, OutputPin, InputPin, Level};
use crate::pilot::Pilot;

/// ## GPIO Pin Configuration
/// Please note the following descriptions refer to GPIO numbers, not actual pin numbers on the Raspberry Pi GPIO connector:

/// - `4` - Power Watchdog Pin: Toggle this pin at 1-10 kHz whenever the relay is powered on. (using the pilot.rs module)
/// - `17` - Power Pin: Set this pin high to turn on the vehicle power. Ensure the power watchdog pin is toggling, else a synthetic GFI event will occur.
/// - `18` - Pilot Pin: Set this pin high for +12, low for -12. Further details on using this pin are discussed below.
/// - `22` - GFI Status Pin: This input pin goes high when the GFI is set, preventing the power from being turned on.
/// - `23` - Relay Test Pin: This input pin provides the status of the HV relay / GCM test. It should go high and remain that way within 100 ms of turning the power on. Similarly, it should go low and remain so within 100 ms of turning the power off.
/// - `24` - GFI Test Pin: This output pin is connected to a wire taking two loops through the GFI CT, then connecting to the ground. Toggle this pin at 60 Hz to simulate a ground fault as part of the GFI test procedure.
///i - `27` - GFI Reset Pin: Pulse this pin high to clear the GFI. This action cannot be performed while vehicle power is on.

// define the GPIO pins

const POWER_WATCHDOG_PIN: u8 = 4;
const CONTACTOR_PIN: u8 = 17;
const GFI_STATUS_PIN: u8 = 22;
const RELAY_TEST_PIN: u8 = 23;
const GFI_TEST_PIN: u8 = 24;
const GFI_RESET_PIN: u8 = 27;

pub struct GpioPeripherals {
    pilot: Pilot,
    contactor_pin: OutputPin,
    gfi_status_pin: InputPin,
    relay_test_pin: InputPin,
    gfi_test_pin: OutputPin,
    gfi_reset_pin: OutputPin,
    power_watchdog: Arc<AtomicBool>,
}

impl GpioPeripherals {
    pub fn new() -> Self {
        // TODO: unwraps
        let gpio = Gpio::new().unwrap();

        let pilot = Pilot::new().unwrap();
        let power_watchdog_pin = gpio.get(POWER_WATCHDOG_PIN).unwrap().into_output();
        let contactor_pin = gpio.get(CONTACTOR_PIN).unwrap().into_output();
        let gfi_status_pin = gpio.get(GFI_STATUS_PIN).unwrap().into_input();
        let relay_test_pin = gpio.get(RELAY_TEST_PIN).unwrap().into_input();
        let gfi_test_pin = gpio.get(GFI_TEST_PIN).unwrap().into_output();
        let gfi_reset_pin = gpio.get(GFI_RESET_PIN).unwrap().into_output();

        let power_watchdog = Arc::new(AtomicBool::new(false));

        Self::power_pin_thread(power_watchdog_pin, Arc::clone(&power_watchdog));

        Self {
            power_watchdog: power_watchdog,
            pilot: pilot,
            contactor_pin,
            gfi_status_pin,
            relay_test_pin,
            gfi_test_pin,
            gfi_reset_pin,
        }
    }

    pub fn set_power_watchdog(&mut self, _level: Level) {
        self.power_watchdog.store(true, std::sync::atomic::Ordering::Relaxed);
    }

    pub fn set_pilot_ampere(&mut self, ampere: f32) {
        let duty_cycle = ampere / 100.0; // This is wrong: TODO: fix
        self.pilot.set_duty_cycle(duty_cycle as f64);
    }

    fn power_pin_thread(mut wdp: OutputPin, watch_dog: Arc<AtomicBool>) {
        let tick = crossbeam_channel::tick(std::time::Duration::from_micros(500));
        let mut state = Level::Low;
        thread::spawn(move || {
            loop {
                tick.recv().unwrap();
                let togle = watch_dog.load(std::sync::atomic::Ordering::Relaxed);
                if togle {
                    state = match state {
                        Level::Low => Level::High,
                        Level::High => Level::Low,
                    };
                    wdp.write(state);
                } else if state == Level::High {
                    wdp.write(Level::Low);
                }
            }
        });
    }

    pub fn set_contactor_pin(&mut self, level: Level) {
        self.contactor_pin.write(level);
    }

    pub fn get_gfi_status_pin(&mut self) -> Level {
        self.gfi_status_pin.read()
    }

    pub fn get_relay_test_pin(&mut self) -> Level {
        self.relay_test_pin.read()
    }

    pub fn set_gfi_test_pin(&mut self, level: Level) {
        self.gfi_test_pin.write(level);
    }

    pub fn set_gfi_reset_pin(&mut self, level: Level) {
        self.gfi_reset_pin.write(level);
    }
}

#[cfg(test)]
mod testgpio {
    use std::time::Duration;

    use super::*;

    #[test]
    fn test_set_power_watchdog() {
        let mut gpio = GpioPeripherals::new();
        gpio.set_power_watchdog(Level::High);
        thread::sleep(Duration::from_millis(100));
        gpio.set_power_watchdog(Level::Low);
    }

    #[test]
    fn test_set_contactor_pin() {
        let mut gpio = GpioPeripherals::new();
        gpio.set_contactor_pin(Level::High);
    }

    #[test]
    fn test_get_gfi_status_pin() {
        let mut gpio = GpioPeripherals::new();
        assert_eq!(gpio.get_gfi_status_pin(), Level::Low);
    }

    #[test]
    fn test_get_relay_test_pin() {
        let mut gpio = GpioPeripherals::new();
        assert_eq!(gpio.get_relay_test_pin(), Level::Low);
    }

    #[test]
    fn test_set_gfi_test_pin() {
        let mut gpio = GpioPeripherals::new();
        gpio.set_gfi_test_pin(Level::High);
    }

    #[test]
    fn test_set_gfi_reset_pin() {
        let mut gpio = GpioPeripherals::new();
        gpio.set_gfi_reset_pin(Level::High);
    }

    #[test]
    fn test_set_pilot_ampere() {
        let mut gpio = GpioPeripherals::new();
        gpio.set_pilot_ampere(32.0);
    }

}