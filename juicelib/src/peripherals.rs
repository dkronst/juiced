use std::{fmt::{self, Display, Formatter}, sync::{Arc, Mutex}, thread};
use std::result::Result as StdResult;
use lazy_static::lazy_static;

use error_stack::{Context, ResultExt, Result, Report, FutureExt};

lazy_static! {
    static ref TOKIO_RT: tokio::runtime::Runtime = tokio::runtime::Runtime::new().unwrap();
}

// use gpio's hal
use rppal::gpio::{Gpio, OutputPin, InputPin, Level};
use apigpio::{Connection, PigpiodError, Pulse, WaveId};
use crate::{pilot::Pilot, adc::Adc};

use log::{debug, info};

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
const DEFAULT_PWM_FREQUENCY: u32 = 10000;
const DEFAULT_DUTY_CYCLE: f64 = 0.5;


pub struct Pins {
    pub contactor_pin: OutputPin,
    pub gfi_status_pin: InputPin,
    pub relay_test_pin: InputPin,
    pub gfi_test_pin: OutputPin,
    pub gfi_reset_pin: OutputPin,
    pub power_watchdog_pin: OutputPinWithPwm,

    power_watchdog_oscillating: bool,
}

pub struct OutputPinWithPwm {
    pin: OutputPin,
    frequency: u32,
    duty_cycle: f64,
    conn: Connection,
    wave_id: Option<WaveId>,
}

impl TryFrom<OutputPin> for OutputPinWithPwm {
    type Error = Report<PeripheralsError>;
    fn try_from(pin: OutputPin) -> StdResult<Self, Self::Error> {
        let rt = &TOKIO_RT;
        let frequency = DEFAULT_PWM_FREQUENCY;
        let duty_cycle = DEFAULT_DUTY_CYCLE;
        // Start with range = 255 (default)
        let pin_number = pin.pin() as u32;
    
        let conn = rt.block_on(Connection::new()).change_context(PeripheralsError)?;
        rt.block_on(conn.wave_clear()).change_context(PeripheralsError)?;
        rt.block_on(conn.set_mode(pin_number, apigpio::GpioMode::Output)).change_context(
            PeripheralsError
        )?;
        let wid = rt.block_on(
                init_pio_pwm(&conn, pin_number, frequency, duty_cycle)
        ).change_context(
            PeripheralsError,
        )?;

        Ok(Self {
            pin: pin,
            frequency: frequency,
            duty_cycle: duty_cycle,
            conn: conn,
            wave_id: Some(wid),
        })
    }
}

async fn init_pio_pwm(connection: &Connection, pin: u32, frequency: u32, duty_cycle: f64) -> Result<WaveId, PeripheralsError> {
    let period = (1. / frequency as f64 * 1_000_000.) as u32;
    let on_pwm = Pulse {
        on_mask: 1 << pin,
        off_mask: 0,
        us_delay: (duty_cycle * period as f64) as u32,
    };
    let off_pwm = Pulse {
        on_mask: 0,
        off_mask: 1 << pin,
        us_delay: period - (duty_cycle * period as f64) as u32,
    };
    info!("Creating PIO PWM with period: {}, on: {}, off: {}", period, on_pwm.us_delay, off_pwm.us_delay);
    debug!("Creating PIO PWM with on: {:?}, off: {:?}", on_pwm, off_pwm);
    connection
        .wave_add_generic(vec![on_pwm, off_pwm].as_ref())
        .await
        .change_context(PeripheralsError)?;
    let wave_id = connection.wave_create().await.change_context(PeripheralsError)?;

    Ok(wave_id)
}

trait PigPioPwm {
    fn start_pio_pwm(&mut self) -> Result<(), PeripheralsError>;
    fn stop_pio_pwm(&mut self) -> Result<(), PeripheralsError>;
}

impl PigPioPwm for OutputPinWithPwm {
    fn start_pio_pwm(&mut self) -> Result<(), PeripheralsError> {
        let rt = &TOKIO_RT;
        let pin_number = self.pin.pin() as u32;
        info!("Starting PIO PWM on pin {}", pin_number);
        match self.wave_id {
            Some(wave_id) => {
                rt.block_on(self.conn.wave_send_repeat(wave_id)).change_context(PeripheralsError)?;
            }
            None => {
                let wid = rt.block_on(init_pio_pwm(&self.conn, pin_number, self.frequency, self.duty_cycle)).change_context(PeripheralsError)?;
                self.wave_id = Some(wid);
            }
        }
        
        Ok(())
    }

    fn stop_pio_pwm(&mut self) -> Result<(), PeripheralsError> {
        let pin_number = self.pin.pin() as u32;
        info!("Stopping PIO PWM on pin {}", pin_number);
        TOKIO_RT.block_on(self.conn.wave_tx_stop()).change_context(PeripheralsError)?;
        Ok(())
    }

}

pub struct GpioPeripherals {
    pilot: Pilot,
    adc: Arc<Mutex<Adc>>,
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


        let adc = Arc::new(Mutex::new(Adc::new().unwrap()));

        let pins = Arc::new(Mutex::new(Pins {
            contactor_pin: contactor_pin,
            gfi_status_pin: gfi_status_pin,
            relay_test_pin: relay_test_pin,
            gfi_test_pin: gfi_test_pin,
            gfi_reset_pin: gfi_reset_pin,
            power_watchdog_pin: power_watchdog_pin.try_into().unwrap(),
            power_watchdog_oscillating: false,
        }));

        ctrlc::set_handler(move || {
            let mut new_pilot = Pilot::new().unwrap();
            new_pilot.set_duty_cycle(0.0).unwrap();
        }).unwrap();

        Self {
            adc: adc,
            pilot: pilot,
            pins: pins,
        }
    }

    pub fn get_adc(&mut self) -> Arc<Mutex<Adc>> {
        Arc::clone(&self.adc)
    }

    pub fn set_oscillate_watchdog(&mut self, oscillate: bool) -> Result<(), PeripheralsError> {
        let mut pins = self.pins.lock().map_err(|_| Report::new(PeripheralsError))?;
        if pins.power_watchdog_oscillating == oscillate {
            debug!("Watchdog already in desired state");
            return Ok(());
        }
        if oscillate {
            debug!("Oscillating watchdog");
            pins.power_watchdog_pin.start_pio_pwm().change_context(PeripheralsError)?;
        } else if pins.power_watchdog_oscillating {
            debug!("Stopping watchdog oscillation");
            pins.power_watchdog_pin.stop_pio_pwm().change_context(PeripheralsError)?;
        }
        pins.power_watchdog_oscillating = oscillate;
        Ok(())
    }

    pub fn set_pilot_ampere(&mut self, ampere: f32) -> Result<(), PeripheralsError> {
        let duty_cycle = ampere / 0.6 / 100.; // Based on: https://www.fveaa.org/fb/J1772_386.pdf
        self.pilot.set_duty_cycle(duty_cycle as f64).change_context(PeripheralsError)
    }

    pub fn set_waiting_for_vehicle(&mut self) -> Result<(), PeripheralsError> {
        self.pilot.set_to_waiting_for_vehicle().change_context(PeripheralsError)
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
        gpio.set_oscillate_watchdog(true).unwrap();
        thread::sleep(Duration::from_millis(100));
        gpio.set_oscillate_watchdog(false).unwrap();
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
        gpio.gfi_reset().unwrap();
    }

    #[test]
    fn test_set_pilot_ampere() {
        let mut gpio = GPIO.lock().unwrap();
        gpio.set_pilot_ampere(32.0).unwrap();
    }

}