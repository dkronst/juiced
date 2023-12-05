// This file contains the different functionality that's associated with the 
// specifics of the EVSE. 

// Define the state machine of an AC EVSE:

use core::panic;
use std::{thread, time::Duration, sync::{atomic::{AtomicBool, Ordering}, Arc, Condvar, RwLock}};

use rppal::gpio::{Level, Trigger};
use rust_fsm::*;

use error_stack::{Context, Report, Result, ResultExt, ensure, report};

use log::{info, error, debug};

use crossbeam_channel::{Receiver, Select, bounded};

use crate::peripherals::GpioPeripherals;
use crate::sensors::SensorsState;

const MAINS_FREQUENCY_HZ: f64 = 50.; // Hz
const GFI_TEST_CYCLES: u64 = 10; // 10 cycles

// The states are summed up in the following table:
// 3. **The Different states** as defined by the SAE_J1772 std:
// | Base status       | Charging status   | Resistance, CP-PE | Resistance, R2 | Voltage, CP-PE |
// |-------------------|-------------------|-------------------|----------------|----------------|
// | Status A          | Standby           | Open, or ∞ Ω      |                | +12 V          |
// | Status B          | Vehicle detected  | 2740 Ω            |                | +9±1 V         |
// | Status C          | Ready (charging)  | 882 Ω             | 1300 Ω         | +6±1 V         |
// | Status D          | With ventilation  | 246 Ω             | 270 Ω          | +3±1 V         |
// | Status E          | No power (shutoff)|                   |                | 0 V            |
// | Status F          | Error             |                   |                | −12 V          |


// Define the State Machine:

// Define the possible states:

state_machine! {
    derive(Debug)
    EVSEMachine(SelfTest)

    SelfTest => {
        SelfTestOk => Standby [SelfTestOk],
        SelfTestFailed => FailedStation [SelfTestError],
        HardwareFault => FailedStation [HardwareFault],
    },
    Standby => {
        PilotIs12V => Standby [NoTransition],
        PilotIs9V => VehicleDetected [ConsiderCharging],
        PilotIs6V => ResetableError[Illegal],
        PilotIs3V => ResetableError[Illegal],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
        PilotIsNegative12V => FailedStation [HardwareFault],
    },
    VehicleDetected => {
        PilotIs12V => Standby [VehicleDisconnected],
        PilotIs9V => VehicleDetected [NoTransition],
        PilotIs6V => StartCharging [OfferCharging],
        PilotIs3V => VentilationNeeded,
        PilotIsNegative12V => NoPower,
        PilotInError => ResetableError [PilotMeasurement],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
    },
    StartCharging => {
        PilotInError => ResetableError [PilotMeasurement],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
        SelfTestOk => Charging [NoTransition],
        SelfTestFailed => ResetableError [SelfTestError],
    },
    Charging => {
        PilotIs12V => StopCharging [VehicleDisconnected],
        PilotIs9V => StopCharging [ChargingFinished],
        PilotIs6V => Charging [NoTransition],
        PilotIs3V => VentilationNeeded [UnsupportedVehicle],
        PilotIsNegative12V => StopCharging [NoPower],
        PilotInError => StopCharging [PilotMeasurement],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
    },
    StopCharging => {
        ChargingFinished => Standby [NoTransition],
        PilotInError => ResetableError [PilotMeasurement],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
    },
    NoPower => {
        PilotIsNegative12V => NoPower [ChargingFinished],
        HardwareFault => FailedStation [HardwareFault],
    },
    ResetableError => {
        PilotIs12V => Standby,    // This error can be reset by the user
    },
}

// The state machine is the continuation of the main thread - i.e. it 
// takes the foreground and is the main loop of the program. 
// The point here is to prevent safety issues by having a state machine
// fail and crash when inside a different thread - thus not disconnecting.

fn run_gfi_self_test(evse: &mut impl EVSEHardware) -> Result<EVSEMachineInput, HwError> {
    evse.set_ground_test_pin(OnOff::Off)?;
    evse.set_contactor(OnOff::Off)?;
    thread::sleep(Duration::from_millis(200));
    evse.gfi_reset()?;
    thread::sleep(Duration::from_millis(100));
    evse.reset_gfi_status_pin()?;   // Reset the interrupt handler and clear the pin
    debug!("GFI Status before self test: {:?}", evse.get_gfi_status_pin());
    if evse.get_gfi_status_pin() != Level::Low {
        return Err(Report::new(HwError::HardwareFault).attach_printable("GFI self test failed: GFI status pin is high before synthetic interrupt."))?;
    }
    
    for _ in 0..GFI_TEST_CYCLES {
        evse.set_ground_test_pin(OnOff::On)?;
        thread::sleep(Duration::from_secs_f64(1./(2.*MAINS_FREQUENCY_HZ))); // Wait for the required number of cycles
        evse.set_ground_test_pin(OnOff::Off)?;
        thread::sleep(Duration::from_secs_f64(1./(2.*MAINS_FREQUENCY_HZ))); // Wait for the required number of cycles
    }
    if evse.get_gfi_status_pin() == Level::Low {
        return Err(Report::new(HwError::HardwareFault).attach_printable("GFI self test failed: mock ground fault not detected."))?;
    }
    thread::sleep(Duration::from_millis(500));
    evse.gfi_reset()?;
    evse.reset_gfi_status_pin()?;

    thread::sleep(Duration::from_millis(100));
    if evse.get_gfi_status_pin() != Level::Low {
        return Err(Report::new(HwError::HardwareFault).attach_printable("GFI self test failed: mock ground fault was not cleared."))?;
    }
    thread::sleep(Duration::from_millis(100));
    if evse.get_gfi_status_pin() != Level::Low {
        return Err(Report::new(HwError::HardwareFault).attach_printable("GFI self test failed: mock ground fault is high after clearning."))?;
    }

    evse.reset_gfi_status_pin()?;


    Ok(EVSEMachineInput::SelfTestOk)
}

#[derive(Debug)]
pub enum Fault {
    GFIInterrupted,
    NoGround,
    PilotInError,
    InternalFaultThreadError,
}

fn get_pilot_state(min_max: (f32, f32)) -> EVSEMachineInput {
    let (vm12, voltage) = min_max;
    if (vm12 > -11.0 && vm12 > -13.0) && vm12 < 0.0 {    // If the pilot is oscilating?
        info!("Pilot minimum voltage is out of range: {}", vm12);
        return EVSEMachineInput::PilotInError;
    }
    if  13.0 > voltage && voltage > 11.0 {
        EVSEMachineInput::PilotIs12V
    } else if 10.0 > voltage && voltage > 8.0 {
        EVSEMachineInput::PilotIs9V
    } else if 7.0 > voltage && voltage > 5.0 {
        EVSEMachineInput::PilotIs6V
    } else if 4.0 > voltage && voltage > 2.0 {
        EVSEMachineInput::PilotIs3V
    } else if -11.0 > voltage && voltage > -13.0 {
        EVSEMachineInput::PilotIsNegative12V
    } else {
        info!("Pilot voltage is out of range: {}", voltage);
        EVSEMachineInput::PilotInError
    }
}

fn get_new_state_input(pilot_voltage_chan: Receiver<(f32, f32)>, fault_channel: Receiver<Fault>) -> EVSEMachineInput {
    // Selecting on the Fault channel and the Pilot channel - 
    // Both are async and require a response quickly. 
    let mut sel = Select::new();
    sel.recv(&pilot_voltage_chan);
    sel.recv(&fault_channel);
    let oper = sel.select();
    match oper.index() {
        0 => {
            let voltage = oper.recv(&pilot_voltage_chan);
            if let Err(_) = voltage {
                return EVSEMachineInput::PilotInError;
            }
            debug!("Pilot voltage: {:?}", voltage);
            let voltage: (f32, f32) = voltage.unwrap();
            get_pilot_state(voltage)
        },
        1 => {
            // Fault channel
            match oper.recv(&fault_channel) {
                Ok(fault) => {
                    error!("Fault: {:?}", fault);
                    match fault {
                        Fault::GFIInterrupted => EVSEMachineInput::GFIInterrupted,
                        Fault::NoGround => EVSEMachineInput::NoGround,
                        Fault::PilotInError => EVSEMachineInput::PilotInError,
                        Fault::InternalFaultThreadError => EVSEMachineInput::HardwareFault,
                    }
                },
                Err(_) => {
                    info!("Fault channel closed");
                    EVSEMachineInput::PilotInError
                }
            }
        },
        _ => {
            panic!("Invalid index");
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum OnOff {
    On,
    Off,
}

#[derive(Debug)]
pub enum HwError {
    PoisonError,
    GpioError,
    HardwareFault,
    InternalFaultThreadError,
}

impl Context for HwError {}

impl std::fmt::Display for HwError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f,"Hardware Error Detected")
    }
}

fn start_adc_thread(periph: &mut GpioPeripherals, listen_to_pilot: Arc<AtomicBool>, sensors_state: Arc<RwLock<SensorsState>>) -> Result<Receiver<(f32, f32)>, HwError> {
    let (pilot_tx, pilot_rx) = bounded(16);
    let adc = periph.get_adc();
    thread::spawn(move || {
        loop {
            let (min, max, current, voltage);
            {
                let locked_adc = adc.lock().unwrap();
                (min, max) = locked_adc.peak_to_peak_pilot().unwrap();
                current = locked_adc.read_current_sense_rms().unwrap();
                voltage = locked_adc.peak_mains_voltage().unwrap();
            }
            thread::sleep(Duration::from_millis(200));
            if listen_to_pilot.load(Ordering::Relaxed) {
                match pilot_tx.send((min, max)) {
                    Err(_) => {
                        error!("Pilot channel closed");
                        break;
                    },
                    _ => {}
                }
            }
            {
                let mut locked_sensors_state = sensors_state.write().unwrap();
                locked_sensors_state.add_cs_reading(current as f64);
                locked_sensors_state.add_mains_peak_reading(voltage as f64);
            }
        }
    });
    Ok(pilot_rx)
}

fn start_fault_thread(periph: &mut GpioPeripherals) -> Result<Receiver<Fault>, HwError> {
    // TODO: This is wrong. fix to not lose the channel.
    let (fault_tx, fault_rx) = bounded(16);
    let pins = periph.get_pins();
    
    thread::spawn(move || {
        let res = || -> Result<(), HwError> {
            let cond = Arc::new(Condvar::new());
            loop {
                let fault_tx = fault_tx.clone();
                let cond = Arc::clone(&cond);
                {
                    let mut locked_pins = pins.lock().
                        map_err(|e| Report::new(HwError::PoisonError).attach_printable(format!("failed locking pins: {:?}", e)))?;

                    let cond_clone = Arc::clone(&cond);
                    locked_pins.gfi_status_pin.set_async_interrupt(Trigger::RisingEdge, move |_| {
                        cond_clone.notify_all();
                    }).change_context(HwError::GpioError)?;
                    let locked_pins = cond.wait(locked_pins).map_err(
                        |e| Report::new(HwError::PoisonError).attach_printable(format!("failed waiting on condvar: {:?}", e))
                    )?;
                    let gfi_status = locked_pins.gfi_status_pin.is_high();
                    let contactor = locked_pins.contactor_pin.is_set_high();
                    if gfi_status && contactor {
                        fault_tx.send(Fault::GFIInterrupted)
                            .map_err(|e| Report::new(HwError::PoisonError).attach_printable(format!("failed sending fault: {:?}", e)))?;
                    } else if !contactor {
                        debug!("GFI Interrupted, but contactor is off. Ignoring.");
                    }
                }
            }
        }();
        if let Err(e) = res {
            error!("Error: {:?}", e);
            fault_tx.send(Fault::InternalFaultThreadError).unwrap();
        }
    });
    
    Ok(fault_rx)
}


pub trait EVSEHardware {
    fn set_contactor(&mut self, state: OnOff) -> Result<(), HwError>;
    fn set_current_offer_ampere(&mut self, ampere: f32) -> Result<(), HwError>;
    fn set_ground_test_pin(&mut self, state: OnOff) -> Result<(), HwError>;
    fn get_contactor_state(&mut self) -> Result<OnOff, HwError>;
    fn get_peripherals(&mut self) -> &mut GpioPeripherals;

    fn get_gfi_status_pin(&mut self) -> Level {
        self.get_peripherals().read_gfi_status_pin()
    }

    fn get_relay_test_pin(&mut self) -> Level {
        self.get_peripherals().read_relay_test_pin()
    }

    fn gfi_reset(&mut self) -> Result<(), HwError> {
        self.get_peripherals().gfi_reset().change_context(HwError::HardwareFault)?;
        Ok(())
    }

    fn reset_gfi_status_pin(&mut self) -> Result<(), HwError> {
        self.get_peripherals().reset_gfi_status_pin().change_context(HwError::HardwareFault)?;
        Ok(())
    }

    fn set_waiting_for_vehicle(&mut self) -> Result<(), HwError> {
        debug!("Setting waiting for vehicle");
        self.set_contactor(OnOff::Off)?;
        self.get_peripherals().set_waiting_for_vehicle().change_context(HwError::HardwareFault)?;
        self.set_ground_test_pin(OnOff::Off)?;
        Ok(())
    }

    const MAX_CURRENT_OFFER: f32 = 32.0;
}

pub struct EVSEHardwareImpl {
    contactor: OnOff,
    current_offer: f32,
    ground_test_pin: OnOff,
    hw_peripherals: GpioPeripherals,
}

impl From<OnOff> for Level {
    fn from(state: OnOff) -> Self {
        match state {
            OnOff::On => Level::High,
            OnOff::Off => Level::Low,
        }
    }
}

impl From<Level> for OnOff {
    fn from(state: Level) -> Self {
        match state {
            Level::High => OnOff::On,
            Level::Low => OnOff::Off,
        }
    }
}

impl EVSEHardwareImpl {
    pub fn new() -> Self {
        let mut ret = Self {
            contactor: OnOff::Off,
            current_offer: 0.0,
            ground_test_pin: OnOff::Off,
            hw_peripherals: GpioPeripherals::new(),
        };
        ret.set_contactor(OnOff::Off).unwrap();
        ret.set_current_offer_ampere(0.0).unwrap();
        ret.set_ground_test_pin(OnOff::Off).unwrap();
        ret
    }
}

impl EVSEHardware for EVSEHardwareImpl {
    fn set_contactor(&mut self, state: OnOff) -> Result<(), HwError> {
        self.contactor = state.clone();
        let oscillate = match state {
            OnOff::On => true,
            OnOff::Off => false
        };

        if oscillate {
            info!("Setting power watchdog to oscillate");
            self.hw_peripherals.set_oscillate_watchdog(oscillate).change_context(HwError::HardwareFault)
                    .attach_printable("Failed to set power watchdog to oscillate")?;
        }
        self.hw_peripherals.set_contactor_pin(state.clone().into());
        if !oscillate {
            info!("Stopping power watchdog");
            self.hw_peripherals.set_oscillate_watchdog(oscillate).change_context(HwError::HardwareFault)
                    .attach_printable("Failed to stop power watchdog")?;
        }
        Ok(())
    }

    fn set_current_offer_ampere(&mut self, ampere: f32) -> Result<(), HwError> {
        self.hw_peripherals.set_pilot_ampere(ampere).change_context(HwError::HardwareFault)?;
        self.current_offer = ampere;
        Ok(())
    }

    fn set_ground_test_pin(&mut self, state: OnOff) -> Result<(), HwError> {
        self.ground_test_pin = state.clone();
        self.hw_peripherals.set_gfi_test_pin(state.into());
        Ok(())
    }

    fn get_contactor_state(&mut self) -> Result<OnOff, HwError> {
        Ok(self.hw_peripherals.read_relay_test_pin().into())
    }

    fn get_peripherals(&mut self) -> &mut GpioPeripherals {
        &mut self.hw_peripherals
    }
}

fn do_state_transition<T>(state: &EVSEMachineState, hw: &mut T, listen_to_pilot: &Arc<AtomicBool>) -> Result<Option<EVSEMachineInput>, HwError> 
where T: EVSEHardware
{
    let mut next_state_input: Option<EVSEMachineInput> = Option::None;
    debug!("Do state transition: {:?}", state);
    match state {
        EVSEMachineState::Standby => {
            hw.set_contactor(OnOff::Off)?;
            hw.set_waiting_for_vehicle()?;

            thread::sleep(Duration::from_millis(1000));
            listen_to_pilot.store(true, Ordering::Relaxed);
            ensure!(hw.get_relay_test_pin() == Level::Low, HwError::HardwareFault);
        },
        EVSEMachineState::VehicleDetected => {
            hw.set_contactor(OnOff::Off)?;
            thread::sleep(Duration::from_millis(100));
            ensure!(hw.get_relay_test_pin() == Level::Low, HwError::HardwareFault);
            hw.set_current_offer_ampere(T::MAX_CURRENT_OFFER)?;
            hw.set_ground_test_pin(OnOff::Off)?;
        },
        EVSEMachineState::StartCharging => {
            thread::sleep(Duration::from_secs(1)); // Protocol requires 1 second of waiting for the vehicle to be ready
            ensure!(hw.get_relay_test_pin() == Level::Low, report!(HwError::HardwareFault).attach_printable("Incorrect relay test pin state"));
            hw.set_contactor(OnOff::Off)?;
            thread::sleep(Duration::from_millis(200));
            next_state_input = Some(EVSEMachineInput::SelfTestOk);
            let state = hw.get_contactor_state()?;
            debug!("Machine is set to 'Charging'. Contactor state: {:?}", state);
            if OnOff::Off == state {
                info!("Contactor is off. Running GFI self test.");
                let self_test_result = run_gfi_self_test(hw)?;
                match self_test_result {
                    EVSEMachineInput::SelfTestFailed => {
                        return Err(Report::new(HwError::HardwareFault).attach_printable("GFI self test failed before power on"));
                    },
                    _ => {
                        info!("Self test passed. Turning on contactor.");
                        hw.set_contactor(OnOff::On)?;
                        thread::sleep(Duration::from_millis(200));
                    }
                }
            }
            // At this point, the contactor is On and the mains should be connected.
            // Make sure that the 220V is high
            ensure!(hw.get_relay_test_pin() == Level::High, HwError::HardwareFault);
        },
        EVSEMachineState::Charging => {
            // Basically, there's nothing to do, for now. Later, we'll need to update the UI etc.
            // The only thing we need to do is to make sure that the contactor is on.
            ensure!(hw.get_relay_test_pin() == Level::High, report!(HwError::HardwareFault).attach_printable("Relay should be on when charging"));
        },
        EVSEMachineState::StopCharging => {
            hw.set_current_offer_ampere(0.)?;
            thread::sleep(Duration::from_secs(2)); // Disconnecting requires that the vehicle stops the charge cycle of else the contactor will be damaged
            hw.set_contactor(OnOff::Off)?;
            thread::sleep(Duration::from_millis(100));
            ensure!(hw.get_relay_test_pin() == Level::Low, report!(HwError::HardwareFault).attach_printable("Relay should be off when charging is stopped"));
            next_state_input = Some(EVSEMachineInput::ChargingFinished);
        },
        EVSEMachineState::NoPower => {
            hw.set_current_offer_ampere(0.)?;
            hw.set_contactor(OnOff::Off)?;
            hw.set_ground_test_pin(OnOff::Off)?;
            ensure!(hw.get_relay_test_pin() == Level::High, HwError::HardwareFault);
        }
        _ => {
            hw.set_contactor(OnOff::Off)?;
            ensure!(hw.get_relay_test_pin() == Level::High, HwError::HardwareFault);
            // Do nothing otherwise
        }
    }
    debug!("State transition done");
    Ok(next_state_input)
}

pub fn start_machine<T>(mut evse: T) -> !
where T: EVSEHardware + Send + Sync
{
    let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
    let liste_to_pilot = Arc::new(AtomicBool::new(false));
    let sensor_state = Arc::new(RwLock::new(SensorsState::new()));

    let pilot_voltage_chan = start_adc_thread(
        evse.get_peripherals(),
        Arc::clone(&liste_to_pilot),
        Arc::clone(&sensor_state)
    ).unwrap();
    let fault_chan = start_fault_thread(evse.get_peripherals()).unwrap();

    loop {
        let mut state_input = EVSEMachineInput::PilotInError;
        match machine.state() {
            EVSEMachineState::SelfTest => {
                // Do the self test
                let self_test_result = run_gfi_self_test(&mut evse);
                liste_to_pilot.store(false, Ordering::Relaxed);
                match self_test_result {
                    Err(e) => {
                        state_input = EVSEMachineInput::SelfTestFailed;
                        error!("Self test failed: {:?}", e);
                    },
                    Ok(EVSEMachineInput::SelfTestOk) => {
                        state_input = EVSEMachineInput::SelfTestOk;
                        info!("Self test passed");
                    },
                    Ok(e) => {
                        error!("Self test returned an unexpected result: {:?}", e);
                        state_input = e;
                    }
                }
            },
            EVSEMachineState::ResetableError => {
                // todo!("Reset the error");
            },
            EVSEMachineState::FailedStation => {
                error!("Station failed. Full reset required, contact admin if the issue persists.");
                panic!("Fatal Error");
            },
            state => {
                // Measure pilot voltage and feed the machine
                // pilot measurements are done on another thread. If a timeout
                // occurs, the machine will be fed with a PilotInError input and 
                // any charging will be stopped immediately.
                // Other possible inputs here are:
                // - GFIInterrupted
                // - NoGround
                // which are fatal since such errors are, in theory, indications of
                // a faulty installation or a hardware failure.

                // select on 2 channels - one for the pilot voltage and one for the
                // GFI/NoGround errors. The latter are performed by listening to GPIOs 
                // while the former is done every 10ms (at most) by the pilot measurement thread.
                // We will decide on a pilot error, or we'll get one from the channel if 
                // a timeout occurs, or an actual error (wrong voltage, for example) occurs.
                
                let res = do_state_transition(state, &mut evse, &liste_to_pilot);
                match res {
                    Err(e) => {
                        error!("Error: {:?}", e);
                        state_input = EVSEMachineInput::HardwareFault;
                        // Turn off the contactor, if we fail, we'll try again in the state transition
                        // This is anyway only a secondary safety measure, since the hardware is already hard-wired
                        // to turn off the contactor immediately if a something dangerous happens (no software), 
                        // such as GFI for example.
                        evse.set_contactor(OnOff::Off).unwrap_or(());
                    },
                    Ok(ok) => {
                        match ok {
                            Some(input) => {
                                state_input = input;
                            },
                            None => {
                                state_input = get_new_state_input(pilot_voltage_chan.clone(), fault_chan.clone());
                            }
                        }
                    }
                }
                debug!("Current sensor state: {}", sensor_state.read().unwrap());
            }
        }
        debug!("State input: {:?}", state_input);
        // TODO: Don't perform state change if there is no transition
        let output = machine.consume(&state_input);
        debug!("Output: {:?}", output);
        debug!("State: {:?}", machine.state());
    } // loop
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_sm() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output);
        info!("State: {:?}", machine.state());
    }

    #[test]
    fn test_self_test() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestFailed).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::FailedStation));
    }

    #[test]
    fn test_standby() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
    }

    #[test]
    fn test_gfi_interrupted() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::GFIInterrupted).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::FailedStation));
    }

    #[test]
    fn test_no_ground() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::NoGround).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::FailedStation));
    }

    #[test]
    fn test_start_charging() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
        let output = machine.consume(&EVSEMachineInput::PilotIs6V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Charging));
    }

    #[test]
    fn test_vehicle_left() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
        let output = machine.consume(&EVSEMachineInput::PilotIs12V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
    }

    #[test]
    fn test_charging_finished() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
        let output = machine.consume(&EVSEMachineInput::PilotIs6V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Charging));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
    }
}