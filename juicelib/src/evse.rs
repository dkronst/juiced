// This file contains the different functionality that's associated with the 
// specifics of the EVSE. 

// Define the state machine of an AC EVSE:

use std::{thread, time::Duration, sync::{atomic::{AtomicBool, Ordering}, Arc, RwLock}};

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
        ChargingFinished => NoPower [ForcedTransition],  // This is required to reset the charging cycle on the vehicle side
        PilotInError => ResetableError [PilotMeasurement],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
    },
    NoPower => {
        PilotIsNegative12V => NoPower [ChargingFinished],
        WaitComplete => Standby [ForcedTransition],
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
    // Missing-diode / oscillating-pilot integrity check: when the pilot is
    // oscillating (i.e. we are offering charging), the minimum must reach
    // near -12 V. A negative-but-shallow minimum (e.g. -5 V) means the
    // protection diode is gone. Only enforce when the max is positive so we
    // do not flag a steady -12 V fault-state pilot.
    if voltage > 0.0 && vm12 > -11.0 {
        info!("Pilot minimum voltage is out of range: {}", vm12);
        return EVSEMachineInput::PilotInError;
    }
    // Classify by max voltage. Bands meet at the midpoint of each adjacent
    // J1772 level so a brief sample during a vehicle transition cannot land
    // in a gap and produce a spurious PilotInError.
    if voltage > 13.5 {
        info!("Pilot voltage above spec: {}", voltage);
        EVSEMachineInput::PilotInError
    } else if voltage > 10.5 {
        EVSEMachineInput::PilotIs12V
    } else if voltage > 7.5 {
        EVSEMachineInput::PilotIs9V
    } else if voltage > 4.5 {
        EVSEMachineInput::PilotIs6V
    } else if voltage > 1.5 {
        EVSEMachineInput::PilotIs3V
    } else if voltage > -10.5 {
        // Voltages near zero or shallow negatives — the pilot is in a bad
        // place that does not correspond to any J1772 level.
        info!("Pilot voltage is out of range: {}", voltage);
        EVSEMachineInput::PilotInError
    } else if voltage >= -13.5 {
        EVSEMachineInput::PilotIsNegative12V
    } else {
        info!("Pilot voltage below spec: {}", voltage);
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
            match oper.recv(&pilot_voltage_chan) {
                Ok(voltage) => {
                    debug!("Pilot voltage: {:?}", voltage);
                    get_pilot_state(voltage)
                },
                Err(_) => {
                    // The ADC thread closed the channel — the sensing subsystem
                    // is dead. Route this to HardwareFault so we land in
                    // FailedStation rather than drifting through StopCharging
                    // with a `PilotInError` that looks vehicle-driven.
                    error!("Pilot channel closed; ADC thread is gone");
                    EVSEMachineInput::HardwareFault
                },
            }
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
        _ => unreachable!("Select returned an index outside 0..2"),
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

fn start_adc_thread(
    periph: &mut GpioPeripherals,
    listen_to_pilot: Arc<AtomicBool>,
    sensors_state: Arc<RwLock<SensorsState>>,
    fault_tx: crossbeam_channel::Sender<Fault>,
) -> Result<Receiver<(f32, f32)>, HwError> {
    let (pilot_tx, pilot_rx) = bounded(16);
    let adc = periph.get_adc();
    thread::spawn(move || {
        loop {
            let (min, max, current, voltage);
            {
                let locked_adc = match adc.lock() {
                    Ok(g) => g,
                    Err(e) => {
                        error!("ADC mutex poisoned: {:?}", e);
                        let _ = fault_tx.send(Fault::InternalFaultThreadError);
                        break;
                    }
                };
                match (
                    locked_adc.peak_to_peak_pilot(),
                    locked_adc.read_current_sense_rms(),
                    locked_adc.peak_mains_voltage(),
                ) {
                    (Ok(p2p), Ok(cs), Ok(mv)) => {
                        (min, max) = p2p;
                        current = cs;
                        voltage = mv;
                    },
                    (p2p, cs, mv) => {
                        error!("ADC read failed: p2p={:?} cs={:?} mv={:?}", p2p.err(), cs.err(), mv.err());
                        let _ = fault_tx.send(Fault::InternalFaultThreadError);
                        break;
                    }
                }
            }
            thread::sleep(Duration::from_millis(200));
            if listen_to_pilot.load(Ordering::Relaxed) {
                if let Err(_) = pilot_tx.send((min, max)) {
                    error!("Pilot channel closed");
                    break;
                }
            }
            match sensors_state.write() {
                Ok(mut locked_sensors_state) => {
                    locked_sensors_state.add_cs_reading(current as f64);
                    locked_sensors_state.add_mains_peak_reading(voltage as f64);
                },
                Err(e) => {
                    error!("Sensors state RwLock poisoned: {:?}", e);
                    let _ = fault_tx.send(Fault::InternalFaultThreadError);
                    break;
                }
            }
        }
    });
    Ok(pilot_rx)
}

fn start_fault_thread(
    periph: &mut GpioPeripherals,
    fault_tx: crossbeam_channel::Sender<Fault>,
) -> Result<(), HwError> {
    let pins = periph.get_pins();

    thread::spawn(move || {
        // The interrupt closure pushes a unit token onto this channel; the
        // loop blocks on `recv`. Channels queue, so notifications fired
        // before the loop reaches `recv` are *not* lost the way a condvar's
        // would have been.
        let (event_tx, event_rx) = bounded::<()>(16);
        let res = || -> Result<(), HwError> {
            // Install the interrupt exactly once. Repeatedly re-installing
            // it (as the previous design did) is wasteful and leaks
            // closures inside rppal.
            {
                let mut locked_pins = pins.lock()
                    .map_err(|e| Report::new(HwError::PoisonError).attach_printable(format!("failed locking pins: {:?}", e)))?;
                let event_tx_cb = event_tx.clone();
                locked_pins.gfi_status_pin.set_async_interrupt(Trigger::RisingEdge, move |_| {
                    let _ = event_tx_cb.try_send(());
                }).change_context(HwError::GpioError)?;
                // If the GFI is already latched high before we installed the
                // handler, no rising edge will fire — synthesize one event so
                // we evaluate the fault on the next iteration.
                if locked_pins.gfi_status_pin.is_high() {
                    let _ = event_tx.try_send(());
                }
            }

            loop {
                event_rx.recv().map_err(|e| {
                    Report::new(HwError::PoisonError)
                        .attach_printable(format!("gfi event channel closed: {:?}", e))
                })?;
                let (gfi_status, contactor) = {
                    let locked_pins = pins.lock()
                        .map_err(|e| Report::new(HwError::PoisonError).attach_printable(format!("failed locking pins: {:?}", e)))?;
                    (locked_pins.gfi_status_pin.is_high(), locked_pins.contactor_pin.is_set_high())
                };
                if gfi_status && contactor {
                    fault_tx.send(Fault::GFIInterrupted)
                        .map_err(|e| Report::new(HwError::PoisonError).attach_printable(format!("failed sending fault: {:?}", e)))?;
                } else if !contactor {
                    debug!("GFI Interrupted, but contactor is off. Ignoring.");
                }
            }
        }();
        if let Err(e) = res {
            error!("Error: {:?}", e);
            if let Err(send_err) = fault_tx.send(Fault::InternalFaultThreadError) {
                error!("Failed to forward fault thread error (channel closed): {:?}", send_err);
            }
        }
    });

    Ok(())
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
    pub fn new() -> Result<Self, HwError> {
        let mut ret = Self {
            contactor: OnOff::Off,
            ground_test_pin: OnOff::Off,
            hw_peripherals: GpioPeripherals::new().change_context(HwError::HardwareFault)?,
        };
        ret.set_contactor(OnOff::Off)?;
        ret.set_current_offer_ampere(0.0)?;
        ret.set_ground_test_pin(OnOff::Off)?;
        Ok(ret)
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
        self.hw_peripherals.set_contactor_pin(state.clone().into()).change_context(HwError::HardwareFault)?;
        if !oscillate {
            debug!("Stopping power watchdog");
            self.hw_peripherals.set_oscillate_watchdog(oscillate).change_context(HwError::HardwareFault)
                    .attach_printable("Failed to stop power watchdog")?;
        }
        Ok(())
    }

    fn set_current_offer_ampere(&mut self, ampere: f32) -> Result<(), HwError> {
        self.hw_peripherals.set_pilot_ampere(ampere).change_context(HwError::HardwareFault)?;
        Ok(())
    }

    fn set_ground_test_pin(&mut self, state: OnOff) -> Result<(), HwError> {
        self.ground_test_pin = state.clone();
        self.hw_peripherals.set_gfi_test_pin(state.into()).change_context(HwError::HardwareFault)?;
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
            let state = hw.get_contactor_state()?;
            debug!("Machine is set to 'Charging'. Contactor state: {:?}", state);
            if OnOff::Off == state {
                info!("Contactor is off. Running GFI self test.");
                let self_test_result = run_gfi_self_test(hw)?;
                match self_test_result {
                    EVSEMachineInput::SelfTestOk => {
                        info!("Self test passed. Turning on contactor.");
                        hw.set_contactor(OnOff::On)?;
                        thread::sleep(Duration::from_millis(200));
                    },
                    other => {
                        return Err(Report::new(HwError::HardwareFault)
                            .attach_printable(format!("GFI self test did not return SelfTestOk: {:?}", other)));
                    }
                }
            }
            // At this point, the contactor is On and the mains should be connected.
            // Make sure that the 220V is high
            ensure!(hw.get_relay_test_pin() == Level::High, HwError::HardwareFault);
            // Only now is it safe to advance the FSM into `Charging`.
            next_state_input = Some(EVSEMachineInput::SelfTestOk);
        },
        EVSEMachineState::Charging => {
            // Basically, there's nothing to do, for now. Later, we'll need to update the UI etc.
            // The only thing we need to do is to make sure that the contactor is on.
            ensure!(hw.get_relay_test_pin() == Level::High, report!(HwError::HardwareFault).attach_printable("Relay should be on when charging"));
        },
        EVSEMachineState::StopCharging => {
            // Per spec §138 the contactor must NOT open while the vehicle is
            // still drawing current. Drop the pilot offer to 0 first so the
            // vehicle observes the signal and winds its draw down to near
            // zero, and only then break the contactor.
            listen_to_pilot.store(false, Ordering::Relaxed);
            hw.set_current_offer_ampere(0.)?;
            thread::sleep(Duration::from_secs(2)); // wind-down for vehicle current
            hw.set_contactor(OnOff::Off)?;
            thread::sleep(Duration::from_secs(3)); // inter-cycle reset on vehicle side
            ensure!(hw.get_relay_test_pin() == Level::Low, report!(HwError::HardwareFault).attach_printable("Relay should be off when charging is stopped"));
            next_state_input = Some(EVSEMachineInput::ChargingFinished);
        },
        EVSEMachineState::NoPower => {
            // This is a transient state in which we tell the vehicle that we want to reset the charging
            // cycle.
            hw.set_contactor(OnOff::Off)?;
            hw.set_ground_test_pin(OnOff::Off)?;
            thread::sleep(Duration::from_millis(100)); // Waiting for the contactor to turn off completely
            ensure!(hw.get_relay_test_pin() == Level::Low, report!(HwError::HardwareFault).attach_printable("Relay should now be off"));
            info!("Waiting 5s for the vehicle to stop charging. (NoPower state)");
            thread::sleep(Duration::from_secs(5));
            next_state_input = Some(EVSEMachineInput::WaitComplete);   // After this, we'll go to Standby starting the cycle again.
        }
        _ => {
            hw.set_contactor(OnOff::Off)?;
            thread::sleep(Duration::from_millis(100)); // 100 ms grace per spec line 55
            ensure!(
                hw.get_relay_test_pin() == Level::Low,
                report!(HwError::HardwareFault).attach_printable("Relay must drop Low within 100 ms of contactor off")
            );
        }
    }
    debug!("State transition done. Next state input: {:?}, current state: {:?}", next_state_input, state);
    Ok(next_state_input)
}

pub fn start_machine<T>(mut evse: T) -> Result<(), HwError>
where T: EVSEHardware + Send + Sync
{
    let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
    let listen_to_pilot = Arc::new(AtomicBool::new(false));
    let sensor_state = Arc::new(RwLock::new(SensorsState::new()));

    let (fault_tx, fault_chan) = bounded::<Fault>(16);
    let pilot_voltage_chan = start_adc_thread(
        evse.get_peripherals(),
        Arc::clone(&listen_to_pilot),
        Arc::clone(&sensor_state),
        fault_tx.clone(),
    )?;
    start_fault_thread(evse.get_peripherals(), fault_tx)?;

    loop {
        let state_input: EVSEMachineInput;
        match machine.state() {
            EVSEMachineState::SelfTest => {
                // Do the self test
                let self_test_result = run_gfi_self_test(&mut evse);
                listen_to_pilot.store(false, Ordering::Relaxed);
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
                // Hold safely until the user drives the pilot to +12 V (i.e.
                // unplugs the vehicle), which transitions us back to Standby.
                // Without blocking on real input here we would busy-loop the
                // outer state-machine match at 100% CPU.
                state_input = get_new_state_input(
                    pilot_voltage_chan.clone(),
                    fault_chan.clone(),
                );
            },
            EVSEMachineState::FailedStation => {
                error!("Station failed. Full reset required, contact admin if the issue persists.");
                // Safe idle: hold the process alive instead of crashing into a 60 s
                // systemd restart cycle. Contactor off, pilot to error (-12 V), watchdog off.
                if let Err(e) = evse.set_contactor(OnOff::Off) {
                    error!("FailedStation: failed to open contactor: {:?}", e);
                }
                if let Err(e) = evse.get_peripherals().set_pilot_ampere(0.0) {
                    error!("FailedStation: failed to drive pilot to error: {:?}", e);
                }
                listen_to_pilot.store(false, Ordering::Relaxed);
                loop {
                    thread::sleep(Duration::from_secs(60));
                }
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
                
                let res = do_state_transition(state, &mut evse, &listen_to_pilot);
                match res {
                    Err(e) => {
                        error!("Error: {:?}", e);
                        state_input = EVSEMachineInput::HardwareFault;
                        // Turn off the contactor, if we fail, we'll try again in the state transition
                        // This is anyway only a secondary safety measure, since the hardware is already hard-wired
                        // to turn off the contactor immediately if a something dangerous happens (no software), 
                        // such as GFI for example.
                        if let Err(e2) = evse.set_contactor(OnOff::Off) {
                            error!("Failed to open contactor on fault path: {:?}", e2);
                        }
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
                if let Ok(state) = sensor_state.read() {
                    debug!("Current sensor state: {}", state);
                }
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
        assert!(matches!(machine.state(), EVSEMachineState::StartCharging));
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
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
        assert!(matches!(machine.state(), EVSEMachineState::StartCharging));
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Charging));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::StopCharging));
        let output = machine.consume(&EVSEMachineInput::ChargingFinished).unwrap();
        info!("Output: {:?}", output.unwrap());
        info!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::NoPower));
    }

    // ----- Bug-driven tests (CTF review) -----
    //
    // The tests below were written to fail on the buggy code described in
    // the audit; once the corresponding fixes land, they should pass.

    use std::time::Instant;

    struct MockHardware {
        contactor: OnOff,
        relay_test_pin: Level,
        gfi_status_pin: Level,
        gfi_test_pin: OnOff,
        current_offer: f32,
        pilot_waiting: bool,
        events: Vec<(Instant, String)>,
    }

    impl MockHardware {
        fn new() -> Self {
            Self {
                contactor: OnOff::Off,
                relay_test_pin: Level::Low,
                gfi_status_pin: Level::Low,
                gfi_test_pin: OnOff::Off,
                current_offer: 0.0,
                pilot_waiting: false,
                events: Vec::new(),
            }
        }

        fn record(&mut self, event: String) {
            self.events.push((Instant::now(), event));
        }

        fn time_of(&self, event: &str) -> Option<Instant> {
            self.events.iter().find(|(_, e)| e == event).map(|(t, _)| *t)
        }
    }

    impl EVSEHardware for MockHardware {
        fn set_contactor(&mut self, state: OnOff) -> Result<(), HwError> {
            self.record(format!("set_contactor({:?})", state));
            self.contactor = state.clone();
            self.relay_test_pin = match state {
                OnOff::On => Level::High,
                OnOff::Off => Level::Low,
            };
            Ok(())
        }

        fn set_current_offer_ampere(&mut self, ampere: f32) -> Result<(), HwError> {
            self.record(format!("set_current_offer({})", ampere));
            self.current_offer = ampere;
            Ok(())
        }

        fn set_ground_test_pin(&mut self, state: OnOff) -> Result<(), HwError> {
            self.record(format!("set_ground_test_pin({:?})", state));
            self.gfi_test_pin = state;
            Ok(())
        }

        fn get_contactor_state(&mut self) -> Result<OnOff, HwError> {
            Ok(self.relay_test_pin.into())
        }

        fn get_peripherals(&mut self) -> &mut GpioPeripherals {
            unimplemented!("MockHardware does not back GpioPeripherals; \
                            do_state_transition must not call get_peripherals \
                            for any state under test")
        }

        // Override every default impl that goes through get_peripherals so the
        // mock does not panic.
        fn get_relay_test_pin(&mut self) -> Level {
            self.relay_test_pin
        }

        fn get_gfi_status_pin(&mut self) -> Level {
            self.gfi_status_pin
        }

        fn gfi_reset(&mut self) -> Result<(), HwError> {
            self.record("gfi_reset".to_string());
            self.gfi_status_pin = Level::Low;
            Ok(())
        }

        fn reset_gfi_status_pin(&mut self) -> Result<(), HwError> {
            self.record("reset_gfi_status_pin".to_string());
            self.gfi_status_pin = Level::Low;
            Ok(())
        }

        fn set_waiting_for_vehicle(&mut self) -> Result<(), HwError> {
            self.record("set_waiting_for_vehicle".to_string());
            self.pilot_waiting = true;
            self.contactor = OnOff::Off;
            self.gfi_test_pin = OnOff::Off;
            Ok(())
        }
    }

    /// Bug 1: the catch-all `_` arm of `do_state_transition` turns the
    /// contactor off and then ensures the relay-test pin is High — backwards.
    /// Hitting `VentilationNeeded` (vehicle status D, 3 V) should not blow up.
    #[test]
    fn test_ventilation_needed_does_not_self_destruct() {
        let mut mock = MockHardware::new();
        // Pretend we arrived from Charging with the contactor on.
        mock.contactor = OnOff::On;
        mock.relay_test_pin = Level::High;

        let listen = Arc::new(AtomicBool::new(true));
        let result = do_state_transition(
            &EVSEMachineState::VentilationNeeded,
            &mut mock,
            &listen,
        );

        assert!(
            result.is_ok(),
            "VentilationNeeded should not return HwError. result = {:?}\nevents = {:?}",
            result, mock.events,
        );
    }

    /// Bug 2: `get_pilot_state` has 1 V dead bands between every level.
    /// A 50 ms sample landing in a gap during a legitimate transition
    /// returns `PilotInError`, which can force a mid-charge `StopCharging`.
    #[test]
    fn test_pilot_state_gap_voltages_not_pilot_error() {
        // Between PilotIs6V (5..7) and PilotIs9V (8..10).
        let input = get_pilot_state((-12.0, 7.5));
        assert!(
            !matches!(input, EVSEMachineInput::PilotInError),
            "7.5 V should be classified, not PilotInError. Got {:?}",
            input
        );

        // Between PilotIs3V (2..4) and PilotIs6V (5..7).
        let input = get_pilot_state((-12.0, 4.5));
        assert!(
            !matches!(input, EVSEMachineInput::PilotInError),
            "4.5 V should be classified, not PilotInError. Got {:?}",
            input
        );

        // Between PilotIs9V (8..10) and PilotIs12V (11..13).
        let input = get_pilot_state((-12.0, 10.5));
        assert!(
            !matches!(input, EVSEMachineInput::PilotInError),
            "10.5 V should be classified, not PilotInError. Got {:?}",
            input
        );

        // Boundary inside an existing band — sanity that we did not over-correct.
        assert!(matches!(get_pilot_state((-12.0, 12.0)), EVSEMachineInput::PilotIs12V));
        assert!(matches!(get_pilot_state((-12.0,  9.0)), EVSEMachineInput::PilotIs9V));
        assert!(matches!(get_pilot_state((-12.0,  6.0)), EVSEMachineInput::PilotIs6V));
        assert!(matches!(get_pilot_state((-12.0,  3.0)), EVSEMachineInput::PilotIs3V));
        assert!(matches!(get_pilot_state((-12.0, -12.0)), EVSEMachineInput::PilotIsNegative12V));
    }

    /// Bug 7: `StopCharging` opens the contactor immediately after dropping the
    /// pilot offer to 0 — under load. Per spec §138 the vehicle must be given
    /// time to wind down before the contactor breaks the circuit.
    #[test]
    fn test_stop_charging_waits_before_opening_contactor() {
        let mut mock = MockHardware::new();
        // Entering StopCharging from Charging: contactor on, relay test high.
        mock.contactor = OnOff::On;
        mock.relay_test_pin = Level::High;

        let listen = Arc::new(AtomicBool::new(true));
        let result = do_state_transition(
            &EVSEMachineState::StopCharging,
            &mut mock,
            &listen,
        );

        assert!(result.is_ok(), "StopCharging returned error: {:?}", result);

        let offer_zero = mock
            .time_of("set_current_offer(0)")
            .expect("set_current_offer(0) was never called");
        let contactor_off = mock
            .time_of("set_contactor(Off)")
            .expect("set_contactor(Off) was never called");

        assert!(
            offer_zero < contactor_off,
            "Pilot offer must drop to 0 strictly before opening the contactor"
        );

        let wind_down = contactor_off.duration_since(offer_zero);
        assert!(
            wind_down >= Duration::from_secs(2),
            "Need >= 2 s between offer=0 and contactor=Off for vehicle wind-down; got {:?}",
            wind_down
        );
    }

    /// Bug 8: the ADC thread closes `pilot_voltage_chan` on its way out (e.g.
    /// SPI failure). `get_new_state_input` currently maps the closed-channel
    /// `Err` to `PilotInError`, which from `Charging` drives us through
    /// `StopCharging` rather than to `FailedStation`. A dead sensing subsystem
    /// is a hardware fault.
    #[test]
    fn test_closed_pilot_channel_signals_hardware_fault() {
        let (pilot_tx, pilot_rx) = bounded::<(f32, f32)>(1);
        let (_fault_tx, fault_rx) = bounded::<Fault>(1);
        drop(pilot_tx); // close the pilot channel

        let input = get_new_state_input(pilot_rx, fault_rx);
        assert!(
            matches!(input, EVSEMachineInput::HardwareFault),
            "Closed pilot channel should yield HardwareFault, not {:?}",
            input
        );
    }
}