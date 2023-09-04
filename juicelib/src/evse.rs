// This file contains the different functionality that's associated with the 
// specifics of the EVSE. 

// Define the state machine of an AC EVSE:

use core::panic;
use std::error::Error;

use rppal::gpio::Level;
use rust_fsm::*;

use crossbeam_channel::{Receiver, Select};

use crate::peripherals::GpioPeripherals;


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
        SelfTestFailed => FailedStation [SelfTestError]
    },
    Standby => {
        PilotIs9V => VehicleDetected [StartCharging],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
    },
    VehicleDetected => {
        PilotIs12V => Standby [VehicleDisconnected],
        PilotIs6V => Charging [ChargingInProgress],
        PilotIs3V => VentilationNeeded,
        PilotIs0V => NoPower,
        PilotInError => ResetableError [PilotMeasurement],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
        HardwareFault => FailedStation [HardwareFault],
    },
    
    Charging => {
        PilotIs12V => ResetableError [VehicleDisconnected],
        PilotIs9V => VehicleDetected [ChargingFinished],
        PilotIs6V => Charging [ChargingInProgress], // Is this correct?
        PilotIs3V => VentilationNeeded [UnsupportedVehicle],
        PilotIs0V => NoPower [ChargingFinished],
        PilotInError => ResetableError [PilotMeasurement],
        GFIInterrupted => FailedStation [GFIError],
        NoGround => FailedStation [NoGroundError],
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

fn run_self_test() -> EVSEMachineInput {
    EVSEMachineInput::SelfTestOk
}

fn listen_to_pilot() -> EVSEMachineInput {
    // Listen to the input from the pilot
    EVSEMachineInput::PilotIs9V
}

pub enum Fault {
    GFIInterrupted,
    NoGround,
    PilotInError,
}

fn get_pilot_state(voltage: f32) -> EVSEMachineInput {
    if  13.0 > voltage && voltage > 11.0 {
        EVSEMachineInput::PilotIs12V
    } else if 10.0 > voltage && voltage > 8.0 {
        EVSEMachineInput::PilotIs9V
    } else if 7.0 > voltage && voltage > 5.0 {
        EVSEMachineInput::PilotIs6V
    } else if 4.0 > voltage && voltage > 2.0 {
        EVSEMachineInput::PilotIs3V
    } else if 1.0 > voltage && voltage > -1.0 {
        EVSEMachineInput::PilotIs0V
    } else {
        println!("Pilot voltage is out of range: {}", voltage);
        EVSEMachineInput::PilotInError
    }
}

fn get_new_state_input(pilot_voltage_chan: Receiver<f32>, fault_channel: Receiver<Fault>) -> EVSEMachineInput {
    let mut sel = Select::new();
    sel.recv(&pilot_voltage_chan);
    sel.recv(&fault_channel);
    let oper = sel.select();
    match oper.index() {
        0 => {
            let voltage = oper.recv(&pilot_voltage_chan);
            if let Err(x) = voltage {
                return EVSEMachineInput::PilotInError;
            }
            let voltage = voltage.unwrap();
            get_pilot_state(voltage)
        },
        1 => {
            // Fault channel
            match oper.recv(&fault_channel) {
                Ok(fault) => {
                    match fault {
                        Fault::GFIInterrupted => EVSEMachineInput::GFIInterrupted,
                        Fault::NoGround => EVSEMachineInput::NoGround,
                        Fault::PilotInError => EVSEMachineInput::PilotInError,
                    }
                },
                Err(_) => {
                    println!("Fault channel closed");
                    EVSEMachineInput::PilotInError
                }
            }
        },
        _ => {
            panic!("Invalid index");
        }
    }
}

#[derive(Debug, Clone)]
pub enum OnOff {
    On,
    Off,
}

#[derive(Debug, Clone)]
pub struct HwError {
    message: String,
}

impl Error for HwError {}

impl std::fmt::Display for HwError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f,"{}", self.message)
    }
}

pub trait EVSEHardware {
    fn set_contactor(&mut self, state: OnOff) -> Result<(), HwError>;
    fn set_current_offer_ampere(&mut self, ampere: f32) -> Result<(), HwError>;
    fn set_ground_test_pin(&mut self, state: OnOff) -> Result<(), HwError>;
    fn get_contactor_state(&mut self) -> Result<OnOff, HwError>;

    const MAX_CURRENT_OFFER: f32 = 32.0;
}

struct EVSEHardwareImpl {
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
    fn new() -> Self {
        let mut ret = Self {
            contactor: OnOff::Off,
            current_offer: 0.0,
            ground_test_pin: OnOff::Off,
            hw_peripherals: GpioPeripherals::new(),
        };
        ret.set_contactor(OnOff::Off).unwrap();
        ret.set_current_offer_ampere(0.0).unwrap();
        ret.set_gfi_test_pin(OnOff::Off).unwrap();
        ret
    }

    fn set_contactor(&mut self, state: OnOff) -> Result<(), HwError> {
        self.contactor = state.clone();
        self.hw_peripherals.set_contactor_pin(state.into());
        Ok(())
    }

    fn set_current_offer_ampere(&mut self, ampere: f32) -> Result<(), HwError> {
        self.hw_peripherals.set_pilot_ampere(ampere);
        self.current_offer = ampere;
        Ok(())
    }

    fn set_gfi_test_pin(&mut self, state: OnOff) -> Result<(), HwError> {
        self.ground_test_pin = state.clone();
        self.hw_peripherals.set_gfi_test_pin(state.into());
        Ok(())
    }

    fn get_contactor_state(&mut self) -> Result<OnOff, HwError> {
        Ok(self.hw_peripherals.get_relay_test_pin().into())
    }
}

fn do_state_transition<T>(state: &EVSEMachineState, hw: &mut T) -> Result<(), HwError> 
where T: EVSEHardware
{
    match state {
        EVSEMachineState::Standby => {
            hw.set_contactor(OnOff::Off)?;
            hw.set_current_offer_ampere(0.0)?;
            hw.set_ground_test_pin(OnOff::Off)?;
        },
        EVSEMachineState::VehicleDetected => {
            hw.set_contactor(OnOff::Off)?;
            hw.set_current_offer_ampere(T::MAX_CURRENT_OFFER)?;
            hw.set_ground_test_pin(OnOff::On)?;
        },        
        EVSEMachineState::Charging => {
            let state = hw.get_contactor_state()?;
            if let OnOff::Off = state {
                println!("Contactor is off. Turning it on");
                hw.set_contactor(OnOff::On)?;
            }
        }
        _ => {
            hw.set_contactor(OnOff::Off)?;
            // Do nothing
        }
    }
    Ok(())
}

pub fn start_machine(pilot_voltage_chan: Receiver<f32>, fault_channel: Receiver<Fault>, evse: &mut impl EVSEHardware) -> ! 
{
    let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
    loop {
        match machine.state() {
            EVSEMachineState::SelfTest => {
                // Do the self test
                let self_test_result = run_self_test(); // TODO: What's needed here?
                let output = machine.consume(&self_test_result).unwrap();
                println!("Output: {:?}", output.unwrap());
                println!("State: {:?}", machine.state());
            },
            EVSEMachineState::ResetableError => {
                todo!("Reset the error");
            },
            EVSEMachineState::FailedStation => {
                eprintln!("Station failed. Full reset required, contact admin if the issue persists.");
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
                
                
                let res = do_state_transition(state, evse);
                let mut state_input = EVSEMachineInput::PilotInError;
                if let Err(e) = res {
                    eprintln!("Error: {}", e);
                    state_input = EVSEMachineInput::HardwareFault;
                    // Turn off the contactor, if we fail, we'll try again in the state transition
                    // This is anyway only a secondary safety measure, since the hardware is already hard-wired
                    // to turn off the contactor immediately if a something dangerous happens (no software), 
                    // such as GFI for example.
                    evse.set_contactor(OnOff::Off).unwrap_or(());     
                } else {
                    state_input = get_new_state_input(pilot_voltage_chan.clone(), fault_channel.clone());
                }
                println!("State input: {:?}", state_input);                
                let output = machine.consume(&state_input);
                println!("Output: {:?}", output);
                println!("State: {:?}", machine.state());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_sm() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        println!("Output: {:?}", output);
        println!("State: {:?}", machine.state());
    }

    #[test]
    fn test_self_test() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestFailed).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::FailedStation));
    }

    #[test]
    fn test_standby() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
    }

    #[test]
    fn test_gfi_interrupted() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::GFIInterrupted).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::FailedStation));
    }

    #[test]
    fn test_no_ground() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::NoGround).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::FailedStation));
    }

    #[test]
    fn test_start_charging() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
        let output = machine.consume(&EVSEMachineInput::PilotIs6V).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Charging));
    }

    #[test]
    fn test_vehicle_left() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
        let output = machine.consume(&EVSEMachineInput::PilotIs12V).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
    }

    #[test]
    fn test_charging_finished() {
        let mut machine: StateMachine<EVSEMachine> = StateMachine::new();
        let output = machine.consume(&EVSEMachineInput::SelfTestOk).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Standby));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
        let output = machine.consume(&EVSEMachineInput::PilotIs6V).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::Charging));
        let output = machine.consume(&EVSEMachineInput::PilotIs9V).unwrap();
        println!("Output: {:?}", output.unwrap());
        println!("State: {:?}", machine.state());
        assert!(matches!(machine.state(), EVSEMachineState::VehicleDetected));
    }
}