// This file contains the different functionality that's associated with the 
// specifics of the EVSE. 

// Define the state machine of an AC EVSE:

use core::panic;

use rust_fsm::*;

use crossbeam_channel::{Receiver, Sender, select};


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
    },
    VehicleDetected => {
        PilotIs12V => Standby [VehicleDisconnected],
        PilotIs6V => Charging [ChargingInProgress],
        PilotIs3V => VentilationNeeded,
        PilotIs0V => NoPower,
        PilotInError => ResetableError [PilotMeasurement],
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
    },
    ResetableError => {
        PilotIs12V => Standby,    // This error can be reset by the user
    }
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

pub fn start_machine(pilot_voltage_chan: Receiver<f32>, fault_channel: Receiver<Fault>) -> ! 
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

                // set_hardware_state(state);

                let mut state_input = EVSEMachineInput::PilotInError;
                let selector = select! {
                    recv(pilot_voltage_chan) -> voltage => {
                        match voltage {
                            Ok(v) => {
                                state_input = match v {
                                    12.0 => EVSEMachineInput::PilotIs12V,
                                    9.0 => EVSEMachineInput::PilotIs9V,
                                    6.0 => EVSEMachineInput::PilotIs6V,
                                    3.0 => EVSEMachineInput::PilotIs3V,
                                    0.0 => EVSEMachineInput::PilotIs0V,
                                    _ => EVSEMachineInput::PilotInError,
                                };
                            },
                            Err(_) => {
                                // Timeout occured, feed the machine with a PilotInError input
                                state_input = EVSEMachineInput::PilotInError;
                            }
                        }
                    },
                    recv(fault_channel) -> fault => {
                        match fault {
                            Ok(f) => {
                                state_input = match f {
                                    Fault::GFIInterrupted => EVSEMachineInput::GFIInterrupted,
                                    Fault::NoGround => EVSEMachineInput::NoGround,
                                    Fault::PilotInError => EVSEMachineInput::PilotInError,
                                };
                            },
                            Err(_) => {
                                // Timeout occured, feed the machine with a PilotInError input
                                state_input = EVSEMachineInput::PilotInError;
                            }
                        }
                    }
                };
                
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

}