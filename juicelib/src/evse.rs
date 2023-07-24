// This file contains the different functionality that's associated with the 
// specifics of the EVSE. 

// Define the state machine of an AC EVSE:

use rust_fsm::*;

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

#[derive(Debug, PartialEq, Clone)]
enum EVSEState {
    SelfTest,
    Standby,
    VehicleDetected,
    Charging,
    VentilationNeeded,
    NoPower,
    ResettableError,
    FailedStation       // Cannot recover from that
}

#[derive(Debug, PartialEq, Clone)]
enum EVSEInput {
    PilotIs12V,
    PilotIs9V,
    PilotIs6V,
    PilotIs3V,
    PilotIs0V,               // A short? Disconnected Pilot?
    PilotInError,            // -12V or any other voltage that's not specified
    GFIInterrupted,
    NoGround,
    SelfTestOk,
    SelfTestFailed
}

state_machine! {
    derive(Debug)
    EVSEMachine(Standby)

    SelfTest => {
        SelfTestOk => Standby,
        SelfTestFailed => FailedStation
    },
    Standby => {
        PilotIs9V => VehicleDetected,
        GFIInterrupted => FailedStation,
        NoGround => FailedStation,
    },
    VehicleDetected => {
        PilotIs12V => Standby,
        PilotIs6V => Charging,
        PilotIs3V => VentilationNeeded,
        PilotIs0V => NoPower,
        PilotInError => ResetableError
    },
    
    Charging => {
        PilotIs12V => ResetableError,
        PilotIs9V => VehicleDetected,
        PilotIs6V => Charging,
        PilotIs3V => VentilationNeeded,
        PilotIs0V => NoPower,
        PilotInError => ResetableError,
        GFIInterrupted => FailedStation,
    },
    ResetableError => {
        PilotIs12V => Standby,   // Vehicle disconnected
    }
}

