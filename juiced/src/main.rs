
use juicelib::evse::{start_machine, Fault, EVSEHardware, EVSEHardwareImpl};
use crossbeam_channel::bounded;

fn main() {
    let mut evse = EVSEHardwareImpl::new();
    
    let (fault_tx, fault_rx) = bounded(1);
    let (pilot_tx, pilot_rx) = bounded(1);
    
    let evse = EVSEHardwareImpl::new();
    start_machine(pilot_rx, fault_rx, &mut evse);
}
