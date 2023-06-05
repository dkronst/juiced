use my_crate::mcp3xxx::MCP3xxx;
use my_crate::analog_in::AnalogIn;
use my_crate::mcp3002::MCP3002;
use embedded_hal_mock::{spi::Transaction as SPITransaction, spi::Mock as MockSPI, pin::Mock as MockPin};

#[test]
fn reads_value_mcp3xxx() {
    let expectations = [
        SPITransaction::write(vec![0x80, 0x00]),
        SPITransaction::transfer(vec![0x00, 0x00], vec![0x03, 0xFF]),
    ];
    let mock_spi = MockSPI::new(&expectations);
    let mock_pin = MockPin::new(&[Ok(())]);
    let mut device = MCP3xxx::new(mock_spi, mock_pin, 3.3);
    assert_eq!(device.read(0, false), 1023);
}

#[test]
fn reads_value_analog_in() {
    let expectations = [
        SPITransaction::write(vec![0x80, 0x00]),
        SPITransaction::transfer(vec![0x00, 0x00], vec![0x03, 0xFF]),
    ];
    let mock_spi = MockSPI::new(&expectations);
    let mock_pin = MockPin::new(&[Ok(())]);
    let mut device = MCP3002::new(mock_spi, mock_pin, 3.3);
    let mut analog_in = AnalogIn::new(device, 0, None);
    assert_eq!(analog_in.value(), 65472);
}

#[test]
fn reads_voltage_analog_in() {
    let expectations = [
        SPITransaction::write(vec![0x80, 0x00]),
        SPITransaction::transfer(vec![0x00, 0x00], vec![0x03, 0xFF]),
    ];
    let mock_spi = MockSPI::new(&expectations);
    let mock_pin = MockPin::new(&[Ok(())]);
    let mut device = MCP3002::new(mock_spi, mock_pin, 3.3);
    let mut analog_in = AnalogIn::new(device, 0, None);
    assert_eq!(analog_in.voltage(), 3.3);
}
