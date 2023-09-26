# EVSE Pi Hat Specification

This document provides detailed instructions for using the PWM overlay, activating the pilot, setting the duty cycle, and other procedures to operate the EVSE Pi Hat. Please follow these steps carefully.

## PWM Overlay

Ensure the PWM overlay is set up as described in previous logs. This can be done either by appending it to `/boot/config.txt` or executing `dtoverlay` at the command line.

## Preparatory Step

As a preliminary measure, open `/sys/class/pwm/pwmchip0/export` and write `0\n` to it.

## Activating the Pilot

To enable the pilot, write `1000000\n` to `/sys/class/pwm/pwmchip0/pwm0/period`. This equates to one million nanoseconds, or one millisecond, which represents a 1 kHz period. 

## Setting the Duty Cycle

To determine the duty cycle, multiply the desired percentage by 10,000. Write this value, followed by a newline, to `/sys/class/pwm/pwmchip0/pwm0/duty_cycle`. For example, a duty cycle of 10% (representing a 6A pilot ampacity) would equate to a `duty_cycle` of 100,000. This implies that the high portion lasts 100,000 nanoseconds, or 100 microseconds. 

Finally, write `1\n` to `/sys/class/pwm/pwmchip0/pwm0/enable`.

## Overriding Output Levels

- To force the output to a constant +12v, set the duty cycle to 101%.
- To force the output to a constant -12v, set the duty cycle to 0%.

## Interpretation of Output Levels

- A solid -12v signal generally indicates a fatal error as the EVSE cannot detect the presence of a vehicle due to the pilot never exceeding 1v.
- A constant +12 volts signifies that the EVSE is ready to charge but is still preparing. Normally, when there's no vehicle connected, the pilot is fixed at +12v. In this state, the EVSE can identify the presence of a vehicle if the +12v is drawn down to +9v.

The vehicle is not allowed to pull to +6 or +3 volts until it receives an "offer". 

## Pilot Oscillation

Pilot oscillation from the EVSE implies an offer to the vehicle to draw the indicated number of amps, as determined by the duty cycle.

## Detecting Pilot Feedback

Pilot feedback detection is handled through the ADC, which is described in a separate project log.

# EVSE Pi Hat - GPIO Pin Configuration and ADC Connection

This section provides detailed instructions for manipulating GPIO pins and configuring the MCP3004 ADC for the EVSE Pi Hat. 

## GPIO Pin Configuration

Please note the following descriptions refer to GPIO numbers, not actual pin numbers on the Raspberry Pi GPIO connector:

- `4` - Power Watchdog Pin: Toggle this pin at 1-10 kHz whenever the relay is powered on.
- `17` - Power Pin: Set this pin high to turn on the vehicle power. Ensure the power watchdog pin is toggling, else a synthetic GFI event will occur.
- `18` - Pilot Pin: Set this pin high for +12, low for -12. Further details on using this pin are discussed below.
- `22` - GFI Status Pin: This input pin goes high when the GFI is set, preventing the power from being turned on.
- `23` - Relay Test Pin: This input pin provides the status of the HV relay / GCM test. It should go high and remain that way within 100 ms of turning the power on. Similarly, it should go low and remain so within 100 ms of turning the power off.
- `24` - GFI Test Pin: This output pin is connected to a wire taking two loops through the GFI CT, then connecting to the ground. Toggle this pin at 60 Hz to simulate a ground fault as part of the GFI test procedure.
- `27` - GFI Reset Pin: Pulse this pin high to clear the GFI. This action cannot be performed while vehicle power is on.

## MCP3004 ADC Configuration

The MCP3004 ADC is connected to the SPI system on device zero (`/dev/spidev0.0`). The first three channels are linked to:

- `0` - Pilot Feedback: Sample this pin several hundred times, recording high and low values. The high value is used to detect vehicle state changes. The low value should stay near -12v (when the pilot is oscillating) otherwise it's a "missing diode" test failure.
- `1` - Current Sense Transformer: Sample this pin across two zero crossings (512 is zero), execute an RMS calculation, and scale the result to determine the vehicle's current draw.
- `2` - AC Voltage Sense: Sample this pin for peak value to determine the AC voltage. Together with the current sense, it can determine the power drawn by the vehicle.

## Raspberry Pi Hardware PWM Configuration

To create a proper pilot, you need to configure the Raspberry Pi hardware PWM timers. The Python GPIO library's PWM functionality is not sufficient.

## Enabling ADC and PWM 

To make the ADC available, you must use `raspi-config` and enable SPI. To turn on the hardware PWM on pin 18, add the following to `/boot/config.txt`: 

```markdown
dtoverlay=pwm,pin=18,func=2
```

This ensures that the hardware PWM is enabled on pin 18.

# Performing a GFI Test

This section outlines the steps required to perform a Ground Fault Interrupter (GFI) test.

## Steps

1. **Initial State**: Start with the GFI cleared. If it's set, clear it. The test will fail if it doesn't clear. It should remain clear for a few dozen milliseconds.

2. **GFI Test Line Toggle**: Toggle the GFI Test line at 60 Hz, which equates to a change every 8.3 ms, for 10 cycles. 

3. **GFI State Check**: At the end of the 10 cycles, the GFI should be set. If it's not, the test is a failure.

4. **GFI Clear Attempt**: Wait around 100 ms, then clear the GFI. If it doesn't clear, the test is a failure.

5. **GFI Status Monitoring**: Monitor the GFI status for another 100 ms. If it doesn't remain clear, the test is a failure.

6. **Test Pass Criteria**: If the GFI status meets all the above conditions, the test is considered a pass.

Remember to perform this GFI test immediately before every attempt to turn the power on. Since the GFI remains in the clear state after the test, it is safe to power on the relay and start toggling the power watchdog pin at this stage.


# EVSE Pi Hat - Service Loop

The following section describes the crucial steps of the service loop that should be executed to successfully manage the EVSE Pi Hat.

## Service Loop Steps

1. **Pilot Feedback Check**: If the pilot is on, perform around 100 ADC conversions on channel 0 (pilot feedback), saving the highest and lowest values seen. If the lowest value does not correspond to -12V, it's an error. You can determine the state from the highest value: near 12v is state A, 9v is B, 6v is C, and 3v is D.

2. **State Change Response**: If the pilot feedback state changes, actions may be required:
    - `A -> B` or `B -> A` are informational, indicating plugging in or out.
    - `B -> C` or `B -> D` are requests for power.
    - `C -> B` or `D -> B` are requests to end power.

3. **The Different states** as defined by the SAE_J1772 std:
| Base status       | Charging status   | Resistance, CP-PE | Resistance, R2 | Voltage, CP-PE |
|-------------------|-------------------|-------------------|----------------|----------------|
| Status A          | Standby           | Open, or ∞ Ω      |                | +12 V          |
| Status B          | Vehicle detected  | 2740 Ω            |                | +9±1 V         |
| Status C          | Ready (charging)  | 882 Ω             | 1300 Ω         | +6±1 V         |
| Status D          | With ventilation  | 246 Ω             | 270 Ω          | +3±1 V         |
| Status E          | No power (shutoff)|                   |                | 0 V            |
| Status F          | Error             |                   |                | −12 V          |

  
3. **Power Request Response**: For a power request, perform a GFI self-test, then turn the power on. For a request to end power, turn the power off.

4. **Power Watchdog**: If power is on, regularly toggle the power watchdog pin (recommended frequency is 1 kHz, but not exceeding 100 kHz and not less than 100 Hz).

5. **Relay Test Line**: If the power is on, the relay test line must be high. If the power is off, the relay test line must be low. Allow up to 100 ms grace period after power transitions; otherwise, an error is triggered.

6. **GFI Line**: If the GFI line is high, it's a ground fault. Turn the power off and report an error on the pilot (stop oscillating - leave it either 0% or 100% duty). Before attempting to turn the power back on, reset the GFI once the clearance interval has elapsed.

7. **Current and Voltage Measurement**: Use ADCs on channel 1 to observe the current drawn by the vehicle. Collect samples for two zero crossings (zero is 512), perform an RMS on the samples, and scale it to obtain an RMS current draw. se ADCs on channel 2 to figure out the AC voltage; look for the peak and scale that to determine the voltage.
U
8. **Pilot Change Grace Period**: Allow the car a 5-second grace period to respond to any changes to the pilot, including withdrawing the pilot while the vehicle is charging.

9. **Power State Changes**: Unless responding to a GFI, do not turn off the power other than in response to a `[C, D] -> B` state change. Doing so is bad for the contactor as the vehicle will arrange for current draw to be reduced to near zero before changing state. Similarly, do not turn the power on other than in response to a `B -> [C, D]` transition.

10. **State A Pilot**: In state A (no vehicle), the pilot should be a steady +12v and should not oscillate. A vehicle transitioning directly from state A to state C or D is illegal. A transition from state B to C or D is also illegal unless the pilot is oscillating (an EVSE can deny permission to charge by refusing to oscillate the pilot).


# Hardware PWM and Build Updates

## Hardware PWM

Good news - hardware PWM works smoothly, eliminating the jitter issues observed with RPi.GPIO. However, it's worth noting that it doesn't seem to be directly supported by Python.

Despite this, implementing it isn't too difficult. It's quite similar to manual GPIO manipulation via `/sys/class`. You start by writing a "0" to `/sys/class/pwm/pwmchip0/export`. In response, you'll find `/sys/class/pwm/pwmchip0/pwm0/` appearing. Inside `pwm0`, you'll encounter "period", "duty_cycle", and "enable". 

To set the cycle period, you write the desired time in nanoseconds (1,000,000 for 1 kHz) into "period". You set the duty cycle by writing the time you want the signal to be high in nanoseconds (e.g., 100,000 for 10%) into "duty_cycle". Finally, to enable the PWM, you write a "1" into "enable".

## Build Report 1.0

The first prototype had a few issues:
- The quad NAND gate had the wrong pinout.
- There was no connection from the Pi GPIO header for the GFI test line.

These issues have been addressed. I also replaced the three diode-plus-pull-up level shifters with a triple bus buffer chip. Despite being slightly pricier, a chip and a bypass cap are more efficient than six passives. Additionally, I replaced the dual JK flip-flop with a single D flip-flop chip. The fact that it's a D versus JK doesn't matter in our case because we only use the set/reset functionality as a latch.

The pilot generator, ADC, and relay test input were tested successfully. 

We are now moving onto version 1.1!

## Some Sample Code

Here's how you can read the ADC:

```python
import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 0
chan = 0  # 0..3 for MCP3004
buf = [1, 0x80 | (chan << 4), 0xff]
spi.xfer(buf)
val = ((buf[1] & 0x3) << 8) | buf[2]  # val is output, from 0-1023.
```

Though the ADC is rated (at Vdd=5V) for 3.6 MHz, it appears that exceeding 1 MHz affects its linearity. The reason for this isn't quite clear. Still, this provides well over 40k samples per second.

To test the pilot, run the above code in a tight loop, around 500 times, capturing the low and high readings. Theoretically, a pilot voltage of -12v should result in an ADC voltage of about 900 mV, and a +12v should read as about 4.55v. These two values from the ADC correspond to 184 and 932, respectively. Between these two values, the scale should be linear.

