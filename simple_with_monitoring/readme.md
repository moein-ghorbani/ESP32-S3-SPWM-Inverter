

# ESP32 SPWM Inverter

## Overview

This project implements a **minimal software-based SPWM (Sinusoidal Pulse Width Modulation) inverter** using an ESP32 microcontroller.
It converts 12V DC input to a controlled AC output via an H-Bridge using high-frequency PWM modulation.

**Key Features:**

* 50Hz SPWM output (configurable)
* 20kHz carrier PWM with dead-time for safe H-Bridge operation
* Soft-start ramp to prevent inrush current
* ADC monitoring for voltage and current
* Over-Voltage (OV) and Over-Current (OC) protection with fault latching
* Resettable fault system via a button

⚠️ **Warning:** High voltage appears at the H-Bridge output. Handle with proper isolation and protection.

---

## Hardware Requirements

* ESP32 development board
* H-Bridge capable of handling your load
* Voltage divider for ADC voltage sensing
* Current sensor (e.g., shunt + amplifier)
* Push-button for fault reset
* LEDs for status indication

**Pin Configuration (default):**

| Function           | ESP32 Pin |
| ------------------ | --------- |
| MCPWM Output A     | 25        |
| MCPWM Output B     | 26        |
| Voltage Sense ADC  | 34        |
| Current Sense ADC  | 35        |
| Fault Reset Button | 4         |
| Status LED         | 2         |

---

## Software Requirements

* Arduino IDE
* ESP32 Board Package ≥ 3.3.1
* No external libraries required (uses `driver/mcpwm.h`)

---

## Installation

1. Clone the repository:

```bash
git clone https://github.com/yourusername/esp32-spwm-inverter.git
```

2. Open `ups.ino` in Arduino IDE.
3. Configure pins and parameters if needed:

```cpp
const float OUTPUT_FREQ = 50.0f;        // AC output frequency
const float SOFT_START_TIME_SEC = 3.0f; // Soft-start duration
const float OV_BUS_V = 380.0f;          // Over-voltage threshold
const float OC_I_RMS = 20.0f;           // Over-current threshold
```

4. Compile and upload to your ESP32.

---

## How It Works

1. **Sine Table Generation:**
   Generates 256 samples per cycle for a smooth SPWM waveform.

2. **PWM Output:**
   MCPWM module drives H-Bridge with complementary duty cycles based on the sine table.

3. **Soft Start:**
   Gradually increases amplitude to prevent sudden inrush current.

4. **ADC Monitoring:**
   Continuously reads bus voltage and current to calculate RMS values.

5. **Protections:**

   * Trips a latched fault if RMS voltage or current exceeds configured limits.
   * Catastrophic over-voltage triggers immediate shutdown.
   * Faults can be reset via a push-button.

---

## Example Output

```
V_rms: 230.12 V  I_rms: 5.32 A  amp: 0.750
```

* `V_rms`: RMS voltage measured from ADC
* `I_rms`: RMS current
* `amp`: Current amplitude factor (used for soft-start)

---

## Safety Notes

* This inverter outputs high voltage; take proper precautions.
* Use fuses, isolation, and protective gear.
* Test first with a resistive load and low voltage.

---

## License

This project is **MIT Licensed** – feel free to use and modify it.

---

