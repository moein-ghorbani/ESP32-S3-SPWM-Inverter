
---

# ESP32 SPWM Inverter (Full Version with Web Interface & Protections)

## Overview

This project implements a **full-featured SPWM (Sinusoidal Pulse Width Modulation) inverter** using the ESP32. It converts 12V DC to a 50Hz AC output through an H-Bridge, with a variety of monitoring, protections, and a web-based interface for real-time control.

⚠️ **Warning:** The H-Bridge output involves high voltage. Proper isolation and safety precautions are required.

---

## Features

* **SPWM Output:** 50 Hz AC using 12V DC input
* **Carrier Frequency:** 20 kHz with configurable dead-time
* **Soft-Start Ramp:** Gradually increases amplitude to prevent inrush current
* **Real-Time ADC Monitoring:** Measures bus voltage and load current
* **Protections:**

  * Over-Voltage (OV)
  * Over-Current (OC) with latch mechanism
* **Web Interface (AP Mode):**

  * Live display of voltage, current, amplitude factor, and fault status
  * Fault reset button
  * Configure soft-start duration and target amplitude

---

## Hardware

* **ESP32 MCU**
* **H-Bridge Driver** (connected to MCPWM outputs)
* **Voltage & Current Sensors** (ADC inputs)
* **Status LED**
* **Fault Reset Button**

**Pin Assignments (Default):**

| Signal             | Pin |
| ------------------ | --- |
| MCPWM Output A     | 25  |
| MCPWM Output B     | 26  |
| Voltage Sense      | 34  |
| Current Sense      | 35  |
| Fault Reset Button | 4   |
| Status LED         | 2   |

---

## Software Configuration

* **PWM Parameters:**

  * Output frequency: 50 Hz
  * Carrier frequency: 20 kHz
  * Dead-time: 2 µs

* **ADC Scaling:**

  * Reference voltage: 3.3 V
  * Voltage divider ratio: 330/3.3
  * Current sense: 0.1 V/A

* **Protections:**

  * Bus OV limit: 380 V
  * RMS current limit: 20 A

* **Soft-Start:**

  * Default time: 3 s
  * Target amplitude: 1.0 (normalized)

---

## Web Interface

After powering up, the ESP32 creates an Access Point:

* **SSID:** `ESP32_INVERTER`
* **Password:** `12345678`

Web dashboard features:

* Display bus voltage, load current, and amplitude factor
* Show fault status (`YES`/`NO`)
* Reset latched faults
* Configure soft-start duration and target amplitude

---

## Usage

1. Power the ESP32 and H-Bridge safely.
2. Connect to the ESP32 WiFi AP.
3. Open the web interface in a browser (`http://192.168.4.1`).
4. Monitor live measurements and manage faults.
5. Adjust soft-start time or target amplitude as needed.

---

## Safety Notes

* Ensure proper isolation between high-voltage H-Bridge output and control circuitry.
* Use appropriate fuses and protective components.
* Never touch H-Bridge output while powered.

---

## License

This project is open-source and free to use for educational and experimental purposes. Proper safety precautions must be followed when handling high-voltage circuits.

---

