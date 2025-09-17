
---

````markdown
# ESP32-S3 DC PWM Controller

This project demonstrates how to generate a **DC PWM output** using the ESP32-S3 (YD-ESP32-S3 DevKitC-1 clone).  
It uses the **MCPWM peripheral** to produce complementary PWM signals with dead-time insertion, which is useful for motor drivers, inverters, and other power electronics applications.  

## Features
- Runs on **ESP32-S3 (DevKitC-1 clone)**.
- Generates complementary PWM signals on:
  - **GPIO0 → PWM A**
  - **GPIO1 → PWM B**
- Dead-time insertion (2 µs) to prevent shoot-through.
- Default PWM frequency: **20 kHz**.
- Default duty cycle: **50%**.
- **Dynamic duty cycle control** via Serial commands.
- Built-in RGB LED heartbeat indicator (blinks every 0.5s).

## Serial Commands
Open the Serial Monitor at **115200 baud**.  
- Set duty cycle (0–100%):
  ```text
  DUTY 30
````

→ Sets duty cycle to 30%.

* Example:

  ```text
  DUTY 75
  ```

  → Sets duty cycle to 75%.

## Hardware

* **Board**: YD-ESP32-S3 DevKitC-1 (clone)
* **Outputs**:

  * GPIO0 = PWM A
  * GPIO1 = PWM B
* **On-board RGB LED** used for activity indicator.

## Usage

1. Flash the code to your ESP32-S3 using Arduino IDE or PlatformIO.
2. Connect GPIO0 and GPIO1 to your circuit (e.g., MOSFET driver, test load).
3. Open the Serial Monitor at **115200 baud**.
4. Send commands to adjust PWM duty cycle.

## Example Output

```
DC PWM output ready (GPIO0=A, GPIO1=B)
Send: DUTY <value>   e.g. DUTY 30
Duty updated: 50.0%
Duty updated: 30.0%
Duty updated: 75.0%
```

## License

MIT License

```

--
