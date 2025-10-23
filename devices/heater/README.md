# Heater Controllers — ESP32-POE-ISO PID Temperature Control

## Overview

ESP32-POE-ISO nodes control heating elements via **SparkFun Qwiic Dual Solid State Relay** with **PID temperature control**. Temperature sensing uses **ADS1015 ADC** with **NTC thermistors**. Features include setpoint control, manual PWM, and comprehensive safety systems.

## Available Firmware

- **`Heater_client.ino`** - Main heater controller with PID temperature control using SparkFun SSR
- **`Heater_Safety_client.ino`** - Safety-enhanced version with watchdog supervision

> **Recommended**: Use `Heater_Safety_client.ino` for production deployments.

## Hardware

* **MCU**: Olimex **ESP32-POE-ISO**
* **SSR**: SparkFun **Qwiic Dual Solid State Relay** (I²C, default `0x0A`)
* **ADC**: **ADS1015** (I²C) for thermistors
* **Thermistors**: NTC 10 k, with divider resistor **RC = 10 kΩ** to **VCC = 3.3 V**
* **Power**: PoE/USB for ESP; heater power routed by SSR (match SSR ratings)

## Wiring

* ESP32 I²C → Dual SSR → heater loads
* ESP32 I²C → ADS1015
* Two thermistor dividers:

  * **VCC(3.3 V)** → fixed **RC=10 kΩ** → **node** → **NTC** → **GND**
  * ADS1015 **A0** reads node (heater 1), **A1** reads node (heater 2)
* Keep sensor wiring away from heater power where possible

## Constants (already in sketch)

```cpp
#define HEATER_MAX 100     // PID output upper limit (% PWM)
#define HEATER_MIN 0       // PID output lower limit
#define RC     10000.0     // Divider fixed resistor (Ω)
#define VCC    3.3         // Reference (V)
#define R25    10000.0     // Thermistor @25°C (Ω)
#define B2585  3977.0      // Beta constant (K)
```

## MQTT

**Base**: `heat/01`

| Topic                |   Dir  | Example Payload                                                                 | Notes                 |
| -------------------- | :----: | ------------------------------------------------------------------------------- | --------------------- |
| `heat/01/cmd/<n>`    | PC→ESP | `ON` · `OFF` · `ON:2000` · `PWM:35` · `SET:42.0` · `PID:ON` · `PID:OFF` · `GET` | Channel `<n>` ∈ {1,2} |
| `heat/01/state/<n>`  | ESP→PC | `ON`/`OFF` *(retained)*                                                         |                       |
| `heat/01/target/<n>` | ESP→PC | `42.0` *(retained)*                                                             | PID setpoint          |
| `heat/01/temp/<n>`   | ESP→PC | `39.6`                                                                          | Current temperature   |
| `heat/01/status`     | ESP→PC | `ONLINE`/`OFFLINE` *(retained)*                                                 |                       |
| `heat/01/heartbeat`  | ESP→PC | `1` every 15 s                                                                  |                       |

**Commands (payloads to `heat/01/cmd/<n>`)**

* **Manual**: `ON`, `OFF`, `ON:<ms>`, `PWM:<0–100>`
* **PID**: `SET:<tempC>`, `PID:ON`, `PID:OFF`
* **Telemetry**: `GET` → immediately publishes `temp/<n>`

## PID & Safety (what the firmware does)

* PID: `AutoPID` using `Kp/Ki/Kd` (in code) drives **slow PWM** (0–100 %) to the SSR.
* Safety:

  * **Sensor validity**: if thermistor read is out-of-range / NaN, SSR is forced **OFF**.
  * **Rate-of-rise brake**: if ΔT/Δt exceeds a threshold (config in code), firmware briefly clamps PWM/OFF to prevent runaway.
    Tune this based on your thermal mass and heater wattage.

## Configure & Build

1. **Arduino IDE Setup**:
   - Install ESP32 board support package
   - Install required libraries: `PubSubClient`, `SparkFun_Qwiic_Relay`, `Adafruit_ADS1X15`, `AutoPID`, `Wire`

2. **Configuration**:
   - Set `MQTT_BROKER_IP` to your broker's IP address
   - Configure MQTT credentials (username/password)
   - Set base topic to `heat/01`
   - Map ADS channels (`TH_CH_1`, `TH_CH_2`) to your thermistor connections

3. **PID Tuning**:
   - Adjust `Kp/Ki/Kd` constants for your thermal system
   - Set rate-of-rise thresholds for safety brake
   - Configure sensor validity limits

4. **Upload**:
   - Select board: **ESP32 Dev Module**
   - Upload via USB to ESP32-POE-ISO

## Required Arduino Libraries

```cpp
#include <PubSubClient.h>           // MQTT communication
#include <SparkFun_Qwiic_Relay.h>   // SSR control
#include <Adafruit_ADS1X15.h>       // ADC for thermistors
#include <AutoPID.h>                // PID control algorithm
#include <Wire.h>                   // I²C communication
```

## Test (Python)

```python
import time
from iot_mqtt import HeatMQTT

# Connect to heater controller
heat = HeatMQTT(broker="192.168.0.100", username="heat1", password="heat", base_topic="heat/01")
heat.ensure_connected()

# Set target temperature and enable PID
heat.set_target(1, 42.0)      # Set 42°C target
heat.pid_on(1)                # Enable PID control

# Monitor temperature for 30 seconds
for _ in range(30):
    try:
        temp = heat.get_base_temp(1, timeout_s=2.0)
        print(f"Heater 1 Temperature: {temp}°C")
    except TimeoutError:
        print("Temperature read timeout")
    time.sleep(1)

# Stop heating
heat.pid_off(1)               # Disable PID
heat.set_pwm(1, 0)            # Set PWM to 0%
heat.off(1)                   # Ensure heater is off
heat.disconnect()
```

See [Python API Documentation](../iot_mqtt/README.md) for complete usage examples.

## Troubleshooting

### Temperature Control Issues
- **Temperature readings erratic**: Check thermistor constants (`R25`, `B2585`), wiring, and tune `Kp/Ki/Kd` parameters
- **Temperature overshoot**: Reduce `Kp` gain or increase derivative term `Kd` for better damping
- **Slow response**: Increase `Kp` gain or reduce integral time `Ki`

### Hardware Issues
- **No ADS1015 detected**: Check I²C address, pull-ups (Qwiic has them), and cable connections
- **Safety brake triggers immediately**: Loosen rate-of-rise limit or verify ADC wiring (floating input can appear as fast temperature change)
- **SSR not switching**: Check I²C communication to relay module and verify SSR power supply

### Safety Issues
- **Heater stays on after disconnect**: Check controller beacon is running (`pyctl/heartbeat` topic)
- **Random safety shutdowns**: Verify Ethernet connection stability and thermistor readings

See [Main Troubleshooting Guide](../../README.md#troubleshooting) for more details.


