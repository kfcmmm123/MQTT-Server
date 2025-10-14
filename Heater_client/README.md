# Heater Node — Dual SSR + Thermistors + PID (+ Safety)

## Overview

ESP32-POE-ISO controls **two heater channels** via a **Qwiic Dual Solid State Relay** and reads **two thermistors** via **ADS1015**. It provides setpoint control (`SET:<tempC>`), **PID ON/OFF**, manual **PWM**, and **safety** (rate-of-rise brake, sensor validity).

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

* Set `MQTT_BROKER_IP`, creds, base `heat/01`.
* Map **ADS channels** (`TH_CH_1`, `TH_CH_2`) to your thermistor nodes.
* Tune **Kp/Ki/Kd**, **rate-of-rise** thresholds, and **sensor limits** for your hardware.
* Libraries: `PubSubClient`, `SparkFun_Qwiic_Relay`, `Adafruit_ADS1X15`, `AutoPID`, `Wire`.

## Test (Python)

```python
from iot_mqtt import HeatMQTT
h = HeatMQTT(broker="192.168.0.101", username="heat1", password="heat", base_topic="heat/01")
h.ensure_connected()

# Set target & enable PID
h.set_base_temp(1, 42.0)          # retained target
h._publish("cmd/1", "PID:ON")

# Watch temps
for _ in range(30):
    try:
        print("T1:", h.get_base_temp(1, timeout_s=2.0))
    except TimeoutError:
        print("No temp")
    time.sleep(1)

# Stop
h._publish("cmd/1", "PID:OFF")
h.set_pwm(1, 0)
h.off(1)
h.disconnect()
```

## Troubleshooting

* Temps jump or overshoot: confirm thermistor constants (`R25`, `B2585`), wiring, and tune `Kp/Ki/Kd`.
* “Safety brake” triggers immediately: loosen the rate-of-rise limit or verify ADC wiring (floating input can look like fast ΔT).
* No ADS detected: check I²C address, pull-ups (Qwiic has them), cable.


