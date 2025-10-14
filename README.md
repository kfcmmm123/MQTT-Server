# ESP32-POE-ISO IoT Control System

**Modular MQTT-Based Control for Pumps, Ultrasonic Drivers, and Heaters**

This project implements a **multi-device IoT control system** using **ESP32-POE-ISO** boards connected via **MQTT over Ethernet**.
Each node manages a specific subsystem — **Pumps**, **Ultrasonic Drivers**, or **Heaters** — with real-time monitoring, PID control, and safety supervision.

---

## Table of Contents

* [Features](#features)
* [Repository Structure](#repository-structure)
* [System Architecture](#system-architecture)
* [Bill of Materials (BOM)](#bill-of-materials-bom)
* [Setup](#setup)

  * [Broker (Mosquitto)](#broker-mosquitto)
  * [ESP32 Nodes](#esp32-nodes)
  * [Python Controller](#python-controller)
* [MQTT Topic Structure](#mqtt-topic-structure)
* [Python API (`iot_mqtt.py`)](#python-api-iot_mqttpy)

  * [Classes](#classes)
  * [Command Reference (payloads to `<base>/cmd/<n>`)](#command-reference-payloads-to-basecmdn)
* [Example Usage](#example-usage)
* [Wiring & Safety Notes](#wiring--safety-notes)
* [Troubleshooting](#troubleshooting)
* [License](#license)

---

## Features

* Unified **MQTT** communication for all devices
* Independent **ESP32-POE-ISO** nodes (PoE, isolated Ethernet)
* **PID temperature control** with thermistors (ADS1015) + Dual SSR
* **Quad-relay pump** control (Qwiic I²C)
* **Ultrasonic driver** control via relay
* **Safety**: over-temperature rate brake, startup delay, fail-safe OFF
* **Python controller** (`iot_mqtt.py`) for orchestration and testing
* Real-time telemetry (temperatures, relay states, online/heartbeat)

---

## Repository Structure

```
├── iot_mqtt.py                # Python control library
├── iot_mqtt_demo.py           # Example usage script
├── Arduino/
│   ├── pumps_node.ino         # 4-channel pump controller
│   ├── ultra_node.ino         # 2-channel ultrasonic controller
│   └── heat_node.ino          # 2-channel heater + PID + safety
├── README.md                  # (this file)
└── assets/
    └── architecture.png       # system diagram (optional)
```

---

## System Architecture

```text
                 ┌─────────────────────────────┐
                 │         MQTT Broker         │
                 │     (Mosquitto on PC)       │
                 │       Port 1883 (TCP)       │
                 └──────────────┬──────────────┘
                                │
                         Ethernet TCP/IP
          ┌─────────────────────┼────────────────────┐
          │                     │                    │                    
┌─────────┴──────────┐  ┌───────┴────────┐   ┌───────┴─────────┐
│ ESP32-POE-ISO      │  │ ESP32-POE-ISO  │   │ ESP32-POE-ISO   │
│ Node: pumps/01     │  │ Node: ultra/01 │   │ Node: heat/01   │
│ Qwiic Quad Relay   │  │ Qwiic Quad     │   │ Dual SSR +      │
│ (4 channels: pumps)│  │ Relay (2 ch:   │   │ ADS1015 + NTCs  │
│                    │  │ ultrasonics)   │   │ PID + Safety    │
└────────────────────┘  └────────────────┘   └─────────────────┘
```

Each node runs an MQTT client, subscribes to control topics (`<base>/cmd/#`), and publishes telemetry (`state/#`, `temp/#`, `status`, `heartbeat`).

---

## Bill of Materials (BOM)

### Server

* **Ethernet Switch:** [Netgear GS308EPP 8-Port PoE+](https://www.amazon.ca/NETGEAR-Gigabit-Ethernet-Managed-GS308EP/dp/B08MBFLMDC/ref=sr_1_3_sspa?crid=1D3BBWKKI60KR&keywords=ethernet+switch&qid=1760448019&sr=8-3-spons&th=1) — Managed Gigabit PoE+ switch (8 ports, 123 W). Powers and networks all ESP32-POE-ISO nodes.
* **Ethernet Cables:** Cat5e / Cat6 (PoE-compatible), one per node.

### ESP Nodes

* **MCU:** [Olimex ESP32-POE-ISO](https://www.digikey.ca/en/products/detail/olimex-ltd/ESP32-POE-ISO-EA/10258722) (PoE power, galvanic isolation, Ethernet + I²C + GPIO).

### Pumps Node

* **Pumps:** [Kamoer KPHM100](https://www.kamoer.com/us/product/detail.html?id=10021) (12 V micro peristaltic).
* **Relay Board:** [SparkFun Qwiic Quad Relay](https://www.sparkfun.com/sparkfun-qwiic-quad-relay.html) (4-ch I²C).
* **Protection:** Flyback diodes (1N4007) across each DC pump.
* **Power:** 12 V DC.

### Ultrasonic Node

* **Driver/Transducer:** [120 W 40 kHz Driver + Transducer Kit](https://www.amazon.ca/Dacvgog-Ultrasonic-Transducer-Cleaning-Machines/dp/B0BC7Q8DY1) (AC-driven ultrasonic cleaning module).
* **Relay:** [SparkFun Qwiic Single Relay](https://www.sparkfun.com/sparkfun-qwiic-single-relay.html) (I²C).
* **Power:** 110–120 VAC.

### Heater Node

* **Heater:** [Aluminum Heater 60 × 28 × 5 mm](https://www.ptcyidu.com/heating-element-hair-dryer-accessories-12-220v-60-270-degrees-coffee-maker-ptc-heaters-with-aluminum-shell-60x28x5mm340) (12–220 V variants).
* **Thermistors:** 10 kΩ NTC (one per channel).
* **Divider Resistors:** 10 kΩ (1%) (one per channel).
* **ADC:** [Adafruit ADS1015](https://www.adafruit.com/product/1083?srsltid=AfmBOop1jAaBsJfcPm18nTynSNd855LKGRBNVP8QOJirZOHIC3O8EmZ_) (I²C, 4-ch, 12-bit).
* **Relay:** [SparkFun Qwiic Dual Solid-State Relay](https://www.sparkfun.com/sparkfun-qwiic-dual-solid-state-relay.html) (2-ch).
* **DC Rail Protection:** 1000 µF electrolytic + 0.1 µF ceramic + TVS diode across 12 V.

---

## Setup

### Broker (Mosquitto)

* Install Mosquitto on your **PC** (Windows/Linux/macOS) or a **Raspberry Pi**.
* Default listener: **TCP 1883**.
* Optional: `iot_mqtt.py` can auto-start Mosquitto on Windows (`start_broker_if_needed()`).

### ESP32 Nodes

* Flash each **ESP32-POE-ISO** with the corresponding sketch in `Arduino/`:

  * `pumps_node.ino` → base `pumps/01`
  * `ultra_node.ino` → base `ultra/01`
  * `heat_node.ino` → base `heat/01` (PID + safety + thermistor/ADS1015)
* Edit Wi-Fi/Ethernet/MQTT credentials and base topics in each sketch if needed.
* Connect I²C (Qwiic), relays, pumps/heaters/ultrasonic hardware.

### Python Controller

* Requires Python 3.10+ and `paho-mqtt` (v2 API):

  ```bash
  pip install "paho-mqtt>=2.0.0"
  ```
* Use `iot_mqtt.py` from your PC to command and monitor all nodes.

---

## MQTT Topic Structure

| Topic pattern                      | Pub/Sub | Who → Who | Example payload                                   | Purpose                           |
| ---------------------------------- | :-----: | --------- | ------------------------------------------------- | --------------------------------- |
| `<base>/cmd/<n>`                   |   Pub   | PC → ESP  | `SET:42` · `ON` · `ON:1500` · `PWM:35` · `PID:ON` | Control channel per channel `<n>` |
| `<base>/state/<n>`                 |   Sub   | ESP → PC  | `ON` · `OFF` *(retained)*                         | Relay/actuator state per channel  |
| `<base>/temp/<n>`                  |   Sub   | ESP → PC  | `39.6`                                            | Temperature (heater node)         |
| `<base>/target/<n>` (or `set/<n>`) |   Sub   | ESP → PC  | `42.0` *(retained)*                               | PID setpoint (heater)             |
| `<base>/status`                    |   Sub   | ESP → PC  | `ONLINE` / `OFFLINE` *(retained)*                 | Node availability                 |
| `<base>/heartbeat`                 |   Sub   | ESP → PC  | `1` every 15 s                                    | Keep-alive                        |

Typical bases:

* Pumps: `pumps/01`
* Ultrasonic: `ultra/01`
* Heaters: `heat/01`

---

## Python API (`iot_mqtt.py`)

### Classes

* **`PumpMQTT`** — 4-channel Qwiic Quad Relay control (pumps).
* **`UltraMQTT`** — 2-channel relay control (ultrasonic drivers).
* **`HeatMQTT`** — Dual SSR heater control with **PID** + **thermistor feedback** (ADS1015).

### Command Reference (payloads to `<base>/cmd/<n>`)

**Pumps (`pumps/01`)**

* `ON` — turn channel `<n>` on
* `OFF` — turn channel `<n>` off
* `ON:<ms>` — on for `<ms>` milliseconds (auto-off)

**Ultrasonic (`ultra/01`)**

* `ON`, `OFF`, `ON:<ms>` — same semantics as pumps

**Heaters (`heat/01`)**

* `ON`, `OFF` — manual control (disables PID on that channel)
* `ON:<ms>` — timed on
* `PWM:<0–100>` — manual slow-PWM duty % (disables PID)
* `SET:<tempC>` — set PID target; retained at `target/<n>`
* `PID:ON` / `PID:OFF` — enable/disable PID loop for `<n>`
* `GET` — request immediate temperature publish on `temp/<n>`

---

## Example Usage

```python
# iot_mqtt_demo.py
import time
from iot_mqtt import start_broker_if_needed, stop_broker
from iot_mqtt import PumpMQTT, UltraMQTT, HeatMQTT

# Optional: start a local Mosquitto on Windows if not already running
proc = start_broker_if_needed()  # comment if your broker is already running

# Create clients (replace IP with your broker's LAN IP)
pumps = PumpMQTT(broker="192.168.0.101", username="pico1",  password="pump",
                 base_topic="pumps/01", client_id="pyctl-pumps")
ultra = UltraMQTT(broker="192.168.0.101", username="ultra1", password="ultra",
                  base_topic="ultra/01", client_id="pyctl-ultra")
heat  = HeatMQTT(broker="192.168.0.101", username="heat1",  password="heat",
                 base_topic="heat/01",  client_id="pyctl-heat")

# Connect + start background loops
pumps.ensure_connected(); ultra.ensure_connected(); heat.ensure_connected()
time.sleep(1)  # let subscriptions settle

# Quick status snapshots (retained + live)
pumps.status(seconds=2.0)
ultra.status(seconds=2.0)
heat.status(seconds=2.0)

# ---- Control examples ----
# Pumps
pumps.on(1); time.sleep(1)
pumps.on(1, 2000)            # auto-off after 2s
time.sleep(2.5)

# Ultrasonic
ultra.on_for(2, 1500)        # ch2 for 1.5s
time.sleep(2)

# Heaters (PID demo)
heat.set_base_temp(1, 42.0)  # retained at heat/01/target/1
heat.pid_on(1)

# Read temp for ~30s
for _ in range(30):
    try:
        t = heat.get_base_temp(1, timeout_s=2.0)
        print("Heater1 Temp =", t)
    except TimeoutError:
        print("Temp read timeout")
    time.sleep(1)

# Stop PID and ensure OFF
heat.pid_off(1)
heat.set_pwm(1, 0)
heat.off(1)

# Cleanup
pumps.disconnect(); ultra.disconnect(); heat.disconnect()
stop_broker(proc)
```

---

## Wiring & Safety Notes

* **PoE**: Each ESP32-POE-ISO is powered via PoE switch (GS308EPP).
* **I²C/Qwiic**: Keep I²C leads short (<30 cm).
* **Pumps**: Add **flyback diodes** across DC pump terminals.
* **Heaters (12–24 V)**: Add **TVS + 1000 µF + 0.1 µF** across the rail; fuse the DC source.
* **Ultrasonic & AC**: Fuse the AC input; switch via relay/SSR rated for mains; isolate mains wiring physically.
* **Grounding/EMI**: Keep sensor lines away from AC/heater wiring; use twisted pairs and strain relief.

---

## Troubleshooting

* **No broker connection**: Confirm broker IP/port (1883), firewall rules, and creds.
* **No ESP topics**: Check node base topics (`pumps/01`, `ultra/01`, `heat/01`), and that Ethernet link is up.
* **Temps look wrong**: Verify ADS1015 gain, thermistor wiring, and constants (R25, B-value, 10 kΩ divider).
* **PID overshoot**: Lower `Kp` / enable safety rate brake in the heater sketch; verify PWM scaling and SSR wiring.
* **Retained state confusion**: Clear retained topics by publishing empty retained messages or restart nodes.

---

## License

MIT (see `LICENSE`)

---
