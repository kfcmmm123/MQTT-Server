# ESP32-POE-ISO IoT Control System

**Modular MQTT-Based Control for Pumps, Ultrasonic Drivers, and Heaters**

This project implements a **multi-device IoT control system** using **ESP32-POE-ISO** boards connected via **MQTT over Ethernet**.
Each node manages a specific subsystem — **Pumps**, **Ultrasonic Drivers**, or **Heaters** — with real-time monitoring, PID control, and safety supervision.

---

## Table of Contents

* [Features](#features)
* [Repository Structure](#repository-structure)
* [System Architecture](#system-architecture)
* [Setup](#setup)

  * [Broker (Mosquitto)](#broker-mosquitto)
  * [ESP32 Nodes](#esp32-nodes)
  * [Python Controller](#python-controller)
* [MQTT Topic Structure](#mqtt-topic-structure)
* [Python API (`iot_mqtt.py`)](#python-api-iot_mqttpy)

  * [Classes](#classes)
  * [Command Reference (payloads to `<base>/cmd/<n>`)](#command-reference-payloads-to-basecmdn)
* [Example Usage](#example-usage)
* [Bill of Materials (BOM)](#bill-of-materials-bom)
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
├── iot_mqtt.py                 # Unified Python MQTT control library
├── MQTT_Demo.ipynb             # Example usage notebook (Jupyter)
│
├── Pump_client/
│   ├── Pump_client.ino         # 4-channel quad relay pump controller (Qwiic Quad Relay)
│   └── README.md               # Pump system documentation
│
├── Ultrasonic_client/
│   ├── Ultrasonic_single_client.ino   # 2 single relay ultrasonic driver controller
│   ├── Ultrasonic_quad_client.ino   # 2-channel quad relay ultrasonic driver controller
│   └── README.md               # Ultrasonic system documentation
│
├── Heater_client/
│   ├── Heater_client.ino       # 2-channel SSR heater controller (PID + safety)
│   └── README.md               # Heater system documentation
│
├── mosquitto_config/
│   ├── mosquitto.conf          # Mosquitto broker configuration
│   └── aclfile.txt             # Access control list (ACL) file
│
└── README.md                   # Main project documentation (this file)

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

## Setup

### Broker (Mosquitto)

1. **Install Mosquitto** on your **PC** (Windows, Linux, or macOS) or a **Raspberry Pi**.

   * Default listener: **TCP 1883**
   * You can download it from [https://mosquitto.org/download/](https://mosquitto.org/download/)

2. **Start the broker:**

   * **Option A — Manual:**
     Run the `mosquitto.exe` executable from PowerShell or Terminal:

     ```bash
     mosquitto -v
     ```
   * **Option B — Auto-launch via Python:**
     Use the helper in `iot_mqtt.py` to start Mosquitto automatically on Windows:

   > It launches `mosquitto.exe` via Python’s `subprocess` — make sure to **edit the executable path** of `mosq_exe` and `mosq_conf` in the script to match your Mosquitto installation directory.

     ```python
     from iot_mqtt import start_broker_if_needed
     proc = start_broker_if_needed()
     ```

     You can do this either in a standard Python session or in Jupyter (`MQTT_Demo.ipynb`).

3. Confirm that the broker is running:

   * Default port: **1883**
   * Test connection using:

     ```bash
     mosquitto_sub -h 127.0.0.1 -t test -v
     mosquitto_pub -h 127.0.0.1 -t test -m "hello"
     ```
   * You should see `test hello` echoed back.

### ESP32 Nodes

* Flash each **ESP32-POE-ISO** with the corresponding sketch in `Arduino/`:

  * `pumps_node.ino` → base `pumps/01`
  * `ultra_node.ino` → base `ultra/01`
  * `heat_node.ino` → base `heat/01` 
* Edit Wi-Fi/Ethernet/MQTT credentials and base topics in each sketch if needed.
* Connect I²C (Qwiic), relays, pumps/heaters/ultrasonic hardware.

### Python Controller

* Requires Python 3.10+ and `paho-mqtt` (v2 API):

  ```bash
  pip install "paho-mqtt>=2.0.0"
  ```

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

## Bill of Materials (BOM)

| Category                 | Item                       | Description                                                        | Example / Source                                                                                                                                                  |
| ------------------------ | -------------------------- | ------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Server**               | Ethernet Switch            | 8-port Gigabit PoE+ switch (123 W budget)                          | [Netgear GS308EPP](https://www.amazon.ca/NETGEAR-Gigabit-Ethernet-Managed-GS308EP/dp/B08MBFLMDC)                                                                  |
|                          | Ethernet Cables            | Cat5e / Cat6 PoE-compatible Ethernet cables (one per node)         | —                                                                                                                                                                 |
| **ESP Node**             | Main Controller            | PoE-enabled microcontroller with Ethernet + isolation + I²C + GPIO | [Olimex ESP32-POE-ISO](https://www.digikey.ca/en/products/detail/olimex-ltd/ESP32-POE-ISO-EA/10258722)                                                            |
| **Pumps Node**           | Peristaltic Pump           | 12 V micro peristaltic pump for fluid dispensing                   | [Kamoer KPHM100](https://www.kamoer.com/us/product/detail.html?id=10021)                                                                                          |
|                          | Relay Board                | 4-channel I²C relay for DC load control                            | [SparkFun Qwiic Quad Relay](https://www.sparkfun.com/sparkfun-qwiic-quad-relay.html)                                                                              |
| **Ultrasonic Node**      | Driver + Transducer        | 120 W / 40 kHz AC ultrasonic cleaning driver + transducer          | [Dacvgog Kit (Amazon)](https://www.amazon.ca/Dacvgog-Ultrasonic-Transducer-Cleaning-Machines/dp/B0BC7Q8DY1)                                                       |
|                          | Relay Board                | Two I²C single relays for AC driver switching                      | [SparkFun Qwiic Single Relay](https://www.sparkfun.com/sparkfun-qwiic-single-relay.html)                                                                          |
| **Heater Node**          | PTC Heater                 | Aluminum-clad PTC heater (12 – 220 V options)                      | [PTC 60×28×5 mm](https://www.ptcyidu.com/heating-element-hair-dryer-accessories-12-220v-60-270-degrees-coffee-maker-ptc-heaters-with-aluminum-shell-60x28x5mm340) |
|                          | Thermistor                 | 10 kΩ NTC temperature sensor (1 per channel)                       | [EPCOS B57560G104F](https://www.digikey.ca/en/products/detail/epcos-tdk-electronics/B57560G104F/1659138)                                                          |
|                          | Divider Resistor           | 10 kΩ 1% precision resistor (for voltage divider)                  | [Yageo RC1206FR-0710KL](https://www.digikey.ca/en/products/detail/yageo/RC1206FR-0710KL/731456)                                                                   |
|                          | ADC                        | 12-bit / 4-channel I²C ADC for thermistor inputs                   | [Adafruit ADS1015](https://www.adafruit.com/product/1083)                                                                                                         |
|                          | Solid-State Relay          | Dual SSR module for heater control                                 | [SparkFun Qwiic Dual Solid-State Relay](https://www.sparkfun.com/sparkfun-qwiic-dual-solid-state-relay.html)                                                      |
| **Power and Protection** | Power Supply               | 12 V DC regulated PSU (≥ 5 A depending on pump count)              | [Mean Well EDR-120-12](https://www.digikey.ca/en/products/detail/mean-well-usa-inc/EDR-120-12/7702913)                                                            |
|                          | Capacitor ( Electrolytic ) | 1000 µF / 25 V for DC rail stabilization                           | —                                                                                                                                                                 |
|                          | Capacitor ( Ceramic )      | 0.1 µF decoupling capacitor                                        | —                                                                                                                                                                 |
|                          | TVS Diode                  | 12 V transient voltage suppressor (600 W)                          | [Littelfuse SMBJ12A](https://www.digikey.ca/en/products/detail/littelfuse-inc/SMBJ12A/1049868)                                                                    |
|                          | MOV                        | 470 V 4.5 kA Varistor                                              | [Bourns MOV-14D471K](https://www.digikey.ca/en/products/detail/bourns-inc/MOV-14D471K/2799087)                                                                    |
|                          | E-Stop Switch              | Normally closed emergency stop button                              | [Omron A22NE-M-P202-N-B](https://www.digikey.ca/en/products/detail/omron-automation-and-safety/A22NE-M-P202-N-B/9190908)                                         |
|                          | Contactor                  | Contactor 3 Pole 12 A 120 VAC                                      | [Schneider Electric DPE12G7](https://www.digikey.ca/en/products/detail/schneider-electric/DPE12G7/15858518)                                                       |

**Full Digi-Key Cart:** [Complete Digi-Key Parts List](https://www.digikey.ca/en/mylists/list/V301MIOCVD)

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
