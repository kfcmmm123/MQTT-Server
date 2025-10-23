# ESP32-POE-ISO IoT Control System
**Modular MQTT-Based Automation for Pumps, Ultrasonic Drivers, and Heaters**

This repository hosts a **multi-device IoT control platform** built on **ESP32-POE-ISO** boards communicating via **MQTT over Ethernet**.  
Each ESP32 node acts as a standalone controller for a subsystem (Pump, Ultrasonic, or Heater), while a Python controller (`iot_mqtt.py`) provides orchestration, supervision, and visualization.

---

## Project Overview

| Component | Description |
|------------|-------------|
| **ESP32-POE-ISO Nodes** | Independent nodes for each subsystem. Each node runs its own safety-supervised firmware. |
| **Python Controller** | Publishes heartbeat and commands; logs telemetry from all devices. |
| **Mosquitto Broker** | Central MQTT message hub managing device topics and access control. |
| **CAD Models** | Modular enclosures for each ESP node and relay system. |

Each subsystem connects through a unified topic schema and supports **autonomous safety shutdown**, **timed operation**, and **real-time telemetry**.

---

## Features

- **Modular MQTT Architecture:** each node operates independently but shares a common protocol.
- **Ethernet + PoE Isolation:** hardware-level safety and simplified wiring.
- **PID Temperature Control:** with ADS1015 ADC + NTC thermistors for precise heater regulation.
- **Relay Control (I²C/Qwiic):** reliable actuation of pumps, heaters, and ultrasonic drivers.
- **Safety Supervision:** heartbeat monitoring, link detection, MQTT reconnection handling, and auto-failsafe.
- **Cross-Platform Python API:** unified control, MQTT utilities, and broker automation.

---

## Repository Structure

```
MQTT-Server/
├── README.md                     # Project overview & quickstart
│
├── devices/                      # ESP32 firmware & documentation
│   ├── pump/                        # Pump control firmware
│   │   ├── Pump_quad_client.ino     # Quad relay pump controller
│   │   ├── Pump_single_client.ino   # Single relay pump controller  
│   │   ├── Pump_Safety_client.ino   # Safety-enabled version for single relay
│   │   └── README.md                # Pump setup guide
│   │
│   ├── ultrasonic/                  # Ultrasonic driver control
│   │   ├── Ultrasonic_quad_client.ino          # Quad relay ultrasonic controller
│   │   ├── Ultrasonic_single_client.ino        # Single relay ultrasonic controller
│   │   ├── Ultrasonic_Safety_client.ino        # Safety-enabled version for single relay
│   │   └── README.md                # Ultrasonic setup guide
│   │
│   └── heater/                      # Heater control with PID
│       ├── Heater_client.ino        # Main heater firmware
│       ├── Heater_Safety_client.ino # Safety-enabled version
│       └── README.md                # Heater setup guide
│
├── iot_mqtt/                     # Python control library
│   ├── iot_mqtt.py                  # Unified MQTT API
│   ├── MQTT_Demo.ipynb              # Example Jupyter notebook
│   └── README.md                    # Python API documentation
│
├── mosquitto_config/             # MQTT broker configuration
│   ├── mosquitto.conf               # Broker settings
│   ├── aclfile.txt                  # Access control list
│   └── README.md                    # Broker setup guide
│
└── hardware/                     # 3D printable enclosures
    ├── CAD_models/                  # 3MF files for printing
    └── README.md                    # Hardware assembly guide
```

---

## System Architecture

```text
                 ┌─────────────────────────────┐
                 │         MQTT Broker         │
                 │        (Mosquitto)          │
                 │       Port 1883 (TCP)       │
                 └──────────────┬──────────────┘
                                │
                         Ethernet TCP/IP
          ┌─────────────────────┼────────────────────┐
          │                     │                    │                    
┌─────────┴──────────┐  ┌───────┴────────┐   ┌───────┴─────────┐
│ ESP32-POE-ISO      │  │ ESP32-POE-ISO  │   │ ESP32-POE-ISO   │
│ Node: pumps/01     │  │ Node: ultra/01 │   │ Node: heat/01   │
│ Relays (pumps)     │  │ Relays (ultra) │   │ SSR + ADS1015   │
└────────────────────┘  └────────────────┘   └─────────────────┘

````

---

## Device Overview

| Node                | Features                                              | Firmware                             |
| ------------------- | ----------------------------------------------------- | ------------------------------------ |
| **Pump Node**       | Controls up to 4 DC peristaltic pumps via Qwiic relay | `devices/pump/Pump_Safety_client.ino`|
| **Ultrasonic Node** | Controls 2 ultrasonic driver board via Qwiic relay    | `devices/ultrasonic/Ultra_Safety_client.ino` |
| **Heater Node**     | Controls 2 heaters via Qwiic SSR + thermistor sensing + PID + safety brake | `devices/heater/Heater_Safety_client.ino`    |

Each firmware implements MQTT topic handlers, safety gating, and local timers for timed activation (`ON:<ms>`).

---

## Software Stack

| Layer             | Technology                      |
| ----------------- | ------------------------------- |
| **Hardware**      | ESP32-POE-ISO (Olimex)          |
| **Firmware**      | Arduino framework               |
| **Communication** | MQTT (paho-mqtt / PubSubClient) |
| **Broker**        | Mosquitto                       |
| **Python API**    | `iot_mqtt.py`                   |
| **Safety**        | Heartbeat + LWT watchdogs       |

---

## Security and Safety Summary

* **MQTT ACLs:** device-level topic restrictions in `mosquitto_config/aclfile.txt`
* **LWT Fail-safe:** controller disconnect triggers `OFFLINE` → all nodes shut down
* **Timeouts:** configurable per-device heartbeat timeouts
* **Hardware:** PoE isolation + fused 12 V supply + SSR isolation
* **Emergency Stop:** physical NC E-stop in series with actuator supply

---

## Quick Start

### Prerequisites
- ESP32-POE-ISO boards (Olimex)
- SparkFun Qwiic relay modules
- Python 3.8+ with `paho-mqtt` library
- Arduino IDE with ESP32 support

### 1. Setup MQTT Broker
```bash
# Install Mosquitto (Windows)
choco install mosquitto

# Or Linux
sudo apt install mosquitto mosquitto-clients

# Configure broker (see mosquitto_config/README.md)
mosquitto -v -c mosquitto_config/mosquitto.conf
```

### 2. Flash ESP32 Devices
1. Open Arduino IDE
2. Install ESP32 board support
3. Flash firmware from `devices/` folders:
    - **Pump Node**: `devices/pump/Pump_Safety_client.ino`
    - **Ultrasonic Node**: `devices/ultrasonic/Ultrasonic_Safety_client.ino`
    - **Heater Node**: `devices/heater/Heater_Safety_client.ino` 

### 3. Run Python Controller
```bash
# Install dependencies
pip install paho-mqtt

# Run example
python iot_mqtt/iot_mqtt.py
```

### 4. Monitor System
```bash
# Watch all MQTT traffic
mosquitto_sub -v -t "#" -u pyctl-controller -P controller
```

### 5. Test Devices
```python
import time
from iot_mqtt import PumpMQTT, UltraMQTT, HeatMQTT

# Control pumps
pumps = PumpMQTT(broker="192.168.0.100", username="pump1", password="pump")
pumps.ensure_connected()
pumps.on(1, 2000)  # Pump 1 for 2 seconds

# Control pumps
ultra = UltraMQTT(broker="192.168.0.100", username="ultra1", password="ultra")
ultra.ensure_connected()
ultra.on(1, 2000)  # Pump 1 for 2 seconds

# Control heaters with PID
heat = HeatMQTT(broker="192.168.0.100", username="heat1", password="heat")
heat.ensure_connected()
heat.set_target(1, 42.0)  # Set 42°C target
heat.pid_on(1)            # Enable PID control
time.sleep(2)             # On for 2 seconds
heat.set_target(1, 42.0)  # Set 42°C target
heat.pid_on(1)            # Enable PID control
```

---

## Documentation & Guides

### Setup & Configuration
- **[MQTT Broker Setup](./mosquitto_config/README.md)** - Mosquitto installation, users, ACL configuration
- **[Python API Guide](./iot_mqtt/README.md)** - Complete Python library documentation with examples
- **[Hardware Assembly](./hardware/README.md)** - 3D printable enclosures and assembly instructions

### Device-Specific Guides  
- **[Pump Controllers](./devices/pump/README.md)** - 4-pump relay control setup and wiring
- **[Ultrasonic Controllers](./devices/ultrasonic/README.md)** - AC driver control via relays
- **[Heater Controllers](./devices/heater/README.md)** - Dual SSR + PID temperature control

### Development Resources
- **[Jupyter Demo](./iot_mqtt/MQTT_Demo.ipynb)** - Interactive examples and testing
- **[CAD Models](./hardware/CAD_models/)** - 3MF files for 3D printing enclosures
- **[BOM](./hardware/BOM/)** - Bill of Materials for the setup

---

## Troubleshooting

### Common Issues

**ESP32 won't connect to MQTT**
- Check broker IP address in firmware
- Verify Ethernet connection and PoE power
- Confirm MQTT credentials match ACL configuration

**Devices show OFFLINE**
- Verify controller beacon is running (`pyctl/heartbeat` topic)
- Check heartbeat intervals (default 15s)
- Ensure LWT (Last Will & Testament) is properly configured

**Python API connection fails**
- Install required dependencies: `pip install paho-mqtt`
- Check broker address and port (default 1883)
- Verify user credentials match broker configuration

**Temperature readings erratic**
- Check thermistor wiring and pull-up resistors
- Verify ADS1015 I²C address and connections
- Calibrate thermistor constants (R25, B2585) for your specific sensors

### Getting Help
- Check device-specific README files for detailed troubleshooting
- Monitor MQTT topics with `mosquitto_sub -v -t "#"` for debugging
- Review broker logs for authentication and connection issues

---

## License

**MIT License**

© 2025 Alan Yang — Acceleration Consortium, University of Toronto

