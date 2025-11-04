# ESP32-POE-ISO IoT Control System
**Modular MQTT-Based Automation for Pumps, Ultrasonic Drivers, and Heaters**

This repository hosts a **multi-device IoT control platform** built on **ESP32-POE-ISO** boards communicating via **MQTT over Ethernet**.  
Each ESP32 node acts as a standalone controller for a subsystem (Pump, Ultrasonic, Heater, pH probe, or Biologic), while a Python controller (`iot_mqtt.py`) provides orchestration, supervision, and visualization.

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
│
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
│   ├── ph/                          # pH probe monitor/controller
│   │   ├── pH_Safety_client.ino     # Safety-enabled pH firmware
│   │   └── README.md                # pH setup guide
│   │
│   ├── biologic/                    # Biologic controller
│   │   ├── Biologic_Safety_client.ino          # Safety-enabled biologic firmware
│   │   └── README.md                # Biologic setup guide
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
         ┌──────────────────┬───────────────────┼──────────────────┬──────────────────┐
         │                  │                   │                  │                  │
┌────────┴───────┐  ┌───────┴────────┐  ┌───────┴───────┐  ┌───────┴───────┐  ┌───────┴───────┐
│ ESP32-POE-ISO  │  │ ESP32-POE-ISO  │  │ ESP32-POE-ISO │  │ ESP32-POE-ISO │  │ ESP32-POE-ISO │
│ Node: pumps/01 │  │ Node: ultra/01 │  │ Node: heat/01 │  │ Node: ph/01   │  │ Node: bio/01  │
│ Relays (pumps) │  │ Relays (ultra) │  │ SSR + ADS1015 │  │ EZO pH meter  │  │ 16 chan relay │
└────────────────┘  └────────────────┘  └───────────────┘  └───────────────┘  └───────────────┘

````

---

## Device Overview

| Node                | Features                                              | Firmware                             |
| ------------------- | ----------------------------------------------------- | ------------------------------------ |
| **Pump Node**       | Controls up to 4 DC peristaltic pumps via Qwiic relay | `devices/pump/Pump_Safety_client.ino`|
| **Ultrasonic Node** | Controls 2 ultrasonic driver boards via Qwiic relay    | `devices/ultrasonic/Ultrasonic_Safety_client.ino` |
| **Heater Node**     | Controls 2 heaters via Qwiic SSR + thermistor sensing + PID + safety brake | `devices/heater/Heater_Safety_client.ino`    |
| **pH Node**         | Publishes pH readings; supports oneshot and periodic polling with safety | `devices/ph/pH_Safety_client.ino` |
| **Biologic Node**   | Controls 16 channels relay via MCP23017 I²C expander | `devices/biologic/Biologic_Safety_client.ino` |

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
- Python 3.8+ with `paho-mqtt` library
- Arduino IDE with ESP32 support
- ESP32-POE-ISO boards (Olimex)
- SparkFun Qwiic relay modules
- Ethernet switch and router
- EZO pH probe (optional, for pH readings)

### 1. Network Setup
1. **Connect all ESP32-POE-ISO devices to an Ethernet switch**
2. **Connect switch to router** (or use router's built-in switch)
3. **Find broker IP address:**
   ```bash
   # Windows
   ipconfig
   
   # Linux/Mac
   ip addr show
   # or
   ifconfig
   ```
   Note the IP address (e.g., `192.168.1.100`) for the next steps.

### 2. Install Mosquitto Broker
```bash
# Windows
choco install mosquitto

# Linux
sudo apt install mosquitto mosquitto-clients
```

### 3. Configure Broker
1. **Copy configuration files:**
   ```bash
   # Copy to Mosquitto directory
   cp mosquitto_config/mosquitto.conf "C:/Program Files/mosquitto/"
   cp mosquitto_config/aclfile.txt "C:/Program Files/mosquitto/"
   ```

2. **Create users:**
   ```bash
   cd "C:/Program Files/mosquitto/"
   mosquitto_passwd -c passwd pyctl-controller
   mosquitto_passwd    passwd pump1
   mosquitto_passwd    passwd ultra1
   mosquitto_passwd    passwd heat1
   mosquitto_passwd    passwd ph1
   mosquitto_passwd    passwd bio1
   ```

> Note: If you use custom usernames/passwords, update both `mosquitto_config/aclfile.txt` and the credentials in the notebook/code.

3. **Start broker:**
   ```bash
   mosquitto -v -c "C:/Program Files/mosquitto/mosquitto.conf"
   ```
   > You can also skip this step and start the broker inside the Jupyter notebook (recommended)

### 4. Flash ESP32 Devices
1. **Open Arduino IDE** and install ESP32 board support
2. **Install required libraries:**
   - `PubSubClient` (MQTT)
   - `SparkFun_Qwiic_Relay` (relay control)
   - `Wire` (I²C)
3. **Update broker IP** in each firmware file to match your network
4. **Flash firmware:**
    - **Pump Node**: `devices/pump/Pump_Safety_client.ino`
    - **Ultrasonic Node**: `devices/ultrasonic/Ultrasonic_Safety_client.ino`
    - **Heater Node**: `devices/heater/Heater_Safety_client.ino`
    - **pH Node**: `devices/ph/pH_Safety_client.ino`
   
   **Note**: ESP32 firmware automatically generates unique client IDs based on MAC address (e.g., `pumps01-A1B2C3D4`). This prevents client ID conflicts when multiple devices connect.
   
5. **Monitor connection:**
   - Use Arduino Serial Monitor to confirm MQTT connection and command logs

### 5. Run Jupyter Notebook
1. **Open `iot_mqtt/MQTT_Demo.ipynb`**

2. **Update IP and credentials** in the notebook to match your broker and ACL users

3. **Execute cells block by block:**
   - **Cell 0**: Install library
   - **Cell 1**: Import libraries and start broker (optional if already running)
   - **Cell 2**: Create device connections
   - **Cell 3**: Test pump control
   - **Cell 4**: Test ultrasonic control
   - **Cell 5**: Test heater PID control
   - **Cell 6**: Test pH monitoring
   - **Cell 7**: Cleanup and shutdown

4. **Monitor output logs:**
   - Check Jupyter cell outputs for connection/status
   - Watch the Mosquitto console (or service logs) for MQTT traffic
   - Verify device status messages in notebook output

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
- **[pH Controller](./devices/ph/README.md)** - pH probe monitoring and commands

### Development Resources
- **[Jupyter Demo](./iot_mqtt/MQTT_Demo.ipynb)** - Interactive examples and testing
- **[CAD Models](./hardware/CAD_models/)** - 3MF files for 3D printing enclosures
- **[BOM](./hardware/BOM/)** - Bill of Materials for the setup

---

## Troubleshooting

### Common Issues

**ESP32 won't connect to MQTT**
- Check broker IP address in firmware (use `ipconfig` to verify network settings)
- Verify Ethernet connection and PoE power supply
- Confirm MQTT credentials match ACL configuration

**Broker stops responding with continuous client reconnections**
> **IMPORTANT**: This indicates a critical broker failure or client ID conflict!

**Root cause**: When a client disconnects without properly closing, the broker retains the session. If the client (or server) tries to reconnect with the same client ID, the broker attempts to connect both the old session and the new connection, causing a conflict loop where it keeps kicking one and connecting the other.

Prevention (already implemented):
- Python clients use unique client IDs (`base-hostname-PID`) and MQTT v5 with clean start + session expiry 0
- ESP32 firmware uses MAC-based unique client IDs (e.g., `pumps01-A1B2C3D4`)
- Broker expires persistent client sessions after 1 day of inactivity

Immediate fix:
1. Stop the broker immediately: `taskkill /IM mosquitto.exe /F` (Windows) or `sudo systemctl stop mosquitto` (Linux)
2. Wait until `netstat -ano | findstr :1883` shows no output
3. Restart Jupyter/kernel to clear Python client connections
4. Restart the broker: `mosquitto -v -c mosquitto.conf`

**Broker startup fails**
- Check for existing broker process: `netstat -ano | findstr :1883`
- If process exists, kill it: `taskkill /IM mosquitto.exe /F`
- Verify broker configuration: `ipconfig`, and port availability (default 1883)

**Devices show OFFLINE**
- Verify controller beacon is running (`pyctl/heartbeat` topic)
- Check heartbeat intervals (default 15s)
- Ensure LWT (Last Will & Testament) is properly configured

**Python API connection fails**
- Install required dependencies: `pip install paho-mqtt`
- Check broker address and port (default 1883)
- Verify user credentials match broker configuration (also check for user access rights in ACL file)
- **Check Python controller log file** - The `iot_mqtt.py` has built-in logging that writes detailed connection and communication logs to a log file

**Python controller issues**
- Check the log file created by `iot_mqtt.py` for detailed error messages and connection status
- Log file contains MQTT connection attempts, device status updates, and communication errors
- Look for authentication failures, connection timeouts, or topic permission errors in the logs

### Getting Help

**Debug MQTT Communication**
```bash
# Monitor all MQTT traffic
mosquitto_sub -v -t "#" -u pyctl-controller -P controller

# Check specific device topics
mosquitto_sub -v -t "pumps/01/#" -u pump1 -P pump
mosquitto_sub -v -t "heat/01/#" -u heat1 -P heat
```

**Additional Resources**
- **Check Python controller log file** - Built-in logging provides detailed connection status, MQTT communication, and error messages
- Check device-specific README files for detailed troubleshooting
- Review broker logs for authentication and connection issues
- Verify network connectivity with `ping <broker-ip>`
- Test broker connectivity: `mosquitto_pub -h <broker-ip> -t "test" -m "hello"`

---

## License

**MIT License**

© 2025 Alan Yang — Acceleration Consortium, University of Toronto

