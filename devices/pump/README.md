# Pump Controllers — ESP32-POE-ISO MQTT Control

## Overview

ESP32-POE-ISO nodes control DC peristaltic pumps via **SparkFun Qwiic Relay** modules over I²C. Multiple firmware variants are available for different relay configurations. Each node exposes MQTT commands for **ON**, **OFF**, and **timed ON** operation with safety supervision.

## Available Firmware

- **`Pump_quad_client.ino`** - 4-pump controller using SparkFun Qwiic Quad Relay
- **`Pump_single_client.ino`** - Single pump controller using one Qwiic Single Relay  
- **`Pump_Safety_client.ino`** - Safety-enhanced version with watchdog supervision

> **Recommended**: Use `Pump_Safety_client.ino` for production deployments.

## Hardware

* **MCU**: Olimex **ESP32-POE-ISO**
* **Relay**: SparkFun **Qwiic Quad Relay** (I²C, default addr `0x6D`), or 3/4 single relays
* **Power**: PoE or 5 V via USB (PoE preferred)
* **Cabling**: Qwiic/JST-SH (I²C); pump power routed through relay contacts

## Wiring

* ESP32-POE-ISO I²C → Qwiic Quad Relay (Qwiic cable)
* Pump positive lead → relay **COM**; relay **NO** → +12 V supply (or whatever your pump supply is)
* Pump negative lead → supply GND (common)
* Add per-pump flyback diode if the pump is inductive and relay is mechanical (across pump leads, band to +)

## MQTT

**Base**: `pumps/01`

**Topics**

| Topic                |   Dir  | Example Payload                   | Notes                  |
| -------------------- | :----: | --------------------------------- | ---------------------- |
| `pumps/01/cmd/<n>`   | PC→ESP | `ON` · `OFF` · `ON:1500`          | Channel `<n>` ∈ {1..4} |
| `pumps/01/state/<n>` | ESP→PC | `ON` / `OFF` *(retained)*         | Relay state            |
| `pumps/01/status`    | ESP→PC | `ONLINE` / `OFFLINE` *(retained)* | Node availability      |
| `pumps/01/heartbeat` | ESP→PC | `1` every 15 s                    | Keep-alive             |

**Commands (payloads to `pumps/01/cmd/<n>`)**

* `ON` — turn pump <n> on
* `OFF` — turn pump <n> off
* `ON:<ms>` — on for `<ms>` ms, then auto-off

## Configure & Build

1. **Arduino IDE Setup**:
   - Install ESP32 board support package
   - Install required libraries: `PubSubClient`, `SparkFun_Qwiic_Relay`, `Wire`

2. **Configuration**:
   - Set `MQTT_BROKER_IP` to your broker's IP address
   - Configure MQTT credentials (username/password)
   - Set base topic to `pumps/01` (or customize)

3. **Upload**:
   - Select board: **ESP32 Dev Module**
   - Upload via USB to ESP32-POE-ISO

## Required Arduino Libraries

```cpp
#include <PubSubClient.h>           // MQTT communication
#include <SparkFun_Qwiic_Relay.h>   // Relay control
#include <Wire.h>                   // I²C communication
```

## Test (Python)

```python
from iot_mqtt import PumpMQTT

# Connect to pump controller
pumps = PumpMQTT(broker="192.168.0.100", username="pump1", password="pump", base_topic="pumps/01")
pumps.ensure_connected()

# Test operations
pumps.on(1, 2000)      # Pump 1 for 2 seconds
pumps.status(2.0)      # Get status for 2 seconds
pumps.off(1)           # Ensure pump 1 is off
pumps.disconnect()
```

See [Python API Documentation](../iot_mqtt/README.md) for complete usage examples.

## Troubleshooting

### Connection Issues
- **No `ONLINE` status**: Check PoE link, USB power, and broker IP address
- **MQTT connection fails**: Verify broker is running and credentials match ACL configuration

### Hardware Issues  
- **I²C not found**: Run I²C scanner; ensure Qwiic cable is properly seated; default address is `0x6D`
- **Pump doesn't run**: Confirm external power supply, correct wiring to NO/COM contacts
- **Relay clicking but no pump**: Check flyback diodes for inductive loads

### Safety Issues
- **Pump stays on after timeout**: Check controller beacon is running (`pyctl/heartbeat` topic)
- **Random shutdowns**: Verify Ethernet connection stability and PoE power supply

See [Main Troubleshooting Guide](../../README.md#troubleshooting) for more details.

