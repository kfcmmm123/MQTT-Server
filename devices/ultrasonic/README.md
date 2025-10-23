# Ultrasonic Controllers — ESP32-POE-ISO Driver Control

## Overview

ESP32-POE-ISO nodes control ultrasonic cleaning drivers via **SparkFun Qwiic Relay** modules. Multiple firmware variants support different relay configurations for AC or DC driver control. MQTT provides **ON**, **OFF**, and **timed ON** operation with safety supervision.

## Available Firmware

- **`Ultrasonic_quad_client.ino`** - 4-channel controller using SparkFun Qwiic Quad Relay
- **`Ultrasonic_single_client.ino`** - 2-channel controller using two Qwiic Single Relays
- **`Ultrasonic_Safety_client.ino`** - Safety-enhanced version with watchdog supervision

> **Recommended**: Use `Ultrasonic_Safety_client.ino` for production deployments.

## Hardware

* **MCU**: Olimex **ESP32-POE-ISO**
* **Relay**: Qwiic Quad Relay (use 2 channels), or two single relays
* **Loads**: Two ultrasonic **driver boards** (your “13913887466 120 W” boards), powered from AC (through SSR) or DC (through relay), per your wiring plan
* **Power**: PoE/USB for ESP, appropriate mains/DC for the drivers (with proper fusing)

## Wiring

* ESP32 I²C → Qwiic Relay
* Each ultrasonic driver input power (or enable line) routed through its relay
* If switching **mains**, prefer **SSR** and keep creepage/clearance; fuse appropriately

## MQTT

**Base**: `ultra/01`

| Topic                |   Dir  | Example Payload                 | Notes                 |
| -------------------- | :----: | ------------------------------- | --------------------- |
| `ultra/01/cmd/<n>`   | PC→ESP | `ON` · `OFF` · `ON:1000`        | Channel `<n>` ∈ {1,2} |
| `ultra/01/state/<n>` | ESP→PC | `ON`/`OFF` *(retained)*         |                       |
| `ultra/01/status`    | ESP→PC | `ONLINE`/`OFFLINE` *(retained)* |                       |
| `ultra/01/heartbeat` | ESP→PC | `1` every 15 s                  |                       |

**Commands**

* `ON`, `OFF`, `ON:<ms>` (same semantics as pumps)

## Configure & Build

1. **Arduino IDE Setup**:
   - Install ESP32 board support package
   - Install required libraries: `PubSubClient`, `SparkFun_Qwiic_Relay`, `Wire`

2. **Configuration**:
   - Set `MQTT_BROKER_IP` to your broker's IP address
   - Configure MQTT credentials (username/password)
   - Set base topic to `ultra/01`
   - Configure relay I²C addresses for your setup

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
import time
from iot_mqtt import UltraMQTT

# Connect to ultrasonic controller
ultra = UltraMQTT(broker="192.168.0.100", username="ultra1", password="ultra", base_topic="ultra/01")
ultra.ensure_connected()

# Test operations
ultra.on(1, 1500)        # Channel 1 for 1.5 seconds
ultra.status(2.0)        # Get status for 2 seconds
ultra.off(1)             # Ensure channel 1 is off
ultra.disconnect()
```

See [Python API Documentation](../iot_mqtt/README.md) for complete usage examples.

## Troubleshooting

### Connection Issues
- **No `ONLINE` status**: Check PoE link, USB power, and broker IP address
- **MQTT connection fails**: Verify broker is running and credentials match ACL configuration

### Hardware Issues
- **Both channels toggle together**: Verify correct relay channel addressing in code and wiring
- **Relay not switching**: Check I²C communication and relay power supply
- **Driver not responding**: Verify driver power supply and relay contact wiring

### Safety Issues
- **Driver stays on after timeout**: Check controller beacon is running (`pyctl/heartbeat` topic)
- **Random shutdowns**: Verify Ethernet connection stability and PoE power supply

### Mains Power Safety
- **AC driver control**: Use SSR and proper snubbers/fusing for mains switching
- **Wiring safety**: Keep low-voltage control and mains power wiring properly segregated
- **Isolation**: Ensure proper creepage/clearance distances for mains connections

See [Main Troubleshooting Guide](../../README.md#troubleshooting) for more details.

