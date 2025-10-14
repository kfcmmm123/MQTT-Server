# Ultrasonic Node — 2× drivers via Qwiic Relay

## Overview

ESP32-POE-ISO toggles **two ultrasonic driver boards** through a Qwiic relay board (can be the same SparkFun Quad Relay; you’ll use 2 channels). MQTT provides **ON**, **OFF**, **timed ON**.

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

* Set `MQTT_BROKER_IP`, credentials, base `ultra/01`.
* Same toolchain as pumps node; same libraries (`PubSubClient`, `SparkFun_Qwiic_Relay`).

## Test

```python
from iot_mqtt import UltraMQTT
u = UltraMQTT(broker="192.168.0.101", username="ultra1", password="ultra", base_topic="ultra/01")
u.ensure_connected()
u.on_for(1, 1500)  # ch1 for 1.5 s
u.status(2.0)
u.off(1)
u.disconnect()
```

## Troubleshooting

* If both channels toggle together: verify you addressed the *correct* relay channels in code and wiring.
* Mains drivers: use SSR and proper snubbers/fusing. Keep low-voltage and mains wiring segregated.

