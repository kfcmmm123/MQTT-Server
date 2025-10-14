# Pumps Node — Qwiic Quad Relay (4× pumps)

## Overview

ESP32-POE-ISO drives a **SparkFun Qwiic Quad Relay** over I²C. Each channel powers a DC pump (through the relay). The node exposes simple MQTT commands for **ON**, **OFF**, and **timed ON**.

## Hardware

* **MCU**: Olimex **ESP32-POE-ISO**
* **Relay board**: SparkFun **Qwiic Quad Relay** (I²C, default addr `0x6D`)
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

* In sketch: set `MQTT_BROKER_IP`, user/pass (if auth), and base `pumps/01` (optional).
* Board: **ESP32** → “ESP32 Dev Module” (Olimex is ESP32), Upload via USB.
* Requires libraries: `PubSubClient`, `SparkFun_Qwiic_Relay`, `Wire`.

## Test (from PC)

```python
from iot_mqtt import PumpMQTT
p = PumpMQTT(broker="192.168.0.101", username="pico1", password="pump", base_topic="pumps/01")
p.ensure_connected()
p.on(1, 2000)   # pump 1 for 2 s
p.status(2.0)
p.off(1)
p.disconnect()
```

## Troubleshooting

* No `ONLINE`: check PoE link / USB power / broker IP.
* I²C not found: run an I²C scanner; ensure Qwiic cable seated; default address is `0x6D`.
* Pump doesn’t run: confirm external supply, correct wiring to NO/COM, and flyback diodes for inductive loads.

