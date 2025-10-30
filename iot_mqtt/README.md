# Python MQTT API — ESP32 Device Control

This document explains how to use the **Python MQTT API (`iot_mqtt.py`)** to control ESP32-POE-ISO device nodes (pumps, heaters, ultrasonic drivers) via MQTT communication.

## Overview

The MQTT network consists of:
- **Broker:** Mosquitto server (usually on your PC or Raspberry Pi)
- **Controller:** Python beacon (publishes heartbeat + commands)
- **ESP32 Nodes:** individual clients (pumps, ultrasonic, heater)

Each device uses authentication and a dedicated topic subtree for clean separation.

## Python MQTT Library (`iot_mqtt.py`)

### Quick Start

 💡 **Use the Jupyter Demo Notebook**

For a guided, ready-to-run experience, open **`MQTT_Demo.ipynb`** in Jupyter Notebook.
It walks through:

* starting the broker
* connecting the beacon and all device nodes
* sending test commands (`ONESHOT`, `START:<ms>`, etc.)
* collecting and plotting live data (e.g., pH vs time)

This notebook provides step-by-step examples using the same API described here — perfect for both testing and visualization.


### Command Payloads

| Category    | Command                | Example Payload | Description                           |
| ----------- | ---------------------- | --------------- | ------------------------------------- |
| **Pump + Ultra** | `ON`, `OFF`, `ON:<ms>` | `ON:2000`       | Turn relay ON for given ms (auto-off) |
| **Heater**  | `PWM:<0–100>`          | `PWM:50`        | Apply PWM % output                    |
|             | `SET:<temp>`           | `SET:42.0`      | Set PID target                        |
|             | `PID:ON` / `PID:OFF`   | —               | Enable/disable PID loop               |
|             | `GET`                  | —               | Request temperature publish           |
| **pH Probe**| `START:<ms>`           | `START:5000`    | Begin periodic polling (reads pH every `<ms>` milliseconds)    |
|             | `STOP`                 | —               | Stop periodic polling                                          |
|             | `ONESHOT`              | —               | Take a single pH reading immediately and publish|
|             | `<raw command>`  | `i`, `Status,?`, `Cal,mid,7.00` | Forward raw text command to EZO pH board and publish its reply |

---

## Topic Map Summary

```       

# Controller supervision (beacon)
pyctl/
 ├─ status      ↔  controller ONLINE/OFFLINE
 └─ heartbeat   ↔  controller heartbeat signal

# Pump / Ultrasonic / Heater nodes
pumps/01/
 ├─ cmd/1..3       ←  pump control (ON, OFF, ON:<ms>)
 ├─ state/1..3     →  relay states
 ├─ status         ↔  ONLINE/OFFLINE
 └─ heartbeat      ↔  periodic keepalive

ultra/01/
 ├─ cmd/1..2       ←  ultrasonic control (ON, OFF, ON:<ms>)
 ├─ state/1..2     →  driver states
 ├─ status         ↔  ONLINE/OFFLINE
 └─ heartbeat      ↔  periodic keepalive

heat/01/
 ├─ cmd/1..2       ←  heater control (ON, OFF, SET:<temp>, PWM:<0–100>, PID:ON/OFF)
 ├─ temp/1..2      →  measured temperature
 ├─ target/1..2    →  PID target
 ├─ status         ↔  ONLINE/OFFLINE
 └─ heartbeat      ↔  periodic keepalive

# pH Probe node
ph/01/
 ├─ cmd            ←  START:<ms>, STOP, ONESHOT, Cal,mid,7.00
 ├─ ph             →  latest pH reading
 ├─ reply          →  command response or warning
 ├─ status         ↔  ONLINE/OFFLINE
 └─ heartbeat      ↔  periodic keepalive

```
---

## Safety Behavior

| Condition               | Action                                            |
| ----------------------- | ------------------------------------------------- |
| No controller heartbeat | All outputs OFF                                   |
| MQTT disconnected       | All outputs OFF                                   |
| Ethernet down           | All outputs OFF                                   |
| Controller crash        | LWT sets `pyctl/status = OFFLINE` → all nodes OFF |
| ON command no duration  | Auto OFF after lease (`MAX_ON_LEASE_MS`)          |

---

## Troubleshooting

* **Broker rejects connection:** recheck user/pass and ACL.
* **No telemetry:** verify ESP node IP and Ethernet link.
* **Device doesn’t turn OFF after PC crash:** check LWT enabled and controller beacon running.
* **“CTRL timeout” warnings:** ensure first heartbeat received before timeout starts.

---

## References

* [Mosquitto Docs](https://mosquitto.org/documentation/)
* [Paho MQTT Python Client](https://pypi.org/project/paho-mqtt/)
* [ESP32 Arduino PubSubClient](https://github.com/knolleary/pubsubclient)

---

## 🪪 License

MIT © 2025 Alan Yang

