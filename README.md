# 🌐 ESP32-POE-ISO IoT Control System  
**Modular MQTT-Based Control for Pumps, Ultrasonic Drivers, and Heaters**

This project implements a **multi-device IoT control system** using **ESP32-POE-ISO** boards connected via **MQTT over Ethernet**.  
Each node manages a specific subsystem — **Pumps**, **Ultrasonic Drivers**, or **Heaters** — with real-time monitoring, PID control, and safety supervision.

---

## 📋 Features

-  Unified **MQTT communication** for all devices  
-  Independent **ESP32-POE-ISO nodes** with Ethernet  
-  **PID temperature control** with thermistors and dual SSRs  
-  **Quad relay pump** control for fluid handling  
-  **Ultrasonic relay** driver control  
-  **Python control script (`iot_mqtt.py`)** for all subsystems  
-  **Safety protection**: over-temp rate brake, startup delay, fail-safe OFF  
-  Real-time telemetry publishing (temperature, states, heartbeat)

---

## 🧭 System Architecture

```text
                ┌─────────────────────────────┐
                │         MQTT Broker         │
                │     (Mosquitto on PC)       │
                │       Port 1883 (TCP)       │
                └──────────────┬──────────────┘
                               │
                        Ethernet TCP/IP
                               │
┌──────────────────────────────┴─────────────────────────────────────┐
│ ESP32-POE-ISO Nodes (Olimex)                                       │
│────────────────────────────────────────────────────────────────────│
│ pumps/01 → Qwiic Quad Relay (4 channels, fluid pumps)              │
│ ultra/01 → Qwiic Quad Relay (2 ultrasonic drivers)                 │
│ heat/01 → Dual SSR + ADS1015 + thermistors + PID + safety          │
└────────────────────────────────────────────────────────────────────┘
```

Each node runs an MQTT client, subscribes to control topics (`cmd/#`),  
and publishes telemetry (`state/#`, `temp/#`, `status`, `heartbeat`).

---

## Common Setup Notes

### Broker

* Use a local Mosquitto broker (PC or Pi). Default port: **1883**.
* Your Windows helper in `iot_mqtt.py` can **start** Mosquitto automatically (optional).

### Credentials

* Use distinct usernames per node if you like (e.g., `pump1`, `ultra1`, `heat1`) or reuse one. Align with broker config.

### Network

* All nodes and PC must share the same LAN/VLAN. If Pi has both Wi-Fi and Ethernet, ensure routing doesn’t isolate topics.

### Safety & Power

* Fuse mains devices; use SSR for AC switching where appropriate.
* Keep sensor wiring away from mains and heater cables.
* Add **TVS + bulk + ceramic** across 12 V rails; flyback diodes across inductive DC loads.

---


## 📡 MQTT Topic Structure

| Topic pattern                           | Pub/Sub | Who → Who    | Example payload                                   | Purpose                           |
| --------------------------------------- | :-----: | ------------ | ------------------------------------------------- | --------------------------------- |
| `<base>/cmd/<n>`                        | **Pub** | **PC → ESP** | `SET:42` · `ON` · `ON:1500` · `PWM:35` · `PID:ON` | Control channel per channel `<n>` |
| `<base>/state/<n>`                      | **Sub** | **ESP → PC** | `ON` · `OFF` *(retained)*                         | Relay/actuator state per channel  |
| `<base>/temp/<n>`                       | **Sub** | **ESP → PC** | `39.6`                                            | Temperature (heater node)         |
| `<base>/target/<n>` or `<base>/set/<n>` | **Sub** | **ESP → PC** | `42.0` *(retained)*                               | PID setpoint (heater)             |
| `<base>/status`                         | **Sub** | **ESP → PC** | `ONLINE` / `OFFLINE` *(retained)*                 | Node availability                 |
| `<base>/heartbeat`                      | **Sub** | **ESP → PC** | `1` every 15 s                                    | Keep-alive                        |

Typical bases:

-  Pumps node: pumps/01
-  Ultrasonic node: ultra/01
-  Heater node: heat/01

---

## 💻 Python Controller — `iot_mqtt.py`

### Overview
`iot_mqtt.py` is a **unified control script** for all ESP32 nodes.  
It uses the `paho-mqtt` v2 client API to send commands and monitor telemetry.

### Classes

| Class       | Device     | What it does                          |
| ----------- | ---------- | ------------------------------------- |
| `PumpMQTT`  | Pumps      | 4-channel Qwiic Quad Relay control    |
| `UltraMQTT` | Ultrasonic | 2-channel Qwiic Single relay control  |
| `HeatMQTT`  | Heaters    | Dual SSR + PID + thermistor feedback  |

### ESP Command Set (payloads sent to <base>/cmd/<n>)

**Pumps** (pumps/01)

- ON — turn channel n on
- OFF — turn channel n off
- ON:<ms> — turn on, auto-off after <ms> milliseconds

**Ultrasonic** (ultra/01)

- ON, OFF, ON:<ms> — same semantics as pumps

**Heaters** (heat/01)
- ON, OFF — manual on/off (PID is disabled for that channel)
- ON:<ms> — timed on
- PWM:<0–100> — manual slow-PWM duty (%) for channel (disables PID)
- SET:<tempC> — set target setpoint (°C) and publish it on target/<n>
- PID:ON — enable PID control for channel <n>
- PID:OFF — disable PID control (forces PWM=0 → OFF)
- GET — request an immediate temperature publish on temp/<n>

### Example Topic Map 

- **Pumps**

  - pumps/01/cmd/1 ← ON, ON:1500, OFF
  - pumps/01/state/1 → ON (retained)
  - pumps/01/status → ONLINE
  - pumps/01/heartbeat → 1

- **Ultrasonic**

  - ultra/01/cmd/2 ← ON:1500
  - ultra/01/state/2 → ON (retained)

- **Heaters**

  - heat/01/cmd/1 ← SET:42, PID:ON
  - heat/01/target/1 → 42.0 (retained)
  - heat/01/temp/1 → 39.6
  - heat/01/state/1 → ON (retained)
  - heat/01/status → ONLINE

### “Cheat Sheet” of High-Level Calls

```python

# Pumps
pumps.on(ch)                 # ch ∈ {1..4}
pumps.on(ch, ms)            # timed on
pumps.off(ch)
pumps.toggle(ch)            # reads retained state and flips it
pumps.status()              # shows status/heartbeat/state/1..4

# Ultrasonic
ultra.on(ch)                # ch ∈ {1,2}
ultra.on_for(ch, ms)
ultra.off(ch)
ultra.status()

# Heaters
heat.set_base_temp(ch, 42.0)   # sets target (retained)
heat.set_pwm(ch, 35)           # manual PWM %, disables PID
# (If you prefer helpers)
# heat.pid_on(ch); heat.pid_off(ch)

temp = heat.get_base_temp(ch)  # requests GET, waits for temp/<ch>
heat.status()

```

### Example Usage
```python
# iot_mqtt_demo.py
import time
from iot_mqtt import start_broker_if_needed, stop_broker
from iot_mqtt import PumpMQTT, UltraMQTT, HeatMQTT

# (Optional) start local Mosquitto on Windows if not already running
proc = start_broker_if_needed()  # comment out if you already have a broker

# Create clients
pumps = PumpMQTT(broker="192.168.0.101", username="pico1",  password="pump",
                 base_topic="pumps/01", client_id="pyctl-pumps")
ultra = UltraMQTT(broker="192.168.0.101", username="ultra1", password="ultra",
                  base_topic="ultra/01", client_id="pyctl-ultra")
heat  = HeatMQTT(broker="192.168.0.101", username="heat1",  password="heat",
                 base_topic="heat/01",  client_id="pyctl-heat")

# Connect + start background loops
pumps.ensure_connected()
ultra.ensure_connected()
heat.ensure_connected()
time.sleep(1)  # let subscriptions settle

# Show live status from each node for a couple seconds
pumps.status(seconds=2.0)
ultra.status(seconds=2.0)
heat.status(seconds=2.0)

# ---- Control examples ----

# Pumps
pumps.on(1)                 # pump 1 ON
time.sleep(1)
pumps.on(1, 2000)           # pump 1 ON for 2 s (auto-off)
time.sleep(2.5)

# Ultrasonic
ultra.on_for(2, 1500)       # ultrasonic ch2 ON for 1.5 s
time.sleep(2)

# Heaters (PID demo)
heat.set_base_temp(1, 42.0) # tells ESP to set target 42 °C (retained on target/1)
heat._publish("cmd/1", "PID:ON")  # or: heat.pid_on(1) if you expose a helper

# Read temperature for ~30 s
for _ in range(30):
    try:
        t = heat.get_base_temp(1, timeout_s=2.0)  # requests GET → reads temp/1
        print("Heater1 Temp =", t)
    except TimeoutError:
        print("Temp read timeout")
    time.sleep(1)

# Stop PID and ensure OFF
heat._publish("cmd/1", "PID:OFF")
heat.set_pwm(1, 0)
heat.off(1)

# Cleanup
pumps.disconnect()
ultra.disconnect()
heat.disconnect()
stop_broker(proc)

```

> **Notes** 
> - Replace `192.168.0.101` with your broker’s IP (your PC if Mosquitto runs there).
> - For long-running apps, prefer non-blocking UI or a scheduler rather than time.sleep.


