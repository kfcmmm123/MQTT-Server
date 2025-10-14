# 🌐 ESP32-POE-ISO IoT Control System  
**Modular MQTT-Based Control for Pumps, Ultrasonic Drivers, and Heaters**

This project implements a **multi-device IoT control system** using **ESP32-POE-ISO** boards connected via **MQTT over Ethernet**.  
Each node manages a specific subsystem — **Pumps**, **Ultrasonic Drivers**, or **Heaters** — with real-time monitoring, PID control, and safety supervision.

---

## 📋 Features

- ✅ Unified **MQTT communication** for all devices  
- 🔌 Independent **ESP32-POE-ISO nodes** with Ethernet  
- 🔥 **PID temperature control** with thermistors and dual SSRs  
- 💧 **Quad relay pump** control for fluid handling  
- 🔊 **Ultrasonic relay** driver control  
- 🧠 **Python control script (`iot_mqtt.py`)** for all subsystems  
- 🛡️ **Safety protection**: over-temp rate brake, startup delay, fail-safe OFF  
- 🌡️ Real-time telemetry publishing (temperature, states, heartbeat)

---

## 🧭 System Architecture

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

Each node runs an MQTT client, subscribes to control topics (`cmd/#`),  
and publishes telemetry (`state/#`, `temp/#`, `status`, `heartbeat`).

---

## 📡 MQTT Topic Structure

| Topic | Direction | Example | Description |
|--------|------------|----------|-------------|
| `<base>/cmd/<n>` | → ESP | `heat/01/cmd/1 → SET:42` | Commands |
| `<base>/state/<n>` | ← ESP | `pumps/01/state/3 → ON` | Relay states |
| `<base>/temp/<n>` | ← ESP | `heat/01/temp/1 → 39.6` | Temperature feedback |
| `<base>/target/<n>` | ← ESP | `heat/01/target/1 → 42.0` | PID setpoint |
| `<base>/status` | ← ESP | `ONLINE` / `OFFLINE` | Connection status |
| `<base>/heartbeat` | ← ESP | `"1"` every 15 s | Keep-alive signal |

---

## 💻 Python Controller — `iot_mqtt.py`

### Overview
`iot_mqtt.py` is a **unified control script** for all ESP32 nodes.  
It uses the `paho-mqtt` v2 client API to send commands and monitor telemetry.

### Classes
| Class | Device | Description |
|--------|---------|-------------|
| `PumpMQTT` | Pumps | Controls 4-channel Qwiic Quad Relay |
| `UltraMQTT` | Ultrasonic | Controls 2-channel ultrasonic drivers |
| `HeatMQTT` | Heaters | Controls dual SSR heater node with PID + feedback |

### Example Usage
```python
from iot_mqtt import PumpMQTT, UltraMQTT, HeatMQTT
import time

# Start Server 
proc = start_broker_if_needed()  # only if you don’t run the Windows service

# Connect devices
broker = "192.168.0.100"

pumps = PumpMQTT(broker=broker, username="pump1", password="pump",
                base_topic="pumps/01", client_id="pyctl-pumps")
ultra = UltraMQTT(broker=broker, username="ultra1", password="ultra",
                base_topic="ultra/01", client_id="pyctl-ultra")
heat = HeatMQTT(broker=broker, username="heat1", password="heat",
                base_topic="heat/01", client_id="pyctl-heat")

# Connect + Start loop
heat.ensure_connected()
pumps.ensure_connected()
ultra.ensure_connected()
time.sleep(1)  # wait for connections to settle

# Control examples

# Pump demo
pumps.on(1)                 # Start pump 1
pumps.on(1, 2000)           # Run pump 1 for 2s
ultra.on_for(2, 1500)       # Trigger ultrasonic 2 for 1.5s
heat.set_base_temp(1, 42.0) # Set heater 1 target 42°C
heat._publish("cmd/1", "PID:ON")


# Heater demo
heat.set_target(1, 42.0)         # set target to 42C (ESP retains on set/1)
heat.pid_on(1)                   # enable PID loop on ESP
# actively request a reading now:
try:
    for i in range(200):
        t = heat.get_base_temp(1, timeout_s=5.0)
        print("Temp(ch1) =", t)
        time.sleep(1)
except TimeoutError as e:
    print("Temp read timeout:", e)

heat.pid_off(1)                  # stop PID
heat.set_pwm(1, 0)               # ensure PWM is off
heat.off(1)                      # ensure relay is off

for _ in range(10):
    print("CH1 Temp:", heat.get_base_temp(1))
    time.sleep(1)

# Shutdown
heat._publish("cmd/1", "PID:OFF")
pumps.off(1)
ultra.off(2)

