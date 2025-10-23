# Hardware Assembly — 3D Printable Enclosures & Mounts

This folder contains the **3D-printable designs** and assembly instructions for ESP32-POE-ISO node enclosures, relay carriers, and accessories for the IoT control system.

## Quick Links

- **[Python API Documentation](../iot_mqtt/README.md)** - Control ESP32 devices via Python
- **[Device Firmware](../devices/)** - ESP32 firmware for pumps, heaters, ultrasonic controllers
- **[MQTT Broker Setup](../mosquitto_config/README.md)** - Mosquitto configuration and security

---

## CAD Files

```
hardware/
├── CAD_models/                 # 3D printable enclosure files
│   ├── Base Power Tower.3mf
│   ├── Heater Module - SSR.3mf
│   ├── Pump Module - 3 single relay.3mf
│   ├── Pump Module - quad relay.3mf
│   ├── Ultrasonic Driver Module.3mf
│   └── Ultrasonic Module - 2 single relay.3mf
└── README.md                   # This assembly guide
```

---

## System Overview

- **Pump Node Enclosure** – Qwiic Quad Relay carrier + cable glands for DC pumps.
- **Ultrasonic Node Enclosure** – Two Qwiic Single Relays for AC driver switching; mains isolation clearances maintained.
- **Heater Node Enclosure** – Dual SSR + ADS1015 mount, thermistor cable inlet, and optional LCD window.
- **Shared Backplate** – 120 mm × 160 mm DIN-slot pattern for wall or rack trays.

👉 **TODO:** Insert one hero render/photo of the assembled stack.

---

## Print Settings (Reference)

- **Material:** PETG (preferred for heat & chemicals); PLA+ acceptable for dry bench use.
- **Nozzle/Layer:** 0.4 mm nozzle; 0.2 mm layer height.
- **Walls:** 3 perimeters; top/bottom 4–5 layers.
- **Infill:** 20–30% grid/gyroid.
- **Supports:** Only for board standoffs that overhang >50° (most parts are support-free).
- **Tolerances:** All snap/groove fits modeled at +0.3 mm clearance for PETG.

---

## Hardware & Fasteners

- **Heat-set inserts:** M2.5 × 4 mm (board mounting, lid screws).  
- **Screws:** M2.5 × 6–8 mm (PCBs), M3 × 8–10 mm (lid/backplate).  
- **Cable glands:** PG7 / M12 for sensor and DC pump leads.  
- **Standoffs:** 10 mm M2.5 for ADS1015 & SSR where needed.  
- **Label plates:** 10 mm high; DXF provided under `dxf/labels/`.

👉 **TODO:** Add the exact insert model/brand you used.

---

## Panel & Cutouts

DXF drawings in `dxf/` include:

- **Ethernet/PoE rectangular cut** (ESP32-POE-ISO)
- **C14 power inlet + fuse + switch** (if used in a combined AC box)
- **E-Stop 22 mm** round cutout
- **IEC/AC warning labels**

> If you laser-cut acrylic/ABS, add **M3 heat-set inserts** in corner bosses and keep 10 mm clearance around mains.

---

## Mounting the Electronics

- **ESP32-POE-ISO**: board posts on a 53 × 58 mm pattern (standoffs or heat-set inserts).  
- **Qwiic Relays / Dual SSR**: side-by-side with 5–7 mm spacing to vent heat.  
- **ADS1015**: stacked above SSR using a two-post bracket to shorten I²C leads.  
- **LCD (optional)**: 16×2 I²C window; snap bezel included.

👉 **TODO:** Paste photos for each node with callouts to connectors.

---

## Cable Management

- **Strain relief**: print the small “fork” clamps for pump/DC leads.  
- **I²C/Qwiic**: keep total bus ≤ 30 cm inside each enclosure; route away from mains.
- **Grounding**: if an AC box is used, bond the chassis ground lug to earth.

---

## Rail/Slide Concept (Sonicator/Cooler Swappable)

A small **T-slot rail** is provided for a 44 mm round sonicator puck **or** a rectangular cooler/fan module:

- **Rail**: 2020-style printed T-slot; 2 × M3 captive nuts; 70 mm travel.  
- **Sonicator puck plate**: 44 mm diameter counterbore with silicone pad (Shore A 40–50).  
- **Cooler plate**: Raspberry Pi 5 Active Cooler or 50×50 mm blower module; spring-loaded clamping bar for contact pressure.

👉 **TODO:** Insert render of the rail with both plates and describe your surface prep/thermal pad.

---

## Safety Notes

- Maintain **creepage/clearance** for mains relay sections (≥6 mm).  
- Use **printed guards** around AC terminals; never expose live screws.  
- Prefer **PETG/ABS** near warm SSRs or heaters; avoid PLA if ambient > 40 °C.  
- Label **inlets/outlets** and emergency-stop clearly (DXF included).

---

## Revision Log

- `v1.0` – Initial enclosure set and rail concept.  
- `v1.1` – Added LCD window and stronger lid ribs.  
- `v1.2` – Cable gland spacing increased, thermistor clamp added.



