# Biologic Controller — Technical Reference

## MQTT Interface

* Base topic: `bio/01`
* Channels: `1..16`
  (Driven via MCP23017 I²C GPIO expander → active-low relay modules)

### Topics

| Topic              | Dir    | Payload              | Retained | Notes                              |
| ------------------ | ------ | -------------------- | -------- | ---------------------------------- |
| `bio/01/cmd/<n>`   | PC→ESP | `ON` · `OFF`         | no       | `<n>` ∈ {1..16}                    |
| `bio/01/state/<n>` | ESP→PC | `ON` · `OFF`         | yes      | Last relay state                   |
| `bio/01/status`    | ESP→PC | `ONLINE` · `OFFLINE` | yes      | Node availability (incl. LWT)      |
| `bio/01/heartbeat` | ESP→PC | `1`                  | no       | Every ~15 s                        |
| `pyctl/status`     | PC↔ESP | `ONLINE` · `OFFLINE` | yes      | Controller supervision (LWT)       |
| `pyctl/heartbeat`  | PC↔ESP | `1`                  | no       | Periodic heartbeat from controller |

Command semantics (to `cmd/<n>`):
- `ON` — turn channel `<n>` on until explicit `OFF` or watchdog timeout
- `OFF` — turn channel `<n>` off immediately

No duration-based command (e.g., `ON:<ms>`) is implemented by default, but could be added if desired.

## Behavior

### On Boot

* Device initializes MCP23017 and drives **all relays OFF** (active-low → output HIGH).
* Publishes retained:

  ```
  bio/01/status = ONLINE
  bio/01/state/<n> = OFF
  ```

### LWT & Disconnects

If the ESP32 loses connection unexpectedly:

* Mosquitto publishes retained:

  ```
  bio/01/status = OFFLINE
  ```

### Safety Rules

| Event                                    | Effect      |
| ---------------------------------------- | ----------- |
| Controller heartbeat missing (> timeout) | **ALL OFF** |
| Controller publishes OFFLINE (LWT)       | **ALL OFF** |
| MQTT reconnect timeout                   | **ALL OFF** |
| Ethernet link down                       | **ALL OFF** |

This ensures unattended hardware always fails safe.

## State Reporting

Each channel publishes its state:

```
bio/01/state/<n> = ON | OFF   (retained)
```

This allows UI or notebooks to reconstruct the full state after reconnect.

Example:

```
bio/01/state/4  →  ON
```

## Error Notes

* Invalid channel index → ignored.
* Unknown payload → ignored.
* If MCP23017 fails to initialize, firmware prints a diagnostic; relay control is disabled.
* Commands are idempotent — repeated `ON` or `OFF` does not cause issues.

## Internal Hardware Notes

* GPIO expansion uses **MCP23017 @ I²C addr 0x20**
* Relay outputs on MCP pins `0..15`
* Relays are **active-low**:

  * `LOW` → ON
  * `HIGH` → OFF

Mapping:

```
ch 1 → MCP pin 0
ch 2 → MCP pin 1
...
ch 16 → MCP pin 15
```

## Related

| Item                  | Location                       |
| --------------------- | ------------------------------ |
| Python API class      | `BioMQTT` (`iot_mqtt.py`)      |
| Mosquitto example ACL | `mosquitto_config/aclfile.txt` |
| ESP firmware          | (Your Arduino `.ino`)          |
| MCP23017 Datasheet    | Microchip                      |

