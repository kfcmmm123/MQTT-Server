# Pump Controllers — Technical Reference

## MQTT Interface

- Base: `pumps/01`
- Channels: 1..4 (depending on hardware)

Topics:

| Topic                | Dir    | Payload                   | Retained | Notes                  |
| -------------------- | ------ | ------------------------- | -------- | ---------------------- |
| `pumps/01/cmd/<n>`   | PC→ESP | `ON` · `OFF` · `ON:<ms>`  | no       | `<n>` ∈ {1..4}         |
| `pumps/01/state/<n>` | ESP→PC | `ON` · `OFF`              | yes      | Last relay state       |
| `pumps/01/status`    | ESP→PC | `ONLINE` · `OFFLINE`      | yes      | Node availability      |
| `pumps/01/heartbeat` | ESP→PC | `1` (fixed)               | no       | Every ~15 s            |
| `pyctl/status`     | PC↔ESP | `ONLINE` · `OFFLINE` | yes      | Controller supervision (LWT)       |
| `pyctl/heartbeat`  | PC↔ESP | `1`                  | no       | Periodic heartbeat from controller |

Command semantics (to `cmd/<n>`):
- `ON` — turn channel `<n>` on until explicit `OFF` or watchdog timeout
- `OFF` — turn channel `<n>` off immediately
- `ON:<ms>` — turn on for `<ms>` milliseconds, then auto-off

## Behavior

- On boot: publishes retained `status=ONLINE`; `state/<n>` reflect current outputs.
- LWT: if client disconnects unexpectedly, broker sets retained `status=OFFLINE`.
- Safety: loss of controller heartbeat (if implemented on device) → all outputs OFF.

## Error Notes

- Sending malformed payloads to `cmd/<n>` is ignored; state remains unchanged.
- If `<n>` is out of range, firmware ignores the command.

## Related

- Python API: `PumpMQTT` in `iot_mqtt/iot_mqtt.py`.
- Broker ACL example: see `mosquitto_config/aclfile.txt`.

