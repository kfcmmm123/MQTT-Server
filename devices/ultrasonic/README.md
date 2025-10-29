# Ultrasonic Controllers — Technical Reference

## MQTT Interface

- Base: `ultra/01`
- Channels: 1..2

Topics:

| Topic                | Dir    | Payload                   | Retained | Notes      |
| -------------------- | ------ | ------------------------- | -------- | ---------- |
| `ultra/01/cmd/<n>`   | PC→ESP | `ON` · `OFF` · `ON:<ms>`  | no       | `<n>`∈{1,2}|
| `ultra/01/state/<n>` | ESP→PC | `ON` · `OFF`              | yes      |            |
| `ultra/01/status`    | ESP→PC | `ONLINE` · `OFFLINE`      | yes      |            |
| `ultra/01/heartbeat` | ESP→PC | `1`                       | no       | ~15 s      |

Command semantics match pumps (`ON`, `OFF`, `ON:<ms>`).

## Behavior

- Retained `status` and `state/<n>` reflect last-known state.
- LWT ensures `status=OFFLINE` on unexpected disconnects.
- Safety watchdogs can force outputs OFF if controller/heartbeat lost.

## Notes

- Duplicate `ON:<ms>` renews the lease from the latest command.
- Overlapping commands per channel are serialized by the device firmware.

## Related

- Python API: `UltraMQTT` in `iot_mqtt/iot_mqtt.py`.
- Broker ACL example: `mosquitto_config/aclfile.txt`.

