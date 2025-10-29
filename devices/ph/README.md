# pH Probe Controller — Technical Reference

## MQTT Interface

- Base: `ph/01`

Topics:

| Topic              | Dir    | Payload                | Retained | Notes                                |
| ------------------ | ------ | ---------------------- | -------- | ------------------------------------ |
| `ph/01/cmd`        | PC→ESP | `ONESHOT`              | no       | One immediate reading                 |
|                    |        | `START:<ms>`           | no       | Periodic readings every `<ms>`        |
|                    |        | `STOP`                 | no       | Stop periodic readings                |
|                    |        | raw probe commands     | no       | e.g., `i`, `Status,?`, `Cal,mid,7.00`|
| `ph/01/ph`         | ESP→PC | numeric string (e.g. 7.03) | no  | Latest pH reading                     |
| `ph/01/reply`      | ESP→PC | text                   | no       | Probe replies / safety notes          |
| `ph/01/status`     | ESP→PC | `ONLINE` · `OFFLINE`   | yes      | Node availability                      |
| `ph/01/heartbeat`  | ESP→PC | `1`                    | no       | Keep-alive                             |

## Behavior

- Subscribes to `cmd`; publishes `ph` and `reply` as events.
- `START:<ms>` is idempotent; issuing again resets the period.
- On STOP or disconnect, device ceases polling.

## Notes

- Raw probe command passthrough enables calibration/status queries.
- Consider rate limits on `ph` publish frequency to avoid broker overload.

## Related

- Python API: `PhMQTT` in `iot_mqtt/iot_mqtt.py`.
- Broker ACL example: `mosquitto_config/aclfile.txt`.


