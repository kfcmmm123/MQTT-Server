# Mosquitto Broker — Configuration & Security

This folder contains the **Mosquitto MQTT broker configuration** and **access control lists** for the ESP32-POE-ISO IoT control system.

## Configuration Files

- **`mosquitto.conf`** – Main broker configuration (listener, persistence, authentication)
- **`aclfile.txt`** – Topic-level permissions and access control per user

## Quick Links

- **[Python API Documentation](../iot_mqtt/README.md)** - Control ESP32 devices via Python
- **[Device Firmware](../devices/)** - ESP32 firmware for pumps, heaters, ultrasonic, and pH controllers
- **[Hardware Assembly](../hardware/README.md)** - 3D printable enclosures and assembly

---

## Configuration Schema

Edit paths in `mosquitto.conf` to match your machine (Windows example shown):

```conf
listener 1883
allow_anonymous false

# Auth files (edit these paths!)
password_file "C:/Program Files/mosquitto/passwd"
acl_file      "C:/Program Files/mosquitto/aclfile.txt"

# Persistence (safe restarts, retained messages)
persistence true
persistence_location "C:/Program Files/mosquitto/data/"

# Optional: logging
log_timestamp true
log_type all
```

> On Linux, typical paths are `/etc/mosquitto/mosquitto.conf`,
> `/etc/mosquitto/passwd`, `/etc/mosquitto/aclfile.txt`, and `/var/lib/mosquitto/`.

---

## Users & Passwords

Create users with `mosquitto_passwd`.
**Important:** `-c` creates/overwrites the file—use it *once*.

```bash
# First time only (creates the file)
mosquitto_passwd -c passwd pyctl-controller

# Add device users
mosquitto_passwd    passwd pump1
mosquitto_passwd    passwd ultra1
mosquitto_passwd    passwd heat1
mosquitto_passwd    passwd ph1
```

Move `passwd` to the path referenced in `mosquitto.conf`.

---

## ACL (Access Control)

`aclfile.txt` grants the minimum required permissions:

```text
# Controller (Python)
user pyctl-controller
topic write pyctl/#
topic read  pumps/01/#
topic read  ultra/01/#
topic read  heat/01/#
topic read  ph/01/#

# Pump device
user pump1
topic read  pyctl/#
topic read  pumps/01/cmd/#
topic write pumps/01/state/#
topic write pumps/01/status
topic write pumps/01/heartbeat

# Ultrasonic device
user ultra1
topic read  pyctl/#
topic read  ultra/01/cmd/#
topic write ultra/01/state/#
topic write ultra/01/status
topic write ultra/01/heartbeat

# Heater device
user heat1
topic read  pyctl/#
topic read  heat/01/cmd/#
topic write heat/01/state/#
topic write heat/01/status
topic write heat/01/heartbeat
topic write heat/01/temp/#
topic write heat/01/target/#
 
# pH device
user ph1
topic read  pyctl/#
topic read  ph/01/cmd
topic write ph/01/ph
topic write ph/01/reply
topic write ph/01/status
topic write ph/01/heartbeat
```

> If you create more nodes (e.g., `pumps/02`), add matching blocks here.

---

## Optional: TLS (Encrypted MQTT)

1. Generate or obtain certificates (self-signed or from your CA).
2. Add to `mosquitto.conf`:

```conf
listener 8883
cafile     /etc/mosquitto/certs/ca.crt
certfile   /etc/mosquitto/certs/server.crt
keyfile    /etc/mosquitto/certs/server.key
require_certificate false
```

3. Point Python/ESP clients to port **8883** and provide CA certs.

---

## Persistence & Backups

* **Retained messages** (e.g., `status`, `target/`) are saved to the persistence store.
* Back up the directory referenced by `persistence_location` regularly.
* To reset the broker state during development, stop Mosquitto and delete the persistence DB (⚠️ clears retained topics).

---

## Troubleshooting

* **`Not authorized`** → user doesn’t match the ACL; verify username, topic base, and that you didn’t recreate the password file with `-c`.
* **No retained messages** → persistence disabled or the broker lacks write permission to the persistence directory.
* **Clients disconnecting** → increase `keepalive` (30–60 s) and check for duplicate client IDs.
* **Windows service can’t read files** → run as a user with access or move config to a user-writable path.

---

## Security Hardening Checklist

* [x] `allow_anonymous false`
* [x] Unique users per device class
* [x] Minimal ACL permissions per subtree
* [x] Enable persistence and log to file
* [x] Optional TLS (port 8883) on shared or Wi-Fi networks
* [x] Use **LWT** in clients for failsafe OFF handling



