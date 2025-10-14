# iot_mqtt.py
# Unified MQTT helpers for ESP32-POE-ISO devices:
# - Pumps (SparkFun Qwiic Quad Relay)
# - Ultrasonic drivers (2 channels on a SparkFun Qwiic Quad Relay)
# - Heaters (SparkFun 2-Channel Solid State Relay)
#
# The script provides reusable Python classes for MQTT-based control
# and monitoring of multiple ESP32-based subsystems.
#
# Requires: paho-mqtt>=1.6 (v2 callback API)

import time, socket, subprocess, shutil, os, threading
from typing import Optional, Callable, Iterable
from paho.mqtt import client as mqtt
from datetime import datetime

# ---------------------------------------------------------------------------
# Utility functions for checking broker ports and starting/stopping Mosquitto
# ---------------------------------------------------------------------------

def _is_port_open(host: str, port: int) -> bool:
    """Return True if something is already listening on (host, port)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0

def _wait_for_port(host: str, port: int, timeout: float = 10.0) -> bool:
    """Wait until a TCP port opens or timeout expires."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _is_port_open(host, port):
            return True
        time.sleep(0.25)
    return False
        
def start_broker_if_needed(
    mosq_exe: str = r"C:\Program Files\mosquitto\mosquitto.exe",
    mosq_conf: str = r"C:\Program Files\mosquitto\mosquitto.conf",
    port: int = 1883,
) -> Optional[subprocess.Popen]:
    """
    Start Mosquitto broker if not already running, and log with timestamps.
    Returns a subprocess handle or None if already listening.
    """
    if _is_port_open("127.0.0.1", port):
        print(f"[broker] Already listening on port {port}")
        return None

    if not (os.path.exists(mosq_exe) or shutil.which(mosq_exe)):
        raise FileNotFoundError(f"mosquitto.exe not found: {mosq_exe}")

    # --- Safe log path -----------------------------------------------------
    default_log = r"C:\Users\13538\OneDrive\Desktop\Arduino\mosq-python.log"
    try:
        os.makedirs(os.path.dirname(default_log), exist_ok=True)
        log_path = default_log
    except (PermissionError, OSError):
        log_base = os.path.join(os.path.expanduser("~"), "mosquitto-logs")
        os.makedirs(log_base, exist_ok=True)
        log_path = os.path.join(log_base, "mosq-python.log")
        print(f"[broker] No write access to default path; using {log_path}")

    print(f"[broker] Logging to: {log_path}")
    logf = open(log_path, "a", buffering=1, encoding="utf-8")

    # --- Start broker ------------------------------------------------------
    print("[broker] Starting Mosquitto (-v for logs)...")
    proc = subprocess.Popen(
        [mosq_exe, "-v", "-c", mosq_conf],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    # Thread to add timestamps to log
    def log_with_timestamps(stream, dest):
        for line in iter(stream.readline, ''):
            ts = datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ")
            dest.write(ts + line)
        stream.close()
    threading.Thread(target=log_with_timestamps, args=(proc.stdout, logf), daemon=True).start()

    # --- Wait for broker ready --------------------------------------------
    if not _wait_for_port("127.0.0.1", port, timeout=10):
        proc.terminate()
        raise RuntimeError(f"Broker failed to open port {port}")

    proc._log_handle = logf
    print("[broker] Broker is ready.")
    return proc


def stop_broker(proc: Optional[subprocess.Popen]) -> None:
    """Gracefully stop Mosquitto broker if we started it."""
    if proc and proc.poll() is None:
        print("[broker] Stopping Mosquitto...")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            print("[broker] Mosquitto did not terminate in time; killing it.")
            proc.kill()

        # Log the stop event with timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        msg = f"[{timestamp}] [broker] Mosquitto stopped.\n"
        print(msg.strip())

        if hasattr(proc, "_log_handle") and proc._log_handle:
            try:
                proc._log_handle.write(msg)
                proc._log_handle.flush()
            except Exception:
                pass

    # Close log handle cleanly
    if proc and hasattr(proc, "_log_handle"):
        try:
            proc._log_handle.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Base MQTT device class providing connection, publish, and monitoring utilities
# ---------------------------------------------------------------------------

class _BaseDevice:
    """
    Common MQTT client wrapper.

    Handles:
      - Connection / disconnection
      - Publishing commands
      - Watching & printing topic messages
      - Temporary subscription for reading retained states
    """

    def __init__(
        self,
        *,
        broker: str = "192.168.0.100",
        port: int = 1883,
        username: Optional[str] = None,
        password: Optional[str] = None,
        base_topic: str = "device/01",
        client_id: str = "pyctl1",
        keepalive: int = 30,
        print_publish: bool = True,
    ):
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.base = base_topic.rstrip("/")
        self.client_id = client_id
        self.keepalive = keepalive
        self.print_publish = print_publish

        # Runtime state
        self._client: Optional[mqtt.Client] = None
        self._loop_running: bool = False          # <--- track loop state here
        self._watch_thread: Optional[threading.Thread] = None
        self._watch_stop = threading.Event()

    # -------------------
    # Connection handling
    # -------------------

    def connect(self, retries: int = 1, delay: float = 0.5) -> None:
        """Create a client and open a TCP connection. Does NOT start the network loop."""
        if self._client is not None:
            return

        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.client_id, protocol=mqtt.MQTTv311)
        if self.username and self.password:
            c.username_pw_set(self.username, self.password)

        def _on_connect(client, userdata, flags, reason_code, properties=None):
            if reason_code == 0:
                print(f"[{self.client_id}] Connected to {self.broker}:{self.port}")
            else:
                print(f"[{self.client_id}] Connection failed (rc={reason_code})")

        c.on_connect = _on_connect

        for attempt in range(max(1, retries)):
            print(f"[{self.client_id}] Connecting to {self.broker}:{self.port} (attempt {attempt + 1})...")
            try:
                c.connect(self.broker, self.port, keepalive=self.keepalive)
                self._client = c
                return
            except OSError:
                if attempt == retries - 1:
                    raise
                print(f"[{self.client_id}] Connection failed; retrying in {delay} seconds...")
                time.sleep(delay)

    def ensure_connected(self, retries: int = 5, delay: float = 0.6) -> None:
        """
        Ensure there is a connected client AND that the background loop is running.
        Call this once at the start of your session.
        """
        if self._client is None:
            self.connect(retries=retries, delay=delay)
        if not getattr(self, "_loop_running", False):
            self._client.loop_start()
            self._loop_running = True

    def disconnect(self) -> None:
        """Stop the background loop (if we started it) and disconnect cleanly."""
        if self._client is not None:
            try:
                if getattr(self, "_loop_running", False):
                    self._client.loop_stop()
                    self._loop_running = False
                self._client.disconnect()
            finally:
                self._client = None

    # -------------------
    # Publishing & topics
    # -------------------

    def _require(self) -> mqtt.Client:
        """Internal: raise error if no client connection."""
        if self._client is None:
            raise RuntimeError("Not connected. Call connect()/ensure_connected() first.")
        return self._client

    def _publish(self, topic_suffix: str, payload: str, *, qos: int = 0, retain: bool = False) -> None:
        """Send a payload to a topic under the device's base topic."""
        c = self._require()
        full = f"{self.base}/{topic_suffix}".replace("//", "/")
        c.publish(full, payload, qos=qos, retain=retain)
        if self.print_publish:
            print(f"[{self.client_id}] Published '{payload}' to {full}")

    # -------------------
    # Monitoring utilities
    # -------------------

    def status(self, topics: Optional[Iterable[str]] = None, seconds: float = 3.0) -> None:
        """Subscribe temporarily to status/heartbeat or custom topics and print messages."""
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("status() requires the background loop running. Call ensure_connected() first.")

        to_sub = list(topics) if topics else [f"{self.base}/status", f"{self.base}/heartbeat"]
        for t in to_sub:
            c.subscribe(t, qos=0)

        def _on_msg(client, userdata, msg):
            try:
                print(f"{msg.topic} {msg.payload.decode('utf-8', errors='ignore')}")
            except Exception:
                print(f"{msg.topic} <{len(msg.payload)} bytes>")

        old = c.on_message
        c.on_message = _on_msg
        try:
            time.sleep(seconds)  # loop is running; we'll receive messages during this window
        finally:
            c.on_message = old
            # Optional: unsubscribe to be tidy
            for t in to_sub:
                c.unsubscribe(t)


    def watch(self, on_message: Optional[Callable[[str, bytes], None]] = None) -> None:
        """
        Start a background watcher that prints all messages under base/#.
        Requires ensure_connected() so the background loop is running.
        """
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("watch() requires the background loop running. Call ensure_connected() first.")

        self._watch_stop.clear()

        def _default_cb(topic: str, payload: bytes):
            try:
                print(f"{topic} {payload.decode('utf-8', errors='ignore')}")
            except Exception:
                print(f"{topic} <{len(payload)} bytes>")

        cb = on_message or _default_cb

        def _run():
            old = c.on_message
            c.subscribe(f"{self.base}/#", qos=0)
            try:
                def _on_msg(_c, _u, msg):
                    cb(msg.topic, msg.payload)
                c.on_message = _on_msg
                # Just park here until stop is requested; network loop is running elsewhere
                while not self._watch_stop.is_set():
                    time.sleep(0.1)
            finally:
                c.on_message = old
                c.unsubscribe(f"{self.base}/#")

        if self._watch_thread and self._watch_thread.is_alive():
            print("[watch] already running")
            return

        self._watch_thread = threading.Thread(target=_run, daemon=True)
        self._watch_thread.start()
        print(f"[watch] {self.base}/# (call .watch_stop() to stop)")

    def watch_stop(self) -> None:
        """Stop the background watch thread if running."""
        self._watch_stop.set()
        if self._watch_thread and self._watch_thread.is_alive():
            self._watch_thread.join(timeout=2)
        self._watch_thread = None

    # Context manager sugar — allows:
    # with PumpMQTT(...) as pumps: pumps.on(1)
    # def __enter__(self):
    #     self.ensure_connected()
    #     return self

    # def __exit__(self, exc_type, exc, tb):
    #     self.watch_stop()
    #     self.disconnect()

# ---------------------------------------------------------------------------
# Helper for validating channel numbers
# ---------------------------------------------------------------------------

def _check_range(value: int, low: int, high: int, label: str = "channel") -> None:
    """Raise ValueError if value is not within the allowed range [low, high]."""
    if not (low <= value <= high):
        raise ValueError(f"{label.capitalize()} must be {low}-{high}")


# ---------------------------------------------------------------------------
# Device-specific MQTT wrappers
# ---------------------------------------------------------------------------

class PumpMQTT(_BaseDevice):
    """Controls 4 pump relays via MQTT topics."""

    def on(self, channel: int, duration_ms: Optional[int] = None) -> None:
        """Turn a pump ON (optionally auto-OFF after duration_ms)."""
        _check_range(channel, 1, 4, "channel")
        cmd = "ON" if duration_ms is None else f"ON:{duration_ms}"
        self._publish(f"cmd/{channel}", cmd, retain=False)

    def off(self, channel: int) -> None:
        """Turn a pump OFF."""
        _check_range(channel, 1, 4, "channel")
        self._publish(f"cmd/{channel}", "OFF", retain=False)

    def toggle(self, channel: int, timeout_s: float = 1.0) -> None:
        """Toggle pump state by reading retained value and flipping it.

        Requires: ensure_connected() has been called (background loop running).
        """
        _check_range(channel, 1, 4, "channel")
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("toggle() requires the background loop. Call ensure_connected() first.")

        state_topic = f"{self.base}/state/{channel}"
        got = {"val": None}

        # temporary per-topic callback
        def _cb(_c, _u, msg):
            got["val"] = msg.payload.decode(errors="ignore")

        c.message_callback_add(state_topic, _cb)
        try:
            # subscribe and give the broker a moment to deliver the retained state
            c.subscribe(state_topic, qos=0)
            t0 = time.time()
            while got["val"] is None and (time.time() - t0) < timeout_s:
                time.sleep(0.02)
        finally:
            # clean up
            c.message_callback_remove(state_topic)
            c.unsubscribe(state_topic)

        # default to ON if we didn't get a retained state
        new_val = "OFF" if (got["val"] == "ON") else "ON"
        self._publish(f"cmd/{channel}", new_val)


    def status(self, seconds: float = 3.0) -> None:
        """Query all pump state and heartbeat topics."""
        topics = [f"{self.base}/status", f"{self.base}/heartbeat"] + \
                 [f"{self.base}/state/{i}" for i in (1, 2, 3, 4)]
        super().status(topics, seconds)


class UltraMQTT(_BaseDevice):
    """Controls 2 ultrasonic channels via relay driver."""

    def on(self, channel: int) -> None:
        """Turn an ultrasonic channel ON."""
        _check_range(channel, 1, 2, "channel")
        self._publish(f"cmd/{channel}", "ON", retain=False)

    def off(self, channel: int) -> None:
        """Turn an ultrasonic channel OFF."""
        _check_range(channel, 1, 2, "channel")
        self._publish(f"cmd/{channel}", "OFF", retain=False)

    def on_for(self, channel: int, ms: int) -> None:
        """Turn ON for ms milliseconds (handled by ESP firmware auto-timer)."""
        _check_range(channel, 1, 2, "channel")
        if ms <= 0:
            raise ValueError("Duration must be positive")
        self._publish(f"cmd/{channel}", f"ON:{ms}", retain=False)

    def status(self, seconds: float = 3.0) -> None:
        """Print current state, heartbeat, and status topics."""
        topics = [f"{self.base}/status", f"{self.base}/heartbeat"] + \
                 [f"{self.base}/state/{i}" for i in (1, 2)]
        super().status(topics, seconds)


class HeatMQTT(_BaseDevice):
    """Controls 2 heater SSR channels and interacts with thermistor readings."""

    # ---- Basic ON/OFF & manual PWM ----
    def on(self, channel: int) -> None:
        _check_range(channel, 1, 2, "heater number")
        self._publish(f"cmd/{channel}", "ON")

    def off(self, channel: int) -> None:
        _check_range(channel, 1, 2, "heater number")
        self._publish(f"cmd/{channel}", "OFF")

    def set_pwm(self, channel: int, percent: float) -> None:
        _check_range(channel, 1, 2, "heater number")
        p = max(0, min(100, int(round(percent))))
        self._publish(f"cmd/{channel}", f"PWM:{p}")

    # ---- PID / Target control ----
    def set_base_temp(self, channel: int, temp_c: float) -> None:
        """Set target temperature on ESP (publishes retained echo on set/<n>)."""
        _check_range(channel, 1, 2, "heater number")
        self._publish(f"cmd/{channel}", f"SET:{temp_c:.1f}")

    # Friendly alias (your example used heat.set_target)
    def set_target(self, channel: int, temp_c: float) -> None:
        self.set_base_temp(channel, temp_c)

    def pid_on(self, channel: int) -> None:
        """Enable PID on the ESP for this channel."""
        _check_range(channel, 1, 2, "heater number")
        self._publish(f"cmd/{channel}", "PID:ON")

    def pid_off(self, channel: int) -> None:
        """Disable PID on the ESP for this channel."""
        _check_range(channel, 1, 2, "heater number")
        self._publish(f"cmd/{channel}", "PID:OFF")

    # ---- Temperature queries ----
    def get_base_temp(self, channel: int, timeout_s: float = 1.5) -> float:
        """
        Actively request a temperature reading:
          - Sends "GET" to cmd/<n>
          - Waits for temp/<n>
        """
        _check_range(channel, 1, 2, "heater number")
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("get_base_temp() requires the background loop. Call ensure_connected() first.")
        topic = f"{self.base}/temp/{channel}"
        result = {"val": None}
        def _cb(_c, _u, msg):
            try:
                result["val"] = float(msg.payload.decode("utf-8", errors="ignore").strip())
            except ValueError:
                pass
        c.message_callback_add(topic, _cb)
        try:
            c.subscribe(topic, qos=0)
            self._publish(f"cmd/{channel}", "GET")
            t0 = time.time()
            while result["val"] is None and (time.time() - t0) < timeout_s:
                time.sleep(0.02)
        finally:
            c.message_callback_remove(topic)
            c.unsubscribe(topic)
        if result["val"] is None:
            raise TimeoutError(f"No temp reading on {topic} within {timeout_s:.1f}s")
        return result["val"]

    def wait_temp(self, channel: int, timeout_s: float = 5.0) -> float:
        """
        Passive wait for the *next* published temp/<n> (useful when ESP is already streaming).
        """
        _check_range(channel, 1, 2, "heater number")
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("wait_temp() requires the background loop. Call ensure_connected() first.")
        topic = f"{self.base}/temp/{channel}"
        result = {"val": None}
        def _cb(_c, _u, msg):
            try:
                result["val"] = float(msg.payload.decode("utf-8", errors="ignore").strip())
            except ValueError:
                pass
        c.message_callback_add(topic, _cb)
        try:
            c.subscribe(topic, qos=0)
            t0 = time.time()
            while result["val"] is None and (time.time() - t0) < timeout_s:
                time.sleep(0.02)
        finally:
            c.message_callback_remove(topic)
            c.unsubscribe(topic)
        if result["val"] is None:
            raise TimeoutError(f"No temp published on {topic} within {timeout_s:.1f}s")
        return result["val"]

    def get_target(self, channel: int, timeout_s: float = 1.5) -> float:
        """
        Read retained target from set/<n>.
        """
        _check_range(channel, 1, 2, "heater number")
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("get_target() requires the background loop. Call ensure_connected() first.")
        topic = f"{self.base}/set/{channel}"
        result = {"val": None}
        def _cb(_c, _u, msg):
            try:
                result["val"] = float(msg.payload.decode("utf-8", errors="ignore").strip())
            except ValueError:
                pass
        c.message_callback_add(topic, _cb)
        try:
            c.subscribe(topic, qos=0)
            t0 = time.time()
            while result["val"] is None and (time.time() - t0) < timeout_s:
                time.sleep(0.02)
        finally:
            c.message_callback_remove(topic)
            c.unsubscribe(topic)
        if result["val"] is None:
            raise TimeoutError(f"No retained target on {topic} within {timeout_s:.1f}s")
        return result["val"]

    def status(self, seconds: float = 3.0) -> None:
        topics = [f"{self.base}/status", f"{self.base}/heartbeat"] \
               + [f"{self.base}/state/{i}" for i in (1, 2)] \
               + [f"{self.base}/set/{i}"  for i in (1, 2)] \
               + [f"{self.base}/pwm/{i}"  for i in (1, 2)] \
               + [f"{self.base}/temp/{i}" for i in (1, 2)]
        super().status(topics, seconds)
# ---------------------------------------------------------------------------
# Example usage: executed only when running this file directly
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Optional: start local broker if needed
    proc = start_broker_if_needed()

    # Point these at your actual broker IP and creds
    pumps = PumpMQTT(broker="192.168.0.101", username="pump1",  password="pump",
                     base_topic="pumps/01", client_id="pyctl-pumps")
    ultra = UltraMQTT(broker="192.168.0.101", username="ultra1", password="ultra",
                      base_topic="ultra/01", client_id="pyctl-ultra")
    heat  = HeatMQTT (broker="192.168.0.101", username="heat1",  password="heat",
                      base_topic="heat/01",  client_id="pyctl-heat")

    pumps.ensure_connected()
    ultra.ensure_connected()
    heat.ensure_connected()

    # Pump demo
    pumps.on(1, duration_ms=1500)    # run pump 1 for 1.5s
    time.sleep(2)

    # Ultrasonic demo
    ultra.on_for(1, 2000)
    time.sleep(1)

    # Heater + thermistor demo
    heat.set_target(1, 42.0)         # set target to 42C (ESP retains on set/1)
    heat.pid_on(1)                   # enable PID loop on ESP
    # actively request a reading now:
    try:
        t = heat.get_base_temp(1, timeout_s=5.0)
        print("Temp(ch1) =", t)
    except TimeoutError as e:
        print("Temp read timeout:", e)

    time.sleep(3)
    heat.pid_off(1)                  # stop PID
    heat.set_pwm(1, 0)               # ensure PWM is off
    heat.off(1)                      # ensure relay is off

    pumps.disconnect()
    ultra.disconnect()
    heat.disconnect()

    stop_broker(proc)