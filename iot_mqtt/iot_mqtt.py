# iot_mqtt.py
# Unified MQTT helpers for ESP32-POE-ISO devices:
# - Pumps (SparkFun Qwiic Relays)
# - Ultrasonic drivers (SparkFun Qwiic Relays)
# - Heaters (SparkFun Dual SSR)
#
# Requires: paho-mqtt>=1.6 (v2 callback API)

from __future__ import annotations

import atexit
import os
import shutil
import signal
import socket
import subprocess
import threading
import time
import random
from datetime import datetime
from typing import Callable, Final, Iterable, Optional

from paho.mqtt import client as mqtt

# -----------------------------------------------------------------------------
# Global device counts (single source of truth for loops / safety shutdowns)
# -----------------------------------------------------------------------------
PUMP_COUNT: Final[int] = 3
ULTRA_COUNT: Final[int] = 2
HEAT_COUNT: Final[int] = 2


# -----------------------------------------------------------------------------
# Helper: Generate unique client IDs
# -----------------------------------------------------------------------------
def _make_unique_client_id(base: str) -> str:
    """
    Generate a unique client ID by appending hostname and PID.
    Prevents MQTT client ID conflicts when multiple instances run.
    """
    ts = time.strftime("%m%d_%H%M")
    rand = f"{random.randint(0, 0xFFFF):04x}"
    return f"{base}-{ts}-{rand}"


# -----------------------------------------------------------------------------
# Helper: MQTT v5 Disconnect Reason Code Mapping
# -----------------------------------------------------------------------------
def _get_disconnect_reason_message(reason_code: int) -> str:
    """
    Map MQTT v5 disconnect reason codes to human-readable messages.
    Reference: MQTT 5.0 Specification - 3.14.2.2 Disconnect Reason Code
    """
    reason_messages = {
        # Normal disconnection
        0x00: "Normal disconnection",
        0x04: "Disconnect with Will Message",
        
        # Server-initiated disconnects
        0x81: "Unspecified error",
        0x82: "Malformed Packet",
        0x83: "Protocol Error",
        0x84: "Implementation specific error",
        0x85: "Unsupported Protocol Version",
        0x86: "Client Identifier not valid",
        0x87: "Bad User Name or Password",
        0x88: "Not authorized",
        0x89: "Server unavailable",
        0x8A: "Server busy",
        0x8B: "Banned",
        0x8F: "Server shutting down",
        0x90: "Bad authentication method",
        0x91: "Keep Alive timeout",
        0x92: "Session taken over",
        0x93: "Topic Filter invalid",
        0x94: "Topic Name invalid",
        0x95: "Packet Identifier in use",
        0x96: "Packet Identifier not found",
        0x97: "Receive Maximum exceeded",
        0x98: "Topic Alias invalid",
        0x99: "Packet too large",
        0x9A: "Message rate too high",
        0x9B: "Quota exceeded",
        0x9C: "Administrative action",
        0x9D: "Payload format invalid",
        0x9E: "Retain not supported",
        0x9F: "QoS not supported",
        0xA0: "Use another server",
        0xA1: "Server moved",
        0xA2: "Shared Subscriptions not supported",
        0xA3: "Connection rate exceeded",
        0xA4: "Maximum connect time",
        0xA5: "Subscription Identifiers not supported",
        0xA6: "Wildcard Subscriptions not supported",
    }
    return reason_messages.get(reason_code, f"Unknown reason code (0x{reason_code:02X})")


def _format_disconnect_info(reason_code: int, properties: Optional[mqtt.Properties] = None) -> str:
    """
    Format disconnect information including reason code, reason string, and user properties.
    """
    msg = _get_disconnect_reason_message(reason_code)
    parts = [f"Reason: {msg} (0x{reason_code:02X})"]
    
    if properties:
        # Extract reason string if available
        if hasattr(properties, 'ReasonString') and properties.ReasonString:
            parts.append(f"Details: {properties.ReasonString}")
        
        # Extract user properties if available
        if hasattr(properties, 'UserProperty') and properties.UserProperty:
            user_props = []
            # UserProperty is typically a list of tuples
            props_list = properties.UserProperty
            if isinstance(props_list, list):
                for prop in props_list:
                    if isinstance(prop, (list, tuple)) and len(prop) >= 2:
                        user_props.append(f"{prop[0]}={prop[1]}")
                if user_props:
                    parts.append(f"User properties: {', '.join(user_props)}")
    
    return " | ".join(parts)


# -----------------------------------------------------------------------------
# Controller beacon (ONLINE/OFFLINE via LWT + periodic heartbeat)
# -----------------------------------------------------------------------------
class ControllerBeacon:
    """
    Publishes controller ONLINE/OFFLINE (retained) and periodic heartbeat so
    devices can fail-safe. Uses MQTT LWT for crash/kill scenarios.

    Topics:
      - status_topic   (retained): "ONLINE"/"OFFLINE"
      - heartbeat_topic          : "1" periodically
    """

    def __init__(
        self,
        *,
        broker: str = "192.168.0.100",
        port: int = 1883,
        username: Optional[str] = None,
        password: Optional[str] = None,
        client_id: str = "pyctl-controller",
        status_topic: str = "pyctl/status",
        heartbeat_topic: str = "pyctl/heartbeat",
        heartbeat_interval: float = 5.0,
        keepalive: int = 30,
    ) -> None:
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        # Make client ID unique to prevent conflicts
        self.client_id = _make_unique_client_id(client_id)
        self.status_topic = status_topic
        self.heartbeat_topic = heartbeat_topic
        self.heartbeat_interval = heartbeat_interval
        self.keepalive = keepalive

        self._client: Optional[mqtt.Client] = None
        self._hb_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._loop_running = False

    def _build_client(self) -> mqtt.Client:
        # Use MQTT v5 with clean start and session expiry 0 to prevent session conflicts
        c = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
            protocol=mqtt.MQTTv5,
        )
        if self.username and self.password:
            c.username_pw_set(self.username, self.password)

        # LWT: if we die unexpectedly, broker publishes OFFLINE (retained)
        c.will_set(self.status_topic, payload="OFFLINE", qos=1, retain=True)

        # MQTT v5: Set clean start and session expiry 0 (no persistent session)
        connect_properties = mqtt.Properties(mqtt.PacketTypes.CONNECT)
        connect_properties.SessionExpiryInterval = 0

        def _on_connect(client: mqtt.Client, _userdata, _flags, reason_code, _props=None) -> None:
            if reason_code == 0:
                print(f"[ctl] Connected -> {self.broker}:{self.port} (client_id={self.client_id})")
                client.publish(self.status_topic, "ONLINE", qos=1, retain=True)
            else:
                # MQTT v5 connect reason codes
                reason_msg = _get_disconnect_reason_message(reason_code)  # Same mapping works for connect
                print(f"[ctl] Connect failed: {reason_msg} (0x{reason_code:02X})")
                # Additional context from properties if available
                if _props and hasattr(_props, 'ReasonString') and _props.ReasonString:
                    print(f"[ctl] Server message: {_props.ReasonString}")

        def _on_disconnect(client: mqtt.Client, _userdata,  disconnect_flags=None, reason_code: int | None = None, properties: Optional[mqtt.Properties] = None) -> None:
            if reason_code == 0:
                print(f"[ctl] Disconnected normally")
            else:
                disconnect_info = _format_disconnect_info(reason_code, properties)
                print(f"[ctl] Disconnected unexpectedly: {disconnect_info}")
                # Log additional diagnostics for common issues
                if reason_code == 0x92:  # Session taken over
                    print(f"[ctl] WARNING: Another client with same ID connected. Client ID: {self.client_id}")
                elif reason_code == 0x87:  # Bad User Name or Password
                    print(f"[ctl] WARNING: Check MQTT credentials (username/password)")
                elif reason_code == 0x88:  # Not authorized
                    print(f"[ctl] WARNING: Check ACL permissions for user")
                elif reason_code == 0x91:  # Keep Alive timeout
                    print(f"[ctl] WARNING: Network connectivity issue or broker overload")

        c.on_connect = _on_connect
        c.on_disconnect = _on_disconnect
        
        # Store properties for use during connect
        c._connect_properties = connect_properties  # type: ignore[attr-defined]
        return c

    def start(self) -> None:
        if self._client is not None:
            return
        self._client = self._build_client()
        # Use connect with properties for MQTT v5 clean start
        connect_properties = getattr(self._client, '_connect_properties', None)
        if connect_properties:
            self._client.connect(self.broker, self.port, keepalive=self.keepalive, properties=connect_properties)
        else:
            self._client.connect(self.broker, self.port, keepalive=self.keepalive)
        self._client.loop_start()
        self._loop_running = True

        # heartbeat thread
        self._stop.clear()

        def _hb() -> None:
            while not self._stop.is_set():
                try:
                    assert self._client is not None
                    self._client.publish(self.heartbeat_topic, "1", qos=0, retain=False)
                except Exception:
                    pass
                # sleep in small chunks so stop reacts quickly
                chunks = max(1, int(self.heartbeat_interval * 10))
                for _ in range(chunks):
                    if self._stop.is_set():
                        break
                    time.sleep(0.1)

        self._hb_thread = threading.Thread(target=_hb, daemon=True)
        self._hb_thread.start()

        # graceful OFFLINE on exit/signals (LWT still covers crashes)
        atexit.register(self.stop)
        try:
            signal.signal(signal.SIGINT, self._sig_stop)
            signal.signal(signal.SIGTERM, self._sig_stop)
        except Exception:
            # Not all environments allow signal handlers (e.g., notebooks on Windows)
            pass

    def _sig_stop(self, *_args) -> None:
        self.stop()
        # Let normal KeyboardInterrupt handling proceed in scripts
        raise KeyboardInterrupt

    def stop(self) -> None:
        if self._client is None:
            return
        print("[ctl] Stopping controller beacon (OFFLINE)...")
        try:
            self._client.publish(self.status_topic, "OFFLINE", qos=1, retain=True)
        except Exception:
            pass

        # stop heartbeat
        self._stop.set()
        if self._hb_thread and self._hb_thread.is_alive():
            self._hb_thread.join(timeout=2.0)
        self._hb_thread = None

        # stop loop and disconnect
        try:
            if self._loop_running and self._client is not None:
                self._client.loop_stop()
                self._loop_running = False
            if self._client is not None:
                self._client.disconnect()
        finally:
            self._client = None


# -----------------------------------------------------------------------------
# Mosquitto helper (optional local broker)
# -----------------------------------------------------------------------------
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
    def log_with_timestamps(stream, dest) -> None:
        for line in iter(stream.readline, ""):
            ts = datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ")
            dest.write(ts + line)
        stream.close()

    threading.Thread(
        target=log_with_timestamps, args=(proc.stdout, logf), daemon=True
    ).start()

    # --- Wait for broker ready --------------------------------------------
    if not _wait_for_port("127.0.0.1", port, timeout=10):
        proc.terminate()
        raise RuntimeError(f"Broker failed to open port {port}")

    proc._log_handle = logf  # type: ignore[attr-defined]
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

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        msg = f"[{timestamp}] [broker] Mosquitto stopped.\n"
        print(msg.strip())

        if hasattr(proc, "_log_handle") and proc._log_handle:  # type: ignore[attr-defined]
            try:
                proc._log_handle.write(msg)  # type: ignore[attr-defined]
                proc._log_handle.flush()     # type: ignore[attr-defined]
            except Exception:
                pass

    # Close log handle cleanly
    if proc and hasattr(proc, "_log_handle"):
        try:
            proc._log_handle.close()  # type: ignore[attr-defined]
        except Exception:
            pass


def _best_effort_all_off(
    pumps: Optional["PumpMQTT"] = None,
    ultra: Optional["UltraMQTT"] = None,
    heat: Optional["HeatMQTT"] = None,
    ph:    Optional["PhMQTT"]    = None,
) -> None:
    """
    Try to turn every output OFF on clean shutdown (best-effort).
    For pH: tell it STOP (no polling) so probe can idle/sleep.
    For pumps/ultra/heaters: turn off relays / set PWM 0.
    """
    try:
        if pumps:
            for ch in range(1, PUMP_COUNT + 1):
                try:
                    pumps.off(ch)
                    time.sleep(0.02)
                except Exception:
                    pass
        if ultra:
            for ch in range(1, ULTRA_COUNT + 1):
                try:
                    ultra.off(ch)
                    time.sleep(0.02)
                except Exception:
                    pass
        if heat:
            for ch in range(1, HEAT_COUNT + 1):
                try:
                    heat.set_pwm(ch, 0)
                    time.sleep(0.02)
                except Exception:
                    pass
                try:
                    heat.off(ch)
                    time.sleep(0.02)
                except Exception:
                    pass
        if ph:
            try:
                ph.stop_poll()  # ask ESP32 to stop periodic "R"
                time.sleep(0.05)
            except Exception:
                pass
    except Exception:
        pass


# -----------------------------------------------------------------------------
# Base MQTT client wrapper
# -----------------------------------------------------------------------------
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
    ) -> None:
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.base = base_topic.rstrip("/")
        # Make client ID unique to prevent conflicts
        self.client_id = _make_unique_client_id(client_id)
        self.keepalive = keepalive
        self.print_publish = print_publish

        self._client: Optional[mqtt.Client] = None
        self._loop_running: bool = False
        self._watch_thread: Optional[threading.Thread] = None
        self._watch_stop = threading.Event()

    # Connection handling
    def connect(self, retries: int = 1, delay: float = 0.5) -> None:
        """Create a client and open a TCP connection. Does NOT start the network loop."""
        if self._client is not None:
            return

        # Use MQTT v5 with clean start and session expiry 0 to prevent session conflicts
        c = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
            protocol=mqtt.MQTTv5,
        )
        if self.username and self.password:
            c.username_pw_set(self.username, self.password)

        # MQTT v5: Set clean start and session expiry 0 (no persistent session)
        connect_properties = mqtt.Properties(mqtt.PacketTypes.CONNECT)
        connect_properties.SessionExpiryInterval = 0

        def _on_connect(client: mqtt.Client, _userdata, _flags, reason_code, _props=None) -> None:
            if reason_code == 0:
                print(f"[{self.client_id}] Connected to {self.broker}:{self.port}")
            else:
                # MQTT v5 connect reason codes
                reason_msg = _get_disconnect_reason_message(reason_code)  # Same mapping works for connect
                print(f"[{self.client_id}] Connection failed: {reason_msg} (0x{reason_code:02X})")
                # Additional context from properties if available
                if _props and hasattr(_props, 'ReasonString') and _props.ReasonString:
                    print(f"[{self.client_id}] Server message: {_props.ReasonString}")

        def _on_disconnect(client: mqtt.Client, _userdata, disconnect_flags=None, reason_code: int | None = None, properties: Optional[mqtt.Properties] = None) -> None:
            if reason_code == 0:
                print(f"[{self.client_id}] Disconnected normally")
            else:
                disconnect_info = _format_disconnect_info(reason_code, properties)
                print(f"[{self.client_id}] Disconnected unexpectedly: {disconnect_info}")
                # Log additional diagnostics for common issues
                if reason_code == 0x92:  # Session taken over
                    print(f"[{self.client_id}] WARNING: Another client with same ID connected")
                elif reason_code == 0x87:  # Bad User Name or Password
                    print(f"[{self.client_id}] WARNING: Check MQTT credentials (username/password)")
                elif reason_code == 0x88:  # Not authorized
                    print(f"[{self.client_id}] WARNING: Check ACL permissions for user")
                elif reason_code == 0x91:  # Keep Alive timeout
                    print(f"[{self.client_id}] WARNING: Network connectivity issue or broker overload")

        c.on_connect = _on_connect
        c.on_disconnect = _on_disconnect

        for attempt in range(max(1, retries)):
            print(f"[{self.client_id}] Connecting to {self.broker}:{self.port} (attempt {attempt + 1})...")
            try:
                c.connect(self.broker, self.port, keepalive=self.keepalive, properties=connect_properties)
                self._client = c
                return
            except OSError:
                if attempt == retries - 1:
                    raise
                print(f"[{self.client_id}] Connection failed; retrying in {delay} seconds...")
                time.sleep(delay)

    def start(self, retries: int = 5, delay: float = 0.6) -> None:
        """
        Ensure there is a connected client AND that the background loop is running.
        Call this once at the start of your session.
        """
        if self._client is None:
            self.connect(retries=retries, delay=delay)
        if not getattr(self, "_loop_running", False):
            assert self._client is not None
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

    # Publishing & topics
    def _require(self) -> mqtt.Client:
        """Raise if no client connection."""
        if self._client is None:
            raise RuntimeError("Not connected. Call start() first.")
        return self._client

    def _publish(self, topic_suffix: str, payload: str, *, qos: int = 0, retain: bool = False) -> None:
        """Send a payload to a topic under the device's base topic."""
        c = self._require()
        full = f"{self.base}/{topic_suffix}".replace("//", "/")
        c.publish(full, payload, qos=qos, retain=retain)
        if self.print_publish:
            print(f"[{self.client_id}] Published '{payload}' to {full}")

    # Monitoring utilities
    def status(self, topics: Optional[Iterable[str]] = None, seconds: float = 3.0) -> None:
        """Subscribe temporarily to status/heartbeat or custom topics and print messages."""
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("status() requires the background loop running. Call start() first.")

        to_sub = list(topics) if topics else [f"{self.base}/status", f"{self.base}/heartbeat"]
        for t in to_sub:
            c.subscribe(t, qos=0)

        def _on_msg(_client: mqtt.Client, _userdata, msg: mqtt.MQTTMessage) -> None:
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
            for t in to_sub:
                c.unsubscribe(t)

    def watch(self, on_message: Optional[Callable[[str, bytes], None]] = None) -> None:
        """
        Start a background watcher that prints all messages under base/#.
        Requires start() so the background loop is running.
        """
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("watch() requires the background loop running. Call start() first.")

        self._watch_stop.clear()

        def _default_cb(topic: str, payload: bytes) -> None:
            try:
                print(f"{topic} {payload.decode('utf-8', errors='ignore')}")
            except Exception:
                print(f"{topic} <{len(payload)} bytes>")

        cb = on_message or _default_cb

        def _run() -> None:
            old = c.on_message
            c.subscribe(f"{self.base}/#", qos=0)
            try:
                def _on_msg(_c: mqtt.Client, _u, msg: mqtt.MQTTMessage) -> None:
                    cb(msg.topic, msg.payload)

                c.on_message = _on_msg
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


# -----------------------------------------------------------------------------
# Device-specific MQTT wrappers
# -----------------------------------------------------------------------------
def _check_range(value: int, low: int, high: int, label: str = "channel") -> None:
    """Raise ValueError if value is not within the allowed range [low, high]."""
    if not (low <= value <= high):
        raise ValueError(f"{label.capitalize()} must be {low}-{high}")


class PumpMQTT(_BaseDevice):
    """Controls pump relays via MQTT topics under base/cmd/<n>."""
    # Keep validation consistent with module-level constant:
    PUMP_COUNT: Final[int] = PUMP_COUNT  # type: ignore[name-defined]

    def on(self, channel: int, duration_ms: Optional[int] = None) -> None:
        """Turn a pump ON (optionally auto-OFF after duration_ms)."""
        _check_range(channel, 1, self.PUMP_COUNT, "channel")
        cmd = "ON" if duration_ms is None else f"ON:{duration_ms}"
        self._publish(f"cmd/{channel}", cmd, retain=False)

    def off(self, channel: int) -> None:
        """Turn a pump OFF."""
        _check_range(channel, 1, self.PUMP_COUNT, "channel")
        self._publish(f"cmd/{channel}", "OFF", retain=False)

    def toggle(self, channel: int, timeout_s: float = 1.0) -> None:
        """Toggle pump by reading retained state and flipping it (requires background loop)."""
        _check_range(channel, 1, self.PUMP_COUNT, "channel")
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("toggle() requires the background loop. Call start() first.")

        state_topic = f"{self.base}/state/{channel}"
        got: dict[str, Optional[str]] = {"val": None}

        def _cb(_c, _u, msg: mqtt.MQTTMessage) -> None:
            got["val"] = msg.payload.decode(errors="ignore")

        c.message_callback_add(state_topic, _cb)
        try:
            c.subscribe(state_topic, qos=0)
            t0 = time.time()
            while got["val"] is None and (time.time() - t0) < timeout_s:
                time.sleep(0.02)
        finally:
            c.message_callback_remove(state_topic)
            c.unsubscribe(state_topic)

        new_val = "OFF" if (got["val"] == "ON") else "ON"
        self._publish(f"cmd/{channel}", new_val)

    def status(self, seconds: float = 3.0) -> None:
        topics = [f"{self.base}/status", f"{self.base}/heartbeat"] + [
            f"{self.base}/state/{i}" for i in range(1, PUMP_COUNT + 1)
        ]
        super().status(topics, seconds)


class UltraMQTT(_BaseDevice):
    """Controls 2 ultrasonic channels via relay driver."""
    ULTRA_COUNT: Final[int] = ULTRA_COUNT  # type: ignore[name-defined]

    def on(self, channel: int, duration_ms: Optional[int] = None) -> None:
        """Turn an ultrasonic ON (optionally auto-OFF after duration_ms)."""
        _check_range(channel, 1, self.ULTRA_COUNT, "channel")
        cmd = "ON" if duration_ms is None else f"ON:{duration_ms}"
        self._publish(f"cmd/{channel}", cmd, retain=False)

    def off(self, channel: int) -> None:
        _check_range(channel, 1, self.ULTRA_COUNT, "channel")
        self._publish(f"cmd/{channel}", "OFF", retain=False)

    def status(self, seconds: float = 3.0) -> None:
        topics = [f"{self.base}/status", f"{self.base}/heartbeat"] + [
            f"{self.base}/state/{i}" for i in range(1, ULTRA_COUNT + 1)
        ]
        super().status(topics, seconds)


class HeatMQTT(_BaseDevice):
    """Controls 2 heater SSR channels and interacts with thermistor readings."""
    HEAT_COUNT: Final[int] = HEAT_COUNT  # type: ignore[name-defined]

    # Basic ON/OFF & manual PWM
    def on(self, channel: int) -> None:
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        self._publish(f"cmd/{channel}", "ON")

    def off(self, channel: int) -> None:
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        self._publish(f"cmd/{channel}", "OFF")

    def set_pwm(self, channel: int, percent: float) -> None:
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        p = max(0, min(100, int(round(percent))))
        self._publish(f"cmd/{channel}", f"PWM:{p}")

    # PID / Target control
    def set_base_temp(self, channel: int, temp_c: float) -> None:
        """Set target temperature on ESP (publishes retained echo on set/<n>)."""
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        self._publish(f"cmd/{channel}", f"SET:{temp_c:.1f}")

    def set_target(self, channel: int, temp_c: float) -> None:
        """Alias for set_base_temp()."""
        self.set_base_temp(channel, temp_c)

    def pid_on(self, channel: int) -> None:
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        self._publish(f"cmd/{channel}", "PID:ON")

    def pid_off(self, channel: int) -> None:
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        self._publish(f"cmd/{channel}", "PID:OFF")

    # Temperature queries
    def get_base_temp(self, channel: int, timeout_s: float = 1.5) -> float:
        """
        Actively request a temperature reading:
          - Sends "GET" to cmd/<n>
          - Waits for temp/<n>
        """
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("get_base_temp() requires the background loop. Call start() first.")
        topic = f"{self.base}/temp/{channel}"
        result: dict[str, Optional[float]] = {"val": None}

        def _cb(_c, _u, msg: mqtt.MQTTMessage) -> None:
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
        return float(result["val"])

    def wait_temp(self, channel: int, timeout_s: float = 5.0) -> float:
        """Passive wait for the next published temp/<n>."""
        _check_range(channel, 1, self.HEAT_COUNT, "heater number")
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("wait_temp() requires the background loop. Call start() first.")
        topic = f"{self.base}/temp/{channel}"
        result: dict[str, Optional[float]] = {"val": None}

        def _cb(_c, _u, msg: mqtt.MQTTMessage) -> None:
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
        return float(result["val"])

    def status(self, seconds: float = 3.0) -> None:
        topics = [f"{self.base}/status", f"{self.base}/heartbeat"] \
               + [f"{self.base}/state/{i}" for i in range(1, HEAT_COUNT + 1)] \
               + [f"{self.base}/set/{i}"  for i in range(1, HEAT_COUNT + 1)] \
               + [f"{self.base}/pwm/{i}"  for i in range(1, HEAT_COUNT + 1)] \
               + [f"{self.base}/temp/{i}" for i in range(1, HEAT_COUNT + 1)]
        super().status(topics, seconds)

class PhMQTT(_BaseDevice):
    """
    Controls/monitors the ESP32-POE-ISO pH probe node.

    ESP32 publishes:
      <base>/status     retained "ONLINE"/"OFFLINE"
      <base>/heartbeat  "1" every HEARTBEAT_MS
      <base>/ph         latest pH reading as ASCII, e.g. "7.03"
      <base>/reply      replies from passthrough commands / safety notes

    We publish:
      <base>/cmd
         START:<ms>   begin periodic polling (ms between "R")
         STOP         stop polling
         ONESHOT      take one reading now and publish
         <raw>        passthrough to the probe ("i", "Status,?", etc.)
    """

    # --- low-level command publisher ---
    def cmd_raw(self, text: str) -> None:
        """
        Send an arbitrary command string to the ESP32 pH node.
        Examples:
          "ONESHOT"
          "START:5000"
          "STOP"
          "Cal,mid,7.00"
          "Status,?"
        """
        self._publish("cmd", text, qos=0, retain=False)

    # --- high-level convenience commands ---
    def start_poll(self, interval_ms: int) -> None:
        """Ask ESP32 to start periodic polling."""
        self.cmd_raw(f"START:{interval_ms}")

    def stop_poll(self) -> None:
        """Tell ESP32 to stop periodic polling."""
        self.cmd_raw("STOP")

    # oneshot() is now just sugar around watch_ph(..., trigger="ONESHOT")
    def oneshot(self, seconds: float = 3.0, collect: bool = True):
        """
        Request one immediate pH reading and capture results for up to `seconds`.
        Returns a list of (timestamp, value_str) if collect is True, else [].
        """
        return self.watch_ph(
            seconds=seconds,
            trigger_cmd="ONESHOT",
            stop_after=False,
            collect=collect,
            print_live=True,
        )

    def watch_poll(
        self,
        interval_ms: int,
        seconds: float = 15.0,
        collect: bool = True,
    ):
        """
        Start periodic polling at `interval_ms` ms, collect readings for `seconds`,
        then STOP polling.

        Returns a list of (timestamp, value_str) tuples if collect is True.
        """
        return self.watch_ph(
            seconds=seconds,
            trigger_cmd=f"START:{interval_ms}",
            stop_after=True,
            collect=collect,
            print_live=True,
        )

    # --- status snapshot ---
    def status(self, seconds: float = 3.0) -> None:
        """
        Subscribe temporarily to core telemetry topics and print what comes in.
        """
        topics = [
            f"{self.base}/status",
            f"{self.base}/heartbeat",
            f"{self.base}/ph",
            f"{self.base}/reply",
        ]
        super().status(topics, seconds)

    # --- live watcher ---
    def watch_ph(
        self,
        seconds: float = 5.0,
        trigger_cmd: str | None = None,
        stop_after: bool = False,
        collect: bool = True,
        print_live: bool = True,
    ):
        """
        Subscribe to <base>/ph and <base>/reply, listen for `seconds`, and optionally:
          - trigger_cmd: send a command (e.g. 'ONESHOT' or 'START:5000') AFTER subscribing
          - stop_after: send 'STOP' at the end
          - collect: store readings in memory and return them
          - print_live: echo to stdout as they arrive

        Returns:
            data : list[(t, val_str)]
                where t is time.time() (float seconds since epoch),
                and val_str is the raw payload from <base>/ph.
                Only ph readings go into data, not reply messages.
        """
        c = self._require()
        if not getattr(self, "_loop_running", False):
            raise RuntimeError("watch_ph() requires the background loop. Call start() first.")

        topic_ph = f"{self.base}/ph"
        topic_reply = f"{self.base}/reply"
        to_sub = [topic_ph, topic_reply]

        # subscribe first so we don't miss fast responses
        for t in to_sub:
            c.subscribe(t, qos=0)

        data: list[tuple[float, str]] = []

        def _on_msg(_client, _userdata, msg):
            ts = time.time()
            payload_txt = msg.payload.decode("utf-8", errors="ignore")

            if print_live:
                print(f"{msg.topic} {payload_txt}")

            # Only log pH readings, not random replies / safety notes
            if msg.topic == topic_ph and collect:
                data.append((ts, payload_txt))

        old_handler = c.on_message
        c.on_message = _on_msg

        try:
            # fire the trigger after subscriptions are active
            if trigger_cmd is not None:
                self.cmd_raw(trigger_cmd)

            # collect for the requested window
            time.sleep(seconds)

            # send STOP if we're doing a START:<ms> style poll session
            if stop_after:
                self.cmd_raw("STOP")

        finally:
            # restore old handler and unsubscribe
            c.on_message = old_handler
            for t in to_sub:
                c.unsubscribe(t)

        return data

# -----------------------------------------------------------------------------
# Example usage (only when running this file directly)
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    proc = start_broker_if_needed()

    broker = "192.168.0.100"

    # Controller beacon (LWT + heartbeat)
    beacon = ControllerBeacon(
        broker=broker,
        port=1883,
        username="pyctl-controller",
        password="controller",
        client_id="pyctl-controller",
        status_topic="pyctl/status",
        heartbeat_topic="pyctl/heartbeat",
        heartbeat_interval=5.0,
        keepalive=30,
    )
    beacon.start()

    # Device wrappers
    pumps = PumpMQTT(
        broker=broker,
        username="pump1",
        password="pump",
        base_topic="pumps/01",
        client_id="pyctl-pumps",
    )
    ultra = UltraMQTT(
        broker=broker,
        username="ultra1",
        password="ultra",
        base_topic="ultra/01",
        client_id="pyctl-ultra",
    )
    heat = HeatMQTT(
        broker=broker,
        username="heat1",
        password="heat",
        base_topic="heat/01",
        client_id="pyctl-heat",
    )
    ph = PhMQTT(
        broker=broker,
        username="ph1",
        password="ph",
        base_topic="ph/01",
        client_id="pyctl-ph",
    )

    pumps.start()
    ultra.start()
    heat.start()
    ph.start()

    try:
        # --------- Pumps demo ---------
        pumps.on(1, duration_ms=1500)
        time.sleep(2)

        # --------- Ultrasound demo ---------
        # You previously called ultra.on_for(), but UltraMQTT doesn't have on_for().
        # EITHER:
        #   ultra.on(1, duration_ms=2000)
        # OR add an on_for() helper in UltraMQTT that just calls on(...).
        ultra.on(1, duration_ms=2000)
        time.sleep(1)

        # --------- Heaters demo ---------
        heat.set_target(1, 42.0)
        heat.pid_on(1)
        try:
            t = heat.get_base_temp(1, timeout_s=5.0)
            print("Temp(ch1) =", t)
        except TimeoutError as e:
            print("Temp read timeout:", e)
        time.sleep(3)
        heat.pid_off(1)
        heat.set_pwm(1, 0)
        heat.off(1)

        # --------- pH demo ---------
        # ask for a single reading
        ph.oneshot()
        # watch pH and reply messages for a few seconds
        ph.watch_ph(seconds=5.0)

        # start periodic polling every 5 seconds
        ph.start_poll(5000)
        time.sleep(6)
        # stop periodic polling
        ph.stop_poll()

    finally:
        # Best-effort tidy OFF on graceful exit
        _best_effort_all_off(pumps, ultra, heat, ph)

        pumps.disconnect()
        ultra.disconnect()
        heat.disconnect()
        ph.disconnect()

        beacon.stop()
        stop_broker(proc)
