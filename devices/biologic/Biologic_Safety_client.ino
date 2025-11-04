/*
  ESP32-POE-ISO (Olimex) — Biologic controller via 2 8-channel Qwiic Relay
  MCP23017 +MQTT + Safety (controller heartbeat/LWT, MQTT timeout,
   link-down, max ON lease)

  Commands (MQTT, bio/01/cmd/<1-16>):
    "ON" | "OFF"

	MQTT topics:
    bio/01/state/<n>   : retained "ON"/"OFF"
    bio/01/status      : retained "ONLINE"/"OFFLINE"
    bio/01/heartbeat   : "1" every 15s

  Controller supervision (subscribe):
    pyctl/status   : retained "ONLINE"/"OFFLINE" (controller's LWT)
    pyctl/heartbeat: "1" periodically
*/

#include <Adafruit_MCP23X17.h>
#include <ETH.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>

/************ MQTT broker config ************/
const char *MQTT_BROKER_IP = "192.168.0.100";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_USER = "bio1";
const char *MQTT_PASS = "bio";
const char *DEV_BASE = "bio/01";

/************ Supervision topics (from Python controller) ************/
const char *CTRL_STATUS_TOPIC = "pyctl/status"; // retained ONLINE/OFFLINE (LWT)
const char *CTRL_HEARTBEAT_TOPIC = "pyctl/heartbeat"; // "1" periodically

/************ Safety timeouts (tune as needed) ************/
const uint32_t CTRL_TIMEOUT_MS = 25000; // if no controller beat -> pause probe
const uint32_t MQTT_DOWN_OFF_MS = 20000; // if MQTT reconnect stalls -> pause probe
const uint32_t HEARTBEAT_MS = 15000; // ESP heartbeat cadence

/************ Ethernet (Olimex ESP32-POE-ISO) ************/
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_PHY_ADDR 0
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_POWER_PIN 12
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

/************ Biologic config ************/
#define BIO_COUNT 16
#define MCP_ADDR 0x20

/************ Globals ************/
WiFiClient netClient;
PubSubClient mqtt(netClient);
bool eth_ready = false;
char clientId[40];

Adafruit_MCP23X17 mcp;
bool mcp_ok = false;
bool relayIsOn[BIO_COUNT + 1] = {false}; // 1-based cache

uint32_t lastBeat = 0;        // last time we sent heartbeat
uint32_t lastCtrlBeat = 0;    // last controller heartbeat we saw
uint32_t lastMqttHealthy = 0; // last time MQTT loop was healthy
bool ctrlSeen = false;        // have we seen controller ONLINE/heartbeat yet?

/************ Helpers ************/
static inline bool validCh(uint8_t ch) { return ch >= 1 && ch <= BIO_COUNT; }

// Map channel 1..16 → MCP pin 0..15
static inline uint8_t chToPin(uint8_t ch) {
  return validCh(ch) ? (ch - 1) : 255;
}

void publishStatus(const char *s) {
  char t[48];
  snprintf(t, sizeof(t), "%s/status", DEV_BASE);
  mqtt.publish(t, s, true);
  Serial.printf("[MQTT] Status published: %s\n", s);
}

void publishState(uint8_t ch) {
  char t[48];
  snprintf(t, sizeof(t), "%s/state/%u", DEV_BASE, ch);
  mqtt.publish(t, relayIsOn[ch] ? "ON" : "OFF", true);
}

bool relayWrite(uint8_t ch, bool on) {
  if (!mcp_ok) return false;
  uint8_t pin = chToPin(ch);
  if (pin == 255) return false;

  // active-low relay inputs
  bool level = on ? LOW : HIGH;
  mcp.digitalWrite(pin, level);

  relayIsOn[ch] = on;
  publishState(ch);
  Serial.printf("[ACTION] Relay %u -> %s\n", ch, on ? "ON" : "OFF");
  return true;
}

/************ Safety: ALL OFF ************/
void allOff(const char *reason) {
  Serial.printf("[SAFETY] ALL OFF (%s)\n", reason ? reason : "unspecified");
  if (!mcp_ok) return;
  for (uint8_t ch = 1; ch <= BIO_COUNT; ch++) {
    relayIsOn[ch] = false;
    uint8_t pin = chToPin(ch);
    if (pin != 255) {
      mcp.digitalWrite(pin, HIGH); // HIGH = OFF (active-low)
    }
    if (mqtt.connected()) publishState(ch);
  }
}

/************ MQTT ************/
void ensureMqtt() {
  if (!eth_ready)
    return;

  while (!mqtt.connected()) {
    uint32_t id = (uint32_t)ESP.getEfuseMac();
    snprintf(clientId, sizeof(clientId), "bio01-%08X", id);

    Serial.printf("[MQTT] Connecting to %s:%d as %s...\n", MQTT_BROKER_IP,
                  MQTT_PORT, clientId);
    if (mqtt.connect(clientId, MQTT_USER, MQTT_PASS,
                     (String(DEV_BASE) + "/status").c_str(), 1, true,
                     "OFFLINE")) {
      Serial.println("[MQTT] Connected successfully!");
      lastMqttHealthy = millis();

      publishStatus("ONLINE");

      // Device command subscription
      char filter[48];
      snprintf(filter, sizeof(filter), "%s/cmd/#", DEV_BASE);
      mqtt.subscribe(filter, 0);
      Serial.printf("[MQTT] Subscribeds to: %s\n", filter);

      // Supervision subscriptions
      mqtt.subscribe(CTRL_STATUS_TOPIC, 1);
      mqtt.subscribe(CTRL_HEARTBEAT_TOPIC, 0);
      Serial.printf("[MQTT] Subscribed to: %s, %s\n", CTRL_STATUS_TOPIC,
                    CTRL_HEARTBEAT_TOPIC);

      lastCtrlBeat = millis();
      ctrlSeen = false;

      // Republish retained state
      for (uint8_t ch = 1; ch <= BIO_COUNT; ch++) publishState(ch);

    } else {
      Serial.printf("[MQTT] Failed (rc=%d). Retrying...\n", mqtt.state());
      delay(1200);
      // If we can't reconnect for long enough, fail-safe OFF
      if (millis() - lastMqttHealthy > MQTT_DOWN_OFF_MS) {
        allOff("MQTT reconnect timeout");
      }
    }
  }
}

void handleCmd(const char *topic, const char *payload) {
  int ch = -1;
  const char *tail = strrchr(topic, '/');
  if (tail)
    ch = atoi(tail + 1);
  if (!validCh(ch)) {
    Serial.printf("[WARN] Invalid topic: %s\n", topic);
    return;
  }

  Serial.printf("[MQTT] Command received: biologic %d <- %s\n", ch, payload);

  if (strcasecmp(payload, "ON") == 0) {
    if (relayWrite(ch, true)) {
      Serial.printf("[ACTION] Biologic %d -> ON\n", ch);
    }
  } else if (strcasecmp(payload, "OFF") == 0) {
    if (relayWrite(ch, false)) {
      Serial.printf("[ACTION] Biologic %d -> OFF\n", ch);
    }
  } else {
    Serial.printf("[WARN] Unknown command payload: %s\n", payload);
  }
}

void onMqtt(char *topic, byte *payload, unsigned int len) {
  char msg[64];
  len = (len < sizeof(msg) - 1) ? len : sizeof(msg) - 1;
  memcpy(msg, payload, len);
  msg[len] = '\0';

  // DEBUG: show what we actually receive
  Serial.printf("[MQTT] RX topic='%s' payload='%s'\n", topic, msg);

  if (strcmp(topic, CTRL_HEARTBEAT_TOPIC) == 0) {
    lastCtrlBeat = millis();
    ctrlSeen = true;
    Serial.printf("[CTRL] heartbeat seen, lastCtrlBeat=%lu\n", lastCtrlBeat);
    return;
  }
  if (strcmp(topic, CTRL_STATUS_TOPIC) == 0) {
    Serial.printf("[CTRL] status='%s'\n", msg);
    if (strcasecmp(msg, "OFFLINE") == 0) {
      ctrlSeen = false;
      allOff("controller LWT OFFLINE");
    } else if (strcasecmp(msg, "ONLINE") == 0) {
      lastCtrlBeat = millis();
      ctrlSeen = true;
    }
    return;
  }

  if (strncmp(topic, DEV_BASE, strlen(DEV_BASE)) == 0) {
    handleCmd(topic, msg);
  }
}

/************ Ethernet events ************/
void onNetEvent(WiFiEvent_t event) {
  switch (event) {
  case ARDUINO_EVENT_ETH_START:
    ETH.setHostname("esp32-bio");
    Serial.println("[ETH] Started");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    Serial.println("[ETH] Connected (link up)");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    eth_ready = true;
    Serial.print("[ETH] IP obtained: ");
    Serial.println(ETH.localIP());
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    Serial.println("[ETH] Disconnected (link down)");
    eth_ready = false;
    allOff("Ethernet link down");
    break;
  case ARDUINO_EVENT_ETH_STOP:
    Serial.println("[ETH] Stopped");
    eth_ready = false;
    allOff("Ethernet stopped");
    break;
  default:
    break;
  }
}

/************ Setup / Loop ************/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32 Biologic Controller starting...");

  Wire.begin();
  Wire.setClock(400000);
  Serial.println("[I2C] Initialized for MCP23017");

  Wire.onEvent(onNetEvent);
  Serial.println("[ETH] Initializing Ethernet...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            ETH_POWER_PIN, ETH_CLK_MODE);

  mcp_ok = mcp.begin_I2C(MCP_ADDR);
  Serial.printf("[I2C] MCP 0x%02X: %s\n", MCP_ADDR, mcp_ok ? "OK" : "NOT FOUND");

  if (!mcp_ok) {
    Serial.println("[FATAL] MCP23017 missing — relays disabled.");
  } else {
    // Configure 16 outputs and default OFF (active-low → drive HIGH)
    for (uint8_t pin = 0; pin < 16; pin++) {
      mcp.pinMode(pin, OUTPUT);
      mcp.digitalWrite(pin, HIGH); // OFF
    }
    for (uint8_t ch = 1; ch <= BIO_COUNT; ch++) relayIsOn[ch] = false;
  }

  mqtt.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setKeepAlive(30);

  lastCtrlBeat = millis(); // start with “recent” so we don’t trip immediately
  lastMqttHealthy = millis();
  ctrlSeen = false;

  Serial.println("[BOOT] Setup complete.\n");
}

void loop() {
  uint32_t now = millis();

  // Maintain MQTT, and fail-safe if reconnect takes too long
  if (eth_ready && !mqtt.connected()) {
    ensureMQTT();
  }
  if (mqtt.connected()) {
    mqtt.loop();
    lastMqttHealthy = now;
  }

  // Refresh 'now' AFTER mqtt.loop() so comparisons use a time >= lastCtrlBeat
  now = millis();

  // Controller heartbeat timeout -> OFF (use signed delta to avoid underflow)
  if (mqtt.connected() && ctrlSeen) {
    int32_t dt = (int32_t)(now - lastCtrlBeat); // signed
    if (dt >= 0 && (uint32_t)dt > CTRL_TIMEOUT_MS) {
      static uint32_t lastTimeoutLog = 0;
      if (now - lastTimeoutLog > 2000) {
        Serial.printf("[SAFETY] CTRL timeout: delta=%ldms (> %lums)\n",
                      (long)dt, (unsigned long)CTRL_TIMEOUT_MS);
        lastTimeoutLog = now;
      }
      allOff("controller heartbeat timeout");
      // Do NOT fake the beat by writing lastCtrlBeat=now; just wait for a real
      // heartbeat.
      ctrlSeen = false; // require a fresh ONLINE/heartbeat before enforcing again
    }
  }

  // Our own heartbeat (nice to have for dashboards)
  if (mqtt.connected()) {
    if (now - lastBeat > HEARTBEAT_MS) {
      lastBeat = now;
      String topic = String(DEV_BASE) + "/heartbeat";
      mqtt.publish(topic.c_str(), "1");
      Serial.println("[HEARTBEAT] Sent");
    }
  }

  delay(10);
}