/*
  This example code gives you all the functionality of the SparkFun Qwiic Relay
  Arduino Library. It shows you how to get the firmware version of the board
  you're using, how to turn on and off a relay, toggle a relay, and get the
  status of the relay.
  By: Elias Santistevan
  SparkFun Electronics
  Date: July 2019

  License: This code is public domain but you buy me a beer if you use 
	this and we meet someday (Beerware license).
*/

#include <Wire.h>
#include <ETH.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "SparkFun_Qwiic_Relay.h"

/************ MQTT broker config ************/
const char* MQTT_BROKER_IP   = "192.168.0.102";   // your laptop/broker IP
const uint16_t MQTT_PORT     = 1883;
const char* MQTT_USER        = "ultra1";
const char* MQTT_PASS        = "ultra";
const char* DEV_BASE         = "ultra/01";

/************ Ethernet (Olimex ESP32-POE-ISO) ************/
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   0
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_POWER_PIN  12
#define ETH_CLK_MODE   ETH_CLOCK_GPIO17_OUT

/************ Relay config ************/
#define RELAY_ADDR_1     0x0A
#define ULTRA_COUNT      2          // exactly two ultrasonic outputs: 1..2
#define HEARTBEAT_MS     15000
#define TIMED_SLOTS      4

/************ Globals ************/
WiFiClient   netClient;
PubSubClient mqtt(netClient);
static bool eth_ready = false;
char clientId[40];

Qwiic_Relay relay(RELAY_ADDR_1); 
bool relay_ok = false;       // whether .begin() succeeded
bool ultraIsOn[ULTRA_COUNT + 1] = {false}; // 1-based cache for retained publishes

// Represents a scheduled auto-OFF task triggered by "ON:<ms>"
struct TimeRun {
  uint8_t ch; 
  uint32_t endMs; 
  bool active; 
} slots[TIMED_SLOTS];

unsigned long lastBeat = 0; // Timestamp of the last heartbeat publish

/************ Helpers ************/

// Convert a user-facing ultrasonic channel number to a relay instance & channel 
bool chanToRelay(uint8_t ch, Qwiic_Relay*& board, uint8_t& relayCh) {
  if (ch < 1 || ch > ULTRA_COUNT) return false; 
  board = &relay; 
  relayCh = ch;   // 1->relay#1, 2->relay#2
  return true;
}

// Force an ultrasonic channel ON or OFF and publish retained state
bool setUltra(uint8_t ch, bool on) {
  Qwiic_Relay* b = nullptr; uint8_t rch = 0;
  if (!chanToRelay(ch, b, rch)) {
    Serial.printf("[WARN] Invalid ultrasonic channel: %d\n", ch);
    return false;
  }

  if (!relay_ok) {
    Serial.println("[INFO] Relay not initialized, trying begin()...");
    relay_ok = b->begin();
    if (!relay_ok) {
      Serial.println("[ERROR] Qwiic Relay not detected on I2C!");
      return false;
    }
  }

  Serial.printf("[ACTION] Ultrasonic %d -> %s\n", ch, on ? "ON" : "OFF");
  if (on) b->turnRelayOn(rch);
  else    b->turnRelayOff(rch);

  ultraIsOn[ch] = on;

  char topic[48];
  snprintf(topic, sizeof(topic), "%s/state/%u", DEV_BASE, ch);
  mqtt.publish(topic, on ? "ON" : "OFF", true); // retained
  return true;
}

// Read the current hardware state of an ultrasonic channel from the relay
bool getUltraStateFromHW(uint8_t ch, bool& on) {
  Qwiic_Relay* b = nullptr; uint8_t rch = 0;
  if (!chanToRelay(ch, b, rch)) return false;

  if (!relay_ok) relay_ok = b->begin();
  if (!relay_ok) return false;

  uint8_t s = b->getState(rch);  // 0 = off, 1 = on
  on = (s == 1);
  Serial.printf("[STATE] Ultrasonic %d current state: %s\n", ch, on ? "ON" : "OFF");
  return true;
}

/************ MQTT ************/

// Publish device status to "<DEV_BASE>/status" as a retained message.
void publishStatus(const char* s) {
  char topic[48]; 
  snprintf(topic, sizeof(topic), "%s/status", DEV_BASE);
  mqtt.publish(topic, s, true);
  Serial.printf("[MQTT] Status published: %s\n", s); 
}

// Ensure MQTT is connected; if not, (re)connect and subscribe to command topics.
void ensureMqtt() {
  if (!eth_ready) return; 

  while (!mqtt.connected()) {
    uint32_t id = (uint32_t)ESP.getEfuseMac(); 
    snprintf(clientId, sizeof(clientId), "ultra01-%08X", id); 

    Serial.printf("[MQTT] Connecting to %s:%d as %s...\n", MQTT_BROKER_IP, MQTT_PORT, clientId);
    if (mqtt.connect(clientId, MQTT_USER, MQTT_PASS,
                     (String(DEV_BASE) + "/status").c_str(), 1, true, "OFFLINE")) {
      Serial.println("[MQTT] Connected successfully!");
      publishStatus("ONLINE");

      // Subscribe to commands: ultra/01/cmd/<n>
      char filter[48];
      snprintf(filter, sizeof(filter), "%s/cmd/#", DEV_BASE);
      mqtt.subscribe(filter, 0);
      Serial.printf("[MQTT] Subscribed to: %s\n", filter);

      // Republish current states (retained)
      for (uint8_t i = 1; i <= ULTRA_COUNT; i++) {
        bool on = false;
        if (getUltraStateFromHW(i, on)) {
          char t[48];
          snprintf(t, sizeof(t), "%s/state/%u", DEV_BASE, i);
          mqtt.publish(t, on ? "ON" : "OFF", true);
        }
      }
    } else {
      Serial.printf("[MQTT] Failed (rc=%d). Retrying...\n", mqtt.state());
      delay(1500);
    }
  }
}

// Handle an incoming command message for a specific channel.
void handleCmd(const char* topic, const char* payload) {
  // topic: ultra/01/cmd/<n>
  // payload: "ON" | "OFF" | "ON:<ms>"
  int ch = -1; 
  const char* p = strrchr(topic, '/');
  if (p) ch = atoi(p + 1); 
  if (ch < 1 || ch > ULTRA_COUNT) {
    Serial.printf("[WARN] Invalid topic: %s\n", topic);
    return; 
  }

  Serial.printf("[MQTT] Command received: ultra %d <- %s\n", ch, payload);

  if (strncasecmp(payload, "ON:", 3) == 0) {
    uint32_t ms = atoi(payload + 3);
    if (ms == 0) return;
    if (!setUltra(ch, true)) return;

    for (int i = 0; i < TIMED_SLOTS; i++) {
      if (!slots[i].active) {
        slots[i].active = true;
        slots[i].ch     = ch;
        slots[i].endMs  = millis() + ms;
        Serial.printf("[TIMER] Ultra %d scheduled OFF in %lu ms\n", ch, ms);
        break;
      }
    }
    return;
  }

  if (strcasecmp(payload, "ON") == 0) setUltra(ch, true);
  else if (strcasecmp(payload, "OFF") == 0) setUltra(ch, false);
}

// PubSubClient callback adapter: converts raw payload to C-string and dispatches to handleCmd().
void onMqtt(char* topic, byte* payload, unsigned int len) {
  char msg[40];
  len = (len < sizeof(msg)-1) ? len : sizeof(msg)-1;
  memcpy(msg, payload, len); msg[len] = '\0';
  handleCmd(topic, msg);
}

// WiFi/Ethernet event handler for the ESP32 Arduino core.
void onNetEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("esp32-ultra");
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
      Serial.println("[ETH] Disconnected");
      eth_ready = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] Stopped");
      eth_ready = false;
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32 Ultrasonic Controller starting...");

  Wire.begin();
  Wire.setClock(400000);
  Serial.println("[I2C] Initialized for Qwiic bus");

  WiFi.onEvent(onNetEvent);
  Serial.println("[ETH] Initializing Ethernet...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_POWER_PIN, ETH_CLK_MODE);

  relay_ok = relay.begin();
  if (relay_ok) Serial.println("[I2C] Qwiic Relay detected.");
  else Serial.println("[I2C] Qwiic Relay NOT found!");

  // Ensure channels are OFF at boot (and publish retained states)
  for (uint8_t i = 1; i <= ULTRA_COUNT; i++) setUltra(i, false);

  mqtt.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setKeepAlive(30);

  Serial.println("[BOOT] Setup complete.\n");
}

void loop() {
  if (eth_ready && !mqtt.connected()) ensureMqtt();

  if (mqtt.connected()) {
    mqtt.loop();

    unsigned long now = millis();
    if (now - lastBeat > HEARTBEAT_MS) {
      lastBeat = now;
      String t = String(DEV_BASE) + "/heartbeat";
      mqtt.publish(t.c_str(), "1");
      Serial.println("[HEARTBEAT] Sent");
    }

    for (int i = 0; i < TIMED_SLOTS; i++) {
      if (slots[i].active && millis() >= slots[i].endMs) {
        setUltra(slots[i].ch, false);
        slots[i].active = false;
        Serial.printf("[TIMER] Ultra %d OFF (timer expired)\n", slots[i].ch);
      }
    }
  }
}