/*
  ESP32-POE-ISO — pH monitor with Atlas EZO pH (UART) + MQTT control

  Hardware:
    - Atlas EZO pH in UART mode
    - ESP32-POE-ISO UART1 wired:
        RX = GPIO36 (ESP32 RX in), TX = GPIO4 (ESP32 TX out)
        Baud: 9600 8N1

  MQTT topics (base = ph/01):

    PC -> ESP32:
      ph/01/cmd
        "START:<ms>"   start periodic pH reads every <ms> milliseconds
        "STOP"         stop periodic reads
        "ONESHOT"      take 1 reading now and publish
        anything else  forward directly to the EZO board
                       reply published to ph/01/reply

    ESP32 -> PC:
      ph/01/ph          latest pH reading as ASCII (ex "7.03")
      ph/01/reply       reply from passthrough commands
      ph/01/status      retained "ONLINE"/"OFFLINE"
      ph/01/heartbeat   "1" every HEARTBEAT_MS

  Controller supervision (subscribe):
    pyctl/status   : retained "ONLINE"/"OFFLINE" (controller's LWT)
    pyctl/heartbeat: "1" periodically

  Safety:
    - If controller heartbeat is missing > CTRL_TIMEOUT_MS → put probe to sleep
    - If MQTT can't reconnect for MQTT_DOWN_OFF_MS → put probe to sleep
    - If Ethernet link drops → put probe to sleep

  Notes:
    - We keep the probe in "quiet" mode (C,0 and *OK,0).
    - We (ESP) actively poll "R" on a timer — we do NOT enable the probe's own
      continuous output. This way we can stop polling instantly on STOP/safety.
*/

#include <ETH.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Ezo_uart.h>

/************ MQTT broker config ************/
const char* MQTT_BROKER_IP = "192.168.0.100";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "ph1";
const char* MQTT_PASS = "ph";
const char* DEV_BASE = "ph/01";

/************ Supervision topics (from Python controller) ************/
const char* CTRL_STATUS_TOPIC = "pyctl/status"; // retained ONLINE/OFFLINE (LWT)
const char* CTRL_HEARTBEAT_TOPIC = "pyctl/heartbeat"; // "1" periodically

/************ Safety timeouts (tune as needed) ************/
const uint32_t CTRL_TIMEOUT_MS = 25000; // if no controller beat -> sleep probe
const uint32_t MQTT_DOWN_OFF_MS =
    20000;                           // if MQTT reconnect stalls -> sleep probe
const uint32_t HEARTBEAT_MS = 15000; // ESP heartbeat cadence

/************ Ethernet (Olimex ESP32-POE-ISO) ************/
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_PHY_ADDR 0
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_POWER_PIN 12
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

/************ Globals ************/
WiFiClient netClient;
PubSubClient mqtt(netClient);
bool eth_ready = false;
char clientId[40];

Ezo_uart PH(Serial1, "pH");
bool ph_ok = false;

uint32_t lastBeat = 0;        // last time we sent heartbeat
uint32_t lastCtrlBeat = 0;    // last controller heartbeat we saw
uint32_t lastMqttHealthy = 0; // last time MQTT loop was healthy
bool ctrlSeen = false;        // have we seen controller ONLINE/heartbeat yet?

// polling control
uint32_t pollIntervalMs = 0; // 0 = not polling
uint32_t lastPollMs = 0;     // last time we asked probe "R"

/************ Probe helpers ************/

// Send a command to EZO and capture reply into respOut
void ezoCommand(const char *cmd, char *respOut, size_t respLen) {
  // Atlas lib blocks for response and null-terminates
  PH.send_cmd(cmd, respOut, respLen);
}

// Take one blocking reading using "R" and publish it
void takeOneReadingAndPublish() {
  char resp[32] = {0};
  ezoCommand("R", resp, sizeof(resp));

  float val = atof(resp);
  bool plausible = (val > 0.0f && val < 20.0f); // rough sanity gate

  if (!plausible) {
    Serial.printf("[WARN] weird pH '%s'\n", resp);
  }
  publishPH(resp);
}

// Put the probe to sleep / safe idle
void sleepPH(const char *reason) {
  Serial.printf("[SAFETY] Sleep pH (%s)\n", reason ? reason : "unspecified");
  // stop polling
  pollIntervalMs = 0;
  // tell probe to stop continuous outputs just in case
  PH.send_cmd_no_resp("C,0");
  delay(50);
  // sleep mode (low power)
  PH.send_cmd_no_resp("Sleep");
}

/************ Publishing helpers ************/
void publishStatus(const char *s) {
  char t[48];
  snprintf(t, sizeof(t), "%s/status", DEV_BASE);
  mqtt.publish(t, s, true); // retained
  Serial.printf("[MQTT] Status published: %s\n", s);
}

void publishHeartbeat() {
  char t[48];
  snprintf(t, sizeof(t), "%s/heartbeat", DEV_BASE);
  mqtt.publish(t, "1");
  Serial.println("[HEARTBEAT] Sent");
}

void publishPH(const char *phTxt) {
  char t[48];
  snprintf(t, sizeof(t), "%s/ph", DEV_BASE);
  mqtt.publish(t, phTxt, false); // not retained
  Serial.printf("[PH] %s\n", phTxt);
}

void publishReply(const char *replyTxt) {
  char t[48];
  snprintf(t, sizeof(t), "%s/reply", DEV_BASE);
  mqtt.publish(t, replyTxt, false);
  Serial.printf("[REPLY] %s\n", replyTxt);
}

/************ MQTT connection ************/
void ensureMqtt() {
  if (!eth_ready)
    return;

  while (!mqtt.connected()) {
    uint32_t id = (uint32_t)ESP.getEfuseMac();
    snprintf(clientId, sizeof(clientId), "ph01-%08X", id);

    Serial.printf("[MQTT] Connecting to %s:%d as %s...\n", MQTT_BROKER_IP,
                  MQTT_PORT, clientId);

    if (mqtt.connect(clientId, MQTT_USER, MQTT_PASS,
                     (String(DEV_BASE) + "/status").c_str(), 1, true,
                     "OFFLINE")) {
      Serial.println("[MQTT] Connected successfully!");
      lastMqttHealthy = millis();

      publishStatus("ONLINE");

      // Device command subscription
      char cmdTopic[48];
      snprintf(cmdTopic, sizeof(cmdTopic), "%s/cmd", DEV_BASE);
      mqtt.subscribe(cmdTopic, 0);
      Serial.printf("[MQTT] Subscribed: %s\n", cmdTopic);

      // Supervision subscriptions
      mqtt.subscribe(CTRL_STATUS_TOPIC, 1);
      mqtt.subscribe(CTRL_HEARTBEAT_TOPIC, 0);
      Serial.printf("[MQTT] Subscribed: %s, %s\n", CTRL_STATUS_TOPIC,
                    CTRL_HEARTBEAT_TOPIC);

      // don't instantly freak out on boot/reconnect
      lastCtrlBeat = millis();
      ctrlSeen = false;

    } else {
      Serial.printf("[MQTT] Failed (rc=%d). Retrying...\n", mqtt.state());
      delay(1200);
      // If we can't get MQTT for a while, go safe
      if (millis() - lastMqttHealthy > MQTT_DOWN_OFF_MS) {
        sleepPH("MQTT reconnect timeout");
      }
    }
  }
}

// payload is a null-terminated ASCII command from ph/01/cmd
void handleCmd(const char *payload) {
  Serial.printf("[MQTT] Command received: ph <- %s\n", payload);

  // START:<ms>
  if (strncasecmp(payload, "START:", 6) == 0) {
    uint32_t ms = atoi(payload + 6);
    if (ms < 2000)
      ms = 2000; // don't hammer UART super fast
    if (ms > 99000)
      ms = 99000; // don't let it go insanely long
    pollIntervalMs = ms;
    lastPollMs = 0; // force immediate sample next loop
    Serial.printf("[POLL] start interval=%lu ms\n", (unsigned long)ms);
    return;
  }

  // STOP
  if (strcasecmp(payload, "STOP") == 0) {
    pollIntervalMs = 0;
    Serial.println("[POLL] stopped");
    return;
  }

  // ONESHOT
  if (strcasecmp(payload, "ONESHOT") == 0) {
    takeOneReadingAndPublish();
    return;
  }

  // passthrough -> send to probe, publish reply
  {
    char resp[64] = {0};
    ezoCommand(payload, resp, sizeof(resp));
    publishReply(resp);
  }
}

/************ MQTT callback ************/
void onMqtt(char *topic, byte *payload, unsigned int len) {
  // copy payload safely into msg[]
  char msg[96];
  len = (len < sizeof(msg) - 1) ? len : sizeof(msg) - 1;
  memcpy(msg, payload, len);
  msg[len] = '\0';

  Serial.printf("[MQTT] RX topic='%s' payload='%s'\n", topic, msg);

  // controller heartbeat?
  if (strcmp(topic, CTRL_HEARTBEAT_TOPIC) == 0) {
    lastCtrlBeat = millis();
    ctrlSeen = true;
    Serial.printf("[CTRL] heartbeat seen, lastCtrlBeat=%lu\n", lastCtrlBeat);
    return;
  }

  // controller ONLINE/OFFLINE (LWT)
  if (strcmp(topic, CTRL_STATUS_TOPIC) == 0) {
    Serial.printf("[CTRL] status='%s'\n", msg);
    if (strcasecmp(msg, "OFFLINE") == 0) {
      ctrlSeen = false;
      sleepPH("controller LWT OFFLINE");
    } else if (strcasecmp(msg, "ONLINE") == 0) {
      lastCtrlBeat = millis();
      ctrlSeen = true;
    }
    return;
  }

  // device command topic?
  char cmdTopic[48];
  snprintf(cmdTopic, sizeof(cmdTopic), "%s/cmd", DEV_BASE);
  if (strcmp(topic, cmdTopic) == 0) {
    handleCmd(msg);
    return;
  }
}

/************ Ethernet events ************/
void onNetEvent(WiFiEvent_t event) {
  switch (event) {
  case ARDUINO_EVENT_ETH_START:
    ETH.setHostname("esp32-ph");
    Serial.println("[ETH] START");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    Serial.println("[ETH] LINK UP");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    eth_ready = true;
    Serial.print("[ETH] IP: ");
    Serial.println(ETH.localIP());
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    Serial.println("[ETH] LINK DOWN");
    eth_ready = false;
    sleepPH("Ethernet link down");
    break;
  case ARDUINO_EVENT_ETH_STOP:
    Serial.println("[ETH] STOP");
    eth_ready = false;
    sleepPH("Ethernet stopped");
    break;
  default:
    break;
  }
}

/************ Setup ************/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32 pH Controller starting...");

  // Bring up Ethernet
  WiFi.onEvent(onNetEvent);
  Serial.println("[ETH] Initializing Ethernet...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            ETH_POWER_PIN, ETH_CLK_MODE);

  // Bring up UART to Atlas EZO pH
  // IMPORTANT: this matches your working test sketch
  Serial1.begin(9600, SERIAL_8N1, 36, 4); // RX=36, TX=4
  delay(500);

  // Put probe in quiet/command mode:
  // 1) Stop internal continuous output
  PH.send_cmd_no_resp("C,0");
  delay(200);
  // 2) Disable "*OK" prefixes so replies are clean
  PH.send_cmd_no_resp("*OK,0");
  delay(200);
  // 3) Clear any junk in RX buffer
  PH.flush_rx_buffer();

  // quick probe ping
  {
    char resp[32] = {0};
    PH.send_cmd("i", resp, sizeof(resp)); // "i" gives device info
    if (strlen(resp) > 0) {
      ph_ok = true;
      Serial.printf("[EZO] probe detected: %s\n", resp);
    } else {
      ph_ok = false;
      Serial.println("[EZO] probe not responding");
    }
  }

  // MQTT client setup
  mqtt.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setKeepAlive(30);

  // seed timing so we don't immediately alarm
  lastCtrlBeat = millis();
  lastMqttHealthy = millis();
  ctrlSeen = false;

  // no polling yet
  pollIntervalMs = 0;
  lastPollMs = 0;

  Serial.println("[BOOT] Setup complete.\n");
}

/************ Loop ************/
void loop() {
  uint32_t now = millis();

  // keep MQTT alive, or go safe if we can't
  if (eth_ready && !mqtt.connected()) {
    ensureMqtt();
  }
  if (mqtt.connected()) {
    mqtt.loop();
    lastMqttHealthy = now;
  }

  // refresh timestamp after mqtt.loop()
  now = millis();

  // controller heartbeat supervision
  if (mqtt.connected() && ctrlSeen) {
    int32_t dt = (int32_t)(now - lastCtrlBeat);
    if (dt >= 0 && (uint32_t)dt > CTRL_TIMEOUT_MS) {
      static uint32_t lastTimeoutLog = 0;
      if (now - lastTimeoutLog > 2000) {
        Serial.printf("[SAFETY] CTRL timeout: delta=%ldms (> %lums)\n",
                      (long)dt, (unsigned long)CTRL_TIMEOUT_MS);
        lastTimeoutLog = now;
      }
      sleepPH("controller heartbeat timeout");
      ctrlSeen = false; // wait for fresh ONLINE/heartbeat
    }
  }

  // publish heartbeat
  if (mqtt.connected() && (now - lastBeat > HEARTBEAT_MS)) {
    lastBeat = now;
    publishHeartbeat();
  }

  // periodic polling if enabled
  if (mqtt.connected() && pollIntervalMs > 0) {
    if (now - lastPollMs >= pollIntervalMs) {
      lastPollMs = now;
      takeOneReadingAndPublish();
    }
  }

  delay(10);
}
