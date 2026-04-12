// ============================================================
//  SunDial_Bridge.ino
//  ESP32-S3  |  WatchTower Protocol v2.0
//  Watches HOUSE_1-5 outputs from the SunDial Arduino and
//  forwards puzzle state to the Alchemy MQTT broker.
//
//  Wiring:
//    Arduino HOUSE_1 (pin 2) ──► ESP32 GPIO 4
//    Arduino HOUSE_2 (pin 3) ──► ESP32 GPIO 5
//    Arduino HOUSE_3 (pin 4) ──► ESP32 GPIO 6
//    Arduino HOUSE_4 (pin 5) ──► ESP32 GPIO 7
//    Arduino HOUSE_5 (pin 6) ──► ESP32 GPIO 15
//    Arduino GND             ──► ESP32 GND  (shared ground required)
//
//  MQTT Topics (publish):
//    MermaidsTale/SunDial/status   – ONLINE, HEARTBEAT, progress, SOLVED
//    MermaidsTale/SunDial/log      – mirrored serial output
//    MermaidsTale/MonkeyDoorsTotems/command – sundialTotemOn/Off
//
//  MQTT Topics (subscribe):
//    MermaidsTale/SunDial/command  – PING | STATUS | RESET | PUZZLE_RESET
// ============================================================

#include <WiFi.h>
#include <PubSubClient.h>

#define FW_VERSION "2.0.0"

// ── WiFi ─────────────────────────────────────────────────────
const char* WIFI_SSID = "AlchemyGuest";
const char* WIFI_PASS = "YourPassword";

// ── MQTT ─────────────────────────────────────────────────────
const char*    MQTT_HOST    = "10.1.10.115";
const uint16_t MQTT_PORT    = 1883;
const char*    DEVICE_NAME  = "SunDial";
const char*    TOPIC_CMD    = "MermaidsTale/SunDial/command";
const char*    TOPIC_STAT   = "MermaidsTale/SunDial/status";
const char*    TOPIC_LOG    = "MermaidsTale/SunDial/log";
const char*    TOPIC_TOTEM  = "MermaidsTale/MonkeyDoorsTotems/command";

// ── Input pins (from Arduino HOUSE_1–5) ──────────────────────
const int NUM_CONDITIONS = 5;
const int HOUSE_PINS[NUM_CONDITIONS] = { 4, 5, 6, 7, 15 };

const char* CONDITION_LABELS[NUM_CONDITIONS] = {
  "Bottle", "Seahorse", "Coconut", "Trident", "Skull"
};

// ── Timing ───────────────────────────────────────────────────
const unsigned long HEARTBEAT_INTERVAL_MS = 300000UL;  // 5 minutes
const unsigned long DEBOUNCE_MS           = 80;

// ── State ────────────────────────────────────────────────────
bool conditionState[NUM_CONDITIONS]     = { false };
bool lastConditionState[NUM_CONDITIONS] = { false };
unsigned long lastChangeMs[NUM_CONDITIONS] = { 0 };

bool puzzleSolved    = false;
bool solvedPublished = false;

unsigned long lastHeartbeatMs = 0;
unsigned long startupMs       = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// ─────────────────────────────────────────────────────────────
//  Logging — mirrors Serial to MQTT /log
// ─────────────────────────────────────────────────────────────
void logLine(const char* msg) {
  Serial.println(msg);
  if (mqtt.connected()) mqtt.publish(TOPIC_LOG, msg);
}

const char* stateString() {
  if (puzzleSolved) return "SOLVED";
  int count = 0;
  for (int i = 0; i < NUM_CONDITIONS; i++) if (conditionState[i]) count++;
  if (count == 0) return "IDLE";
  return "ACTIVE";
}

int progressCount() {
  int count = 0;
  for (int i = 0; i < NUM_CONDITIONS; i++) if (conditionState[i]) count++;
  return count;
}

// ─────────────────────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────────────────────
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected – IP %s\n", WiFi.localIP().toString().c_str());
}

// ─────────────────────────────────────────────────────────────
//  WatchTower publish helpers
// ─────────────────────────────────────────────────────────────
void publishHeartbeat() {
  char buf[96];
  unsigned long uptime = (millis() - startupMs) / 1000;
  snprintf(buf, sizeof(buf), "HEARTBEAT:%s:UP%lus:RSSI%d",
           stateString(), uptime, WiFi.RSSI());
  mqtt.publish(TOPIC_STAT, buf);
  Serial.printf("[MQTT] %s\n", buf);
}

void publishStatusReply() {
  char buf[160];
  unsigned long uptime = (millis() - startupMs) / 1000;
  snprintf(buf, sizeof(buf),
    "STATUS:%s:UP%lus:RSSI%d:PROGRESS%d/%d:VER%s",
    stateString(), uptime, WiFi.RSSI(),
    progressCount(), NUM_CONDITIONS, FW_VERSION);
  mqtt.publish(TOPIC_STAT, buf);
  Serial.printf("[MQTT] %s\n", buf);
}

void publishProgress() {
  char buf[64];
  snprintf(buf, sizeof(buf), "PROGRESS:%d/%d:%s",
           progressCount(), NUM_CONDITIONS, stateString());
  mqtt.publish(TOPIC_STAT, buf);
  Serial.printf("[MQTT] %s\n", buf);
}

void publishSolved() {
  mqtt.publish(TOPIC_STAT, "SOLVED", true);  // retained
  mqtt.publish(TOPIC_TOTEM, "sundialTotemOn");
  logLine("[MQTT] *** SunDial SOLVED ***");
  logLine("[MQTT] -> MonkeyDoorsTotems: sundialTotemOn");
}

void clearSolvedRetained() {
  mqtt.publish(TOPIC_STAT, "", true);
  Serial.println("[MQTT] Solved retained cleared");
}

// ─────────────────────────────────────────────────────────────
//  Reset helpers
// ─────────────────────────────────────────────────────────────
void resetPuzzleState() {
  for (int i = 0; i < NUM_CONDITIONS; i++) {
    conditionState[i]     = false;
    lastConditionState[i] = false;
  }
  puzzleSolved    = false;
  solvedPublished = false;
  clearSolvedRetained();
  mqtt.publish(TOPIC_TOTEM, "sundialTotemOff");
  Serial.println("[MQTT] -> MonkeyDoorsTotems: sundialTotemOff");
}

// ─────────────────────────────────────────────────────────────
//  MQTT callback (incoming commands)
// ─────────────────────────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[64] = {0};
  memcpy(msg, payload, min((unsigned int)63, length));
  Serial.printf("[MQTT] <- %s : %s\n", topic, msg);

  if (strcmp(msg, "PING") == 0) {
    mqtt.publish(TOPIC_STAT, "PONG");
    Serial.println("[MQTT] PONG");

  } else if (strcmp(msg, "STATUS") == 0) {
    publishStatusReply();

  } else if (strcmp(msg, "RESET") == 0) {
    mqtt.publish(TOPIC_STAT, "OK");
    logLine("[CMD] RESET — stopping and rebooting");
    resetPuzzleState();
    delay(200);
    ESP.restart();

  } else if (strcmp(msg, "PUZZLE_RESET") == 0) {
    mqtt.publish(TOPIC_STAT, "OK");
    logLine("[CMD] PUZZLE_RESET — clearing state");
    resetPuzzleState();
    publishProgress();
  }
}

// ─────────────────────────────────────────────────────────────
//  MQTT connect / reconnect
// ─────────────────────────────────────────────────────────────
void ensureMqtt() {
  if (mqtt.connected()) return;

  Serial.printf("[MQTT] Connecting to %s:%d ...", MQTT_HOST, MQTT_PORT);
  String clientId = String(DEVICE_NAME) + "-" + String(random(0xffff), HEX);

  if (mqtt.connect(clientId.c_str())) {
    Serial.println(" connected");
    mqtt.subscribe(TOPIC_CMD);
    mqtt.publish(TOPIC_STAT, "ONLINE");
    Serial.println("[MQTT] ONLINE published");
    publishHeartbeat();
  } else {
    Serial.printf(" failed (state=%d), retry in 3s\n", mqtt.state());
    delay(3000);
  }
}

// ─────────────────────────────────────────────────────────────
//  setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=============================");
  Serial.println("  SunDial Bridge – booting");
  Serial.printf("  FW %s\n", FW_VERSION);
  Serial.println("=============================");

  startupMs = millis();

  for (int i = 0; i < NUM_CONDITIONS; i++) {
    pinMode(HOUSE_PINS[i], INPUT);
    conditionState[i]     = false;
    lastConditionState[i] = false;
  }

  connectWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);

  ensureMqtt();
}

// ─────────────────────────────────────────────────────────────
//  loop
// ─────────────────────────────────────────────────────────────
void loop() {
  connectWiFi();
  ensureMqtt();
  mqtt.loop();

  unsigned long now = millis();

  // ── Read and debounce each HOUSE pin ──────────────────────
  bool anyChange = false;

  for (int i = 0; i < NUM_CONDITIONS; i++) {
    bool raw = digitalRead(HOUSE_PINS[i]) == HIGH;

    if (raw != lastConditionState[i]) {
      lastChangeMs[i] = now;
      lastConditionState[i] = raw;
    }

    if ((now - lastChangeMs[i]) >= DEBOUNCE_MS) {
      if (raw != conditionState[i]) {
        conditionState[i] = raw;
        anyChange = true;
        char line[64];
        snprintf(line, sizeof(line), "[PIN] %s -> %s",
          CONDITION_LABELS[i], raw ? "CORRECT" : "cleared");
        logLine(line);
      }
    }
  }

  // ── Publish progress on every change, handle solve edge ───
  if (anyChange) {
    bool allSolved = (progressCount() == NUM_CONDITIONS);

    publishProgress();

    if (allSolved && !solvedPublished) {
      puzzleSolved    = true;
      solvedPublished = true;
      publishSolved();
    } else if (!allSolved && puzzleSolved) {
      puzzleSolved    = false;
      solvedPublished = false;
      clearSolvedRetained();
      mqtt.publish(TOPIC_TOTEM, "sundialTotemOff");
      Serial.println("[MQTT] -> MonkeyDoorsTotems: sundialTotemOff");
    }
  }

  // ── Heartbeat (WatchTower: every 5 min) ───────────────────
  if ((now - lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
    publishHeartbeat();
    lastHeartbeatMs = now;
  }
}
