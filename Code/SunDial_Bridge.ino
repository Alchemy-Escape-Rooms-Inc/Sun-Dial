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
//    MermaidsTale/SunDial            – status JSON (boot + 5-min heartbeat)
//    MermaidsTale/SunDialSolved      – "triggered" when all 5 conditions met
//
//  MQTT Topics (subscribe):
//    MermaidsTale/SunDial            – commands: PING | STATUS | RESET | PUZZLE_RESET
// ============================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── WiFi ─────────────────────────────────────────────────────
const char* WIFI_SSID = "AlchemyNetwork";
const char* WIFI_PASS = "YourPassword";

// ── MQTT ─────────────────────────────────────────────────────
const char*    MQTT_HOST   = "10.1.10.115";
const uint16_t MQTT_PORT   = 1860;
const char*    DEVICE_NAME = "SunDial";
const char*    TOPIC_CMD   = "MermaidsTale/SunDial";
const char*    TOPIC_STAT  = "MermaidsTale/SunDial";       // status on same topic
const char*    TOPIC_SOLVED  = "MermaidsTale/SunDialSolved";
const char*    TOPIC_TOTEM  = "MermaidsTale/MonkeyDoorsTotems/command";

// ── Input pins (from Arduino HOUSE_1–5) ──────────────────────
const int NUM_CONDITIONS = 5;
const int HOUSE_PINS[NUM_CONDITIONS] = { 4, 5, 6, 7, 15 };

// Human-readable label for each condition (matches puzzle symbols)
const char* CONDITION_LABELS[NUM_CONDITIONS] = {
  "Bottle",    // correct_outer_1 / correct_inner_1
  "Seahorse",  // correct_outer_2 / correct_inner_2
  "Coconut",   // correct_outer_3 / correct_inner_3
  "Trident",   // correct_outer_4 / correct_inner_4
  "Skull"      // correct_outer_5 / correct_inner_5
};

// ── Timing ───────────────────────────────────────────────────
const unsigned long STATUS_INTERVAL_MS = 5UL * 60UL * 1000UL;  // 5 minutes
const unsigned long DEBOUNCE_MS        = 80;

// ── State ────────────────────────────────────────────────────
bool     conditionState[NUM_CONDITIONS]  = { false };
bool     lastConditionState[NUM_CONDITIONS] = { false };
unsigned long lastChangeMs[NUM_CONDITIONS] = { 0 };

bool     puzzleSolved       = false;
bool     solvedPublished     = false;

unsigned long lastStatusMs   = 0;
unsigned long startupMs      = 0;

// ── MQTT client ──────────────────────────────────────────────
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

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
//  MQTT publish helpers
// ─────────────────────────────────────────────────────────────
void publishStatus() {
  StaticJsonDocument<256> doc;
  doc["device"]  = DEVICE_NAME;
  doc["solved"]  = puzzleSolved;
  doc["uptime"]  = (millis() - startupMs) / 1000;

  JsonObject conds = doc.createNestedObject("conditions");
  for (int i = 0; i < NUM_CONDITIONS; i++) {
    conds[CONDITION_LABELS[i]] = conditionState[i];
  }

  int solvedCount = 0;
  for (int i = 0; i < NUM_CONDITIONS; i++) {
    if (conditionState[i]) solvedCount++;
  }
  doc["progress"] = solvedCount;   // 0-5

  char buf[256];
  serializeJson(doc, buf);
  mqtt.publish(TOPIC_STAT, buf, false);
  Serial.printf("[MQTT] Status published – progress %d/5\n", solvedCount);
}

void publishSolved() {
  mqtt.publish(TOPIC_SOLVED, "triggered", true);  // retained
  mqtt.publish(TOPIC_TOTEM, "sundialTotemOn");
  Serial.println("[MQTT] *** SunDial SOLVED – triggered published ***");
  Serial.println("[MQTT] → MonkeyDoorsTotems: sundialTotemOn");
}

void clearSolvedRetained() {
  // Publish empty retained message to clear the solved flag in broker
  mqtt.publish(TOPIC_SOLVED, "", true);
  Serial.println("[MQTT] Solved retained cleared");
}

// ─────────────────────────────────────────────────────────────
//  MQTT callback (incoming commands)
// ─────────────────────────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[64] = {0};
  memcpy(msg, payload, min((unsigned int)63, length));
  Serial.printf("[MQTT] ← %s : %s\n", topic, msg);

  // WatchTower Protocol v2.0 commands
  if (strcmp(msg, "PING") == 0) {
    mqtt.publish(TOPIC_STAT, "{\"device\":\"SunDial\",\"event\":\"PONG\"}");

  } else if (strcmp(msg, "STATUS") == 0) {
    publishStatus();

  } else if (strcmp(msg, "RESET") == 0 || strcmp(msg, "PUZZLE_RESET") == 0) {
    Serial.println("[CMD] RESET received – clearing state");
    for (int i = 0; i < NUM_CONDITIONS; i++) {
      conditionState[i]     = false;
      lastConditionState[i] = false;
    }
    puzzleSolved    = false;
    solvedPublished = false;
    clearSolvedRetained();
    mqtt.publish(TOPIC_TOTEM, "sundialTotemOff");
    Serial.println("[MQTT] → MonkeyDoorsTotems: sundialTotemOff");
    publishStatus();
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
    publishStatus();   // WatchTower: status on boot
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
  Serial.println("=============================");

  startupMs = millis();

  for (int i = 0; i < NUM_CONDITIONS; i++) {
    pinMode(HOUSE_PINS[i], INPUT);   // Arduino drives these HIGH; no pull needed
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
        Serial.printf("[PIN] %s → %s\n",
          CONDITION_LABELS[i], raw ? "CORRECT ✓" : "cleared");
      }
    }
  }

  // ── Check for full solve ───────────────────────────────────
  if (anyChange) {
    int count = 0;
    for (int i = 0; i < NUM_CONDITIONS; i++) {
      if (conditionState[i]) count++;
    }

    bool allSolved = (count == NUM_CONDITIONS);

    if (allSolved && !solvedPublished) {
      puzzleSolved    = true;
      solvedPublished = true;
      publishSolved();
      publishStatus();
    } else if (!allSolved && puzzleSolved) {
      // Arduino reset the puzzle (HOUSE pins dropped)
      puzzleSolved    = false;
      solvedPublished = false;
      clearSolvedRetained();
      mqtt.publish(TOPIC_TOTEM, "sundialTotemOff");
      Serial.println("[MQTT] → MonkeyDoorsTotems: sundialTotemOff");
      publishStatus();
    }
  }

  // ── Heartbeat status (WatchTower: every 5 min) ─────────────
  if ((now - lastStatusMs) >= STATUS_INTERVAL_MS) {
    publishStatus();
    lastStatusMs = now;
  }
}
