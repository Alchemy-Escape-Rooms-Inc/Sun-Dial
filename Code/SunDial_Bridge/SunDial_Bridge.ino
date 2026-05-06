// ============================================================
//  SunDial_Bridge.ino
//  ESP32-S3
//
//  Logic:
//    - Watch 5 input pins from the SunDial Arduino.
//    - Each pin pulses HIGH for ~2s when its symbol is correct,
//      then returns LOW. The Arduino is 5V; pins go through a
//      level shifter / divider to 3.3V at the ESP32.
//    - Pin events are captured in hardware ISRs (rising edge)
//      so MQTT/WiFi work in loop() can never block us long
//      enough to miss a pulse.
//    - Each pin only counts once per round.
//    - When 5 distinct pins have fired, publish SOLVED + totem,
//      then wait for ALL pins to return LOW (so we don't
//      immediately re-latch on stale HIGHs) before re-arming.
//
//  Wiring (any pin can carry any symbol — order doesn't matter):
//    Arduino HOUSE_1..5 -> level shifter -> ESP32 GPIO 4,5,6,7,15
//    Arduino GND        -> ESP32 GND
//
//  MQTT topics (publish):
//    MermaidsTale/SunDial/status            - ONLINE, HEARTBEAT, STATUS,
//                                              "Correct N/5", SOLVED, OK, PONG
//    MermaidsTale/SunDial/log               - mirrored serial output
//    MermaidsTale/MonkeyDoorsTotems/command - sundialTotemOn / sundialTotemOff
//
//  MQTT topics (subscribe):
//    MermaidsTale/SunDial/command           - PING | STATUS | RESET |
//                                              PUZZLE_RESET | CLEAR_STATUS
// ============================================================

#include <WiFi.h>
#include <PubSubClient.h>

#define FW_VERSION "3.2.0"

const char* WIFI_SSID = "AlchemyGuest";
const char* WIFI_PASS = "VoodooVacation5601";

const char*    MQTT_HOST   = "10.1.10.115";
const uint16_t MQTT_PORT   = 1883;
const char*    DEVICE_NAME = "SunDial";
const char*    TOPIC_CMD   = "MermaidsTale/SunDial/command";
const char*    TOPIC_STAT  = "MermaidsTale/SunDial/status";
const char*    TOPIC_LOG   = "MermaidsTale/SunDial/log";
const char*    TOPIC_TOTEM = "MermaidsTale/MonkeyDoorsTotems/command";

const int NUM_PINS = 5;
const int HOUSE_PINS[NUM_PINS] = { 4, 5, 6, 7, 15 };

const unsigned long HEARTBEAT_INTERVAL_MS = 300000UL;  // 5 minutes

// Set HIGH by the ISR on a rising edge. Cleared by loop() once consumed.
volatile bool pendingEdge[NUM_PINS] = { false, false, false, false, false };
// Tracks which pins have already scored this round.
bool counted[NUM_PINS] = { false, false, false, false, false };

unsigned long bootMs = 0;
unsigned long lastHeartbeatMs = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// One ISR per pin. Kept tiny — no Serial, no MQTT, no delay.
void IRAM_ATTR isrPin0() { pendingEdge[0] = true; }
void IRAM_ATTR isrPin1() { pendingEdge[1] = true; }
void IRAM_ATTR isrPin2() { pendingEdge[2] = true; }
void IRAM_ATTR isrPin3() { pendingEdge[3] = true; }
void IRAM_ATTR isrPin4() { pendingEdge[4] = true; }
void (*const ISRS[NUM_PINS])() = { isrPin0, isrPin1, isrPin2, isrPin3, isrPin4 };

int countedTotal() {
  int n = 0;
  for (int i = 0; i < NUM_PINS; i++) if (counted[i]) n++;
  return n;
}

void logLine(const char* msg) {
  Serial.println(msg);
  if (mqtt.connected()) mqtt.publish(TOPIC_LOG, msg);
}

void publishHeartbeat() {
  char buf[80];
  unsigned long uptime = (millis() - bootMs) / 1000UL;
  const char* state = (countedTotal() == NUM_PINS) ? "SOLVED" : "RUNNING";
  snprintf(buf, sizeof(buf), "HEARTBEAT:%s:UP%lus:RSSI%d",
           state, uptime, WiFi.RSSI());
  mqtt.publish(TOPIC_STAT, buf);
}

void publishStatus() {
  char buf[120];
  unsigned long uptime = (millis() - bootMs) / 1000UL;
  const char* state = (countedTotal() == NUM_PINS) ? "SOLVED" : "RUNNING";
  snprintf(buf, sizeof(buf),
           "STATUS:%s:UP%lus:RSSI%d:PROGRESS%d/%d:VER%s",
           state, uptime, WiFi.RSSI(), countedTotal(), NUM_PINS, FW_VERSION);
  mqtt.publish(TOPIC_STAT, buf);
}

void resetState() {
  noInterrupts();
  for (int i = 0; i < NUM_PINS; i++) { counted[i] = false; pendingEdge[i] = false; }
  interrupts();
  mqtt.publish(TOPIC_TOTEM, "sundialTotemOff");
  logLine("[RESET] state cleared");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[32] = {0};
  memcpy(msg, payload, min((unsigned int)31, length));

  if (strcmp(msg, "PING") == 0) {
    mqtt.publish(TOPIC_STAT, "PONG");
    Serial.println("[MQTT] PONG");
  } else if (strcmp(msg, "STATUS") == 0) {
    publishStatus();
    Serial.println("[MQTT] STATUS published");
  } else if (strcmp(msg, "RESET") == 0) {
    mqtt.publish(TOPIC_STAT, "OK");
    logLine("[CMD] RESET — rebooting");
    mqtt.publish(TOPIC_TOTEM, "sundialTotemOff");
    delay(200);
    ESP.restart();
  } else if (strcmp(msg, "PUZZLE_RESET") == 0) {
    mqtt.publish(TOPIC_STAT, "OK");
    logLine("[CMD] PUZZLE_RESET — clearing state");
    resetState();
  } else if (strcmp(msg, "CLEAR_STATUS") == 0) {
    mqtt.publish(TOPIC_STAT, "OK");
    mqtt.publish(TOPIC_STAT, "", true);  // wipe retained
    logLine("[CMD] CLEAR_STATUS — wiped retained status");
  }
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
}

void ensureMqtt() {
  if (mqtt.connected()) return;
  String clientId = String(DEVICE_NAME) + "-" + String(random(0xffff), HEX);
  if (mqtt.connect(clientId.c_str())) {
    mqtt.subscribe(TOPIC_CMD);
    mqtt.publish(TOPIC_STAT, "ONLINE");
    Serial.println("[MQTT] ONLINE published");
  }
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();
  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(HOUSE_PINS[i], INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(HOUSE_PINS[i]), ISRS[i], RISING);
  }
  ensureWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  ensureMqtt();
  Serial.printf("SunDial Bridge ready (FW %s)\n", FW_VERSION);
}

void loop() {
  ensureWiFi();
  ensureMqtt();
  mqtt.loop();

  // Drain pending edges captured by ISRs.
  for (int i = 0; i < NUM_PINS; i++) {
    if (!pendingEdge[i]) continue;
    noInterrupts();
    pendingEdge[i] = false;
    interrupts();

    if (counted[i]) {
      Serial.printf("[PIN %d edge ignored — already counted]\n", HOUSE_PINS[i]);
      continue;
    }
    counted[i] = true;
    int total = countedTotal();
    char buf[32];
    snprintf(buf, sizeof(buf), "Correct %d/%d", total, NUM_PINS);
    mqtt.publish(TOPIC_STAT, buf);
    Serial.printf("[PIN %d HIGH] %s\n", HOUSE_PINS[i], buf);
  }

  if (countedTotal() == NUM_PINS) {
    mqtt.publish(TOPIC_STAT, "SOLVED");
    mqtt.publish(TOPIC_TOTEM, "sundialTotemOn");
    logLine("SOLVED");
    delay(500);

    // Wait for the Arduino to drop all HOUSE pins LOW before re-arming,
    // so a stale HIGH doesn't immediately re-latch a pin in the next round.
    unsigned long t0 = millis();
    while (millis() - t0 < 8000) {
      bool anyHigh = false;
      for (int i = 0; i < NUM_PINS; i++)
        if (digitalRead(HOUSE_PINS[i]) == HIGH) { anyHigh = true; break; }
      if (!anyHigh) break;
      mqtt.loop();
      delay(50);
    }
    resetState();
  }

  unsigned long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    if (mqtt.connected()) publishHeartbeat();
  }
}
