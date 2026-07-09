// ============================================================
//  SunDial_Bridge.ino
//  ESP32-S3
//
//  Logic:
//    - Watch 5 input pins from the SunDial Arduino. Each pin is
//      pulsed HIGH for ~2s when its symbol is correctly chosen.
//    - Pin events are captured in hardware ISRs (rising edge) so
//      MQTT/WiFi work in loop() can never block long enough to
//      miss a pulse.
//    - On each rising edge, publish "true" (NOT retained) to that
//      symbol's MQTT topic. M3 owns the round / SOLVED logic.
//      Retained was removed in 4.1.0: a retained "true" replays into
//      M3 on reconnect and re-fires the Correct/Solved events.
//    - On PUZZLE_RESET / RESET (and at boot), wipe any retained
//      residue off the broker, then publish "false" (not retained)
//      to all 5 symbol topics so M3 sees a clean slate.
//
//  Wiring:
//    Arduino HOUSE_1..5 -> level shifter -> ESP32 GPIO 4,5,6,7,15
//    Arduino GND        -> ESP32 GND
//
//  Symbol mapping (HOUSE_n -> ESP32 GPIO -> Symbol):
//    HOUSE_1 -> GPIO 4  -> Bottle
//    HOUSE_2 -> GPIO 5  -> Crab
//    HOUSE_3 -> GPIO 6  -> Turtle
//    HOUSE_4 -> GPIO 7  -> Coconut
//    HOUSE_5 -> GPIO 15 -> Trident
//
//  MQTT topics (publish, not retained):
//    MermaidsTale/SunDial/Bottle    "true" | "false"
//    MermaidsTale/SunDial/Crab      "true" | "false"
//    MermaidsTale/SunDial/Turtle    "true" | "false"
//    MermaidsTale/SunDial/Coconut   "true" | "false"
//    MermaidsTale/SunDial/Trident   "true" | "false"
//
//  MQTT topics (publish, not retained):
//    MermaidsTale/SunDial/status    ONLINE, HEARTBEAT, STATUS, OK
//    MermaidsTale/SunDial/log       mirrored serial output
//
//  MQTT topics (subscribe; PONG is answered back on this topic):
//    MermaidsTale/SunDial/command   PING | STATUS | RESET |
//                                    PUZZLE_RESET | CLEAR_STATUS
// ============================================================

#include <WiFi.h>
#include <PubSubClient.h>

#define FW_VERSION "4.2.0"

const char* WIFI_SSID = "AlchemyGuest";
const char* WIFI_PASS = "VoodooVacation5601";

const char*    MQTT_HOST   = "10.1.10.115";
const uint16_t MQTT_PORT   = 1883;
const char*    DEVICE_NAME = "SunDial";
const char*    TOPIC_CMD   = "MermaidsTale/SunDial/command";
const char*    TOPIC_STAT  = "MermaidsTale/SunDial/status";
const char*    TOPIC_LOG   = "MermaidsTale/SunDial/log";

const int NUM_PINS = 5;
const int HOUSE_PINS[NUM_PINS] = { 4, 5, 6, 7, 15 };
const char* const SYMBOL_TOPICS[NUM_PINS] = {
  "MermaidsTale/SunDial/Bottle",
  "MermaidsTale/SunDial/Crab",
  "MermaidsTale/SunDial/Turtle",
  "MermaidsTale/SunDial/Coconut",
  "MermaidsTale/SunDial/Trident"
};

const unsigned long HEARTBEAT_INTERVAL_MS = 300000UL;  // 5 minutes

// Glitch rejection (added 4.2.0). Wire logs on 2026-07-09 show phantom
// edges on several pins at once right at power-on (18:03:10: three
// symbols "true" 60ms after boot) — electrical noise while the Arduino
// side powers up, not guest input.
//   - Edges within BOOT_EDGE_MASK_MS of boot are discarded outright.
//   - Any other edge must still read HIGH after EDGE_CONFIRM_MS before
//     it is published. Real pulses hold ~2s; noise doesn't survive 30ms.
const unsigned long BOOT_EDGE_MASK_MS = 3000;
const unsigned long EDGE_CONFIRM_MS   = 30;

// Set HIGH by the ISR on a rising edge. Cleared by loop() once consumed.
volatile bool pendingEdge[NUM_PINS] = { false, false, false, false, false };

// millis() when a drained edge started its confirm window; 0 = none.
unsigned long edgeSeenMs[NUM_PINS] = { 0, 0, 0, 0, 0 };

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

void logLine(const char* msg) {
  Serial.println(msg);
  if (mqtt.connected()) mqtt.publish(TOPIC_LOG, msg);
}

void publishAllSymbols(const char* value) {
  for (int i = 0; i < NUM_PINS; i++) {
    mqtt.publish(SYMBOL_TOPICS[i], value, false);
  }
}

// Publish a retained empty payload to each symbol topic. This deletes
// any retained message still stored on the broker (including leftovers
// from firmware <= 4.0.0, which published retained values).
void wipeRetainedSymbols() {
  for (int i = 0; i < NUM_PINS; i++) {
    mqtt.publish(SYMBOL_TOPICS[i], "", true);
  }
}

void publishHeartbeat() {
  char buf[80];
  unsigned long uptime = (millis() - bootMs) / 1000UL;
  snprintf(buf, sizeof(buf), "HEARTBEAT:RUNNING:UP%lus:RSSI%d",
           uptime, WiFi.RSSI());
  mqtt.publish(TOPIC_STAT, buf);
}

void publishStatus() {
  char buf[120];
  unsigned long uptime = (millis() - bootMs) / 1000UL;
  snprintf(buf, sizeof(buf),
           "STATUS:RUNNING:UP%lus:RSSI%d:VER%s",
           uptime, WiFi.RSSI(), FW_VERSION);
  mqtt.publish(TOPIC_STAT, buf);
}

void resetSymbols() {
  noInterrupts();
  for (int i = 0; i < NUM_PINS; i++) pendingEdge[i] = false;
  interrupts();
  for (int i = 0; i < NUM_PINS; i++) edgeSeenMs[i] = 0;
  wipeRetainedSymbols();
  publishAllSymbols("false");
  logLine("[RESET] all symbols set to false");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[32] = {0};
  memcpy(msg, payload, min((unsigned int)31, length));

  if (strcmp(msg, "PING") == 0) {
    // WatchTower protocol: PONG goes back on /command, the same topic
    // the PING arrived on. Our own subscription echoes it back to us;
    // it matches no command so the callback ignores it.
    mqtt.publish(TOPIC_CMD, "PONG");
    Serial.println("[MQTT] PONG");
  } else if (strcmp(msg, "STATUS") == 0) {
    publishStatus();
    Serial.println("[MQTT] STATUS published");
  } else if (strcmp(msg, "RESET") == 0) {
    mqtt.publish(TOPIC_STAT, "OK");
    logLine("[CMD] RESET — rebooting");
    wipeRetainedSymbols();
    publishAllSymbols("false");
    delay(200);
    ESP.restart();
  } else if (strcmp(msg, "PUZZLE_RESET") == 0) {
    mqtt.publish(TOPIC_STAT, "OK");
    logLine("[CMD] PUZZLE_RESET — clearing symbols");
    resetSymbols();
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
  // Clean slate on boot so M3 sees a known state. The retained wipe
  // also clears anything left on the broker by older firmware.
  wipeRetainedSymbols();
  publishAllSymbols("false");
  Serial.printf("SunDial Bridge ready (FW %s)\n", FW_VERSION);
}

void loop() {
  ensureWiFi();
  ensureMqtt();
  mqtt.loop();

  // Drain pending edges captured by ISRs. Each edge opens a confirm
  // window; the pin must still be HIGH when it closes or the edge is
  // dropped as noise. No dedupe and no counting — M3 owns that.
  unsigned long nowMs = millis();
  for (int i = 0; i < NUM_PINS; i++) {
    if (pendingEdge[i]) {
      noInterrupts();
      pendingEdge[i] = false;
      interrupts();

      if (nowMs - bootMs < BOOT_EDGE_MASK_MS) {
        Serial.printf("[PIN %d] edge ignored (boot mask)\n", HOUSE_PINS[i]);
      } else if (edgeSeenMs[i] == 0) {
        edgeSeenMs[i] = nowMs;
      }
    }

    if (edgeSeenMs[i] != 0 && nowMs - edgeSeenMs[i] >= EDGE_CONFIRM_MS) {
      edgeSeenMs[i] = 0;
      if (digitalRead(HOUSE_PINS[i]) == HIGH) {
        mqtt.publish(SYMBOL_TOPICS[i], "true", false);
        Serial.printf("[PIN %d HIGH] %s = true\n",
                      HOUSE_PINS[i], SYMBOL_TOPICS[i]);
      } else {
        Serial.printf("[PIN %d] edge dropped (glitch, low at confirm)\n",
                      HOUSE_PINS[i]);
      }
    }
  }

  unsigned long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    if (mqtt.connected()) publishHeartbeat();
  }
}
