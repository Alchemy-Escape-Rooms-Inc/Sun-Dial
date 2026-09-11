// ============================================================
//  SunDial_Bridge.ino
//  ESP32-S3
//
//  Logic:
//    - Watch 6 input pins from the SunDial Arduino. HOUSE_1..5 are
//      each pulsed HIGH for ~2s when their symbol is correctly
//      chosen; HOUSE_6 is pulsed HIGH for ~3s on a WRONG guess
//      (the controller's red-bezel branch drives it).
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
//    Arduino HOUSE_1..6 -> level shifter -> ESP32 GPIO 4,5,6,7,15,16
//    Arduino GND        -> ESP32 GND
//
//  Symbol mapping (HOUSE_n -> ESP32 GPIO -> Symbol):
//    HOUSE_1 -> GPIO 4  -> Bottle
//    HOUSE_2 -> GPIO 5  -> Crab
//    HOUSE_3 -> GPIO 6  -> Turtle
//    HOUSE_4 -> GPIO 7  -> Coconut
//    HOUSE_5 -> GPIO 15 -> Trident
//    HOUSE_6 -> GPIO 16 -> Wrong   (wrong-guess pulse, 4.4.0)
//
//  MQTT topics (publish, not retained):
//    MermaidsTale/SunDial/Bottle    "true" | "false"
//    MermaidsTale/SunDial/Crab      "true" | "false"
//    MermaidsTale/SunDial/Turtle    "true" | "false"
//    MermaidsTale/SunDial/Coconut   "true" | "false"
//    MermaidsTale/SunDial/Trident   "true" | "false"
//    MermaidsTale/SunDial/Wrong     "true" | "false"
//
//  MQTT topics (publish, not retained):
//    MermaidsTale/SunDial/status    ONLINE, HEARTBEAT, STATUS, OK
//    MermaidsTale/SunDial/log       mirrored serial output
//
//  MQTT topics (subscribe; PONG is answered back on this topic):
//    MermaidsTale/SunDial/command   PING | STATUS | RESET |
//                                    PUZZLE_RESET | CLEAR_STATUS
//
//  4.3.0 — hang recovery (board hung silently ~2-5min after connect
//  3x on 2026-07-15/16; root cause unknown, puzzle logic unchanged):
//    - 30s task watchdog reboots the chip if loop() ever stalls.
//    - 2min offline self-reboot catches wedges where loop() still
//      runs but WiFi/MQTT never recovers.
//    - LWT: broker publishes retained OFFLINE to /status when the
//      connection dies, so WatchTower sees the death without a PING.
//    - Heartbeat 5min -> 5s (MANIFEST.h) to match the fleet.
//
//  4.4.0 — wrong-guess reporting: the controller Arduino has always
//  pulsed HOUSE_6 HIGH (~3s, the red-bezel branch) on an incorrect
//  choose-button press, but the bridge never watched it. A 6th input
//  (GPIO 16) now publishes MermaidsTale/SunDial/Wrong "true" per
//  wrong guess so the AI character can tell players the combination
//  was incorrect. Same ISR + confirm-window path as the symbols;
//  reset/boot publish "false" on it like the rest. Requires wiring
//  Arduino HOUSE_6 (D7) through the level shifter to GPIO 16.
//
//  4.5.0 — guided-mode support (controller Nano v2, 2026-09-11):
//    - TRIGGER OUT: GPIO 17 -> Nano A6 (10k pulldown on the Nano side).
//      A 500 ms HIGH pulse tells the Nano "start over at step 1". Fired on
//      MermaidsTale/GameStart (any payload) and on PUZZLE_RESET. GameStart
//      arriving within 3 s of (re)subscribing is ignored: that is a retained
//      replay, not a real start, and must not reset a live puzzle.
//    - NANO SERIAL IN: Nano D1 (TX, 5V) -> divider (2k2 top / 3k3 bottom)
//      -> GPIO 18 (Serial1 RX, 115200). The Nano's own prints are parsed:
//        "outer counter = N" -> MermaidsTale/SunDial/Outer  N
//        "inner counter = N" -> MermaidsTale/SunDial/Inner  N
//      every other line is mirrored to MermaidsTale/SunDial/nano (deduped,
//      max ~20/s) so the dial can be watched without a USB cable.
// ============================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include "MANIFEST.h"  // single source of truth for device identity/broker/heartbeat

#define FW_VERSION FIRMWARE_VERSION

const char* WIFI_SSID = "AlchemyGuest";
const char* WIFI_PASS = "VoodooVacation5601";

const char*    MQTT_HOST   = BROKER_IP;
const uint16_t MQTT_PORT   = BROKER_PORT;
const char*    TOPIC_CMD   = "MermaidsTale/SunDial/command";
const char*    TOPIC_STAT  = "MermaidsTale/SunDial/status";
const char*    TOPIC_LOG   = "MermaidsTale/SunDial/log";
const char*    TOPIC_GAMESTART = "MermaidsTale/GameStart";     // 4.5.0: pulse the Nano
const char*    TOPIC_OUTER = "MermaidsTale/SunDial/Outer";     // 4.5.0: outer ring position
const char*    TOPIC_INNER = "MermaidsTale/SunDial/Inner";     // 4.5.0: inner ring position
const char*    TOPIC_NANO  = "MermaidsTale/SunDial/nano";      // 4.5.0: mirrored Nano serial

// 4.5.0 Nano link
const int      TRIGGER_OUT_PIN   = 17;    // -> Nano A6
const int      NANO_RX_PIN       = 18;    // <- Nano D1 via divider
const unsigned long TRIGGER_PULSE_MS     = 500;
const unsigned long GAMESTART_GUARD_MS   = 3000;  // ignore GameStart this soon after subscribing (retained replay)
const unsigned long NANO_MIRROR_MIN_GAP_MS = 50;  // ~20 lines/s max on /nano

const int NUM_PINS = 6;
const int HOUSE_PINS[NUM_PINS] = { 4, 5, 6, 7, 15, 16 };
const char* const SYMBOL_TOPICS[NUM_PINS] = {
  "MermaidsTale/SunDial/Bottle",
  "MermaidsTale/SunDial/Crab",
  "MermaidsTale/SunDial/Turtle",
  "MermaidsTale/SunDial/Coconut",
  "MermaidsTale/SunDial/Trident",
  "MermaidsTale/SunDial/Wrong"   // HOUSE_6: wrong-guess pulse (4.4.0)
};

const unsigned long HEARTBEAT_INTERVAL_MS = HEARTBEAT_MS;  // 5 seconds, from MANIFEST.h

// Hang recovery (4.3.0). The watchdog timeout must exceed the worst
// blocking path in loop(): ensureWiFi's 10s wait (fed inside) plus a
// blocking mqtt.connect() attempt.
const uint32_t      WDT_TIMEOUT_S     = 30;
const unsigned long MQTT_RETRY_MS     = 3000;    // min gap between connect attempts
const unsigned long OFFLINE_REBOOT_MS = 120000;  // no broker for 2min -> restart

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
volatile bool pendingEdge[NUM_PINS] = { false, false, false, false, false, false };

// millis() when a drained edge started its confirm window; 0 = none.
unsigned long edgeSeenMs[NUM_PINS] = { 0, 0, 0, 0, 0, 0 };

unsigned long bootMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastMqttOkMs = 0;      // last time the broker connection was up
unsigned long lastMqttAttemptMs = 0; // last connect attempt (retry backoff)
unsigned long triggerHighMs = 0;      // 0 = trigger line idle
unsigned long subscribedMs = 0;       // when we last (re)subscribed; GameStart guard
char  nanoLine[96];
int   nanoLineLen = 0;
char  nanoLastMirror[96] = {0};
unsigned long nanoLastMirrorMs = 0;
int   lastOuter = -1;
int   lastInner = -1;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// One ISR per pin. Kept tiny — no Serial, no MQTT, no delay.
void IRAM_ATTR isrPin0() { pendingEdge[0] = true; }
void IRAM_ATTR isrPin1() { pendingEdge[1] = true; }
void IRAM_ATTR isrPin2() { pendingEdge[2] = true; }
void IRAM_ATTR isrPin3() { pendingEdge[3] = true; }
void IRAM_ATTR isrPin4() { pendingEdge[4] = true; }
void IRAM_ATTR isrPin5() { pendingEdge[5] = true; }
void (*const ISRS[NUM_PINS])() = { isrPin0, isrPin1, isrPin2, isrPin3, isrPin4, isrPin5 };

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
           "STATUS:RUNNING:UP%lus:RSSI%d:VER%s:OUTER%d:INNER%d",
           uptime, WiFi.RSSI(), FW_VERSION, lastOuter, lastInner);
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

// 4.5.0: raise the trigger line; loop() drops it after TRIGGER_PULSE_MS.
void pulseNano(const char* why) {
  digitalWrite(TRIGGER_OUT_PIN, HIGH);
  triggerHighMs = millis();
  if (triggerHighMs == 0) triggerHighMs = 1;
  char buf[64];
  snprintf(buf, sizeof(buf), "[NANO] trigger pulse (%s)", why);
  logLine(buf);
}

void serviceTriggerPulse() {
  if (triggerHighMs != 0 && millis() - triggerHighMs >= TRIGGER_PULSE_MS) {
    digitalWrite(TRIGGER_OUT_PIN, LOW);
    triggerHighMs = 0;
  }
}

// 4.5.0: one complete line from the Nano.
void handleNanoLine(char* line) {
  // trim leading spaces (the Nano prints " reset wheels")
  while (*line == ' ') line++;
  if (*line == 0) return;
  int n;
  if (sscanf(line, "outer counter = %d", &n) == 1) {
    lastOuter = n;
    char v[8]; snprintf(v, sizeof(v), "%d", n);
    if (mqtt.connected()) mqtt.publish(TOPIC_OUTER, v, false);
    return;
  }
  if (sscanf(line, "inner counter = %d", &n) == 1) {
    lastInner = n;
    char v[8]; snprintf(v, sizeof(v), "%d", n);
    if (mqtt.connected()) mqtt.publish(TOPIC_INNER, v, false);
    return;
  }
  // The Nano prints a bare number + " off" per LED at boot; skip that chatter.
  if (strcmp(line, "off") == 0 || (line[0] >= '0' && line[0] <= '9' && line[1] == 0)) return;
  unsigned long now = millis();
  if (strcmp(line, nanoLastMirror) == 0 && now - nanoLastMirrorMs < 2000) return;  // dedupe repeats
  if (now - nanoLastMirrorMs < NANO_MIRROR_MIN_GAP_MS) return;                      // rate cap
  strncpy(nanoLastMirror, line, sizeof(nanoLastMirror) - 1);
  nanoLastMirrorMs = now;
  if (mqtt.connected()) mqtt.publish(TOPIC_NANO, line, false);
}

void pollNanoSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      nanoLine[nanoLineLen] = 0;
      handleNanoLine(nanoLine);
      nanoLineLen = 0;
    } else if (nanoLineLen < (int)sizeof(nanoLine) - 1) {
      nanoLine[nanoLineLen++] = c;
    } else {
      nanoLineLen = 0;  // overlong garbage: drop the line
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[32] = {0};
  memcpy(msg, payload, min((unsigned int)31, length));

  if (strcmp(topic, TOPIC_GAMESTART) == 0) {
    // 4.5.0: any GameStart payload restarts the dial - unless it is the
    // retained replay we get right after subscribing.
    if (length == 0) return;                                   // retained-wipe (empty) payload
    if (millis() - subscribedMs < GAMESTART_GUARD_MS) {
      logLine("[NANO] GameStart ignored (arrived right after subscribe = retained replay)");
      return;
    }
    pulseNano("GameStart");
    return;
  }

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
    pulseNano("PUZZLE_RESET");
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
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    esp_task_wdt_reset();
    delay(200);
  }
}

void ensureMqtt() {
  if (mqtt.connected()) return;
  if (millis() - lastMqttAttemptMs < MQTT_RETRY_MS) return;
  lastMqttAttemptMs = millis();
  String clientId = String(DEVICE_NAME) + "-" + String(random(0xffff), HEX);
  // LWT: broker publishes retained OFFLINE to /status if this
  // connection dies (keepalive timeout ~22s after a silent hang).
  if (mqtt.connect(clientId.c_str(), TOPIC_STAT, 0, true, "OFFLINE")) {
    mqtt.subscribe(TOPIC_CMD);
    mqtt.subscribe(TOPIC_GAMESTART);           // 4.5.0
    subscribedMs = millis();
    mqtt.publish(TOPIC_STAT, "ONLINE", true);  // retained: overwrite stale OFFLINE
    Serial.println("[MQTT] ONLINE published");
  }
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();

  // Task watchdog on the loop task: if loop() ever stalls (blocked
  // call, interrupt storm), the chip panics and reboots itself.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = {};
  wdtCfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  wdtCfg.idle_core_mask = 0;
  wdtCfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&wdtCfg);  // core 3.x inits the WDT itself
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
  pinMode(TRIGGER_OUT_PIN, OUTPUT);
  digitalWrite(TRIGGER_OUT_PIN, LOW);
  Serial1.begin(115200, SERIAL_8N1, NANO_RX_PIN, -1);  // 4.5.0: RX only from the Nano
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
  lastMqttOkMs = millis();
  Serial.printf("SunDial Bridge ready (FW %s)\n", FW_VERSION);
}

void loop() {
  esp_task_wdt_reset();
  ensureWiFi();
  ensureMqtt();
  mqtt.loop();
  serviceTriggerPulse();   // 4.5.0
  pollNanoSerial();        // 4.5.0

  // Offline self-reboot: loop() can be alive while the WiFi/MQTT
  // stack is wedged (the watchdog can't see that). If the broker has
  // been unreachable for OFFLINE_REBOOT_MS, restart and start clean.
  if (mqtt.connected()) {
    lastMqttOkMs = millis();
  } else if (millis() - lastMqttOkMs >= OFFLINE_REBOOT_MS) {
    Serial.println("[WDT] no broker for 2min — restarting");
    ESP.restart();
  }

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
