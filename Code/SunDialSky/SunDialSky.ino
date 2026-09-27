//  SunDialSky  -  ESP32-S3 driver for the sky-writing video beside the SunDial
//  v1.0.0 (2026-09-27)
//
//  The screen next to the sundial shows Red Beard's ship in the distance. Each
//  sundial question gets its own video file with the clue written in the
//  clouds. The SunDial bridge publishes the current question RETAINED on
//  MermaidsTale/SunDial/Clue ("0" idle, "1".."5", "solved"); this board maps it
//  to a file on the Sprite's card and tells the player to LOOP that file:
//
//      Clue payload    Sprite file
//      0 / anything    000   plain sky, ship in the distance (idle)
//      1..5            001..005   clue for question N in the clouds
//      solved          006   (SOLVED_FILE; set 0 to just go back to the sky)
//
//  Player: MedeaWiz Sprite DV-S1, Control Mode = Serial Control, 9600 baud,
//  firmware >= 20180704. Wire GPIO4 (Serial1 TX) -> Sprite I/O plug serial RX,
//  GND -> GND. 0xFC <file> = "select loop file": the file loops until told
//  otherwise, so a clue stays up for as long as the question is open.
//
//  Own topics MermaidsTale/SunDialSky/{command,status,log}; WatchTower protocol
//  (PING/PONG, STATUS, RESET, PUZZLE_RESET, 5 s heartbeat, LWT OFFLINE) and
//  ArduinoOTA (mandatory): hostname SunDialSky, port 3232, Wi-Fi password.
//
//  Build:  arduino-cli compile --fqbn esp32:esp32:esp32s3 --output-dir build Code/SunDialSky
//  OTA:    arduino-cli upload --fqbn esp32:esp32:esp32s3 -p <board IP> --protocol network
//              --upload-field password=<Wi-Fi password> Code/SunDialSky

#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <stdarg.h>
#include "MANIFEST.h"
#include "secrets.h"            // WIFI_SSID / WIFI_PASS (same as every other board)

#define VERSION    FIRMWARE_VERSION
#define PROP_NAME  DEVICE_NAME

#define TOPIC_COMMAND  "MermaidsTale/SunDialSky/command"
#define TOPIC_STATUS   "MermaidsTale/SunDialSky/status"
#define TOPIC_LOG      "MermaidsTale/SunDialSky/log"
#define TOPIC_CLUE     "MermaidsTale/SunDial/Clue"      // retained, from the SunDial bridge 4.6.0

#define SPRITE_TX_PIN  4          // Serial1 TX -> Sprite serial RX
#define SPRITE_RX_PIN  5          // Serial1 RX (unused, keeps the UART happy)
#define SPRITE_BAUD    9600
#define LAST_CLUE      5          // files 001..005
#define SOLVED_FILE    6          // file shown after the fifth solve (0 = back to the idle sky)
#define MQTT_RETRY_MS  5000UL

static const char* OTA_PASSWORD = WIFI_PASS;   // protocol: OTA password = Wi-Fi password

WiFiClient   espClient;
PubSubClient mqtt(espClient);
Preferences  prefs;

uint8_t       currentFile = 0;      // what the player is looping right now
char          currentClue[12] = "0";
unsigned long lastHeartbeatMs = 0, lastMqttAttemptMs = 0, lastWifiKickMs = 0;
bool          otaReady = false;

void mqttLogf(const char* format, ...) {
  char buffer[256];
  va_list args; va_start(args, format); vsnprintf(buffer, sizeof(buffer), format, args); va_end(args);
  if (mqtt.connected()) mqtt.publish(TOPIC_LOG, buffer);
  Serial.println(buffer);
}

const char* stateStr() {
  if (currentFile == 0) return "IDLE";
  if (currentFile == SOLVED_FILE && SOLVED_FILE != 0) return "SOLVED";
  return "CLUE";
}

// ---------------------------------------------------------------- Sprite
void spriteLoopFile(uint8_t fileNum) {
  Serial1.write((uint8_t)0xFC);
  Serial1.write(fileNum);
  currentFile = fileNum;
  prefs.putUChar("file", currentFile);
}
void showClue(const char* clue) {
  strncpy(currentClue, clue, sizeof(currentClue) - 1);
  uint8_t file = 0;
  if (strcmp(clue, "solved") == 0) file = SOLVED_FILE;
  else {
    int n = atoi(clue);
    if (n >= 1 && n <= LAST_CLUE) file = (uint8_t)n;
  }
  spriteLoopFile(file);
  mqttLogf("%s: clue '%s' -> Sprite file %03d", PROP_NAME, clue, file);
}

// ---------------------------------------------------------------- Wi-Fi / OTA / MQTT
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) { delay(100); Serial.print("-"); }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nWiFi up" : "\nWiFi not up yet - retrying in loop");
}
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiKickMs < 10000UL) return;
  lastWifiKickMs = millis();
  Serial.println("[WIFI] link down, reconnecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}
void setupOTA() {                          // once, after Wi-Fi is up
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { mqttLogf("OTA update starting - sky holds, back in ~30 s"); });
  ArduinoOTA.onEnd([]()   { Serial.println("OTA done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", (unsigned)e); });
  ArduinoOTA.begin();
  otaReady = true;
  Serial.printf("OTA ready: %s @ %s:%d\n", OTA_HOSTNAME, WiFi.localIP().toString().c_str(), OTA_PORT);
}
void publishHeartbeat() {
  char hb[96];
  snprintf(hb, sizeof(hb), "HEARTBEAT:%s:UP%lus:RSSI%d", stateStr(), millis() / 1000UL, WiFi.RSSI());
  mqtt.publish(TOPIC_STATUS, hb);
}
void publishStatusReply() {
  char reply[192];
  snprintf(reply, sizeof(reply), "%s|v%s|UP%lus|RSSI%d|IP:%s|FILE%03d|CLUE%s|OTA:%d",
           stateStr(), VERSION, millis() / 1000UL, WiFi.RSSI(),
           WiFi.localIP().toString().c_str(), currentFile, currentClue, OTA_PORT);
  mqtt.publish(TOPIC_COMMAND, reply);
  Serial.printf("[MQTT] STATUS -> %s\n", reply);
}
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char message[64];
  if (length >= sizeof(message)) length = sizeof(message) - 1;
  memcpy(message, payload, length); message[length] = '\0';
  char* msg = message;
  while (*msg == ' ' || *msg == '\r' || *msg == '\n') msg++;
  char* end = msg + strlen(msg) - 1;
  while (end > msg && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) { *end = '\0'; end--; }
  if (*msg == '\0') return;
  Serial.printf("[MQTT] %s: %s\n", topic, msg);

  if (strcmp(topic, TOPIC_CLUE) == 0) { showClue(msg); return; }
  if (strcmp(topic, TOPIC_COMMAND) != 0) return;

  if (strcmp(msg, "PING") == 0)        { mqtt.publish(TOPIC_COMMAND, "PONG"); }
  else if (strcmp(msg, "STATUS") == 0) { publishStatusReply(); }
  else if (strcmp(msg, "RESET") == 0)  { mqtt.publish(TOPIC_COMMAND, "OK"); delay(100); ESP.restart(); }
  else if (strcmp(msg, "PUZZLE_RESET") == 0) { showClue("0"); mqtt.publish(TOPIC_COMMAND, "OK"); }
  else if (strncmp(msg, "FILE ", 5) == 0) {                 // bench: FILE 3 -> loop file 003
    int n = atoi(msg + 5);
    if (n >= 0 && n <= 255) { spriteLoopFile((uint8_t)n); mqttLogf("%s: FILE -> %03d (bench)", PROP_NAME, n); mqtt.publish(TOPIC_COMMAND, "OK"); }
  }
}
void connectMQTT() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastMqttAttemptMs < MQTT_RETRY_MS) return;
  lastMqttAttemptMs = millis();
  String clientId = String(PROP_NAME) + "-" + String(random(0xffff), HEX);
  if (mqtt.connect(clientId.c_str(), NULL, NULL, TOPIC_STATUS, 1, true, "OFFLINE")) {
    mqtt.subscribe(TOPIC_COMMAND);
    mqtt.subscribe(TOPIC_CLUE);                 // retained: the current clue arrives right away
    mqtt.publish(TOPIC_STATUS, "ONLINE", true);
    mqttLogf("%s v%s online IP:%s OTA:%d file=%03d", PROP_NAME, VERSION,
             WiFi.localIP().toString().c_str(), OTA_PORT, currentFile);
  } else {
    Serial.printf("MQTT connect failed rc=%d\n", mqtt.state());
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial1.begin(SPRITE_BAUD, SERIAL_8N1, SPRITE_RX_PIN, SPRITE_TX_PIN);
  prefs.begin("sky", false);
  currentFile = prefs.getUChar("file", 0);
  Serial.printf("%s v%s boot, last file %03d\n", PROP_NAME, VERSION, currentFile);
  setupWiFi();
  if (WiFi.status() == WL_CONNECTED) setupOTA();
  mqtt.setServer(BROKER_IP, BROKER_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(3);
  mqtt.setKeepAlive(30);
  delay(500);
  spriteLoopFile(currentFile);               // re-assert what the player should be looping
}

void loop() {
  ensureWiFi();
  if (!otaReady && WiFi.status() == WL_CONNECTED) setupOTA();
  if (otaReady) ArduinoOTA.handle();
  connectMQTT();
  mqtt.loop();
  if (mqtt.connected() && millis() - lastHeartbeatMs >= HEARTBEAT_MS) { lastHeartbeatMs = millis(); publishHeartbeat(); }
}
