//  SunDialSky  -  ESP32-S3 driver for the sky-writing video beside the SunDial
//  v2.0.0 (2026-10-08): drives the Fright Ideas "Sprite Input Adapter" (PN 437)
//  instead of raw serial - that adapter is the only Sprite accessory on site.
//
//  The screen next to the sundial shows Red Beard's ship in the distance. Each
//  sundial question has its own video file with the clue written in the
//  clouds. The SunDial bridge publishes the current question RETAINED on
//  MermaidsTale/SunDial/Clue ("0" idle, "1".."5", "solved"):
//
//      Clue payload    adapter input pressed    Sprite file
//      0 / anything    none                     000 (the player's own idle loop)
//      1..5            1..5                     001..005 (words fade in and out, 16 s)
//      solved          6 (SOLVED_FILE)          006
//
//  HOW THE ADAPTER WORKS (maker's quick-start, PN 437): input N plays file 00N
//  ONCE, then the player falls back to looping 000. Its eight inputs are
//  OPTICALLY ISOLATED and need 9-24 V across C and the input - a bare 3.3 V
//  pin cannot fire one. So each input goes through a switch this board drives:
//
//      9-24 V supply (+) ---------------> adapter C   (both C terminals)
//      adapter input N -----------------> relay N "NO" (or ULN2803 output N)
//      relay N "COM" (or ULN2803 GND) --> supply (-)
//      TRIGGER_PINS[N-1] ---------------> relay module IN N (or ULN2803 input N)
//      board GND -----------------------> relay module / ULN2803 GND
//
//  ("Common + or -": C may be on either supply pole, the inputs on the other.)
//  Relay modules with blue relays are usually ACTIVE LOW (TRIGGER_ACTIVE_LOW 1);
//  a ULN2803 or MOSFET board is active high (0).
//
//  A clue has to stay up for the whole question, so while a clue is open its
//  input is pressed again every CLIP_MS (the clip length): the clip ends on the
//  same frame the idle file starts on, so the re-press is not seen. Going back
//  to idle needs no press - the current clip just finishes.
//
//  Player: MedeaWiz Sprite, Control Mode = Serial Control, 9600 baud (the
//  adapter talks serial to it through the 4-pole plug). Files 000..006 in the
//  card's root.
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

const uint8_t TRIGGER_PINS[] = { 4, 5, 6, 7, 15, 16 };   // adapter inputs 1..6, in order
#define NUM_TRIGGERS       (sizeof(TRIGGER_PINS) / sizeof(TRIGGER_PINS[0]))
#define TRIGGER_ACTIVE_LOW 1          // 1 = relay module that switches on a LOW pin, 0 = ULN2803 / MOSFET (HIGH = on)
#define TRIGGER_PULSE_MS   300UL      // how long an input is held "pressed"
#define CLIP_MS            16300UL    // clue clips are 16.0 s: press again this often while a clue is open
#define LAST_CLUE      5          // files 001..005
#define SOLVED_FILE    6          // file shown after the fifth solve (0 = back to the idle sky)
#define MQTT_RETRY_MS  5000UL

static const char* OTA_PASSWORD = WIFI_PASS;   // protocol: OTA password = Wi-Fi password

WiFiClient   espClient;
PubSubClient mqtt(espClient);
Preferences  prefs;

uint8_t       currentFile = 0;      // the clue file being held on screen (0 = idle sky)
unsigned long lastPressMs = 0, pressStartMs = 0;
int           pressedInput = 0;     // adapter input held right now (0 = none)
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

// ---------------------------------------------------------------- Sprite (through the input adapter)
void triggerWrite(int input, bool on) {
  digitalWrite(TRIGGER_PINS[input - 1], (on != (bool)TRIGGER_ACTIVE_LOW) ? HIGH : LOW);
}
void releaseAll() {
  for (unsigned i = 1; i <= NUM_TRIGGERS; i++) triggerWrite(i, false);
  pressedInput = 0;
}
void pressInput(int input) {                     // non-blocking: serviceTriggers() lets go after TRIGGER_PULSE_MS
  if (input < 1 || input > (int)NUM_TRIGGERS) return;
  releaseAll();
  triggerWrite(input, true);
  pressedInput = input;
  pressStartMs = lastPressMs = millis();
}
void serviceTriggers() {
  unsigned long now = millis();
  if (pressedInput && now - pressStartMs >= TRIGGER_PULSE_MS) releaseAll();
  if (currentFile && !pressedInput && now - lastPressMs >= CLIP_MS) pressInput(currentFile);   // keep the clue on screen
}
void spriteShow(uint8_t file) {
  currentFile = file;
  prefs.putUChar("file", currentFile);
  if (file) pressInput(file);                    // idle (0): nothing to press, the running clip ends and 000 loops
}
void showClue(const char* clue) {
  strncpy(currentClue, clue, sizeof(currentClue) - 1);
  uint8_t file = 0;
  if (strcmp(clue, "solved") == 0) file = SOLVED_FILE;
  else {
    int n = atoi(clue);
    if (n >= 1 && n <= LAST_CLUE) file = (uint8_t)n;
  }
  bool changed = (file != currentFile);
  if (changed) spriteShow(file);                 // a retained replay of the same clue must not restart the clip
  mqttLogf("%s: clue '%s' -> file %03d%s", PROP_NAME, clue, file, changed ? "" : " (unchanged)");
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
  ArduinoOTA.onStart([]() { releaseAll(); mqttLogf("OTA update starting - sky idle, back in ~30 s"); });
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
  else if (strncmp(msg, "FILE ", 5) == 0) {                 // bench: FILE 3 -> hold file 003, FILE 0 -> idle
    int n = atoi(msg + 5);
    if (n >= 0 && n <= (int)NUM_TRIGGERS) { spriteShow((uint8_t)n); mqttLogf("%s: FILE -> %03d (bench)", PROP_NAME, n); mqtt.publish(TOPIC_COMMAND, "OK"); }
  }
  else if (strncmp(msg, "PRESS ", 6) == 0) {                // bench: PRESS 3 -> one press of adapter input 3, no hold
    int n = atoi(msg + 6);
    if (n >= 1 && n <= (int)NUM_TRIGGERS) { pressInput(n); mqttLogf("%s: PRESS input %d (bench)", PROP_NAME, n); mqtt.publish(TOPIC_COMMAND, "OK"); }
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
  for (unsigned i = 1; i <= NUM_TRIGGERS; i++) { triggerWrite(i, false); pinMode(TRIGGER_PINS[i - 1], OUTPUT); triggerWrite(i, false); }   // released before anything else
  Serial.begin(115200);
  delay(1500);
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
  if (currentFile) pressInput(currentFile);  // put back the clue that was up before the restart
}

void loop() {
  ensureWiFi();
  if (!otaReady && WiFi.status() == WL_CONNECTED) setupOTA();
  if (otaReady) ArduinoOTA.handle();
  connectMQTT();
  mqtt.loop();
  serviceTriggers();
  if (mqtt.connected() && millis() - lastHeartbeatMs >= HEARTBEAT_MS) { lastHeartbeatMs = millis(); publishHeartbeat(); }
}
