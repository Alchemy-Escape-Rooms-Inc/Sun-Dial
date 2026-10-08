// ============================================================
// MANIFEST.h — WatchTower Device Manifest (SunDialSky)
// Parsed by sync_manifests.py AND #included by SunDialSky.ino.
// ============================================================
#pragma once

#define DEVICE_NAME           "SunDialSky"
#define FIRMWARE_VERSION      "2.0.0"
#define BOARD_TYPE            "ESP32-S3"
#define ROOM                  "MermaidsTale"
#define DESCRIPTION           "Drives the sky-writing video beside the SunDial (MedeaWiz Sprite DV-S1 showing Red Beard's ship in the distance). Listens to the retained MermaidsTale/SunDial/Clue from the SunDial bridge and tells the player which file to loop: 000 = plain sky (idle), 001..005 = the clue for question 1..5 written in the clouds, 006 = solved; 011..016 = play-once transition clips (old words fade out, new fade in) sent right after the new hold file so the hand-over is seamless. Serial 0xFC select-loop-file, so a clue HOLDS on screen until the next one."

#define BUILD_STATUS          "new"
#define CODE_HEALTH           "good"
#define WATCHTOWER_COMPLIANCE "full"

#define BROKER_IP             "10.1.10.115"
#define BROKER_PORT           1883
#define HEARTBEAT_MS          5000

#define SUBSCRIBE_TOPICS      "MermaidsTale/SunDialSky/command, MermaidsTale/SunDial/Clue"
#define PUBLISH_TOPICS        "MermaidsTale/SunDialSky/status, MermaidsTale/SunDialSky/log"
#define HEARTBEAT_FORMAT      "HEARTBEAT:{state}:UP{uptime}s:RSSI{signal}"
#define STATUS_FORMAT         "{state}|v{ver}|UP{uptime}s|RSSI{signal}|IP:{ip}|FILE{n}|CLUE{clue}|OTA:{port}"
#define SUPPORTED_COMMANDS    "PING, STATUS, RESET, PUZZLE_RESET, FILE <n>"

// Over-the-air updates (MANDATORY per mqtt-protocol.md, 2026-09-22).
// Password = the Wi-Fi password (OTA_PASSWORD aliases WIFI_PASS in the sketch).
#define OTA_ENABLED           "yes"
#define OTA_HOSTNAME          "SunDialSky"          // = DEVICE_NAME
#define OTA_PORT              3232

#define PIN_CONFIG            "GPIO4 (Serial1 TX, 9600) -> Sprite I/O plug serial RX, GND -> Sprite GND. GPIO5 = Serial1 RX (unused). USB CDC = debug."
#define COMPONENTS            "ESP32-S3 + MedeaWiz Sprite DV-S1 set to Control Mode = Serial Control, 9600 baud, firmware >= 20180704 (0xFC select-loop-file). SD card: 000 idle sky, 001-005 clue clouds (loop), 006 solved sky (loop), 011-016 transitions (play once). Recipe in Docs/SkyVideos.md."
#define KNOWN_QUIRKS          "The Sprite MUST be in Serial Control mode or 0xFC is ignored (a player that will not save that setting falls back to trigger mode where files only play once - see StarTableSprite). Clue topic is retained so a reboot re-shows the current clue. Listens only; never publishes on SunDial/* topics."

#define REPO_URL              "https://github.com/Alchemy-Escape-Rooms-Inc/Sun-Dial"
