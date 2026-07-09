// ============================================================
// MANIFEST.h — WatchTower Device Manifest
// This file is parsed by sync_manifests.py for the WatchTower dashboard,
// AND #included by SunDial_Bridge.ino so the values below are the single
// source of truth for the firmware. Keep all values as #define strings
// unless noted otherwise.
// ============================================================

#define DEVICE_NAME           "SunDial"
#define FIRMWARE_VERSION      "4.2.0"
#define BOARD_TYPE            "ESP32-S3"
#define ROOM                  "MermaidsTale"
#define DESCRIPTION           "Stateless MQTT bridge for the SunDial puzzle: captures 5 pulse inputs from the Sand Dial Arduino via ISRs and publishes per-symbol true events; M3 owns dedupe, counting, SOLVED, and the totem trigger"

#define BUILD_STATUS          "stable"
#define CODE_HEALTH           "good"
#define WATCHTOWER_COMPLIANCE "full"

// MQTT
#define BROKER_IP             "10.1.10.115"
#define BROKER_PORT           1883
#define HEARTBEAT_MS          300000

#define SUBSCRIBE_TOPICS      "MermaidsTale/SunDial/command"
#define PUBLISH_TOPICS        "MermaidsTale/SunDial/status, MermaidsTale/SunDial/log, MermaidsTale/SunDial/Bottle, MermaidsTale/SunDial/Crab, MermaidsTale/SunDial/Turtle, MermaidsTale/SunDial/Coconut, MermaidsTale/SunDial/Trident"
#define SUPPORTED_COMMANDS    "PING, STATUS, RESET, PUZZLE_RESET, CLEAR_STATUS"

// Hardware
#define PIN_CONFIG            "HOUSE_1/Bottle=4, HOUSE_2/Crab=5, HOUSE_3/Turtle=6, HOUSE_4/Coconut=7, HOUSE_5/Trident=15"
#define COMPONENTS            "Level shifter on Sand Dial Arduino HOUSE_1..5 outputs (2s HIGH pulse per correct symbol)"
#define KNOWN_QUIRKS          "Power-on electrical noise fires phantom rising edges on symbol inputs (2026-07-09: 3-5 pins within 100ms of boot); FW 4.2.0 discards edges in the first 3s and requires the pin still HIGH 30ms after an edge. Symbol topics are deliberately NOT retained (retained true replayed into M3 on reconnect and re-fired Correct/Solved events); boot/RESET/PUZZLE_RESET wipe retained residue. PONG is answered on /command. Serial prints go to UART0, not native USB CDC, so a USB serial monitor shows nothing."

#define REPO_URL              "https://github.com/Alchemy-Escape-Rooms-Inc/Sun-Dial"
