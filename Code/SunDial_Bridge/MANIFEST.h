// ============================================================
// MANIFEST.h — WatchTower Device Manifest
// This file is parsed by sync_manifests.py for the WatchTower dashboard,
// AND #included by SunDial_Bridge.ino so the values below are the single
// source of truth for the firmware. Keep all values as #define strings
// unless noted otherwise.
// ============================================================

#define DEVICE_NAME           "SunDial"
#define FIRMWARE_VERSION      "4.5.0"
#define BOARD_TYPE            "ESP32-S3"
#define ROOM                  "MermaidsTale"
#define DESCRIPTION           "Stateless MQTT bridge for the SunDial puzzle: captures 6 pulse inputs from the Sand Dial Arduino via ISRs and publishes per-symbol true events plus a Wrong event on incorrect guesses; M3 owns dedupe, counting, SOLVED, and the totem trigger. 4.5.0: also pulses the controller Nano on GameStart/PUZZLE_RESET (guided mode restart) and mirrors the Nano serial (ring counts) to MQTT"

#define BUILD_STATUS          "stable"
#define CODE_HEALTH           "good"
#define WATCHTOWER_COMPLIANCE "full"

// MQTT
#define BROKER_IP             "10.1.10.115"
#define BROKER_PORT           1883
#define HEARTBEAT_MS          5000

#define SUBSCRIBE_TOPICS      "MermaidsTale/SunDial/command, MermaidsTale/GameStart"
#define PUBLISH_TOPICS        "MermaidsTale/SunDial/status, MermaidsTale/SunDial/log, MermaidsTale/SunDial/Bottle, MermaidsTale/SunDial/Crab, MermaidsTale/SunDial/Turtle, MermaidsTale/SunDial/Coconut, MermaidsTale/SunDial/Trident, MermaidsTale/SunDial/Wrong, MermaidsTale/SunDial/Outer, MermaidsTale/SunDial/Inner, MermaidsTale/SunDial/nano"
#define SUPPORTED_COMMANDS    "PING, STATUS, RESET, PUZZLE_RESET, CLEAR_STATUS"

// Hardware
#define PIN_CONFIG            "HOUSE_1/Bottle=4, HOUSE_2/Crab=5, HOUSE_3/Turtle=6, HOUSE_4/Coconut=7, HOUSE_5/Trident=15, HOUSE_6/Wrong=16, TRIGGER_OUT->NanoA6=17, NANO_SERIAL_RX<-NanoD1=18"
#define COMPONENTS            "Level shifter on Sand Dial Arduino HOUSE_1..6 outputs (2s HIGH pulse per correct symbol; HOUSE_6 = 3s pulse per WRONG guess — needs the HOUSE_6/D7 line wired through the shifter to GPIO 16). 4.5.0 guided mode: GPIO 17 -> Nano A6 (10k pulldown to GND at the Nano), Nano D1 TX -> 2k2/3k3 divider -> GPIO 18"
#define KNOWN_QUIRKS          "Power-on electrical noise fires phantom rising edges on symbol inputs (2026-07-09: 3-5 pins within 100ms of boot); FW 4.2.0 discards edges in the first 3s and requires the pin still HIGH 30ms after an edge. Board hung silently ~2-5min after MQTT connect 3x on 2026-07-15/16 (root cause unknown, suspect Arduino-side electrical noise); FW 4.3.0 adds 30s task watchdog, 2min offline self-reboot, LWT (retained OFFLINE on /status), and 5s heartbeat so a hang self-recovers and is visible. Symbol topics are deliberately NOT retained (retained true replayed into M3 on reconnect and re-fired Correct/Solved events); boot/RESET/PUZZLE_RESET wipe retained residue. PONG is answered on /command. Serial prints go to UART0, not native USB CDC, so a USB serial monitor shows nothing. FLASH VIA NATIVE USB CONNECTOR ONLY (enumerates as Espressif VID_303A, no BOOT-hold needed): the CP210x/UART0 connector has corrupt data lines (2026-07-16, every esptool sync failed with garbage; DTR/RTS reset still works through it)."

#define REPO_URL              "https://github.com/Alchemy-Escape-Rooms-Inc/Sun-Dial"
