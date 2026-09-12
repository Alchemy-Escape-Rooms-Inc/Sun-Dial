// SunDial_Probe.ino  (2026-09-12, v2 = bezel readout)
// Hardware probe for the SunDial CONTROLLER Nano socket. Flash this onto the
// controller Nano, put it in the prop, power up. Motors are held STOPPED, so
// the rings only move while a rotate button is held.
//
// The FIRST FOUR bezel LEDs show the four sensor pins, live:
//   bezel LED 1 (first board, LED 0)  RED    = pin A0 sees a tab
//   bezel LED 2 (first board, LED 1)  GREEN  = pin A1 sees a tab
//   bezel LED 3 (first board, LED 2)  BLUE   = pin A2 sees a tab
//   bezel LED 4 (first board, LED 3)  WHITE  = pin A3 sees a tab
// Hold the INNER button and watch which colours blink, and whether they
// blink on every tab or once per lap. Then the OUTER button. That gives the
// true wiring: "every tab" = counter sensor, "once per lap" = zero mark.
//
// Nano onboard LED (D13): 3 blinks at boot, then ON while any sensor is HIGH.
// With USB attached it also prints the four pin states 5x per second.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define STEPPER_O_ENABLE 11
#define STEPPER_I_ENABLE 10
#define CHOOSE           12
#define OUTER_CONTROL     9
#define INNER_CONTROL     8
#define LED              13

const int SENSOR_PINS[4] = {A0, A1, A2, A3};
const int HOUSE_PINS[6] = {2, 3, 4, 5, 6, 7};

// channel order on these boards is whatever the main sketch calls "red"/"green"/...
const int COL_RED[3]   = {0, 0, 255};
const int COL_GREEN[3] = {0, 255, 0};
const int COL_BLUE[3]  = {255, 0, 0};
const int COL_WHITE[3] = {255, 255, 255};
const int* const COLORS[4] = {COL_RED, COL_GREEN, COL_BLUE, COL_WHITE};

Adafruit_PWMServoDriver pwmBoard[] = { Adafruit_PWMServoDriver(0x40), Adafruit_PWMServoDriver(0x41) };

void setLed(int board, int led, const int* c, bool on) {
  for (int k = 0; k < 3; k++) pwmBoard[board].setPWM(k + led * 3, 0, on ? c[k] * 16 : 0);
}

void setup() {
  for (int i = 0; i < 4; i++) pinMode(SENSOR_PINS[i], INPUT);
  pinMode(CHOOSE, INPUT_PULLUP);
  pinMode(OUTER_CONTROL, OUTPUT);  digitalWrite(OUTER_CONTROL, LOW);   // LOW = stop
  pinMode(INNER_CONTROL, OUTPUT);  digitalWrite(INNER_CONTROL, LOW);
  pinMode(STEPPER_O_ENABLE, OUTPUT); digitalWrite(STEPPER_O_ENABLE, LOW);  // LOW = driver enabled
  pinMode(STEPPER_I_ENABLE, OUTPUT); digitalWrite(STEPPER_I_ENABLE, LOW);
  for (int i = 0; i < 6; i++) { pinMode(HOUSE_PINS[i], OUTPUT); digitalWrite(HOUSE_PINS[i], LOW); }
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  Serial.println("SunDial PROBE v2 boot");
  for (int i = 0; i < 3; i++) { digitalWrite(LED, HIGH); delay(120); digitalWrite(LED, LOW); delay(120); }

  for (int b = 0; b < 2; b++) {
    pwmBoard[b].begin();
    pwmBoard[b].setOscillatorFrequency(27000000);
    pwmBoard[b].setPWMFreq(50);
    for (int ch = 0; ch < 16; ch++) pwmBoard[b].setPWM(ch, 0, 0);
  }
  // wink each readout LED once in its colour so you know which is which
  for (int i = 0; i < 4; i++) { setLed(0, i, COLORS[i], true); delay(400); setLed(0, i, COLORS[i], false); }
}

int lastState[4] = {-1, -1, -1, -1};
unsigned long lastPrint = 0;

void loop() {
  int v[4]; bool any = false;
  for (int i = 0; i < 4; i++) { v[i] = digitalRead(SENSOR_PINS[i]); if (v[i]) any = true; }
  digitalWrite(LED, any ? HIGH : LOW);
  digitalWrite(OUTER_CONTROL, LOW);
  digitalWrite(INNER_CONTROL, LOW);

  for (int i = 0; i < 4; i++) {
    if (v[i] != lastState[i]) { setLed(0, i, COLORS[i], v[i] == HIGH); lastState[i] = v[i]; }
  }

  unsigned long now = millis();
  if (now - lastPrint >= 200) {
    lastPrint = now;
    Serial.print("A0="); Serial.print(v[0]);
    Serial.print(" A1="); Serial.print(v[1]);
    Serial.print(" A2="); Serial.print(v[2]);
    Serial.print(" A3="); Serial.print(v[3]);
    Serial.print(" select="); Serial.println(digitalRead(CHOOSE) == LOW ? "PRESSED" : "up");
  }
}
