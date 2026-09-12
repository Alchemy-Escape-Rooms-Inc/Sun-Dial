// SunDial_Probe.ino  (2026-09-12)
// Hardware probe for the SunDial CONTROLLER Nano socket. Flash this onto the
// controller Nano, put it in the prop, power up, and read the prop itself:
//
//   Nano onboard LED (D13):  3 quick blinks at boot = the Nano is running.
//                            Then ON whenever ANY of the four optical sensors
//                            sees a tab (block a slot with paper to test).
//   Bezel LEDs:              go WHITE = 5V rail + I2C to the PWM boards OK.
//   Wheels:                  motors are held STOPPED (D8/D9 LOW, drivers
//                            enabled). If a wheel spins anyway, that stop
//                            line is not reaching the motor board
//                            (bad joint / seating on D8 or D9).
//
// With USB attached it also prints the four sensor states 5x per second.
// Pin map is identical to Sand_dial_new_boards_FINAL.ino.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define IR_OUTER_COUNTER A3
#define IR_OUTER_0       A2
#define IR_INNER_COUNTER A1
#define IR_INNER_0       A0

#define STEPPER_O_ENABLE 11
#define STEPPER_I_ENABLE 10
#define CHOOSE           12
#define OUTER_CONTROL     9
#define INNER_CONTROL     8
#define LED              13

const int HOUSE_PINS[6] = {2, 3, 4, 5, 6, 7};

Adafruit_PWMServoDriver pwmBoard[] = { Adafruit_PWMServoDriver(0x40), Adafruit_PWMServoDriver(0x41) };

void bezelWhite() {
  for (int b = 0; b < 2; b++)
    for (int ch = 0; ch < 15; ch++)
      pwmBoard[b].setPWM(ch, 0, 4080);
}

void setup() {
  pinMode(IR_OUTER_COUNTER, INPUT);
  pinMode(IR_OUTER_0, INPUT);
  pinMode(IR_INNER_COUNTER, INPUT);
  pinMode(IR_INNER_0, INPUT);
  pinMode(CHOOSE, INPUT_PULLUP);
  pinMode(OUTER_CONTROL, OUTPUT);  digitalWrite(OUTER_CONTROL, LOW);   // LOW = stop
  pinMode(INNER_CONTROL, OUTPUT);  digitalWrite(INNER_CONTROL, LOW);
  pinMode(STEPPER_O_ENABLE, OUTPUT); digitalWrite(STEPPER_O_ENABLE, LOW);  // LOW = driver enabled (holds the wheel)
  pinMode(STEPPER_I_ENABLE, OUTPUT); digitalWrite(STEPPER_I_ENABLE, LOW);
  for (int i = 0; i < 6; i++) { pinMode(HOUSE_PINS[i], OUTPUT); digitalWrite(HOUSE_PINS[i], LOW); }
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  Serial.println("SunDial PROBE boot");

  for (int i = 0; i < 3; i++) { digitalWrite(LED, HIGH); delay(120); digitalWrite(LED, LOW); delay(120); }

  for (int b = 0; b < 2; b++) {
    pwmBoard[b].begin();
    pwmBoard[b].setOscillatorFrequency(27000000);
    pwmBoard[b].setPWMFreq(50);
  }
  bezelWhite();
}

unsigned long lastPrint = 0, lastBezel = 0;

void loop() {
  int o0 = digitalRead(IR_OUTER_0);
  int oc = digitalRead(IR_OUTER_COUNTER);
  int i0 = digitalRead(IR_INNER_0);
  int ic = digitalRead(IR_INNER_COUNTER);
  int choose = digitalRead(CHOOSE);

  digitalWrite(LED, (o0 || oc || i0 || ic) ? HIGH : LOW);
  digitalWrite(OUTER_CONTROL, LOW);   // keep insisting: stop
  digitalWrite(INNER_CONTROL, LOW);

  unsigned long now = millis();
  if (now - lastPrint >= 200) {
    lastPrint = now;
    Serial.print("outer0="); Serial.print(o0);
    Serial.print(" outerCnt="); Serial.print(oc);
    Serial.print(" inner0="); Serial.print(i0);
    Serial.print(" innerCnt="); Serial.print(ic);
    Serial.print(" select="); Serial.print(choose == LOW ? "PRESSED" : "up");
    Serial.print(" up="); Serial.println(now / 1000);
  }
  if (now - lastBezel >= 2000) { lastBezel = now; bezelWhite(); }
}
