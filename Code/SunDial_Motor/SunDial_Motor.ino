//  SunDial_Motor  -  motor board Nano firmware v2.0.0  (2026-09-27)
//
//  Replaces motor_control_code_FINAL.ino (2/25/25) for the "spin and stop"
//  game. Same pins, same meaning of the two control lines from the controller
//  Nano (HIGH = turn that wheel, LOW = stop), two differences:
//
//    1. Each wheel is stepped on its OWN timer (micros), so the outer wheel
//       runs at exactly the same speed whether or not the inner wheel is
//       moving. The old sketch stepped both inside one loop with blocking
//       delays, so the outer slowed down whenever the inner ran - the
//       controller keeps the outer position by TIME, so that mattered.
//    2. Speeds are plain constants below. The wheels now spin on their own and
//       the players have to STOP them on a symbol / a number, so they must be
//       slow enough to read: about one symbol per second for the outer.
//
//  The two player buttons (D4 inner, D5 outer) are no longer expected here:
//  they now go to the controller Nano, which decides what a press means. If
//  they are still wired here they keep working as "hold to turn" (harmless,
//  but the controller will not know the wheel moved).
//
//  Build: arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 --output-dir build Code/SunDial_Motor

#define OUTER_CONTROL 11        // from controller Nano D9: HIGH = turn outer
#define INNER_CONTROL 10        // from controller Nano D8: HIGH = turn inner
#define ROTATE_INNER  4         // legacy player buttons (to GND), optional
#define ROTATE_OUTER  5
#define OUTER_STEPPER_DIR  A0
#define OUTER_STEPPER_STEP A1
#define INNER_STEPPER_DIR  A2
#define INNER_STEPPER_STEP A3

// Step period in microseconds (one HIGH+LOW pulse). Bigger = slower.
// Old sketch: outer 2 x 15 us = 30 us/step, inner 2 x 45 us = 90 us/step.
// Start slow and tune in the room: the controller prints "outer lap = N ms"
// at every game start; aim for roughly 8-12 s per outer lap.
#ifndef OUTER_STEP_US
#define OUTER_STEP_US 120
#endif
#ifndef INNER_STEP_US
#define INNER_STEP_US 180
#endif
#define PULSE_HIGH_US 5         // step pulse width (drivers need >= 1-2 us)

unsigned long outer_next_us = 0, inner_next_us = 0;

void setup () {
  pinMode (OUTER_CONTROL, INPUT_PULLUP);
  pinMode (INNER_CONTROL, INPUT_PULLUP);
  pinMode (ROTATE_OUTER, INPUT_PULLUP);
  pinMode (ROTATE_INNER, INPUT_PULLUP);
  pinMode (OUTER_STEPPER_DIR, OUTPUT);
  pinMode (OUTER_STEPPER_STEP, OUTPUT);
  pinMode (INNER_STEPPER_DIR, OUTPUT);
  pinMode (INNER_STEPPER_STEP, OUTPUT);
  digitalWrite (OUTER_STEPPER_DIR, LOW);      // same directions as the original sketch
  digitalWrite (INNER_STEPPER_DIR, HIGH);
  digitalWrite (OUTER_STEPPER_STEP, LOW);
  digitalWrite (INNER_STEPPER_STEP, LOW);
  Serial.begin (115200);
  Serial.println (F("SunDial motor v2.0.0 (independent step timers)"));
}

static inline void pulse (uint8_t pin) {
  digitalWrite (pin, HIGH);
  delayMicroseconds (PULSE_HIGH_US);
  digitalWrite (pin, LOW);
}

void loop () {
  unsigned long now = micros ();
  bool run_outer = digitalRead (OUTER_CONTROL) == HIGH || digitalRead (ROTATE_OUTER) == LOW;
  bool run_inner = digitalRead (INNER_CONTROL) == HIGH || digitalRead (ROTATE_INNER) == LOW;

  if (run_outer) {
    if ((long)(now - outer_next_us) >= 0) { pulse (OUTER_STEPPER_STEP); outer_next_us = now + OUTER_STEP_US; }
  } else outer_next_us = now;

  if (run_inner) {
    if ((long)(now - inner_next_us) >= 0) { pulse (INNER_STEPPER_STEP); inner_next_us = now + INNER_STEP_US; }
  } else inner_next_us = now;
}
