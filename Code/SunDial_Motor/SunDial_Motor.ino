//  SunDial_Motor  -  motor board Nano firmware v2.1.0  (2026-10-03)
//
//  Replaces motor_control_code_FINAL.ino (2/25/25) for the "spin and stop"
//  game. Same pins, three differences:
//
//    0. v2.1.0 (needs controller SpinStop v3.2.0+): a wheel turns only while
//       its control line is TOGGLING (the controller sends a 500 Hz square
//       wave). Any steady level = stop. Before, HIGH meant turn and the lines
//       have pull-ups, so a missing or silent controller spun both wheels
//       forever.
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

#define OUTER_CONTROL 11        // from controller Nano D9: square wave = turn outer
#define INNER_CONTROL 10        // from controller Nano D8: square wave = turn inner
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
#define KEEPALIVE_US  10000UL   // control line quiet this long = stop (the controller toggles it every 1 ms)

unsigned long outer_next_us = 0, inner_next_us = 0;

// One control line: "live" while it has changed level within KEEPALIVE_US.
struct Line { uint8_t pin; bool level, live; unsigned long changed_us; };
Line outer_line = { OUTER_CONTROL, true, false, 0 }, inner_line = { INNER_CONTROL, true, false, 0 };
bool line_live (Line& l, unsigned long now) {
  bool lv = digitalRead (l.pin) == HIGH;
  if (lv != l.level) { l.level = lv; l.changed_us = now; l.live = true; }
  else if (l.live && now - l.changed_us > KEEPALIVE_US) l.live = false;
  return l.live;
}

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
  Serial.println (F("SunDial motor v2.1.0 (wheels turn only on a live control signal)"));
}

static inline void pulse (uint8_t pin) {
  digitalWrite (pin, HIGH);
  delayMicroseconds (PULSE_HIGH_US);
  digitalWrite (pin, LOW);
}

void loop () {
  unsigned long now = micros ();
  bool run_outer = line_live (outer_line, now) || digitalRead (ROTATE_OUTER) == LOW;
  bool run_inner = line_live (inner_line, now) || digitalRead (ROTATE_INNER) == LOW;

  if (run_outer) {
    if ((long)(now - outer_next_us) >= 0) { pulse (OUTER_STEPPER_STEP); outer_next_us = now + OUTER_STEP_US; }
  } else outer_next_us = now;

  if (run_inner) {
    if ((long)(now - inner_next_us) >= 0) { pulse (INNER_STEPPER_STEP); inner_next_us = now + INNER_STEP_US; }
  } else inner_next_us = now;
}
