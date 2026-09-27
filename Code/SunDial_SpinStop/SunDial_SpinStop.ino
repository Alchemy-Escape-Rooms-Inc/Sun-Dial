//  SunDial_SpinStop  -  controller Nano firmware v3.1.0  (2026-09-27)
//
//  THE GAME ("spin and stop"):
//    1. Select (or the bridge trigger) starts a game: the dial homes, then BOTH
//       wheels spin on their own and the clue light(s) for step 1 pulse gold.
//    2. Players work out which symbol the clues point at and how many of that
//       thing are in the jungle. They STOP the outer wheel on the symbol with
//       the outer button and the inner wheel on the number with the inner
//       button (each press stops that wheel at its next stop; a press while
//       stopped nudges it ONE stop forward). The symbol the outer is resting on
//       lights white so they can see what they picked.
//    3. CHOOSE. Right = every light green 2 s, that symbol stays green, HOUSE
//       pulse to the bridge (MQTT SunDial/<Symbol> = true for M3 + Evalee),
//       twinkle, then the next step's clues pulse and both wheels spin again.
//       Wrong (either wheel off, or a wheel still spinning) = red 2 s + HOUSE_6
//       (SunDial/Wrong), then both wheels spin again.
//    4. Five steps: two plain (the clue IS the symbol), two with a pair of clues
//       and one with three clues, where the answer is what the clues point at
//       (see STEPS). Each step also prints "step=N ..." which the bridge turns
//       into MermaidsTale/SunDial/Clue = N for the sky-writing video next door. All five answers are the five countable things in
//       the jungle, so the bridge/M3 wiring is unchanged: HOUSE_1..5 = five
//       symbols, HOUSE_6 = wrong.
//    5. After the fifth: green celebration, wheels home, attract light show
//       until the next select.
//
//  WIRING CHANGES vs guided v2.x (REQUIRED):
//    * BOTH player buttons move from the motor Arduino (D4 inner, D5 outer) to
//      THIS board: OUTER button -> A3 (old outer tooth-sensor connector),
//      INNER button -> D7 (old HOUSE_6 "wrong" connector, never wired to the
//      bridge). Button to GND, internal pull-ups. See the defines below for
//      polarity and the A6/A7 alternative. With D7 as a button the Wrong pulse
//      to the bridge is off (it never reached the bridge anyway).
//    * Motor board gets SunDial_Motor v2.0 (independent step timers, slow
//      speeds). The old motor sketch also works but the outer slows whenever
//      the inner runs, which upsets the timed position.
//
//  POSITION: the outer tooth sensor is dead, so the outer position is kept by
//  TIME from the home-mark edge (one lap measured at every game start unless
//  OUTER_LAP_MS is baked in; one stop = lap / OUTER_TEETH). Every home-mark
//  edge re-syncs to position 1. The inner ring still has its tooth sensor and
//  snaps to a number exactly as before.
//
//  LIGHT MAP: each symbol's etched light is one RGB LED on the two PCA9685
//  boards. Five are known from the old code; the rest must be read off the
//  prop: hold SELECT while powering up = MAP MODE, the lights come on one at
//  a time for 3 s each in table order (board 0 LED 0..4, then board 1 LED
//  0..4) - write down which symbol lights at each place and fill SYMBOLS[].
//
//  SERIAL (115200, D1 TX -> divider -> bridge GPIO18): still prints
//  "outer counter = N" / "inner counter = N" (MermaidsTale/SunDial/Outer,
//  /Inner). USB RX bench keys: o = outer button, i = inner button, c = choose,
//  r = restart, s = status, l = light the next LED (map helper).
//
//  Build: arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 --output-dir build Code/SunDial_SpinStop

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define FW_VERSION "3.1.1"

// ---------------------------------------------------------------- pins
#define IR_OUTER_0        A2      // outer home mark (HIGH = mark in the beam)
#define IR_INNER_COUNTER  A1      // inner tooth sensor (HIGH = tooth in beam)
#define IR_INNER_0        A0      // inner home mark
#define STEPPER_O_ENABLE  11
#define STEPPER_I_ENABLE  10
#define CHOOSE            12      // select button, to GND, internal pull-up
#define OUTER_CONTROL     9       // HIGH = motor board turns the outer wheel
#define INNER_CONTROL     8       // HIGH = motor board turns the inner wheel
#define HOUSE_1           2       // bottle  -> bridge GPIO4  -> SunDial/Bottle
#define HOUSE_2           3       // skull   -> bridge GPIO5  -> SunDial/Crab   (topic keeps its old name)
#define HOUSE_3           4       // turtle  -> bridge GPIO6  -> SunDial/Turtle
#define HOUSE_4           5       // coconut -> bridge GPIO7  -> SunDial/Coconut
#define HOUSE_5           6       // trident -> bridge GPIO15 -> SunDial/Trident
#define HOUSE_6           7       // wrong   -> bridge GPIO16 -> SunDial/Wrong

// Player buttons (moved off the motor Arduino). 2026-09-27 photo of the carrier
// board: two connectors are free and already broken out, so no resistors:
//   A3 = the dead outer tooth-sensor input  -> OUTER button
//   D7 = HOUSE_6 "wrong" line, never wired to the bridge -> INNER button
// Button between the pin and GND, internal pull-up, pressed = LOW. If a pad
// has a pull-down resistor on the carrier (the old A3 sensor pad did), wire
// that button between the pin and +5 V instead and set its ACTIVE_LOW to 0.
// The boot log says "reads PRESSED at boot" when the polarity is wrong.
// A6/A7 (analog-only, need an external 10k pull-up) still work: set *_ANALOG 1.
#ifndef OUTER_BUTTON_PIN
#define OUTER_BUTTON_PIN     A3
#endif
#ifndef OUTER_BUTTON_ANALOG
#define OUTER_BUTTON_ANALOG  0
#endif
#ifndef OUTER_BUTTON_ACTIVE_LOW
#define OUTER_BUTTON_ACTIVE_LOW 1
#endif
#ifndef INNER_BUTTON_PIN
#define INNER_BUTTON_PIN     7
#endif
#ifndef INNER_BUTTON_ANALOG
#define INNER_BUTTON_ANALOG  0
#endif
#ifndef INNER_BUTTON_ACTIVE_LOW
#define INNER_BUTTON_ACTIVE_LOW 1
#endif
#define ANALOG_PRESSED_BELOW 400  // analogRead under this = pressed (pulled up, button to GND)

// ---------------------------------------------------------------- tuning
#define OUTER_TEETH            10      // symbol stops on the outer ring
#define OUTER_LAP_MS           0       // 0 = measure at every game start; or bake in the "outer lap = N ms" serial value
#define OUTER_PARK_OFFSET_MS   0       // + stops a little later after each stop boundary (tune if symbols sit off-centre)
#define BUTTON_DEBOUNCE_MS     30
#define SEEK_TIMEOUT_MS        90000UL // homing / lap measure longer than this stops the motor (select retries)
#define PULSE_PERIOD_MS        1400    // breathing period of the clue lights
#define ATTRACT_STEP_MS        600
#define TWINKLE_MS             1500
#define FEEDBACK_MS            2000    // green / red hold

// ---------------------------------------------------------------- symbols and lights
// One entry per etched symbol. pos = outer stop that puts it under the pointer
// (0 = not known / not needed). led_board/led = its RGB light. Lights marked
// MAP are guesses: run MAP MODE on the prop and correct them.
enum Sym { S_BOTTLE, S_TRIDENT, S_SKULL, S_COCONUT, S_TURTLE, S_LIGHTHOUSE, S_SHARK, S_CRAB, S_SEAHORSE, S_ANCHOR, NUM_SYMBOLS };
struct Symbol { const char* name; uint8_t pos; uint8_t led_board; uint8_t led; };
const Symbol SYMBOLS[NUM_SYMBOLS] = {
  { "bottle",     2, 0, 1 },   // known (old code: board 0 addr 4)
  { "trident",    9, 1, 3 },   // known (board 1 addr 10)
  { "skull",      5, 0, 2 },   // known (board 0 addr 7, wire name "Crab")
  { "coconut",    8, 1, 2 },   // known (board 1 addr 7)
  { "turtle",     6, 1, 0 },   // known (board 1 addr 1)
  { "lighthouse", 0, 0, 0 },   // MAP
  { "shark",      0, 0, 3 },   // MAP
  { "crab",       0, 0, 4 },   // MAP
  { "seahorse",   0, 1, 1 },   // MAP
  { "anchor",     0, 1, 4 },   // MAP
};

// The five steps. clues = symbols whose lights pulse; answer = symbol the
// outer must stop on; inner = the count to dial; house = pulse pin.
#define MAX_CLUES 3
struct Step { uint8_t nclues; uint8_t clues[MAX_CLUES]; uint8_t answer; uint8_t inner; uint8_t house; };
const Step STEPS[] = {
  { 1, { S_BOTTLE,     0,          0          }, S_BOTTLE,  3, HOUSE_1 },   // plain
  { 1, { S_TRIDENT,    0,          0          }, S_TRIDENT, 1, HOUSE_5 },   // plain
  { 2, { S_COCONUT,    S_CRAB,     0          }, S_TURTLE,  7, HOUSE_3 },   // hard shells: what's the third?
  { 2, { S_LIGHTHOUSE, S_ANCHOR,   0          }, S_COCONUT, 9, HOUSE_4 },   // land ahoy, drop anchor: what's on the island?
  { 3, { S_SHARK,      S_TRIDENT,  S_BOTTLE   }, S_SKULL,   4, HOUSE_2 },   // three ways a pirate dies: eaten, speared, rum
};
const int NUM_STEPS = sizeof (STEPS) / sizeof (STEPS[0]);

const int LED_BOARDS = 2;
const int LEDS_PER_BOARD = 5;
Adafruit_PWMServoDriver pwmBoard[] = { Adafruit_PWMServoDriver (0x40), Adafruit_PWMServoDriver (0x41) };

// ---------------------------------------------------------------- state
enum Phase { PH_ATTRACT, PH_HOMING, PH_PLAY, PH_DONE };
Phase phase = PH_ATTRACT;

// outer wheel: timed position
enum OuterState { OUT_SPIN, OUT_STOPPING, OUT_STOPPED, OUT_NUDGE, OUT_SEEK_LEAVE, OUT_SEEK_ZERO, OUT_FAULT };
OuterState outer_state = OUT_STOPPED;
int  outer_pos = 1;                 // believed outer position 1..OUTER_TEETH (last reported)
unsigned long lap_ms = OUTER_LAP_MS;
unsigned long spin_zero_ms = 0;     // time of the (real or virtual) home-mark edge the spin is timed from
unsigned long stop_at_ms = 0;       // OUT_STOPPING / OUT_NUDGE: stop when now >= this
int  stop_pos = 1;                  // position we will be on when stop_at_ms passes
unsigned long seek_started_ms = 0, cal_edge_ms = 0;
bool zero_prev = false;
bool outer_press_pending = false;   // pressed during lap measure: stop at the first stop after it

// inner wheel
enum InnerState { IN_SPIN, IN_SNAP, IN_STOPPED, IN_NUDGE };
InnerState inner_state = IN_STOPPED;
int  inner_counter = 1;
int  prev_inner_counter_state = 0;
unsigned long inner_gate_ms = 0;

int  current_step = 0;
bool solved_sym[NUM_SYMBOLS];

// buttons
struct Button { bool state, prev, raw_prev, edge; unsigned long raw_since; };
Button btn_outer, btn_inner;
bool choose_prev = false, choose_edge = false;
bool trigger_pending = false;
unsigned long attract_ms = 0;
int attract_color = 0;
const uint8_t ATTRACT_COLORS[][3] = { {0,0,255},{0,255,0},{55,0,127},{0,255,255},{51,51,255},{255,0,255},{255,255,255},{255,0,0} };
const int NUM_ATTRACT_COLORS = 8;
int map_led = -1;                   // bench 'l' helper

// ---------------------------------------------------------------- LEDs
// Each LED uses three PCA channels: idx*3+0 = blue, +1 = green, +2 = red.
void set_led (int board, int idx, uint8_t r, uint8_t g, uint8_t b) {
  pwmBoard[board].setPWM (idx * 3 + 0, 0, (uint16_t)b * 16);
  pwmBoard[board].setPWM (idx * 3 + 1, 0, (uint16_t)g * 16);
  pwmBoard[board].setPWM (idx * 3 + 2, 0, (uint16_t)r * 16);
}
void sym_led (int s, uint8_t r, uint8_t g, uint8_t b) { set_led (SYMBOLS[s].led_board, SYMBOLS[s].led, r, g, b); }
void all_leds (uint8_t r, uint8_t g, uint8_t b) {
  for (int bd = 0; bd < LED_BOARDS; bd++)
    for (int i = 0; i < LEDS_PER_BOARD; i++) set_led (bd, i, r, g, b);
}
bool is_clue (int s) {
  const Step& st = STEPS[current_step];
  for (int i = 0; i < st.nclues; i++) if (st.clues[i] == s) return true;
  return false;
}
int symbol_at (int pos) {           // which symbol sits under the pointer at this outer stop (-1 = unknown)
  for (int s = 0; s < NUM_SYMBOLS; s++) if (SYMBOLS[s].pos == pos) return s;
  return -1;
}
// The play picture, repainted every 25 ms: solved = green, clues = pulsing
// gold, the symbol the stopped outer rests on = white, the rest dark.
void paint_play (unsigned long now) {
  static unsigned long last_paint = 0;
  static int last_white = -1;
  if (now - last_paint < 25) return;
  last_paint = now;
  float ph = (float)(now % PULSE_PERIOD_MS) / (float)PULSE_PERIOD_MS;
  float tri = ph < 0.5f ? ph * 2.0f : (1.0f - ph) * 2.0f;
  uint8_t level = (uint8_t)(20 + tri * tri * 235);
  int white = (outer_state == OUT_STOPPED) ? symbol_at (outer_pos) : -1;
  for (int s = 0; s < NUM_SYMBOLS; s++) {
    if (solved_sym[s])       sym_led (s, 0, 255, 0);
    else if (is_clue (s))    sym_led (s, level, (uint8_t)((uint16_t)level * 2 / 3), 0);
    else if (s == white)     { if (last_white != s) sym_led (s, 255, 255, 255); }
    else if (s == last_white) sym_led (s, 0, 0, 0);
  }
  last_white = white;
}
void repaint_all () {
  all_leds (0, 0, 0);
  for (int s = 0; s < NUM_SYMBOLS; s++) if (solved_sym[s]) sym_led (s, 0, 255, 0);
}
void twinkle (unsigned long ms) {
  unsigned long t0 = millis ();
  while (millis () - t0 < ms) {
    int bd = random (LED_BOARDS), i = random (LEDS_PER_BOARD);
    set_led (bd, i, 255, 200, 40);
    delay (60);
    set_led (bd, i, 0, 0, 0);
  }
  repaint_all ();
}

// ---------------------------------------------------------------- outer wheel
void outer_run ()  { digitalWrite (OUTER_CONTROL, HIGH); }
void outer_stop () { digitalWrite (OUTER_CONTROL, LOW); }
unsigned long tooth_ms () { return lap_ms / OUTER_TEETH; }
bool zero_now () { return digitalRead (IR_OUTER_0) == HIGH; }
bool outer_moving () { return outer_state != OUT_STOPPED && outer_state != OUT_FAULT; }
void report_outer (int pos) {
  if (pos != outer_pos) { outer_pos = pos; Serial.print (F("outer counter = ")); Serial.println (outer_pos); }
}
// position the timed model says we are at right now (while spinning)
int outer_pos_now (unsigned long now) {
  if (lap_ms == 0) return outer_pos;
  return (int)(((now - spin_zero_ms) / tooth_ms ()) % OUTER_TEETH) + 1;
}
void outer_begin_spin (unsigned long now) {
  // continue the timed model from the known stop: pretend the mark passed (pos-1) stops ago
  spin_zero_ms = now - (unsigned long)(outer_pos - 1) * tooth_ms ();
  outer_state = OUT_SPIN;
  outer_run ();
}
// player pressed while spinning: stop at the next stop boundary
void outer_request_stop (unsigned long now) {
  if (lap_ms == 0) { outer_press_pending = true; return; }          // lap not measured yet: stop at the mark
  unsigned long el = now - spin_zero_ms;
  unsigned long k = el / tooth_ms () + 1;                            // next boundary index after the mark
  stop_at_ms = spin_zero_ms + k * tooth_ms () + OUTER_PARK_OFFSET_MS;
  stop_pos = (int)(k % OUTER_TEETH) + 1;
  outer_state = OUT_STOPPING;
}
// player pressed while stopped: one stop forward
void outer_nudge (unsigned long now) {
  stop_at_ms = now + tooth_ms () + OUTER_PARK_OFFSET_MS;
  stop_pos = outer_pos % OUTER_TEETH + 1;
  spin_zero_ms = now - (unsigned long)(outer_pos - 1) * tooth_ms ();
  outer_state = OUT_NUDGE;
  outer_run ();
}
void outer_fault (const __FlashStringHelper* why) {
  outer_stop ();
  outer_state = OUT_FAULT;
  Serial.print (F("ERROR outer wheel ")); Serial.print (why); Serial.println (F(" - motor stopped (select to retry)"));
}
// Non-blocking, every loop pass.
void drive_outer (unsigned long now) {
  bool zero = zero_now ();
  bool zero_edge = zero && !zero_prev;
  zero_prev = zero;

  switch (outer_state) {
    case OUT_STOPPED:
    case OUT_FAULT:
      outer_stop ();
      break;

    case OUT_SEEK_LEAVE:                        // lap measure: drive off the mark first
      outer_run ();
      if (!zero) outer_state = OUT_SEEK_ZERO;
      else if (now - seek_started_ms > SEEK_TIMEOUT_MS) outer_fault (F("never left the home mark"));
      break;

    case OUT_SEEK_ZERO:                         // lap measure: two mark edges one lap apart
      outer_run ();
      if (zero_edge) {
        if (cal_edge_ms == 0) { cal_edge_ms = now; Serial.println (F("outer: measuring one lap")); }
        else {
          lap_ms = now - cal_edge_ms; cal_edge_ms = 0;
          Serial.print (F("outer lap = ")); Serial.print (lap_ms); Serial.println (F(" ms  (bake into OUTER_LAP_MS)"));
          spin_zero_ms = now; report_outer (1);
          outer_state = OUT_SPIN;               // keep spinning: play has begun
          if (outer_press_pending) { outer_press_pending = false; outer_stop (); outer_state = OUT_STOPPED; Serial.println (F("outer: stopped on 1 (press during lap measure)")); }
        }
      } else if (now - seek_started_ms > SEEK_TIMEOUT_MS) outer_fault (F("never saw the home mark"));
      break;

    case OUT_SPIN:
      outer_run ();
      if (zero_edge) { spin_zero_ms = now; report_outer (1); }
      else report_outer (outer_pos_now (now));
      break;

    case OUT_STOPPING:
    case OUT_NUDGE:
      outer_run ();
      if (zero_edge) {                          // the mark itself is stop 1: land there, re-synced
        if (OUTER_PARK_OFFSET_MS > 0) delay (OUTER_PARK_OFFSET_MS);
        outer_stop (); outer_state = OUT_STOPPED; spin_zero_ms = now;
        report_outer (1);
        Serial.println (F("outer: stopped"));
      } else if ((long)(now - stop_at_ms) >= 0) {
        outer_stop (); outer_state = OUT_STOPPED;
        report_outer (stop_pos);
        Serial.println (F("outer: stopped"));
      }
      break;
  }
}

// ---------------------------------------------------------------- inner wheel
// IN_SPIN: motor runs. IN_SNAP: stop at the next tooth (control LOW while a
// tooth is in the beam, HIGH otherwise - the original snap rule). IN_NUDGE:
// run off the current tooth, then snap to the next one.
void drive_inner () {
  bool tooth = digitalRead (IR_INNER_COUNTER) == HIGH;
  switch (inner_state) {
    case IN_SPIN:    digitalWrite (INNER_CONTROL, HIGH); break;
    case IN_STOPPED: digitalWrite (INNER_CONTROL, LOW);  break;
    case IN_SNAP:
      digitalWrite (INNER_CONTROL, tooth ? LOW : HIGH);
      if (tooth) { inner_state = IN_STOPPED; Serial.println (F("inner: stopped")); }
      break;
    case IN_NUDGE:
      digitalWrite (INNER_CONTROL, HIGH);
      if (!tooth) inner_state = IN_SNAP;
      break;
  }
}
void count_inner (unsigned long now) {
  if (now - inner_gate_ms <= 105) return;       // 105 ms gate like the original sketch
  inner_gate_ms = now;
  int st = digitalRead (IR_INNER_COUNTER);
  if (st == HIGH && prev_inner_counter_state == LOW) {
    inner_counter++;
    if (digitalRead (IR_INNER_0) == HIGH) inner_counter = 1;
    Serial.print (F("inner counter = ")); Serial.println (inner_counter);
  }
  prev_inner_counter_state = st;
}

// Blocking homing of BOTH wheels (same as the original reset_the_wheels).
void home_wheels () {
  digitalWrite (STEPPER_O_ENABLE, LOW); digitalWrite (STEPPER_I_ENABLE, LOW);
  outer_stop (); digitalWrite (INNER_CONTROL, LOW);
  delay (500);
  Serial.println (F(" reset wheels"));
  unsigned long t0 = millis ();
  while (true) {
    bool oz = zero_now (), iz = digitalRead (IR_INNER_0) == HIGH;
    digitalWrite (OUTER_CONTROL, oz ? LOW : HIGH);
    digitalWrite (INNER_CONTROL, iz ? LOW : HIGH);
    if (oz && iz) break;
    if (millis () - t0 > SEEK_TIMEOUT_MS) { Serial.println (F("ERROR homing timed out")); break; }
  }
  outer_stop (); digitalWrite (INNER_CONTROL, LOW);
  outer_pos = 0; report_outer (1);
  inner_counter = 1; Serial.println (F("inner counter = 1"));
  prev_inner_counter_state = digitalRead (IR_INNER_COUNTER);
  zero_prev = zero_now ();
  outer_state = OUT_STOPPED; inner_state = IN_STOPPED;
}

// ---------------------------------------------------------------- inputs
bool read_button_raw (uint8_t pin, bool analog, bool active_low) {
  bool low = analog ? (analogRead (pin) < ANALOG_PRESSED_BELOW) : (digitalRead (pin) == LOW);
  return active_low ? low : !low;
}
void debounce (Button& b, bool raw, unsigned long now) {
  if (raw != b.raw_prev) { b.raw_prev = raw; b.raw_since = now; }
  b.prev = b.state;
  if (now - b.raw_since >= BUTTON_DEBOUNCE_MS) b.state = raw;
  b.edge = b.state && !b.prev;
}
void read_inputs (unsigned long now) {
  debounce (btn_outer, read_button_raw (OUTER_BUTTON_PIN, OUTER_BUTTON_ANALOG, OUTER_BUTTON_ACTIVE_LOW), now);
  debounce (btn_inner, read_button_raw (INNER_BUTTON_PIN, INNER_BUTTON_ANALOG, INNER_BUTTON_ACTIVE_LOW), now);
  if (btn_outer.edge) Serial.println (F("outer button: PRESSED"));
  if (btn_inner.edge) Serial.println (F("inner button: PRESSED"));

  bool ch = digitalRead (CHOOSE) == LOW;
  choose_edge = ch && !choose_prev;
  choose_prev = ch;
  if (choose_edge) Serial.println (F("choose pin: PRESSED"));

  count_inner (now);

  while (Serial.available ()) {
    char c = Serial.read ();
    if      (c == 'o') { btn_outer.edge = true; Serial.println (F("bench: outer")); }
    else if (c == 'i') { btn_inner.edge = true; Serial.println (F("bench: inner")); }
    else if (c == 'c') { choose_edge = true; Serial.println (F("bench: choose")); }
    else if (c == 'r') { trigger_pending = true; Serial.println (F("bench: restart")); }
    else if (c == 's') { print_status (); }
    else if (c == 'l') {                        // light the next LED so the symbol map can be read off the prop
      all_leds (0, 0, 0);
      map_led = (map_led + 1) % (LED_BOARDS * LEDS_PER_BOARD);
      set_led (map_led / LEDS_PER_BOARD, map_led % LEDS_PER_BOARD, 255, 255, 255);
      Serial.print (F("map: board ")); Serial.print (map_led / LEDS_PER_BOARD); Serial.print (F(" led ")); Serial.println (map_led % LEDS_PER_BOARD);
    }
  }
}
void print_status () {
  Serial.print (F("status: phase=")); Serial.print ((int)phase);
  Serial.print (F(" step=")); Serial.print (current_step + 1); Serial.print (F("/")); Serial.print (NUM_STEPS);
  Serial.print (F(" answer=")); Serial.print (current_step < NUM_STEPS ? SYMBOLS[STEPS[current_step].answer].name : "done");
  Serial.print (F(" outer=")); Serial.print (outer_pos); Serial.print (F(" (")); Serial.print ((int)outer_state); Serial.print (F(")"));
  Serial.print (F(" inner=")); Serial.print (inner_counter); Serial.print (F(" (")); Serial.print ((int)inner_state); Serial.print (F(")"));
  Serial.print (F(" lap_ms=")); Serial.println (lap_ms);
}

// ---------------------------------------------------------------- game flow
void announce_step () {
  if (current_step >= NUM_STEPS) { Serial.println (F("step=done")); return; }
  const Step& st = STEPS[current_step];
  Serial.print (F("step=")); Serial.print (current_step + 1);
  Serial.print (F(" clues="));
  for (int i = 0; i < st.nclues; i++) { if (i) Serial.print (F("+")); Serial.print (SYMBOLS[st.clues[i]].name); }
  Serial.print (F(" answer=")); Serial.print (SYMBOLS[st.answer].name);
  Serial.print (F(" target_outer=")); Serial.print (SYMBOLS[st.answer].pos);
  Serial.print (F(" need_inner=")); Serial.println (st.inner);
}
void spin_both (unsigned long now) {
  outer_begin_spin (now);
  inner_state = IN_SPIN;
  Serial.println (F("wheels spinning"));
}
void start_game () {
  for (int s = 0; s < NUM_SYMBOLS; s++) solved_sym[s] = false;
  for (int p = 2; p <= 7; p++) if (p != OUTER_BUTTON_PIN && p != INNER_BUTTON_PIN) digitalWrite (p, LOW);
  current_step = 0;
  all_leds (0, 0, 0);
  home_wheels ();
  Serial.println (F("game start"));
  announce_step ();
  phase = PH_HOMING;
  if (OUTER_LAP_MS == 0 && lap_ms == 0) {         // measure the lap while the wheels start spinning
    inner_state = IN_SPIN;
    outer_press_pending = false; cal_edge_ms = 0; seek_started_ms = millis ();
    outer_state = OUT_SEEK_LEAVE;
  } else {
    spin_both (millis ());
    phase = PH_PLAY;
  }
}
void next_step () {
  current_step++;
  if (current_step >= NUM_STEPS) { finish_game (); return; }
  twinkle (TWINKLE_MS);
  announce_step ();
  spin_both (millis ());
}
void handle_choose () {
  const Step& st = STEPS[current_step];
  bool both_stopped = (outer_state == OUT_STOPPED && inner_state == IN_STOPPED);
  bool right = both_stopped && outer_pos == SYMBOLS[st.answer].pos && inner_counter == st.inner;
  Serial.print (F("choose outer=")); Serial.print (outer_pos); Serial.print (F(" inner=")); Serial.print (inner_counter);
  Serial.println (right ? F(" -> CORRECT") : (both_stopped ? F(" -> wrong") : F(" -> wrong (a wheel is still moving)")));
  outer_stop (); digitalWrite (INNER_CONTROL, LOW);
  if (right) {
    digitalWrite (st.house, HIGH);              // 2 s pulse -> bridge -> MQTT SunDial/<Symbol> true
    all_leds (0, 255, 0);
    delay (FEEDBACK_MS);
    digitalWrite (st.house, LOW);
    solved_sym[st.answer] = true;
    next_step ();
  } else {
    if (INNER_BUTTON_PIN != HOUSE_6 && OUTER_BUTTON_PIN != HOUSE_6) digitalWrite (HOUSE_6, HIGH);   // -> MQTT SunDial/Wrong true (not when D7 is a button)
    all_leds (255, 0, 0);
    delay (FEEDBACK_MS);
    if (INNER_BUTTON_PIN != HOUSE_6 && OUTER_BUTTON_PIN != HOUSE_6) digitalWrite (HOUSE_6, LOW);
    repaint_all ();
    if (outer_state == OUT_FAULT) return;
    spin_both (millis ());                      // both wheels go again
  }
}
void finish_game () {
  Serial.println (F("SOLVED - all symbols"));
  phase = PH_DONE;
  outer_stop (); digitalWrite (INNER_CONTROL, LOW);
  for (int i = 0; i < 5; i++) { all_leds (0, 255, 0); delay (150); all_leds (0, 0, 0); delay (150); }
  all_leds (0, 255, 0);
  delay (5000);
  all_leds (0, 0, 0);
  home_wheels ();
  digitalWrite (STEPPER_O_ENABLE, HIGH); digitalWrite (STEPPER_I_ENABLE, HIGH);
  phase = PH_ATTRACT; attract_ms = 0;
  Serial.println (F("attract (select to start)"));
}
void run_attract (unsigned long now) {
  if (now - attract_ms >= ATTRACT_STEP_MS) {
    attract_ms = now;
    const uint8_t* c = ATTRACT_COLORS[attract_color];
    all_leds (c[2], c[1], c[0]);                // table is stored blue,green,red like the original sketch
    attract_color = (attract_color + 1) % NUM_ATTRACT_COLORS;
  }
}
// Hold SELECT at power-up: light every LED in turn so the symbol map can be read.
void map_mode () {
  Serial.println (F("MAP MODE: each light 3 s, in table order board0 led0..4 then board1 led0..4"));
  for (int round = 0; round < 3; round++)
    for (int bd = 0; bd < LED_BOARDS; bd++)
      for (int i = 0; i < LEDS_PER_BOARD; i++) {
        all_leds (0, 0, 0);
        set_led (bd, i, 255, 255, 255);
        Serial.print (F("map: board ")); Serial.print (bd); Serial.print (F(" led ")); Serial.println (i);
        delay (3000);
      }
  all_leds (0, 0, 0);
}

// ---------------------------------------------------------------- setup / loop
void setup () {
  Serial.begin (115200);
  pinMode (IR_OUTER_0, INPUT); pinMode (IR_INNER_0, INPUT); pinMode (IR_INNER_COUNTER, INPUT);
  pinMode (STEPPER_O_ENABLE, OUTPUT); pinMode (STEPPER_I_ENABLE, OUTPUT);
  pinMode (OUTER_CONTROL, OUTPUT); outer_stop ();
  pinMode (INNER_CONTROL, OUTPUT); digitalWrite (INNER_CONTROL, LOW);
  pinMode (CHOOSE, INPUT_PULLUP);
  for (int p = 2; p <= 7; p++) { if (p == OUTER_BUTTON_PIN || p == INNER_BUTTON_PIN) continue; pinMode (p, OUTPUT); digitalWrite (p, LOW); }
#if !OUTER_BUTTON_ANALOG
  pinMode (OUTER_BUTTON_PIN, INPUT_PULLUP);
#endif
#if !INNER_BUTTON_ANALOG
  pinMode (INNER_BUTTON_PIN, INPUT_PULLUP);
#endif
  delay (5);
  Serial.print (F("SunDial SpinStop v")); Serial.println (FW_VERSION);
  bool select_held = digitalRead (CHOOSE) == LOW;
  if (read_button_raw (OUTER_BUTTON_PIN, OUTER_BUTTON_ANALOG, OUTER_BUTTON_ACTIVE_LOW)) Serial.println (F("WARNING: outer button reads PRESSED at boot - check its wire / pull-up"));
  if (read_button_raw (INNER_BUTTON_PIN, INNER_BUTTON_ANALOG, INNER_BUTTON_ACTIVE_LOW)) Serial.println (F("WARNING: inner button reads PRESSED at boot - check its wire / pull-up"));

  for (int i = 0; i < LED_BOARDS; i++) { pwmBoard[i].begin (); pwmBoard[i].setOscillatorFrequency (27000000); pwmBoard[i].setPWMFreq (50); }
  all_leds (0, 0, 0);
  digitalWrite (STEPPER_O_ENABLE, HIGH); digitalWrite (STEPPER_I_ENABLE, HIGH);
  if (select_held) map_mode ();
  delay (2000);
  all_leds (255, 255, 255);
  home_wheels ();
  delay (1000);
  all_leds (0, 0, 0);
  digitalWrite (STEPPER_O_ENABLE, HIGH); digitalWrite (STEPPER_I_ENABLE, HIGH);
  randomSeed (analogRead (A7));
  phase = PH_ATTRACT;
  Serial.println (F("attract (select to start)"));
}

void loop () {
  unsigned long now = millis ();
  read_inputs (now);

  if (trigger_pending) { trigger_pending = false; Serial.println (F("trigger -> restart")); start_game (); return; }

  switch (phase) {
    case PH_ATTRACT:
      run_attract (now);
      if (choose_edge) start_game ();
      break;

    case PH_HOMING:                              // wheels spinning, lap being measured
      if (btn_outer.edge) outer_request_stop (now);
      if (btn_inner.edge && inner_state == IN_SPIN) inner_state = IN_SNAP;
      drive_outer (now);
      drive_inner ();
      paint_play (now);
      if (outer_state == OUT_SPIN || outer_state == OUT_STOPPED) { phase = PH_PLAY; Serial.println (F("play")); }
      if (outer_state == OUT_FAULT && choose_edge) { Serial.println (F("select -> retry")); cal_edge_ms = 0; seek_started_ms = now; outer_state = OUT_SEEK_LEAVE; }
      break;

    case PH_PLAY:
      if (btn_outer.edge) {
        if (outer_state == OUT_SPIN) outer_request_stop (now);
        else if (outer_state == OUT_STOPPED) outer_nudge (now);
      }
      if (btn_inner.edge) {
        if (inner_state == IN_SPIN) inner_state = IN_SNAP;
        else if (inner_state == IN_STOPPED) inner_state = IN_NUDGE;
      }
      drive_outer (now);
      drive_inner ();
      paint_play (now);
      if (choose_edge) {
        if (outer_state == OUT_FAULT) { Serial.println (F("select -> retry (re-home)")); home_wheels (); spin_both (millis ()); }
        else handle_choose ();
      }
      break;

    case PH_DONE:
      break;
  }
}
