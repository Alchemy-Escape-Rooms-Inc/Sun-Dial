//  SunDial_SpinStop  -  controller Nano firmware v3.4.0  (2026-10-07)
//
//  THE GAME (owner's flow, 2026-10-07 - "show spin, then the players dial"):
//    1. Select (or the bridge trigger) starts a game: the dial homes, the outer
//       wheel makes one show lap on its own and stops, the inner ring returns
//       to 1, and only then the clue light(s) for step 1 pulse gold.
//    2. Nothing moves by itself during play. Players work out which symbol the
//       clues point at and how many of that thing are in the jungle, then turn
//       the OUTER wheel to the symbol with the outer button and the INNER ring
//       to the number with the inner button: one press = one stop forward,
//       hold = keeps stepping. Only one wheel moves at a time. The symbol the
//       outer is resting on lights white so they can see what they picked.
//    3. CHOOSE (ignored while a wheel is moving). Right = every light green
//       2 s, that symbol stays green, HOUSE pulse to the bridge (MQTT
//       SunDial/<Symbol> = true for M3 + Evalee), twinkle, the outer makes its
//       show lap again, the inner returns to 1, then the next step's clues
//       pulse. Wrong = red 2 s + HOUSE_6 (SunDial/Wrong); the wheels stay
//       where the players left them.
//    4. Five steps, all riddles: the pulsing clue light(s) give one half and
//       the words in the clouds (TV next door) give the other; the answer is
//       never one of the lit clues, and no clue is an already-solved symbol
//       (solved lights stay green and cannot pulse) (see STEPS). Each step
//       prints "step=N ..." AFTER its show lap, which the bridge turns into
//       MermaidsTale/SunDial/Clue = N for the sky-writing video. All five
//       answers are the five countable things in the jungle, so the bridge/M3
//       wiring is unchanged: HOUSE_1..5 = five symbols, HOUSE_6 = wrong.
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
//    * Motor board MUST run SunDial_Motor v2.1+ (flash both boards together).
//      Since v3.2.0 "turn this wheel" is a 500 Hz square wave on the control
//      line and any steady level means stop, so a controller that is missing,
//      unpowered or hung leaves the wheels standing still instead of spinning
//      forever. An older motor sketch would read the square wave as stutter.
//
//  POSITION: the outer tooth sensor is dead, so the outer position is kept by
//  TIME from the home-mark edge (one lap measured on the show lap unless
//  OUTER_LAP_MS is baked in; one stop = lap / OUTER_TEETH). Every show lap
//  ends ON the home mark, so every step starts re-synced at position 1. The inner ring still has its tooth sensor and
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

#define FW_VERSION "3.4.0"

// ---------------------------------------------------------------- pins
#define IR_OUTER_0        A2      // outer home mark (HIGH = mark in the beam)
#define IR_INNER_COUNTER  A1      // inner tooth sensor (HIGH = tooth in beam)
#define IR_INNER_0        A0      // inner home mark
#define STEPPER_O_ENABLE  11
#define STEPPER_I_ENABLE  10
#define CHOOSE            12      // select button, to GND, internal pull-up
#define OUTER_CONTROL     9       // PB1: 500 Hz square wave = motor board turns the outer wheel, steady = stop
#define INNER_CONTROL     8       // PB0: same for the inner wheel
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
#define OUTER_LAP_MS           0       // 0 = measure on the show lap; or bake in the "outer lap = N ms" serial value
#define OUTER_PARK_OFFSET_MS   0       // + stops a little later after each stop boundary (tune if symbols sit off-centre)
#define BUTTON_DEBOUNCE_MS     30
#define SEEK_TIMEOUT_MS        90000UL // homing / show lap longer than this stops the motor (select retries)
#define MARK_SETTLE_MS         40      // ignore home-mark chatter this long after an edge
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
  { "bottle",     2, 0, 1 },   // light map read off the prop in MAP MODE 2026-10-07:
  { "trident",    9, 1, 3 },   //   board 0 led 0..4 = seahorse, bottle, crab, skull, shark
  { "skull",      5, 0, 3 },   //   board 1 led 0..4 = turtle, lighthouse, coconut, trident, anchor
  { "coconut",    8, 1, 2 },   // (skull was 0/2 in the old code; the owner reads 0/2 as the crab, 0/3 as the skull)
  { "turtle",     6, 1, 0 },
  { "lighthouse", 0, 1, 1 },
  { "shark",      0, 0, 4 },
  { "crab",       0, 0, 2 },
  { "seahorse",   0, 0, 0 },
  { "anchor",     0, 1, 4 },
};

// The five steps. clues = symbols whose lights pulse; answer = symbol the
// outer must stop on; inner = the count to dial; house = pulse pin.
#define MAX_CLUES 3
struct Step { uint8_t nclues; uint8_t clues[MAX_CLUES]; uint8_t answer; uint8_t inner; uint8_t house; };
const Step STEPS[] = {
  { 1, { S_LIGHTHOUSE, 0,          0          }, S_BOTTLE,  3, HOUSE_1 },   // sends word across the water; neck no head, mouth no voice
  { 2, { S_CRAB,       S_SEAHORSE, 0          }, S_TURTLE,  7, HOUSE_3 },   // born under sand, raised under waves, never leaves home
  { 2, { S_LIGHTHOUSE, S_ANCHOR,   0          }, S_COCONUT, 9, HOUSE_4 },   // three eyes, a beard, waits up high then falls
  { 2, { S_SHARK,      S_CRAB,     0          }, S_SKULL,   4, HOUSE_2 },   // held a pirate's secrets, now empty and grinning
  { 2, { S_SHARK,      S_SEAHORSE, 0          }, S_TRIDENT, 1, HOUSE_5 },   // three teeth, never tasted; its holder rules all that swim
};
const int NUM_STEPS = sizeof (STEPS) / sizeof (STEPS[0]);

const int LED_BOARDS = 2;
const int LEDS_PER_BOARD = 5;
Adafruit_PWMServoDriver pwmBoard[] = { Adafruit_PWMServoDriver (0x40), Adafruit_PWMServoDriver (0x41) };

// ---------------------------------------------------------------- state
enum Phase { PH_ATTRACT, PH_PLAY, PH_DONE };
Phase phase = PH_ATTRACT;

// outer wheel: timed position
enum OuterState { OUT_STOPPED, OUT_NUDGE, OUT_FAULT };
OuterState outer_state = OUT_STOPPED;
int  outer_pos = 1;                 // believed outer position 1..OUTER_TEETH (last reported)
unsigned long lap_ms = OUTER_LAP_MS;
unsigned long stop_at_ms = 0;       // OUT_NUDGE: stop when now >= this
int  stop_pos = 1;                  // position we will be on when stop_at_ms passes
bool zero_prev = false;

// inner wheel
enum InnerState { IN_SNAP, IN_STOPPED, IN_NUDGE };
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
// Control lines to the motor board: Timer2 toggles a line at 1 kHz while its
// wheel should turn and holds it LOW otherwise (see the header note).
volatile bool outer_on = false, inner_on = false;
ISR (TIMER2_COMPA_vect) {
  if (outer_on) PINB = _BV (PB1); else PORTB &= ~_BV (PB1);
  if (inner_on) PINB = _BV (PB0); else PORTB &= ~_BV (PB0);
}
void outer_run ()  { outer_on = true; }
void outer_stop () { outer_on = false; }
void inner_run ()  { inner_on = true; }
void inner_stop () { inner_on = false; }
unsigned long tooth_ms () { return lap_ms / OUTER_TEETH; }
bool zero_now () { return digitalRead (IR_OUTER_0) == HIGH; }
bool outer_moving () { return outer_state != OUT_STOPPED && outer_state != OUT_FAULT; }
void report_outer (int pos) {
  if (pos != outer_pos) { outer_pos = pos; Serial.print (F("outer counter = ")); Serial.println (outer_pos); }
}
// player pressed (or is holding) while stopped: one stop forward
void outer_nudge (unsigned long now) {
  stop_at_ms = now + tooth_ms () + OUTER_PARK_OFFSET_MS;
  stop_pos = outer_pos % OUTER_TEETH + 1;
  outer_state = OUT_NUDGE;
  outer_run ();
}
void outer_fault (const __FlashStringHelper* why) {
  outer_stop ();
  outer_state = OUT_FAULT;
  inner_state = IN_STOPPED;                     // do not leave the inner wheel turning on its own
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

    case OUT_NUDGE:
      outer_run ();
      if (zero_edge) {                          // the mark itself is stop 1: land there, re-synced
        if (OUTER_PARK_OFFSET_MS > 0) delay (OUTER_PARK_OFFSET_MS);
        outer_stop (); outer_state = OUT_STOPPED;
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
// IN_SNAP: stop at the next tooth (control LOW while a
// tooth is in the beam, HIGH otherwise - the original snap rule). IN_NUDGE:
// run off the current tooth, then snap to the next one.
void drive_inner () {
  bool tooth = digitalRead (IR_INNER_COUNTER) == HIGH;
  switch (inner_state) {
    case IN_STOPPED: inner_stop ();  break;
    case IN_SNAP:
      inner_on = !tooth;
      if (tooth) { inner_state = IN_STOPPED; Serial.println (F("inner: stopped")); }
      break;
    case IN_NUDGE:
      inner_run ();
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

// Blocking homing, one wheel at a time (outer to its mark, then inner to 1).
void home_inner () {
  unsigned long t0 = millis ();
  while (digitalRead (IR_INNER_0) != HIGH) {
    inner_on = true;
    if (millis () - t0 > SEEK_TIMEOUT_MS) { Serial.println (F("ERROR inner homing timed out")); break; }
  }
  inner_stop ();
  inner_counter = 1; Serial.println (F("inner counter = 1"));
  prev_inner_counter_state = digitalRead (IR_INNER_COUNTER);
  inner_state = IN_STOPPED;
}
void home_wheels () {
  digitalWrite (STEPPER_O_ENABLE, LOW); digitalWrite (STEPPER_I_ENABLE, LOW);
  outer_stop (); inner_stop ();
  delay (500);
  Serial.println (F(" reset wheels"));
  unsigned long t0 = millis ();
  while (!zero_now ()) {
    outer_on = true;
    if (millis () - t0 > SEEK_TIMEOUT_MS) { Serial.println (F("ERROR homing timed out")); break; }
  }
  outer_stop ();
  outer_pos = 0; report_outer (1);
  zero_prev = zero_now ();
  outer_state = OUT_STOPPED;
  home_inner ();
}

// Blocking wait for the outer home mark to read `want`; false = timed out.
bool wait_mark (bool want, unsigned long t0) {
  while (zero_now () != want) if (millis () - t0 > SEEK_TIMEOUT_MS) return false;
  delay (MARK_SETTLE_MS);
  return true;
}
// The show lap before every clue: the outer wheel turns on its own and stops
// ON the home mark (position 1), then the inner ring returns to 1. From the
// mark it is exactly one lap (and the lap time is measured); from an answer
// it runs on to the mark, plus one more lap if that was under half a turn.
// Returns false on a fault (outer_state = OUT_FAULT, select retries).
bool show_spin () {
  Serial.println (F("show spin"));
  repaint_all ();
  bool from_mark = zero_now ();
  unsigned long t0 = millis ();
  outer_run ();
  if (!wait_mark (false, t0)) { outer_fault (F("never left the home mark")); return false; }
  if (!wait_mark (true, t0))  { outer_fault (F("never saw the home mark")); return false; }
  unsigned long edge = millis () - MARK_SETTLE_MS;
  if (from_mark) lap_ms = edge - t0;
  else if (lap_ms == 0 || edge - t0 < lap_ms / 2) {       // too short to read as a spin: one more full lap
    if (!wait_mark (false, edge)) { outer_fault (F("never left the home mark")); return false; }
    if (!wait_mark (true, edge))  { outer_fault (F("never saw the home mark")); return false; }
    lap_ms = millis () - MARK_SETTLE_MS - edge;
  }
  outer_stop ();
  if (OUTER_LAP_MS) lap_ms = OUTER_LAP_MS;
  Serial.print (F("outer lap = ")); Serial.print (lap_ms); Serial.println (F(" ms  (bake into OUTER_LAP_MS)"));
  outer_pos = 0; report_outer (1);
  zero_prev = zero_now ();
  outer_state = OUT_STOPPED;
  home_inner ();
  return true;
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
// show lap, then the clue: at game start, for every next step and on a select-retry after a fault
void begin_step () {
  if (!show_spin ()) return;
  announce_step ();
}
void start_game () {
  for (int s = 0; s < NUM_SYMBOLS; s++) solved_sym[s] = false;
  for (int p = 2; p <= 7; p++) if (p != OUTER_BUTTON_PIN && p != INNER_BUTTON_PIN) digitalWrite (p, LOW);
  current_step = 0;
  all_leds (0, 0, 0);
  home_wheels ();
  Serial.println (F("game start"));
  phase = PH_PLAY;
  begin_step ();
}
void next_step () {
  current_step++;
  if (current_step >= NUM_STEPS) { finish_game (); return; }
  twinkle (TWINKLE_MS);
  begin_step ();
}
void handle_choose () {
  const Step& st = STEPS[current_step];
  bool right = outer_pos == SYMBOLS[st.answer].pos && inner_counter == st.inner;
  Serial.print (F("choose outer=")); Serial.print (outer_pos); Serial.print (F(" inner=")); Serial.print (inner_counter);
  Serial.println (right ? F(" -> CORRECT") : F(" -> wrong"));
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
    repaint_all ();                             // wheels stay where the players left them
  }
}
void finish_game () {
  Serial.println (F("SOLVED - all symbols"));
  phase = PH_DONE;
  outer_stop (); inner_stop ();
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
  pinMode (INNER_CONTROL, OUTPUT); inner_stop ();
  TCCR2A = _BV (WGM21); TCCR2B = _BV (CS22); OCR2A = 249; TIMSK2 = _BV (OCIE2A);   // Timer2: 1 kHz tick for the control lines
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

    case PH_PLAY: {
      bool idle = (outer_state == OUT_STOPPED && inner_state == IN_STOPPED);   // one wheel at a time
      if (idle && (btn_outer.edge || btn_outer.state)) outer_nudge (now);       // press = one stop, hold = keep stepping
      else if (idle && (btn_inner.edge || btn_inner.state)) inner_state = IN_NUDGE;
      drive_outer (now);
      drive_inner ();
      paint_play (now);
      if (choose_edge) {
        if (outer_state == OUT_FAULT) { Serial.println (F("select -> retry (re-home)")); home_wheels (); begin_step (); }
        else if (idle) handle_choose ();
        else Serial.println (F("choose ignored (a wheel is moving)"));
      }
      break;
    }

    case PH_DONE:
      break;
  }
}
