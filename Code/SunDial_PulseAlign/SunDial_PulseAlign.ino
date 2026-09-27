//  SunDial_PulseAlign  -  controller Nano firmware v3.0.0  (2026-09-27)
//
//  THE GAME ("pulse, align, count, choose"):
//    1. After the select press (or the bridge trigger pulse) the dial homes,
//       measures its lap, then ONE symbol light pulses. Everything else is dark.
//    2. Players push the OUTER button. Each push (or a held button) moves the
//       symbol wheel ONE stop. When the pulsing symbol reaches the pointer the
//       light goes STEADY: "you've got it". The wheel is now held there -
//       further outer pushes just wink the light and do not move it.
//    3. Players dial the count on the INNER ring (unchanged: their button on the
//       motor board spins it, this board snaps it to the next number).
//    4. CHOOSE. Right = every light green 2 s, that symbol stays green, HOUSE
//       pulse to the bridge (MQTT SunDial/<Symbol> = true for M3 + Evalee), a
//       short twinkle, then the NEXT symbol pulses and the hold releases.
//       Wrong (wrong number, or choose before the wheel is aligned) = red 2 s +
//       HOUSE_6 pulse (SunDial/Wrong).
//    5. Three symbols in a fixed order = solved: green celebration, wheels home,
//       attract light show until the next select.
//
//  WIRING CHANGE vs the guided v2.x build (REQUIRED):
//    The players' OUTER rotate button must come to THIS board, not the motor
//    board. Move its lead from motor Nano D5 to controller Nano D3
//    (OUTER_BUTTON_PIN). D3 used to be HOUSE_2 (the "Crab" pulse to the
//    bridge) - DISCONNECT that D3 -> level-shifter wire, otherwise every button
//    release looks like a Crab solve to M3. The button's other lead stays on
//    GND (internal pull-up, pressed = LOW). If D3 is unreachable, set
//    OUTER_BUTTON_ANALOG 1 and use A7 (or A6) with a 10k pull-up to 5 V.
//    Motor board firmware unchanged: with nothing on its D5 it only turns the
//    outer wheel when this board raises OUTER_CONTROL.
//
//  POSITION: the outer tooth sensor is dead, so the wheel position is kept by
//  TIME (v2.2 method): one lap measured from two home-mark edges, one stop =
//  lap / OUTER_TEETH. Every home-mark edge re-syncs to position 1, so drift
//  never lasts more than a lap. If the inner ring moves while the outer steps
//  (the motor board slows the outer then) the position is marked unsure and
//  the wheel quietly re-centres itself via the home mark once the buttons are
//  quiet.
//
//  SERIAL (115200, D1 TX -> bridge divider -> ESP32 GPIO18): keeps printing
//  "outer counter = N" / "inner counter = N" so MermaidsTale/SunDial/Outer and
//  /Inner still work. USB RX bench commands: o = outer push, c = choose,
//  r = restart, s = status.
//
//  Build: arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 --output-dir build Code/SunDial_PulseAlign

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define FW_VERSION "3.0.0"

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
#define HOUSE_2           3       // (was crab) NOW THE OUTER BUTTON INPUT, see below
#define HOUSE_3           4       // turtle  -> bridge GPIO6  -> SunDial/Turtle   (unused in v3)
#define HOUSE_4           5       // coconut -> bridge GPIO7  -> SunDial/Coconut
#define HOUSE_5           6       // trident -> bridge GPIO15 -> SunDial/Trident
#define HOUSE_6           7       // wrong   -> bridge GPIO16 -> SunDial/Wrong

#ifndef OUTER_BUTTON_PIN
#define OUTER_BUTTON_PIN  3       // players' outer rotate button (moved off the motor board)
#endif
#ifndef OUTER_BUTTON_ANALOG
#define OUTER_BUTTON_ANALOG 0     // 1 = pin is A6/A7 (analog-only, external 10k pull-up to 5 V)
#endif

#define TRIGGER_ENABLED   0       // 1 only if bridge GPIO17 -> A6 (+10k pull-down) is wired
#define TRIGGER_PIN       A6
#define TRIGGER_THRESHOLD 400

// ---------------------------------------------------------------- tuning
#define OUTER_TEETH            10      // symbol stops on the outer ring
#define OUTER_LAP_MS           0       // 0 = measure at every game start; or bake in the "outer lap = N ms" serial value
#define OUTER_PARK_OFFSET_MS   0       // + stops a little later after each home-mark edge (tune if position 1 sits off-centre)
#define OUTER_STEP_DWELL_MS    150     // pause between stops while the button is held
#define BUTTON_DEBOUNCE_MS     30
#define RECENTER_QUIET_MS      1500    // buttons quiet this long before an unsure wheel re-centres itself
#define SEEK_TIMEOUT_MS        90000UL // a home-mark search longer than this stops the motor (select retries)
#define PULSE_PERIOD_MS        1400    // breathing period of the target symbol
#define ATTRACT_STEP_MS        600
#define TWINKLE_MS             1500

// ---------------------------------------------------------------- the three symbols, in order
struct Step {
  const char* name;
  uint8_t outer;      // wheel stop that puts the symbol under the pointer
  uint8_t inner;      // the count the players must dial
  uint8_t house;      // HOUSE pin pulsed on a correct choose
  uint8_t led_board;  // PCA9685 board (0 = 0x40, 1 = 0x41)
  uint8_t led;        // LED index on that board (0..4), 3 channels each: blue, green, red
};
const Step STEPS[] = {
  { "bottle",  2, 3, HOUSE_1, 0, 1 },
  { "coconut", 8, 9, HOUSE_4, 1, 2 },
  { "trident", 9, 1, HOUSE_5, 1, 3 },
};
const int NUM_STEPS = sizeof (STEPS) / sizeof (STEPS[0]);

// Other symbol lights (decor only for now). Known: skull (0,2) and turtle (1,0).
const int LED_BOARDS = 2;
const int LEDS_PER_BOARD = 5;
Adafruit_PWMServoDriver pwmBoard[] = { Adafruit_PWMServoDriver (0x40), Adafruit_PWMServoDriver (0x41) };

// ---------------------------------------------------------------- state
enum Phase { PH_ATTRACT, PH_HOMING, PH_PLAY, PH_DONE };
Phase phase = PH_ATTRACT;

enum OuterState { OUT_IDLE, OUT_STEP, OUT_SEEK_LEAVE, OUT_SEEK_ZERO, OUT_SEEK_RUN, OUT_FAULT };
OuterState outer_state = OUT_IDLE;

int  current_step = 0;              // index into STEPS
bool solved[NUM_STEPS];
bool locked = false;                // wheel sits on the target symbol
bool outer_unsure = false;          // inner moved during a step -> position may be off
int  outer_pos = 1;                 // believed outer position 1..OUTER_TEETH
unsigned long lap_ms = OUTER_LAP_MS;
unsigned long step_started_ms = 0;
unsigned long seek_started_ms = 0;
unsigned long cal_edge_ms = 0;      // first home-mark edge of a calibration lap
int  seek_target = 1;               // position to stop on after the home mark
bool seek_measure_lap = false;      // this seek also measures the lap
bool zero_prev = false;
int  inner_snapshot = 0;

int  inner_counter = 1;
int  prev_inner_counter_state = 0;
unsigned long inner_gate_ms = 0;

bool outer_btn = false, outer_btn_prev = false;
unsigned long outer_btn_raw_since = 0;
bool outer_btn_raw_prev = false;
bool outer_press_edge = false;
unsigned long last_button_activity_ms = 0;
unsigned long dwell_until_ms = 0;

bool choose_prev = false;
bool choose_edge = false;
bool trigger_pending = false;
bool trigger_last = false;
unsigned long trigger_high_since = 0;

unsigned long attract_ms = 0;
int attract_color = 0;
const uint8_t ATTRACT_COLORS[][3] = { {0,0,255},{0,255,0},{55,0,127},{0,255,255},{51,51,255},{255,0,255},{255,255,255},{255,0,0} };
const int NUM_ATTRACT_COLORS = 8;

// ---------------------------------------------------------------- LEDs
// Each LED uses three PCA channels: idx*3+0 = blue, +1 = green, +2 = red.
void set_led (int board, int idx, uint8_t r, uint8_t g, uint8_t b) {
  pwmBoard[board].setPWM (idx * 3 + 0, 0, (uint16_t)b * 16);
  pwmBoard[board].setPWM (idx * 3 + 1, 0, (uint16_t)g * 16);
  pwmBoard[board].setPWM (idx * 3 + 2, 0, (uint16_t)r * 16);
}
void all_leds (uint8_t r, uint8_t g, uint8_t b) {
  for (int bd = 0; bd < LED_BOARDS; bd++)
    for (int i = 0; i < LEDS_PER_BOARD; i++) set_led (bd, i, r, g, b);
}
void solved_leds_green () {
  for (int s = 0; s < NUM_STEPS; s++)
    if (solved[s]) set_led (STEPS[s].led_board, STEPS[s].led, 0, 255, 0);
}
// Normal play picture: solved symbols green, target pulsing (or steady when
// aligned), everything else dark. Called every loop pass in PH_PLAY.
void paint_play (unsigned long now) {
  static unsigned long last_paint = 0;
  if (now - last_paint < 25) return;
  last_paint = now;
  const Step& st = STEPS[current_step];
  uint8_t level;
  if (locked) level = 255;
  else {
    float ph = (float)(now % PULSE_PERIOD_MS) / (float)PULSE_PERIOD_MS;   // 0..1
    float tri = ph < 0.5f ? ph * 2.0f : (1.0f - ph) * 2.0f;                  // triangle 0..1..0
    level = (uint8_t)(20 + tri * tri * 235);                                 // eased, never fully off
  }
  // gold "sun": full red, 2/3 green, no blue
  set_led (st.led_board, st.led, level, (uint8_t)((uint16_t)level * 2 / 3), 0);
}
void repaint_all () {           // dark ring + solved greens (target is painted by paint_play)
  all_leds (0, 0, 0);
  solved_leds_green ();
}
void wink_target () {           // acknowledgement when the outer button is pushed while aligned
  const Step& st = STEPS[current_step];
  for (int i = 0; i < 2; i++) {
    set_led (st.led_board, st.led, 0, 0, 0); delay (70);
    set_led (st.led_board, st.led, 255, 170, 0); delay (70);
  }
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

// ---------------------------------------------------------------- motors
void outer_run ()  { digitalWrite (OUTER_CONTROL, HIGH); }
void outer_stop () { digitalWrite (OUTER_CONTROL, LOW); }
bool outer_moving () { return outer_state == OUT_STEP || outer_state == OUT_SEEK_LEAVE || outer_state == OUT_SEEK_ZERO || outer_state == OUT_SEEK_RUN; }
unsigned long tooth_ms () { return lap_ms / OUTER_TEETH; }
bool zero_now () { return digitalRead (IR_OUTER_0) == HIGH; }

void report_outer () { Serial.print (F("outer counter = ")); Serial.println (outer_pos); }

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
  outer_pos = 1; inner_counter = 1;
  prev_inner_counter_state = digitalRead (IR_INNER_COUNTER);
  zero_prev = zero_now ();
  outer_state = OUT_IDLE; locked = false; outer_unsure = false;
  report_outer ();
  Serial.println (F("inner counter = 1"));
}

// Start a home-mark search that ends on `target` (and measures the lap if asked).
void begin_seek (int target, bool measure) {
  seek_target = target; seek_measure_lap = measure;
  cal_edge_ms = 0; seek_started_ms = millis ();
  inner_snapshot = inner_counter;
  outer_state = OUT_SEEK_LEAVE;
  Serial.print (F("outer: seek home, target ")); Serial.print (target);
  Serial.println (measure ? " (measuring lap)" : "");
}
void outer_fault (const char* why) {
  outer_stop ();
  outer_state = OUT_FAULT;
  Serial.print (F("ERROR outer wheel ")); Serial.print (why); Serial.println (F(" - motor stopped (select to retry)"));
}
void arrived (int pos) {
  outer_stop ();
  outer_pos = pos;
  outer_state = OUT_IDLE;
  dwell_until_ms = millis () + OUTER_STEP_DWELL_MS;
  report_outer ();
  if (phase == PH_PLAY && !locked && outer_pos == STEPS[current_step].outer) {
    locked = true;
    Serial.print (F("aligned on ")); Serial.println (STEPS[current_step].name);
  }
}

// Non-blocking outer wheel driver, every loop pass.
void drive_outer (unsigned long now) {
  bool zero = zero_now ();
  bool zero_edge = zero && !zero_prev;
  zero_prev = zero;
  bool inner_moved = (inner_counter != inner_snapshot);

  switch (outer_state) {
    case OUT_IDLE:
    case OUT_FAULT:
      outer_stop ();
      break;

    case OUT_STEP:                              // one stop forward
      outer_run ();
      if (inner_moved && !outer_unsure) { outer_unsure = true; Serial.println (F("outer: inner moved during a step - position unsure")); }
      if (zero_edge) {                          // home mark: re-sync, this step ends at position 1
        if (OUTER_PARK_OFFSET_MS > 0) delay (OUTER_PARK_OFFSET_MS);
        arrived (1);
        outer_unsure = false;
      } else if (now - step_started_ms >= tooth_ms ()) {
        arrived (outer_pos % OUTER_TEETH + 1);
      }
      break;

    case OUT_SEEK_LEAVE:                        // drive off the mark first (we may be sitting on it)
      outer_run ();
      if (!zero) outer_state = OUT_SEEK_ZERO;
      else if (now - seek_started_ms > SEEK_TIMEOUT_MS) outer_fault ("never left the home mark");
      break;

    case OUT_SEEK_ZERO:
      outer_run ();
      if (cal_edge_ms != 0 && inner_moved) {    // calibration lap spoiled by the inner ring
        Serial.println (F("outer: inner moved during the lap measure - measuring again"));
        cal_edge_ms = 0; inner_snapshot = inner_counter;
      }
      if (zero_edge) {
        if (seek_measure_lap && lap_ms == 0) {
          if (cal_edge_ms == 0) { cal_edge_ms = now; inner_snapshot = inner_counter; Serial.println (F("outer: measuring one lap")); break; }
          lap_ms = now - cal_edge_ms; cal_edge_ms = 0;
          Serial.print (F("outer lap = ")); Serial.print (lap_ms); Serial.println (F(" ms  (bake into OUTER_LAP_MS)"));
        }
        outer_pos = 1; report_outer ();
        inner_snapshot = inner_counter;
        unsigned long run = (unsigned long)(seek_target - 1) * tooth_ms () + OUTER_PARK_OFFSET_MS;
        if (run == 0) { arrived (1); outer_unsure = false; }
        else { step_started_ms = now; outer_state = OUT_SEEK_RUN; }
      } else if (now - seek_started_ms > SEEK_TIMEOUT_MS) outer_fault ("never saw the home mark");
      break;

    case OUT_SEEK_RUN: {                        // timed run from the mark to the target stop
      outer_run ();
      unsigned long run = (unsigned long)(seek_target - 1) * tooth_ms () + OUTER_PARK_OFFSET_MS;
      if (inner_moved) {                        // timing spoiled: go round again
        Serial.println (F("outer: inner moved during travel - redoing from the home mark"));
        seek_started_ms = now; inner_snapshot = inner_counter; outer_state = OUT_SEEK_LEAVE;
      } else if (now - step_started_ms >= run) {
        arrived (seek_target); outer_unsure = false;
      }
      break; }
  }
}

// ---------------------------------------------------------------- inputs
bool read_outer_button_raw () {
#if OUTER_BUTTON_ANALOG
  return analogRead (OUTER_BUTTON_PIN) < TRIGGER_THRESHOLD;    // pulled up, pressed = low
#else
  return digitalRead (OUTER_BUTTON_PIN) == LOW;
#endif
}
void read_inputs (unsigned long now) {
  // outer button, debounced
  bool raw = read_outer_button_raw ();
  if (raw != outer_btn_raw_prev) { outer_btn_raw_prev = raw; outer_btn_raw_since = now; }
  outer_btn_prev = outer_btn;
  if (now - outer_btn_raw_since >= BUTTON_DEBOUNCE_MS) outer_btn = raw;
  outer_press_edge = outer_btn && !outer_btn_prev;
  if (outer_btn) last_button_activity_ms = now;
  if (outer_press_edge) Serial.println (F("outer button: PRESSED"));

  // choose, edge only
  bool ch = digitalRead (CHOOSE) == LOW;
  choose_edge = ch && !choose_prev;
  choose_prev = ch;
  if (choose_edge) Serial.println (F("choose pin: PRESSED"));

  // inner ring ticks (105 ms gate like the original)
  if (now - inner_gate_ms > 105) {
    inner_gate_ms = now;
    int st = digitalRead (IR_INNER_COUNTER);
    if (st == HIGH && prev_inner_counter_state == LOW) {
      inner_counter++;
      if (digitalRead (IR_INNER_0) == HIGH) inner_counter = 1;
      Serial.print (F("inner counter = ")); Serial.println (inner_counter);
      last_button_activity_ms = now;
    }
    prev_inner_counter_state = st;
  }

  // bridge trigger pulse on A6 (optional)
#if TRIGGER_ENABLED
  bool hi = analogRead (TRIGGER_PIN) > TRIGGER_THRESHOLD;
  if (hi) {
    if (!trigger_last) trigger_high_since = now;
    else if (!trigger_pending && trigger_high_since != 0 && now - trigger_high_since >= 100) {
      trigger_pending = true; trigger_high_since = 0; Serial.println (F("trigger pulse"));
    }
  }
  trigger_last = hi;
#endif

  // USB bench commands
  while (Serial.available ()) {
    char c = Serial.read ();
    if (c == 'o') { outer_press_edge = true; outer_btn = true; Serial.println (F("bench: outer push")); }
    else if (c == 'c') { choose_edge = true; Serial.println (F("bench: choose")); }
    else if (c == 'r') { trigger_pending = true; Serial.println (F("bench: restart")); }
    else if (c == 's') { print_status (); }
  }
}
void print_status () {
  Serial.print (F("status: phase=")); Serial.print ((int)phase);
  Serial.print (F(" step=")); Serial.print (current_step + 1); Serial.print (F("/")); Serial.print (NUM_STEPS);
  Serial.print (F(" symbol=")); Serial.print (current_step < NUM_STEPS ? STEPS[current_step].name : "done");
  Serial.print (F(" outer=")); Serial.print (outer_pos);
  Serial.print (F(" inner=")); Serial.print (inner_counter);
  Serial.print (F(" locked=")); Serial.print (locked);
  Serial.print (F(" unsure=")); Serial.print (outer_unsure);
  Serial.print (F(" lap_ms=")); Serial.println (lap_ms);
}

// Inner ring snap-to-tooth (unchanged behaviour), held off while the outer moves.
void drive_inner () {
  if (outer_moving ()) { digitalWrite (INNER_CONTROL, LOW); return; }
  digitalWrite (INNER_CONTROL, digitalRead (IR_INNER_COUNTER) == HIGH ? LOW : HIGH);
}

// ---------------------------------------------------------------- game flow
void announce_step () {
  if (current_step >= NUM_STEPS) { Serial.println (F("step=done")); return; }
  Serial.print (F("step=")); Serial.print (current_step + 1);
  Serial.print (F(" symbol=")); Serial.print (STEPS[current_step].name);
  Serial.print (F(" target_outer=")); Serial.print (STEPS[current_step].outer);
  Serial.print (F(" need_inner=")); Serial.println (STEPS[current_step].inner);
}
void start_game () {
  for (int s = 0; s < NUM_STEPS; s++) solved[s] = false;
  for (int p = 2; p <= 7; p++) if (p != OUTER_BUTTON_PIN) digitalWrite (p, LOW);
  current_step = 0; locked = false; outer_unsure = false;
  all_leds (0, 0, 0);
  home_wheels ();
  phase = PH_HOMING;
  begin_seek (1, OUTER_LAP_MS == 0);      // measure the lap (unless baked in), end on position 1
  Serial.println (F("game start"));
}
void enter_play () {
  phase = PH_PLAY;
  repaint_all ();
  announce_step ();
  if (outer_pos == STEPS[current_step].outer) { locked = true; Serial.println (F("aligned already")); }
}
void handle_choose () {
  const Step& st = STEPS[current_step];
  if (locked && inner_counter == st.inner) {
    Serial.print (F("correct step=")); Serial.print (current_step + 1); Serial.print (F(" ")); Serial.println (st.name);
    digitalWrite (st.house, HIGH);          // 2 s pulse -> bridge -> MQTT SunDial/<Symbol> true
    all_leds (0, 255, 0);
    delay (2000);
    digitalWrite (st.house, LOW);
    solved[current_step] = true;
    current_step++;
    locked = false;
    if (current_step >= NUM_STEPS) { finish_game (); return; }
    twinkle (TWINKLE_MS);                   // decor between steps
    announce_step ();
    if (outer_pos == STEPS[current_step].outer) { locked = true; Serial.println (F("aligned already")); }
  } else {
    Serial.println (locked ? "wrong number" : "wrong: wheel not on the symbol");
    digitalWrite (HOUSE_6, HIGH);           // -> MQTT SunDial/Wrong true
    all_leds (255, 0, 0);
    delay (2000);
    digitalWrite (HOUSE_6, LOW);
    repaint_all ();
  }
}
void finish_game () {
  Serial.println (F("SOLVED - all symbols"));
  phase = PH_DONE;
  for (int i = 0; i < 5; i++) { all_leds (0, 255, 0); delay (150); all_leds (0, 0, 0); delay (150); }
  all_leds (0, 255, 0);
  delay (5000);
  all_leds (0, 0, 0);
  home_wheels ();
  digitalWrite (STEPPER_O_ENABLE, HIGH); digitalWrite (STEPPER_I_ENABLE, HIGH);   // let the wheels rest
  phase = PH_ATTRACT;
  attract_ms = 0;
  Serial.println (F("attract (select to start)"));
}
void run_attract (unsigned long now) {
  if (now - attract_ms >= ATTRACT_STEP_MS) {
    attract_ms = now;
    const uint8_t* c = ATTRACT_COLORS[attract_color];
    all_leds (c[2], c[1], c[0]);           // table is stored blue,green,red like the original sketch
    attract_color = (attract_color + 1) % NUM_ATTRACT_COLORS;
  }
}

// ---------------------------------------------------------------- setup / loop
void setup () {
  Serial.begin (115200);
  pinMode (IR_OUTER_0, INPUT); pinMode (IR_INNER_0, INPUT); pinMode (IR_INNER_COUNTER, INPUT);
  pinMode (STEPPER_O_ENABLE, OUTPUT); pinMode (STEPPER_I_ENABLE, OUTPUT);
  pinMode (OUTER_CONTROL, OUTPUT); outer_stop ();
  pinMode (INNER_CONTROL, OUTPUT); digitalWrite (INNER_CONTROL, LOW);
  pinMode (CHOOSE, INPUT_PULLUP);
  for (int p = 2; p <= 7; p++) { if (p == OUTER_BUTTON_PIN) continue; pinMode (p, OUTPUT); digitalWrite (p, LOW); }
#if OUTER_BUTTON_ANALOG
  // analog-only pin: nothing to configure, external pull-up required
#else
  pinMode (OUTER_BUTTON_PIN, INPUT_PULLUP);
#endif
  delay (5);
  Serial.print (F("SunDial PulseAlign v")); Serial.println (FW_VERSION);
  if (digitalRead (CHOOSE) == LOW) Serial.println (F("WARNING: select pin reads PRESSED at boot - stuck button or short on D12"));
  if (read_outer_button_raw ()) Serial.println (F("WARNING: outer button reads PRESSED at boot - check the D3 wire / pull-up"));

  for (int i = 0; i < LED_BOARDS; i++) { pwmBoard[i].begin (); pwmBoard[i].setOscillatorFrequency (27000000); pwmBoard[i].setPWMFreq (50); }
  all_leds (0, 0, 0);
  digitalWrite (STEPPER_O_ENABLE, HIGH); digitalWrite (STEPPER_I_ENABLE, HIGH);
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
      if (choose_edge) { start_game (); }
      break;

    case PH_HOMING:                              // measuring the lap / settling on position 1
      drive_outer (now);
      drive_inner ();
      if (outer_state == OUT_IDLE) enter_play ();
      if (outer_state == OUT_FAULT && choose_edge) { Serial.println (F("select -> retry")); begin_seek (1, OUTER_LAP_MS == 0); }
      break;

    case PH_PLAY: {
      // outer button: one stop per push, repeats while held, ignored while aligned
      if (outer_state == OUT_FAULT) {
        if (choose_edge) { Serial.println (F("select -> retry")); begin_seek (outer_pos, false); choose_edge = false; }
      } else if (outer_state == OUT_IDLE) {
        if (locked) {
          if (outer_press_edge) { Serial.println (F("outer push ignored (aligned)")); wink_target (); }
        } else if (outer_btn && now >= dwell_until_ms) {      // dwell = short pause between stops while held
          inner_snapshot = inner_counter;
          step_started_ms = now;
          outer_state = OUT_STEP;
        } else if (outer_unsure && now - last_button_activity_ms >= RECENTER_QUIET_MS) {
          Serial.println (F("outer: re-centring via the home mark"));
          begin_seek (outer_pos, false);
        }
      }
      drive_outer (now);
      drive_inner ();
      paint_play (now);
      if (choose_edge && !outer_moving ()) handle_choose ();
      break; }

    case PH_DONE:
      break;
  }
}
