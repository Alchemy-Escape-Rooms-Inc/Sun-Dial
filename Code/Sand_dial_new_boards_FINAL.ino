  //  working controller code 2/25/25  -  FINAL UPLOAD VERSION
  //
  //  2026-09-11  GUIDED MODE (v2)
  //    The outer (symbol) wheel is driven by this board, not by the players.
  //    It parks on the first symbol; players dial the inner number and press
  //    CHOOSE. Correct -> green, symbol LED, HOUSE pulse to the bridge, then the
  //    outer wheel spins itself to the next symbol. Wrong -> red, stays put.
  //    Sequence = the five combos below in order (bottle, skull, turtle,
  //    coconut, trident). The outer rotate button must be DISCONNECTED.
  //
  //    TRIGGER (optional): ESP32 bridge GPIO 17 -> Nano A6 (10k pulldown to
  //    GND). A HIGH pulse >= 100 ms = "start over at step 1" (GameStart or
  //    PUZZLE_RESET). Without the wire the dial still parks on step 1 at
  //    power-up and after the 5th solve.
  //
  //    v2.1 (2026-09-12): the outer wheel now STOPS ON EVERY TOOTH while
  //    travelling (250 ms dwell) exactly like the original snap-to-tooth
  //    behaviour, and the guided driver counts the teeth itself on every
  //    loop pass. v2.0 let the wheel run flat out between teeth and only
  //    sampled the beam every 35 ms, so fast teeth were missed, the count
  //    never reached the target and the wheel spun forever.
  //
  //    SERIAL -> MQTT: Nano D1 (TX) -> divider (2k2 top / 3k3 bottom) -> ESP32
  //    GPIO 18. The bridge parses "outer counter = N" / "inner counter = N" and
  //    publishes MermaidsTale/SunDial/Outer and /Inner. Set GUIDED_MODE 0
  //    to get the original free-order puzzle back.


                                        #include <Wire.h>
                                        #include <Adafruit_PWMServoDriver.h>




                                        #define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



                                        const int LED_num = 5;
                                        const int board_num = 2;


                                        const int colors [20][3] = {{0,0,255},{0,255,0},{55,0,127}, {0,255,255}, {51,51,255}, {255,0,255}, {255,255, 255},  {255,0,0} };
                                        const int number_of_colors = 8;


                                        int red [3]= {0,0,255};         //0,0,255
                                        int green [3] = {0,255,0};
                                        int purple [3] = {55,0,127};
                                        int yellow [3] = {0,255,255};
                                        int cyan [3] = {51,51,255};
                                        int white [3] = {255,255,255};

                                        int pink [3] = {255,0,255};
                                        int blue[3] = {255,0,0};    //255,0,0
                                        int off [3] = {0,0,0};
                                        int color [3] = {0,255,0};


                                        Adafruit_PWMServoDriver pwmBoard[] = {Adafruit_PWMServoDriver(0x40), Adafruit_PWMServoDriver(0x41) };



//#define IR_OUTER_0 A0
//#define IR_OUTER_COUNTER A1
//#define IR_INNER_0 A2
//#define IR_INNER_COUNTER A3

#define IR_OUTER_COUNTER A3         // this is for new code upload to device
#define IR_OUTER_0 A2
#define IR_INNER_COUNTER A1
#define IR_INNER_0 A0


#define STEPPER_O_ENABLE 11
#define STEPPER_I_ENABLE 10
#define CHOOSE 12
#define STEPPERS_ENABLE 13

#define OUTER_CONTROL 9
#define INNER_CONTROL 8

#define HOUSE_1 2
#define HOUSE_2 3
#define HOUSE_3 4
#define HOUSE_4 5
#define HOUSE_5 6
#define HOUSE_6 7


bool outer_0_state;
bool outer_counter_state;
bool inner_0_state;
bool inner_counter_state;

bool rotate_outer_state;
bool rotate_inner_state;
bool choose_state;
bool break_from_lightshow = 0;
bool outer_counter_flag = 0;
bool inner_counter_flag = 0;


unsigned long outer_counter_time;
unsigned long outer_counter_interval = 750;
unsigned long inner_counter_interval = 500;
unsigned long inner_counter_time;
unsigned long inner_loop_timer;
unsigned long outer_loop_timer;

int outer_counter = 0;
int previous_outer_counter_state = 0;
int inner_counter = 0;
int previous_inner_counter_state = 0;
int previous_outer_counter;
int previous_inner_counter;
int inner_counter_checker;
int counter = 0;


int outer_choice;
int inner_choice;

int correct_outer_1 = 2;   //bottle
int correct_inner_1 = 3;

int correct_outer_2 = 5;   //skull
int correct_inner_2 = 4;

int correct_outer_3 = 6;   // turtle
int correct_inner_3 = 7;

int correct_outer_4 = 8;  //coconut
int correct_inner_4 = 9;

int correct_outer_5 = 9;   //trident
int correct_inner_5 = 1;

int correct_blink_delay = 250;
int blink_times = 3;
int previous_condition_1;
int previous_condition_2;
int previous_condition_3;
int previous_condition_4;
int previous_condition_5;



int I_c = 0;
int O_c = 0;

// ---------------- GUIDED MODE ----------------
#define GUIDED_MODE 1
#define TRIGGER_ENABLED 0              // OFF: the select button is the start, as always. Set 1 only if ESP32 GPIO17 -> A6 (+10k pulldown) is wired.
#define TRIGGER_PIN A6                 // analog-only pin: read with analogRead, needs external 10k pulldown
const int TRIGGER_THRESHOLD = 400;     // 3.3V from the ESP32 reads ~675 of 1023
const int NUM_STEPS = 5;
const int step_outer[NUM_STEPS] = {2, 5, 6, 8, 9};      // symbol positions, same as correct_outer_1..5
const int step_inner[NUM_STEPS] = {3, 4, 7, 9, 1};      // numbers,          same as correct_inner_1..5
const int step_house[NUM_STEPS] = {HOUSE_1, HOUSE_2, HOUSE_3, HOUSE_4, HOUSE_5};
const int step_led_board[NUM_STEPS] = {0, 0, 1, 1, 1};  // green symbol LED address, copied from the original branches
const int step_led_addr[NUM_STEPS]  = {4, 7, 1, 7, 10};
const char* const step_name[NUM_STEPS] = {"bottle", "skull", "turtle", "coconut", "trident"};
const unsigned long PARK_TIMEOUT_MS = 90000;            // outer wheel must reach its target within this or we stop the motor

int current_step = 0;                  // 0..4 = waiting for that combo, 5 = all done
bool trigger_pending = 0;
bool trigger_last = 0;
unsigned long trigger_high_since = 0;
unsigned long park_started_ms = 0;
bool park_timed_out = 0;
bool was_parked = 0;

// outer-wheel driver state (guided mode)
#define OD_DRIVE  0   // spinning, waiting for the next tooth to break the beam
#define OD_DWELL  1   // stopped on a tooth, counted it, short pause
#define OD_LEAVE  2   // spinning off the tooth until the beam clears
#define OD_PARKED 3   // sitting on the target tooth
int outer_drive_state = OD_DRIVE;
bool outer_beam_prev = 0;
unsigned long outer_dwell_start = 0;
const unsigned long OUTER_DWELL_MS = 250;

void setup() {



pinMode (IR_OUTER_0, INPUT);
pinMode (IR_OUTER_COUNTER, INPUT);
pinMode (IR_INNER_0, INPUT);
pinMode (IR_INNER_COUNTER, INPUT);

pinMode (STEPPER_O_ENABLE, OUTPUT);
pinMode (STEPPER_I_ENABLE, OUTPUT);
pinMode (CHOOSE, INPUT_PULLUP);

pinMode (OUTER_CONTROL, OUTPUT);
digitalWrite (OUTER_CONTROL, LOW);
pinMode (INNER_CONTROL, OUTPUT);
digitalWrite (INNER_CONTROL, LOW);


pinMode (HOUSE_1, OUTPUT);
pinMode (HOUSE_2, OUTPUT);
pinMode (HOUSE_3, OUTPUT);
pinMode (HOUSE_4, OUTPUT);
pinMode (HOUSE_5, OUTPUT);
pinMode (HOUSE_6, OUTPUT);


for (int i = 0; i < board_num ; i++){
pwmBoard [i].begin();
pwmBoard [i].setOscillatorFrequency(27000000);
pwmBoard [i].setPWMFreq(SERVO_FREQ); }

 for(int board = 0 ; board < board_num ; board ++){      //shut downt the lights
  for (int j = 0; j< LED_num; j++){
        Serial.println (j );
        Serial.println (" off");
      for (int k = 0; k < 3 ; k++) {
       pwmBoard [board].setPWM (k + (j*3), 0, (off [k]));
       }}}

Serial.begin (115200);

digitalWrite (HOUSE_1, LOW);
digitalWrite (HOUSE_2, LOW);
digitalWrite (HOUSE_3, LOW);
digitalWrite (HOUSE_4, LOW);
digitalWrite (HOUSE_5, LOW);
digitalWrite (HOUSE_6, LOW);


disable_inner_stepper ();
disable_outer_stepper ();

delay (3000);

turn_bezel_white ();

reset_the_wheels ();
sync_counters_to_home ();


delay (2000);

turn_bezel_off();

disable_inner_stepper ();
disable_outer_stepper ();


check_buttons ();

light_show ();

enable_inner_stepper ();
enable_outer_stepper ();


turn_bezel_white ();

if (GUIDED_MODE) guided_begin ();

}


void loop() {


check_buttons ();


if (!GUIDED_MODE && millis () - outer_loop_timer > 35){   // this timer code is required so excessive counts are not registered, once a count is registered
       count_outer_ticks ();                           // the count function can not be reentered until the timer reaches its value.
       outer_loop_timer = millis ();}                  // (guided mode: drive_outer_to_target() counts the outer teeth itself)


if (millis () - inner_loop_timer > 105){
       count_inner_ticks ();
       inner_loop_timer = millis ();}



check_trigger ();
if (GUIDED_MODE && trigger_pending){ trigger_pending = 0; guided_restart (); }

check_choose_button ();


if (GUIDED_MODE) {
  drive_outer_to_target ();                                                          // this board owns the outer wheel now
} else {
if(outer_counter_state == HIGH){digitalWrite (OUTER_CONTROL, LOW);}                   // counters return a HIGH value when the beam is inturrupted
      //if beam is interupted write outer OUTER_CONTROL pin low, which signals motor control code on Arduino 2, to stop outer spin
if(outer_counter_state == LOW){digitalWrite (OUTER_CONTROL, HIGH);}
           // if beam isnt interupted write OUTER_CONTROL pin high, signals to motor control code to spin outer
}
if(inner_counter_state == HIGH){digitalWrite (INNER_CONTROL, LOW); }
     //if beam is interupted write INNER_CONTROL pin low which signals the motor control that it can stop inner spin
if(inner_counter_state == LOW){digitalWrite (INNER_CONTROL, HIGH);}
     // if beam isnt interupted write INNER_CONTROL pin low, signals to motor controller to spin inner


}




void light_show () {


while (choose_state == 1){                              // SLOW ROUTINE V

for (int l = 0; l < number_of_colors ; l++){       // choose color from array
  for(int board = 0 ; board < board_num ; board ++){   //choose board

    for (int j = 0; j< LED_num; j++){                //choose led 5 per board

        for (int k = 0; k < 3 ; k++) {
   pwmBoard [board].setPWM (k + (j*3), 0, (colors [l][k]*16)) ;                  //  turns on first led
  }



  smartDelay (600);
  if(break_from_lightshow == 1){ break_from_lightshow == 0 ; return; }
  }
  }


}


 for(int board = 0 ; board < board_num ; board ++){
  for (int j = 0; j< LED_num; j++){

  for (int k = 0; k < 3 ; k++) {
       pwmBoard [board].setPWM (k + (j*3), 0, (off [k]));
       }
     delay (250);
       }
 }
 smartDelay (5000);
if(break_from_lightshow == 1){ break_from_lightshow == 0 ; return; }





for (int l = 0; l < number_of_colors ; l++){       // choose color from array             FAST ROUTINE V
  for(int board = 0 ; board < board_num ; board ++){   //choose board
    for (int j = 0; j< LED_num; j++){                //choose led 5 per board

        for (int k = 0; k < 3 ; k++) {
   pwmBoard [board].setPWM (k + (j*3), 0, (colors [l][k]*16)) ;                  //  turns on first led
  }



  smartDelay (125);
  if(break_from_lightshow == 1){ break_from_lightshow == 0 ; return; }
  }
  }

  digitalWrite(HOUSE_6, LOW);
}


//unsigned long off_timer = millis ();

 for(int board = 0 ; board < board_num ; board ++){
  for (int j = 0; j< LED_num; j++){

  for (int k = 0; k < 3 ; k++) {
       pwmBoard [board].setPWM (k + (j*3), 0, (off [k]));
       }
     delay (150);
       }
 }
 smartDelay (4000);
if(break_from_lightshow == 1){ break_from_lightshow == 0 ; return; }



}

}



void check_buttons (){

outer_0_state = digitalRead (IR_OUTER_0) ;                //checking the pins
outer_counter_state = digitalRead (IR_OUTER_COUNTER) ;

inner_0_state = digitalRead (IR_INNER_0);
inner_counter_state = digitalRead (IR_INNER_COUNTER) ;

choose_state = digitalRead (CHOOSE) ;


}

void disable_outer_stepper (){
digitalWrite (STEPPER_O_ENABLE, HIGH);

}

void disable_inner_stepper (){
digitalWrite (STEPPER_I_ENABLE, HIGH);

}

void enable_outer_stepper (){
digitalWrite (STEPPER_O_ENABLE, LOW);
}

void enable_inner_stepper (){
digitalWrite (STEPPER_I_ENABLE, LOW);
}


void rotate_outer (){
digitalWrite (OUTER_CONTROL, HIGH);   //HIGH level of pin to motor controller arduino rotates wheel
}

void stop_outer (){                               // LOW level stops wheels
digitalWrite (OUTER_CONTROL, LOW);
}

void rotate_inner (){
digitalWrite (INNER_CONTROL, HIGH);        //HIGH level of pin to motor controller arduino rotates wheel
}

void stop_inner (){
digitalWrite (INNER_CONTROL, LOW);             // LOW level stops wheels
}

void count_outer_ticks () {

outer_counter_state = digitalRead (IR_OUTER_COUNTER) ;


      if (outer_counter_state == 1 && previous_outer_counter_state == 0 ){

       outer_counter = outer_counter + 1;

       outer_0_state = digitalRead (IR_OUTER_0);
         if(outer_0_state == 1 ){outer_counter = 1;}
       previous_outer_counter_state = 1;


        Serial.print ("outer counter = ");
         Serial.println ( outer_counter);

         Serial.println ( " ");

         }


        previous_outer_counter_state = outer_counter_state;
}



void count_inner_ticks (){



inner_counter_state = digitalRead (IR_INNER_COUNTER) ;


      if (inner_counter_state == 1 && previous_inner_counter_state == 0 ){

       inner_counter = inner_counter + 1;

       inner_0_state = digitalRead (IR_INNER_0);
         if(inner_0_state == 1 ){inner_counter = 1;}
       previous_inner_counter_state = 1;


        Serial.print ("inner counter = ");
         Serial.println ( inner_counter);

         Serial.println ( " ");

         }


        previous_inner_counter_state = inner_counter_state;
}





void check_choose_button (){

if (GUIDED_MODE) { guided_choose (); return; }

bool correct = 0;


      if(choose_state == 0){                     //choose button pushed
       outer_choice = outer_counter;
       inner_choice = inner_counter;



       if (outer_choice == correct_outer_1 && inner_choice == correct_inner_1){
          digitalWrite (HOUSE_1, HIGH);



          turn_bezel_green ();
          delay (2000);
          turn_bezel_off ();

          pwmBoard [0].setPWM (4 , 0, 4080);        // first board, address 4, full power  (green) bottle  //3 ADDRESSES required for one LED  RGB  all correct choices are green
          previous_condition_1 = 1;                // so we use the second address to light it  eg.   board addresses are 0 1 2     3 4 5     6 7 8     9 10 11     12 13 14
          correct = 1;                             // 1, 4, 7 , 10 , 13 are all green as green is , (0, 255, 0)   ---  4080 is  (255 * 16) which is the max power (brightness for an LED)
          digitalWrite (HOUSE_1, LOW);            //  there are 10 symbols which require 30 addressses each PWM board (0 , 1 )can accomodate 16 addresses  0-15  - there is an address on each
          turn_bezel_white ();                   // board that is not used.   Changing the  location of a correct character must be changed in this section and below in
          turn_former_greens_on ();              // "turn_former_greens_on" to ensure only the proper addresses are turned on
          delay (2000);}


               if (outer_choice == correct_outer_2 && inner_choice == correct_inner_2){
                  digitalWrite (HOUSE_2, HIGH);

                  turn_bezel_green ();
                  delay (2000);
                  turn_bezel_off ();

                  pwmBoard [0].setPWM (7 , 0, 4080);  // first board, address 7, full power  (green) skull
                  previous_condition_2 = 1;
                  correct = 1;
                  digitalWrite (HOUSE_2, LOW);
                  turn_bezel_white ();
                  turn_former_greens_on ();}



                   if (outer_choice == correct_outer_3 && inner_choice == correct_inner_3){
                      digitalWrite (HOUSE_3, HIGH);


                      turn_bezel_green ();
                      delay (2000);
                      turn_bezel_off ();

                      pwmBoard [1].setPWM (1 , 0, 4080);              // second board adress 1 , full power  (green) turtle
                      previous_condition_3 = 1;
                      correct = 1;
                      digitalWrite (HOUSE_3, LOW);
                      turn_bezel_white ();
                      turn_former_greens_on ();}


                           if (outer_choice == correct_outer_4 && inner_choice == correct_inner_4){
                              digitalWrite (HOUSE_4, HIGH);


                              turn_bezel_green ();
                              delay (2000);
                              turn_bezel_off ();

                              pwmBoard [1].setPWM (7 , 0, 4080);       // second board, address 7, full power  (green) coconut
                              previous_condition_4 = 1;
                              correct = 1;
                              digitalWrite (HOUSE_4, LOW);
                              turn_bezel_white ();
                              turn_former_greens_on ();
                              }


                               if (outer_choice == correct_outer_5 && inner_choice == correct_inner_5){
                                  digitalWrite (HOUSE_5, HIGH);
                                  Serial.println("HOUSE_5 HIGH (trident) - D6");

                                  turn_bezel_green ();
                                  delay (2000);

                                  turn_bezel_off ();

                                  pwmBoard [1].setPWM (10 , 0, 4080);     // second board, address 10 full power  (green)  trident
                                  previous_condition_5 = 1;
                                  correct = 1;
                                  digitalWrite (HOUSE_5, LOW);
                                  Serial.println("HOUSE_5 LOW (trident) - D6");
                                  turn_bezel_white ();
                                  turn_former_greens_on ();}


          if (correct == 0){

          digitalWrite(HOUSE_6, HIGH);

          turn_bezel_off ();

          turn_bezel_red () ;
          Serial.println ( "turn bezel red ");
          delay (3000);   /////////////////////////////////////

          digitalWrite(HOUSE_6, LOW);
          turn_bezel_off ();
          turn_bezel_white ();
          turn_former_greens_on ();}}

                                //******************************

              if((previous_condition_1 + previous_condition_2 + previous_condition_3 + previous_condition_4 + previous_condition_5 == 5)) {
                    finish_sequence ();
                  }
}


// Everything that used to run after the 5th correct combo: celebrate, home the
// wheels, attract light show until CHOOSE (or a trigger pulse), then re-arm.
void finish_sequence (){
                    delay (3000);

                   blink_bezel_green ();


                   turn_bezel_green ();
                          delay (5000);

                    turn_bezel_off ();
                    delay (1000);


                  reset_the_wheels ();
                  disable_inner_stepper ();
                  disable_outer_stepper ();

                  previous_condition_1 = 0;
                  previous_condition_2 = 0;
                  previous_condition_3 = 0;
                  previous_condition_4 = 0;
                  previous_condition_5 = 0;
                  digitalWrite (HOUSE_1, LOW);
                  digitalWrite (HOUSE_2, LOW);
                  digitalWrite (HOUSE_3, LOW);
                  digitalWrite (HOUSE_4, LOW);
                  digitalWrite (HOUSE_5, LOW);
                  digitalWrite (HOUSE_6, LOW);

                  break_from_lightshow = 0;
                  light_show ();
                  inner_counter = 0;
                  outer_counter = 0;
                  previous_inner_counter_state = 0;
                  previous_outer_counter_state = 0;


                  enable_inner_stepper ();
                  enable_outer_stepper ();


                  turn_bezel_white ();

                  if (GUIDED_MODE) guided_begin ();
}


void reset_the_wheels (){
 enable_inner_stepper ();
 enable_outer_stepper ();
  //Serial.println (" reset wheels");
  stop_outer ();
  stop_inner ();
  delay (500);

Serial.println (" reset wheels");
while ( outer_0_state == 0 || inner_0_state == 0 ){                          /// pin low motor stop
  check_buttons ();
  if (outer_0_state == 1 && inner_0_state == 1){ digitalWrite (OUTER_CONTROL, LOW); delay (80); digitalWrite (INNER_CONTROL, LOW);  break;}   // low stops the motors
           // both stopped
  if (outer_0_state == 0 && inner_0_state == 1 ){digitalWrite (OUTER_CONTROL, HIGH); delay (80); digitalWrite (INNER_CONTROL, LOW); }    // outer spins inner stopped


  if (outer_0_state == 1 && inner_0_state == 0 ){digitalWrite (OUTER_CONTROL, LOW);  digitalWrite (INNER_CONTROL, HIGH);         // outer stopped inner spins

   }
  if (outer_0_state == 0 && inner_0_state == 0 ){digitalWrite (OUTER_CONTROL, HIGH);digitalWrite (INNER_CONTROL, HIGH);      //

  }     // both spinning



}
}

void smartDelay (unsigned long delay_interval) {
     unsigned long delay_time = millis ();

     while (millis () -  delay_time < delay_interval){
       check_buttons ();
       check_trigger ();
         if (choose_state == 0 || trigger_pending){


           for(int board = 0 ; board < board_num ; board ++){
                for (int j = 0; j< LED_num; j++){
              Serial.println (j );
              Serial.println (" off");
                for (int k = 0; k < 3 ; k++) {
                     pwmBoard [board].setPWM (k + (j*3), 0, (off [k]));

       }}}

     break_from_lightshow = 1;
     delay (500);
     return;}





     }

}
void turn_bezel_off (){

   for(int board = 0 ; board < board_num ; board ++){
  for (int j = 0; j< LED_num; j++){

  for (int k = 0; k < 3 ; k++) {
       pwmBoard [board].setPWM (k + (j*3), 0, (off [k]));
       }

       }
}
}

void blink_bezel_green (){

for (int i = 0 ; i < 5 ; i++){
turn_bezel_green ();

delay (150);

turn_bezel_off ();
delay (150);
}
}



void blink_bezel_red (){

for (int i = 0 ; i < 5 ; i++){
turn_bezel_red ();

delay (150);

turn_bezel_off ();
delay (150);
}}


void turn_bezel_white (){

 for(int board = 0 ; board < board_num ; board ++){
  for (int j = 0; j< LED_num; j++){

  for (int k = 0; k < 3 ; k++) {
       pwmBoard [board].setPWM (k + (j*3), 0, (white [k] * 16));
       }

       }

}



}
void turn_bezel_red (){

 for(int board = 0 ; board < board_num ; board ++){
  for (int j = 0; j< LED_num; j++){

  for (int k = 0; k < 3 ; k++) {
       pwmBoard [board].setPWM (k + (j*3), 0, (red [k] * 16));
       }

       }

} }

void turn_bezel_green (){
  for(int board = 0 ; board < board_num ; board ++){
  for (int j = 0; j< LED_num; j++){

  for (int k = 0; k < 3 ; k++) {
       pwmBoard [board].setPWM (k + (j*3), 0, (green [k]*16));
       }

       }

}



}

void turn_former_greens_on () {

              if (previous_condition_1 == 1){pwmBoard [0].setPWM (3 , 0, 0); pwmBoard [0].setPWM (5, 0, 0); } //pwmBoard [0].setPWM (1 , 0, 4080);}    // turns on former greens back on  // note addresses on bezel
              delay (100);
              if (previous_condition_2 == 1){pwmBoard [0].setPWM (6 , 0, 0); pwmBoard [0].setPWM (8 , 0, 0); }  // pwmBoard [0].setPWM (7 , 0, 4080);}
              delay (100);
              if (previous_condition_3 == 1){pwmBoard [1].setPWM (0 , 0, 0); pwmBoard [1].setPWM (2 , 0, 0);}    // pwmBoard [0].setPWM (13 , 0, 4080);}
              delay (100);
              if (previous_condition_4 == 1){pwmBoard [1].setPWM (6 , 0, 0);  pwmBoard [1].setPWM (8, 0, 0);}   // pwmBoard [1].setPWM (4 , 0, 4080);}
              delay (100);
              if (previous_condition_5 == 1){pwmBoard [1].setPWM (9 , 0, 0); pwmBoard [1].setPWM (11 , 0, 0);}        //pwmBoard [1].setPWM (10 , 0, 4080);}

              }


// ======================= GUIDED MODE =======================

// Home tooth is position 1 by this sketch's own rule (a tick with the zero beam
// broken sets the counter to 1). After reset_the_wheels() both wheels sit there.
void sync_counters_to_home (){
  outer_counter = 1;  inner_counter = 1;
  previous_outer_counter_state = digitalRead (IR_OUTER_COUNTER);
  previous_inner_counter_state = digitalRead (IR_INNER_COUNTER);
}

int guided_target (){
  if (current_step >= NUM_STEPS) return step_outer[NUM_STEPS - 1];
  return step_outer[current_step];
}

// Parked = counter says we're on the target AND the beam is broken (sitting on a tooth).
bool outer_parked (){
  return outer_drive_state == OD_PARKED;
}

// Called whenever the target changes or the counters were just re-synced.
void outer_drive_rearm (){
  outer_beam_prev = (digitalRead (IR_OUTER_COUNTER) == HIGH);
  if (outer_counter == guided_target () && outer_beam_prev) outer_drive_state = OD_PARKED;
  else outer_drive_state = OD_DRIVE;
  park_started_ms = millis ();
  park_timed_out = 0;
  was_parked = 0;
}

void announce_step (){
  Serial.print ("step=");   Serial.print (current_step + 1);
  Serial.print (" symbol="); Serial.print (current_step < NUM_STEPS ? step_name[current_step] : "done");
  Serial.print (" target_outer="); Serial.print (guided_target ());
  Serial.print (" need_inner="); Serial.println (current_step < NUM_STEPS ? step_inner[current_step] : 0);
}

void guided_begin (){
  current_step = 0;
  Serial.println ("guided begin");
  outer_drive_rearm ();
  announce_step ();
}

// Trigger pulse: wipe greens, home both wheels, back to step 1.
void guided_restart (){
  Serial.println ("trigger -> restart");
  digitalWrite (OUTER_CONTROL, LOW);
  digitalWrite (INNER_CONTROL, LOW);
  previous_condition_1 = 0; previous_condition_2 = 0; previous_condition_3 = 0;
  previous_condition_4 = 0; previous_condition_5 = 0;
  digitalWrite (HOUSE_1, LOW); digitalWrite (HOUSE_2, LOW); digitalWrite (HOUSE_3, LOW);
  digitalWrite (HOUSE_4, LOW); digitalWrite (HOUSE_5, LOW); digitalWrite (HOUSE_6, LOW);
  turn_bezel_off ();
  reset_the_wheels ();
  sync_counters_to_home ();
  enable_inner_stepper ();
  enable_outer_stepper ();
  turn_bezel_white ();
  if (GUIDED_MODE) guided_begin ();
}

// Non-blocking: called every loop pass. Spins the outer wheel until it sits on
// the target tooth, then holds it there. If it gets nudged off, it goes around
// again (the zero mark re-syncs the count every lap).
// Non-blocking, called every loop pass. Moves the outer wheel ONE TOOTH AT A
// TIME (stop on the tooth, count it, pause, move off) until it sits on the
// target tooth. Same stop-on-every-tooth rhythm the original code had, so the
// beam is never missed at speed. If the wheel gets knocked off the target it
// goes around again (the zero mark re-syncs the count every lap).
void drive_outer_to_target (){
  if (park_timed_out) { digitalWrite (OUTER_CONTROL, LOW); return; }
  bool beam = (digitalRead (IR_OUTER_COUNTER) == HIGH);   // HIGH = tooth in the beam

  switch (outer_drive_state) {

    case OD_DRIVE:
      digitalWrite (OUTER_CONTROL, HIGH);
      if (beam && !outer_beam_prev) {                     // a tooth just arrived
        digitalWrite (OUTER_CONTROL, LOW);
        outer_counter = outer_counter + 1;
        if (digitalRead (IR_OUTER_0) == HIGH) outer_counter = 1;   // zero mark = position 1
        Serial.print ("outer counter = "); Serial.println (outer_counter);
        outer_dwell_start = millis ();
        outer_drive_state = OD_DWELL;
      } else if (millis () - park_started_ms > PARK_TIMEOUT_MS) {
        park_timed_out = 1;
        digitalWrite (OUTER_CONTROL, LOW);
        Serial.println ("ERROR outer wheel never reached target - motor stopped (select to retry)");
      }
      break;

    case OD_DWELL:
      digitalWrite (OUTER_CONTROL, LOW);
      if (outer_counter == guided_target ()) {
        outer_drive_state = OD_PARKED;
        was_parked = 1;
        Serial.print ("parked outer="); Serial.println (outer_counter);
      } else if (millis () - outer_dwell_start >= OUTER_DWELL_MS) {
        outer_drive_state = OD_LEAVE;
      }
      break;

    case OD_LEAVE:
      digitalWrite (OUTER_CONTROL, HIGH);
      if (!beam) outer_drive_state = OD_DRIVE;            // cleared the tooth, look for the next one
      break;

    case OD_PARKED:
      digitalWrite (OUTER_CONTROL, LOW);
      if (!beam) {                                        // knocked off the tooth: go around again
        Serial.println ("outer nudged off target - re-parking");
        park_started_ms = millis ();
        was_parked = 0;
        outer_drive_state = OD_DRIVE;
      }
      break;
  }
  outer_beam_prev = beam;
}

// CHOOSE handling for guided mode: only the current step's combo counts.
void guided_choose (){
  if (choose_state != 0) return;                       // not pressed
  if (current_step >= NUM_STEPS) return;               // finished, waiting for restart
  if (park_timed_out) {                                // motor was stopped by the safety timeout: select retries
    Serial.println ("select -> retry outer travel");
    outer_drive_rearm ();
    delay (500);
    return;
  }
  if (!outer_parked ()) {                              // wheel still travelling - ignore the press
    Serial.println ("choose ignored (outer moving)");
    delay (300);
    return;
  }
  outer_choice = outer_counter;
  inner_choice = inner_counter;
  Serial.print ("choose outer="); Serial.print (outer_choice);
  Serial.print (" inner=");       Serial.println (inner_choice);

  if (outer_choice == step_outer[current_step] && inner_choice == step_inner[current_step]) {
    int s = current_step;
    digitalWrite (step_house[s], HIGH);                // 2s pulse to the bridge -> MQTT symbol true
    Serial.print ("correct step="); Serial.print (s + 1); Serial.print (" "); Serial.println (step_name[s]);
    turn_bezel_green ();
    delay (2000);
    turn_bezel_off ();
    pwmBoard [step_led_board[s]].setPWM (step_led_addr[s], 0, 4080);   // symbol LED green
    if (s == 0) previous_condition_1 = 1;
    if (s == 1) previous_condition_2 = 1;
    if (s == 2) previous_condition_3 = 1;
    if (s == 3) previous_condition_4 = 1;
    if (s == 4) previous_condition_5 = 1;
    digitalWrite (step_house[s], LOW);
    turn_bezel_white ();
    turn_former_greens_on ();
    current_step++;
    outer_drive_rearm ();
    announce_step ();
    if (current_step >= NUM_STEPS) { finish_sequence (); }
    else { delay (1000); }                             // let go of CHOOSE before the wheel moves
  } else {
    digitalWrite (HOUSE_6, HIGH);                      // 3s pulse -> MQTT Wrong true
    Serial.println ("wrong");
    turn_bezel_off ();
    turn_bezel_red ();
    delay (3000);
    digitalWrite (HOUSE_6, LOW);
    turn_bezel_off ();
    turn_bezel_white ();
    turn_former_greens_on ();
  }
}

// Trigger input: A6 is analog-only, so poll it. Needs to read HIGH for 100 ms
// straight (rejects noise on a long wire); one event per pulse.
void check_trigger (){
  if (!TRIGGER_ENABLED) return;
  bool hi = analogRead (TRIGGER_PIN) > TRIGGER_THRESHOLD;
  if (hi) {
    if (!trigger_last) trigger_high_since = millis ();
    else if (!trigger_pending && trigger_high_since != 0 && millis () - trigger_high_since >= 100) {
      trigger_pending = 1;
      trigger_high_since = 0;                          // one event per pulse
      Serial.println ("trigger pulse");
    }
  }
  trigger_last = hi;
}
