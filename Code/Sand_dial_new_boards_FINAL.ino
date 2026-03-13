  //  working controller code 2/25/25  -  FINAL UPLOAD VERSION
                                        
                             
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

int correct_outer_2 = 3;   //seahorse
int correct_inner_2 = 6;

int correct_outer_3 = 6;   // coconut
int correct_inner_3 = 7;

int correct_outer_4 = 8;  //trident
int correct_inner_4 = 9;

int correct_outer_5 = 9;   //skull
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


delay (2000);

turn_bezel_off();

disable_inner_stepper ();
disable_outer_stepper ();


check_buttons ();

light_show ();

enable_inner_stepper ();
enable_outer_stepper ();


turn_bezel_white ();



}


void loop() {


check_buttons ();
 
 
if (millis () - outer_loop_timer > 35){                // this timer code is required so excessive counts are not registered, once a count is registered
       count_outer_ticks ();                           // the count function can not be reentered until the timer reaches its value.
       outer_loop_timer = millis ();}


if (millis () - inner_loop_timer > 105){
       count_inner_ticks ();
       inner_loop_timer = millis ();}



check_choose_button ();


if(outer_counter_state == HIGH){digitalWrite (OUTER_CONTROL, LOW);}                   // counters return a HIGH value when the beam is inturrupted 
      //if beam is interupted write outer OUTER_CONTROL pin low, which signals motor control code on Arduino 2, to stop outer spin 
if(outer_counter_state == LOW){digitalWrite (OUTER_CONTROL, HIGH);}
           // if beam isnt interupted write OUTER_CONTROL pin high, signals to motor control code to spin outer 
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

  digitalWrite(HOUSE_6, HIGH);
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
                            
                  pwmBoard [0].setPWM (7 , 0, 4080);  // first board, address 7, full power  (green) seahorse
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
                      
                      pwmBoard [1].setPWM (1 , 0, 4080);              // second board adress 1 , full power  (green) coconut
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
                                                                        
                              pwmBoard [1].setPWM (7 , 0, 4080);       // second board, address 7, full power  (green) trident
                              previous_condition_4 = 1;
                              correct = 1;
                              digitalWrite (HOUSE_4, LOW);
                              turn_bezel_white (); 
                              turn_former_greens_on (); 
                              }         

       
                               if (outer_choice == correct_outer_5 && inner_choice == correct_inner_5){
                                  digitalWrite (HOUSE_5, HIGH);
                                  
                                  
                                  turn_bezel_green (); 
                                  delay (2000);
                        
                                  turn_bezel_off ();
                                  
                                  pwmBoard [1].setPWM (10 , 0, 4080);     // second board, address 10 full power  (green)  skull
                                  previous_condition_5 = 1;
                                  correct = 1;
                                  digitalWrite (HOUSE_5, LOW);
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
                  digitalWrite (HOUSE_5, LOW);
                  
                  break_from_lightshow = 0;
                  light_show ();
                  inner_counter = 0;
                  outer_counter = 0;
                  previous_inner_counter_state = 0;
                  previous_outer_counter_state = 0;

                  
                  enable_inner_stepper ();
                  enable_outer_stepper ();

                  
                  turn_bezel_white (); 
                  loop ();
                  
                  }
}
          

void reset_the_wheels (){
 enable_inner_stepper ();
 enable_outer_stepper ();
  //Serial.println (" reset wheels");
  stop_outer ();
  stop_inner ();
  delay (500);

while ( outer_0_state == 0 || inner_0_state == 0 ){                          /// pin low motor stop
  Serial.println (" reset wheels");
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
         if (choose_state == 0){
           
           
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
