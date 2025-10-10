
     //working code - motor controllers 2/25/25  -  UPLOAD VERSION
                                        

 



#define OUTER_CONTROL 11   // comes in from other arudino board high wheel spins, low wheel stops
#define INNER_CONTROL 10   // comes in from other arudino board high wheel spins, low wheel stops

#define ROTATE_INNER 4   // rotate inner button
#define ROTATE_OUTER 5  // rotate outer button

#define OUTER_STEPPER_DIR A0
#define OUTER_STEPPER_STEP A1
#define INNER_STEPPER_DIR A2
#define INNER_STEPPER_STEP A3

bool outer_control_state;
bool inner_control_state;
bool rotate_outer_state ;
bool rotate_inner_state ;
bool enable_status;


int pulse_delay_outer = 15;    //controls the speed of the outer wheel  the lower the faster
int pulse_delay_inner = 45;    // controls the speed of the inner wheel
int combo_case;







void setup() {

pinMode (OUTER_CONTROL, INPUT_PULLUP);
pinMode (INNER_CONTROL, INPUT_PULLUP);
pinMode (ROTATE_OUTER, INPUT_PULLUP);
pinMode (ROTATE_INNER, INPUT_PULLUP);



pinMode (OUTER_STEPPER_DIR, OUTPUT); 
pinMode (OUTER_STEPPER_STEP, OUTPUT); 
pinMode (INNER_STEPPER_DIR, OUTPUT);
pinMode (INNER_STEPPER_STEP, OUTPUT); 


  digitalWrite (INNER_STEPPER_DIR, HIGH);



Serial.begin (115200);


}


void loop() {


check_pins ();


if (rotate_outer_state == LOW || outer_control_state == HIGH || (rotate_outer_state == LOW && outer_control_state == LOW) ) {      //any combo rotates the outer wheel
    run_outer_stepper ();
             }  
if (rotate_outer_state == HIGH && outer_control_state == LOW){     // wheel stopped
    stop_outer_stepper ();
    
}

  

check_pins ();


if (rotate_inner_state == LOW || inner_control_state == HIGH || (rotate_inner_state == LOW && inner_control_state == LOW)) {       //any combo rotates inner wheel
                run_inner_stepper ();
             }

if (rotate_inner_state == HIGH && inner_control_state == LOW){  // wheel stopped
stop_inner_stepper ();}
             
             

}


 
void check_pins (){

rotate_outer_state = digitalRead (ROTATE_OUTER); 
rotate_inner_state = digitalRead (ROTATE_INNER); 
inner_control_state = digitalRead (INNER_CONTROL); 
outer_control_state = digitalRead (OUTER_CONTROL); 

}

void run_outer_stepper () {
int multiplier ;
if (inner_control_state == LOW) {multiplier = 1;}   //multiplier slows down outer wheel when inner wheel stops

  digitalWrite (OUTER_STEPPER_STEP, HIGH);
  delayMicroseconds (pulse_delay_outer * multiplier);
  digitalWrite (OUTER_STEPPER_STEP, LOW);
  delayMicroseconds (pulse_delay_outer * multiplier);

}

void stop_outer_stepper () {

  digitalWrite (OUTER_STEPPER_STEP, LOW);
  digitalWrite (OUTER_STEPPER_STEP, LOW);
  digitalWrite (OUTER_STEPPER_STEP, LOW);
  digitalWrite (OUTER_STEPPER_STEP, LOW);


}


void run_inner_stepper () {

  digitalWrite (INNER_STEPPER_STEP, HIGH);
  delayMicroseconds (pulse_delay_inner);
  digitalWrite (INNER_STEPPER_STEP, LOW);
  delayMicroseconds (pulse_delay_inner);

}

void stop_inner_stepper () {

  digitalWrite (INNER_STEPPER_STEP, LOW);
  digitalWrite (INNER_STEPPER_STEP, LOW);
  digitalWrite (INNER_STEPPER_STEP, LOW);
  digitalWrite (INNER_STEPPER_STEP, LOW);
 
}



