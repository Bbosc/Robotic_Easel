
#include <TimerThree.h>
#include <TMCStepper.h>

//TCM stepper driver horizontal
#define DIR_PIN_H          34 // Direction
#define STEP_PIN_H         2 // Step pwm pin
#define CS_PIN_H           36 // Chip select
#define EN_PIN_H           38 // Enable

//TCM stepper drivers vertical
#define DIR_PIN_V          40 // Direction
#define STEP_PIN_V         3 // Step pwm pin
#define CS_PIN_V          44 // Chip select
#define EN_PIN_V           46 // Enable

#define SW_MOSI            48 // Software Master Out Slave In (MOSI)
#define SW_MISO            50 // Software Master In Slave Out (MISO)
#define SW_SCK             52 // Software Slave Clock (SCK)
#define R_SENSE 0.11f // Match to your driver

// States for motors
#define UP                    1
#define DOWN                  2
#define RIGHT                 1
#define LEFT                  2
#define STOP                  0

#define DUTY_CYCLE            75.0f

// Command number
#define LEFT_COMMAND          "0"
#define RIGHT_COMMAND         "1"
#define UP_COMMAND            "2"
#define DOWN_COMMAND          "3"
#define END_COURSE_COMMAND    "4"
#define INCREASE_COMMAND      "5"
#define DECREASE_COMMAND      "6"
#define EXTREME_COMMAND       "7"
#define STOP_COMMAND          "8"

#define MAX_PERIOD_PWM        500
#define MIN_PERIOD_PWM        300

// For distance calculation
#define MICROSTEPS            4
#define STEP_PER_TURN         200.0f
#define TIME_PER_LOOP         2       //ms

#define DIST_PER_TURN         2.0f    //mm
#define END_COURSE_DIST       10       //mm
#define EXTREME_DIST          1000    //mm
#define DIST_INCREMENT        50      //mm

// Mode for end_course and extreme implementation
#define NORMAL_MODE           0
#define END_COURSE_MODE       1
#define EXTREME_MODE          2

TMC2130Stepper driver_H(CS_PIN_H, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driver_V(CS_PIN_V, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

int dist_to_travel = 50;
float dist_H = 0;
float dist_V = 0;
int mode = NORMAL_MODE;

int state_V = 0; // 0 stop, 1 up, 2 down
int state_H = 0;  // 0 stop, 1 right, 2 left
int period_pwm_H = MAX_PERIOD_PWM;
int period_pwm_V = MAX_PERIOD_PWM;

bool shaft_H = false;
bool shaft_V = false;

String nom = "Arduino";
String msg;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  //initialize TMC drivers
  pinMode(STEP_PIN_H, OUTPUT);
  pinMode(DIR_PIN_H, OUTPUT);
  pinMode(EN_PIN_H, OUTPUT);

  pinMode(STEP_PIN_V, OUTPUT);
  pinMode(DIR_PIN_V, OUTPUT);
  pinMode(EN_PIN_V, OUTPUT);

  digitalWrite(EN_PIN_H, LOW);      // Enable driver in hardware
  digitalWrite(EN_PIN_V, LOW);      // Enable driver in hardware
  digitalWrite(STEP_PIN_H, LOW);
  digitalWrite(STEP_PIN_V, LOW);

  driver_H.begin();                 // Init CS pins and SW SPI pins
  driver_H.toff(3);                 // Enables driver in software
  driver_H.rms_current(1200);        // Set motor RMS current
  driver_H.microsteps(MICROSTEPS);          // Set microsteps to 1/4th
  //driver_H.pwm_autoscale(true);     // Needed for stealthChop
  //driver_H.en_pwm_mode(true);       // enable stealthChop

  driver_V.begin();                 // Init CS pins and SW SPI pins
  driver_V.toff(3);                 // Enables driver in software
  driver_V.rms_current(1200);        // Set motor RMS current
  driver_V.microsteps(MICROSTEPS);          // Set microsteps to 1/4th
  //driver_V.pwm_autoscale(true);     // Needed for stealthChop
  //driver_V.en_pwm_mode(true);         // enable stealthChop

  digitalWrite(EN_PIN_H, HIGH);      // De-enable driver in hardware
  digitalWrite(EN_PIN_V, HIGH);      // De-enable driver in hardware

  Timer3.initialize(MAX_PERIOD_PWM);
}

//loop
void loop()
{
  update_dist();
  check_for_stop();
  readSerialPort();
  
  if (msg != "") {
     sendData();
     command_process();
     set_motor_V();
     set_motor_H();
  }
  delay(TIME_PER_LOOP);
}

// decell V
void decceleration_V(){
  while(period_pwm_V < MAX_PERIOD_PWM){
    period_pwm_V +=4;
    Timer3.pwm(STEP_PIN_V, (DUTY_CYCLE/ 100.0) * 1023.0, period_pwm_V);
    delay(TIME_PER_LOOP);
  }
}

//decell H
void decceleration_H(){
  while(period_pwm_H < MAX_PERIOD_PWM){
    period_pwm_H +=3;  
    Timer3.pwm(STEP_PIN_H, (DUTY_CYCLE/ 100.0) * 1023.0, period_pwm_H);
    delay(TIME_PER_LOOP);
  }
}

void acceleration_H(){
  while(period_pwm_H > MIN_PERIOD_PWM){
    period_pwm_H --;
    Timer3.pwm(STEP_PIN_H, (DUTY_CYCLE/ 100.0) * 1023.0, period_pwm_H);
    delay(TIME_PER_LOOP);
  }
}

void acceleration_V(){
  while(period_pwm_V > MIN_PERIOD_PWM){
    period_pwm_V --;
    Timer3.pwm(STEP_PIN_V, (DUTY_CYCLE/ 100.0) * 1023.0, period_pwm_V);
    delay(TIME_PER_LOOP);
  }
}

// Process command
void command_process(){
  if (msg.indexOf(END_COURSE_COMMAND) != -1){
    end_of_course();
   }

   else if(msg.indexOf(STOP_COMMAND) != -1){
    stop_move_V();
    stop_move_H();
   }
   
  else if (msg.indexOf(LEFT_COMMAND) != -1){
      stop_move_H();
      state_H = LEFT;
   }
   
   else if (msg.indexOf(RIGHT_COMMAND) != -1){
    stop_move_H();
    state_H = RIGHT;
   }
   
   else if (msg.indexOf(UP_COMMAND) != -1){
    stop_move_V();
    state_V = UP;
   }
   
   else if (msg.indexOf(DOWN_COMMAND) != -1){
    stop_move_V();
    state_V = DOWN;
   }
   
   else if (msg.indexOf(INCREASE_COMMAND) != -1){
      dist_to_travel += DIST_INCREMENT;
   }
   
   else if (msg.indexOf(DECREASE_COMMAND) != -1 && dist_to_travel > 50){
      dist_to_travel -= DIST_INCREMENT;
   }

   if (msg.indexOf(EXTREME_COMMAND) != -1){
      mode = EXTREME_MODE;
   }
}

// check for stop depending on the mode
void check_for_stop(){
  switch(mode){
    case(NORMAL_MODE):
      check_dist(dist_to_travel);
      break;
      
    case(EXTREME_MODE):
      check_dist(EXTREME_DIST);
      break;
      
    case(END_COURSE_MODE):
      check_dist(END_COURSE_DIST);
      break;
  }
}

// check limit distance and stop if needed
void check_dist(int max_dist){
  if (dist_V > max_dist) {
      stop_move_V();
    }
    if (dist_H > max_dist) {
      stop_move_H();
    }
}

//Stop vertical motor
void stop_move_V(){
  decceleration_V();
  state_V = STOP;
  dist_V = 0;
  digitalWrite(EN_PIN_V, HIGH);
  if(state_H == STOP)
    mode = NORMAL_MODE;

  period_pwm_V = MAX_PERIOD_PWM;
}

//Stop horizontal motor
void stop_move_H(){
  decceleration_H();
  state_H = STOP;
  dist_H= 0;
  digitalWrite(EN_PIN_H, HIGH);
  if(state_V == STOP)
    mode = NORMAL_MODE;
    
  period_pwm_H = MAX_PERIOD_PWM;
}

//update the distance travelled by the cart
void update_dist(){
  if (state_V != STOP) {
    float time_per_turn = MICROSTEPS*STEP_PER_TURN*period_pwm_V/1000;
    dist_V += DIST_PER_TURN*TIME_PER_LOOP/time_per_turn;
  }

  if (state_H != STOP) {
    float time_per_turn = MICROSTEPS*STEP_PER_TURN*period_pwm_H/1000;
    dist_H += DIST_PER_TURN*TIME_PER_LOOP/time_per_turn;
  }
}

//end of course, reverse direction and switch mode
void end_of_course(){
  if(state_V == DOWN){
    stop_move_V();
    state_V = UP;
  }
        
  else if(state_V == UP){
    stop_move_V();
    state_V = DOWN;
  }
    
  if(state_H == RIGHT){
    stop_move_H();
    state_H = LEFT;
  }
    
  else if(state_H == LEFT){
    stop_move_H();
    state_H = RIGHT;
  }
  mode = END_COURSE_MODE;
}

//start vertical motor and set direction
void set_motor_V()
{
  if(state_V != STOP){
    digitalWrite(EN_PIN_V, LOW);      // Enable driver in hardware
    if (state_V == UP)
      driver_V.shaft(true);
      
    else if (state_V == DOWN)
      driver_V.shaft(false);
    
    acceleration_V();
  }
}

//start horizontal motor and set direction
void set_motor_H()
{   
   if(state_H != STOP) {
    digitalWrite(EN_PIN_H, LOW);      // Enable driver in hardware
    if (state_H == RIGHT)
      driver_H.shaft(true); 
   
    else if (state_H == LEFT)
      driver_H.shaft(false); 
      
      
      acceleration_H();
  }
}


void readSerialPort() {
 msg = "";
 if (Serial.available()) {
   delay(10);
   while (Serial.available() > 0) {
     msg += (char)Serial.read();
   }
   Serial.flush();
 }
}

void sendData() {
 //write data
 Serial.print(nom);
 Serial.print(" received : ");
 Serial.print(msg); 
}
