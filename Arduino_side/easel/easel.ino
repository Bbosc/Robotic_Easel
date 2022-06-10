#include <Wire.h>
#include <VL53L0X.h>
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

#define UP                    1
#define DOWN                  2
#define RIGHT                 1
#define LEFT                  2
#define STOP                  0

#define DUTY_CYCLE            75.0f
#define PERIOD_PWM            75    // decreasing period <--> decreasing rotation time

#define LEFT_COMMAND          "0"
#define RIGHT_COMMAND         "1"
#define UP_COMMAND            "2"
#define DOWN_COMMAND          "3"
#define END_COURSE_COMMAND    "4"
#define INCREASE_COMMAND      "5"
#define DECREASE_COMMAND      "6"
#define EXTREME_COMMAND       "7"
#define STOP_COMMAND          "8"


#define LOOP_PER_TURN         13.0f
#define DIST_PER_TURN         2.0f
#define END_COURSE_DIST       5
#define EXTREME_DIST          1000
#define DIST_INCREMENT        50

#define NORMAL_MODE           0
#define END_COURSE_MODE       1
#define EXTREME_MODE          2

TMC2130Stepper driver_H(CS_PIN_H, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driver_V(CS_PIN_V, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

VL53L0X sensor_L;
VL53L0X sensor_R;
VL53L0X sensor_U;
VL53L0X sensor_D;
VL53L0X sensor_RTH;
VL53L0X sensor_H;
VL53L0X sensor_V;

int state_V = 0; // 0 stop, 1 up, 2 down
int state_H = 0;  // 0 stop, 1 right, 2 left

bool shaft_H = false;
bool shaft_V = false;

String nom = "Arduino";
String msg;

void setup()
{
  Serial.begin(9600);
  delay(2000);
  Wire.begin();

  //initialize TMC drivers
  pinMode(STEP_PIN_H, OUTPUT);
  pinMode(DIR_PIN_H, OUTPUT);
  pinMode(EN_PIN_H, OUTPUT);

  pinMode(STEP_PIN_V, OUTPUT);
  pinMode(DIR_PIN_V, OUTPUT);
  pinMode(EN_PIN_V, OUTPUT);

  digitalWrite(EN_PIN_H, LOW);      // Enable driver in hardware
  digitalWrite(EN_PIN_V, LOW);      // Enable driver in hardware

  driver_H.begin();                 // Init CS pins and SW SPI pins
  driver_H.toff(5);                 // Enables driver in software
  driver_H.rms_current(1200);        // Set motor RMS current
  driver_H.microsteps(16);          // Set microsteps to 1/4th
  driver_H.pwm_autoscale(true);     // Needed for stealthChop
  driver_H.en_pwm_mode(true);       // enable stealthChop

  driver_V.begin();                 // Init CS pins and SW SPI pins
  driver_V.toff(5);                 // Enables driver in software
  driver_V.rms_current(1200);        // Set motor RMS current
  driver_V.microsteps(16);          // Set microsteps to 1/4th
  driver_V.pwm_autoscale(true);     // Needed for stealthChop
  driver_V.en_pwm_mode(true);         // enable stealthChop

  digitalWrite(EN_PIN_H, HIGH);      // De-enable driver in hardware
  digitalWrite(EN_PIN_V, HIGH);      // De-enable driver in hardware

  Timer3.initialize(40);
  Timer3.pwm(STEP_PIN_V, (DUTY_CYCLE / 100.0) * 1023.0, PERIOD_PWM);
  Timer3.pwm(STEP_PIN_H, (DUTY_CYCLE/ 100.0) * 1023.0, PERIOD_PWM);
}

int prev_dist = 50;
int dist_to_travel = 50;
float dist_H = 0;
float dist_V = 0;
int mode = NORMAL_MODE;

void loop()
{
  update_dist();
  check_for_stop();
  
  readSerialPort();
  //sendData();
  
  if (msg != "") {
     sendData();
     command_process();
     set_motor_V();
     set_motor_H();
  }
  delay(10);
}


void command_process(){

  if (msg.indexOf(END_COURSE_COMMAND) != -1){// fin de course
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

void check_dist(int max_dist){
  if (dist_V > max_dist) {
      stop_move_V();
    }
  
    if (dist_H > max_dist) {
      stop_move_H();
    }
}

void stop_move_V(){
  state_V = STOP;
  dist_V = 0;
  digitalWrite(EN_PIN_V, HIGH);      // Disable driver in hardware

  if(state_H == STOP)
    mode = NORMAL_MODE;
}

void stop_move_H(){
  state_H = STOP;
  dist_H= 0;
  digitalWrite(EN_PIN_H, HIGH);      // Disable driver in hardware

  if(state_V == STOP)
    mode = NORMAL_MODE;
}

void update_dist(){
  if (state_V != STOP) {
    dist_V += DIST_PER_TURN/LOOP_PER_TURN;
  }

  if (state_H != STOP) {
    dist_H += DIST_PER_TURN/LOOP_PER_TURN;
  }
}

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
    
  mode = END_COURSE_MODE; // distance retour fin de course
}

void set_motor_V()
{
  if(state_V != STOP){
    if (state_V == UP)
      shaft_V = true;
      
    else if (state_V == DOWN)
      shaft_V = false;
      
    driver_V.shaft(shaft_V);
    digitalWrite(EN_PIN_V, LOW);      // Enable driver in hardware
  }
}

void set_motor_H()
{    
   if(state_H != STOP) {
    if (state_H == RIGHT)
      shaft_H = true;

    else if (state_H == LEFT)
      shaft_H = false;
   
    driver_H.shaft(shaft_H); // set the horizontal sens of rotation
    digitalWrite(EN_PIN_H, LOW);      // Enable driver in hardware

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
