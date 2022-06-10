#include <Wire.h>
#include <VL53L0X.h>
#include <TimerThree.h>
#include <TMCStepper.h>

#define XSHUT_L_PIN        4
#define XSHUT_R_PIN        5
#define XSHUT_U_PIN        6
#define XSHUT_D_PIN        8
#define XSHUT_RTH_PIN      7
#define XSHUT_H_PIN        9
#define XSHUT_V_PIN        10

#define LED_L_PIN          22
#define LED_R_PIN          24
#define LED_U_PIN          26
#define LED_D_PIN          30
#define LED_RTH_PIN        28

//TCM stepper driver horizontal
#define DIR_PIN_H          34 // Direction
#define STEP_PIN_H         2 // Step pwm pin
#define CS_PIN_H           36 // Chip select
#define EN_PIN_H           38 // Enable

//TCM stepper drivers vertical
#define DIR_PIN_V          40 // Direction
#define STEP_PIN_V         3 // Step pwm pin
#define CS_PIN_VL          43 // Chip select
#define CS_PIN_VR          44 // Chip select
#define EN_PIN_V           46 // Enable

#define SW_MOSI            48 // Software Master Out Slave In (MOSI)
#define SW_MISO            50 // Software Master In Slave Out (MISO)
#define SW_SCK             52 // Software Slave Clock (SCK)
#define R_SENSE 0.11f // Match to your driver
// SilentStepStick series use 0.11
// UltiMachine Einsy and Archim2 boards use 0.2
// Panucatt BSD2660 uses 0.1
// Watterott TMC5160 uses 0.075

#define FAN_PIN            33

#define SENSOR_TIME_THSD       500
#define RTH_TIME_THSD          2000
#define COMMAND_DIST_THSD      100
#define ORIG_THSD              2 // mm
#define ORIG_POS_H             350
#define ORIG_POS_V             350

#define UP                    1
#define DOWN                  2
#define RIGHT                 1
#define LEFT                  2

TMC2130Stepper driver_H(CS_PIN_H, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driver_VL(CS_PIN_VL, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driver_VR(CS_PIN_VR, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

VL53L0X sensor_L;
VL53L0X sensor_R;
VL53L0X sensor_U;
VL53L0X sensor_D;
VL53L0X sensor_RTH;
VL53L0X sensor_H;
VL53L0X sensor_V;

int period_pwm_motor = 75; // decreasing period <--> decreasing rotation time
int state_V = 0; // 0 stop, 1 up, 2 down
int state_H = 0;  // 0 stop, 1 right, 2 left

int distance_mm = 1000;
int distance_L_mm;
int distance_R_mm;
int distance_U_mm;
int distance_D_mm;

int measure_L;
int measure_R;
int measure_U;
int measure_D;

bool start_H = 0;
bool start_V = 0;
bool shaft_H = false;
bool shaft_V = false;

int count_V = 0;
int count_H = 0;

String nom = "Arduino";
String msg;

void setup()

{
  Serial.begin(9600);
  delay(2000);
  Wire.begin();

  pinMode(XSHUT_L_PIN, OUTPUT);
  pinMode(XSHUT_R_PIN, OUTPUT);
  pinMode(XSHUT_U_PIN, OUTPUT);
  pinMode(XSHUT_D_PIN, OUTPUT);
  //pinMode(XSHUT_RTH_PIN, OUTPUT);
  //pinMode(XSHUT_H_PIN, OUTPUT);
  //pinMode(XSHUT_V_PIN, OUTPUT);

  pinMode(LED_L_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_U_PIN, OUTPUT);
  pinMode(LED_D_PIN, OUTPUT);
  pinMode(LED_RTH_PIN, OUTPUT);

  //initialize TMC drivers
  pinMode(STEP_PIN_H, OUTPUT);
  pinMode(DIR_PIN_H, OUTPUT);
  pinMode(EN_PIN_H, OUTPUT);

  pinMode(STEP_PIN_V, OUTPUT);
  pinMode(DIR_PIN_V, OUTPUT);
  pinMode(EN_PIN_V, OUTPUT);

  pinMode(FAN_PIN, OUTPUT);

  digitalWrite(EN_PIN_H, LOW);      // Enable driver in hardware
  digitalWrite(EN_PIN_V, LOW);      // Enable driver in hardware

  digitalWrite(LED_L_PIN, LOW);
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_U_PIN, LOW);
  digitalWrite(LED_D_PIN, LOW);
  digitalWrite(LED_RTH_PIN, LOW);

  digitalWrite(FAN_PIN, LOW);

  driver_H.begin();                 // Init CS pins and SW SPI pins
  driver_H.toff(5);                 // Enables driver in software
  driver_H.rms_current(1200);        // Set motor RMS current
  driver_H.microsteps(16);          // Set microsteps to 1/16th
  driver_H.pwm_autoscale(true);     // Needed for stealthChop

  driver_VL.begin();                 // Init CS pins and SW SPI pins
  driver_VL.toff(5);                 // Enables driver in software
  driver_VL.rms_current(1200);        // Set motor RMS current
  driver_VL.microsteps(16);          // Set microsteps to 1/16th
  driver_VL.pwm_autoscale(true);     // Needed for stealthChop

  driver_VR.begin();                 // Init CS pins and SW SPI pins
  driver_VR.toff(5);                 // Enables driver in software
  driver_VR.rms_current(1200);        // Set motor RMS current
  driver_VR.microsteps(16);          // Set microsteps to 1/16th
  driver_VR.pwm_autoscale(true);     // Needed for stealthChop

  digitalWrite(EN_PIN_H, HIGH);      // De-enable driver in hardware
  digitalWrite(EN_PIN_V, HIGH);      // De-enable driver in hardware

  Timer3.initialize(40);
 

}
int current_nb_loop = 10;
int nb_loop = 10;
void loop()
{
  readSerialPort();
  if (msg != "") {
     sendData();
     if (msg == "0"){
      state_H = LEFT;
      state_V = 0;
     }
     else if (msg == "1"){
      state_H = RIGHT;
      state_V = 0;
     }
     else if (msg == "2"){
      state_V = UP;
      state_H = 0;
     }
     else if (msg == "3"){
      state_V = DOWN;
      state_H = 0;
     }

     //else if (msg =="4"){// fin de course
     else if (msg.indexOf("4") != -1){
      if(state_V == DOWN)
        state_V = UP;
        
      else if(state_V == UP)
        state_V = DOWN;
        
      if(state_H == RIGHT)
        state_H = LEFT;
        
      else if(state_H == LEFT)
        state_H = RIGHT;

      current_nb_loop = nb_loop;
      nb_loop = 3; // distance retour fin de course
     }
     
     else if (msg == "5"){
      nb_loop += 5;
     }
     else if (msg == "6" && nb_loop > 5){
      nb_loop -= 5;
     }
  }
  delay(500);

  
  if (count_V > nb_loop) { // 10 loops is around 6.8 motor rotation with 120 period pwm and 75% duty cycle
    state_V = 0;
    count_V= 0;
    
    if(nb_loop == 3)// si fin de course retourne à vitesse init
      nb_loop = current_nb_loop;
  }

  
  if (count_H > nb_loop) { // 10 loops is around 6.8 motor rotation with 120 period pwm and 75% duty cycle
    state_H = 0;
    count_H= 0;

    if(nb_loop == 3) // si fin de course retourne à vitesse init
      nb_loop = current_nb_loop;
  }

  if (state_V != 0) {
    count_V += 1;
  }

  if (state_H != 0) {
    count_H += 1;
  }

  set_motor();
}


void set_motor()
{
  digitalWrite(FAN_PIN, HIGH);

  float dutyCycle_H = 0.0; // switched off
  float dutyCycle_V = 0.0; // switched off

  set_movement_V(); // depending on the distances measured, the new sens and direction of movement are set
  set_movement_H();

  if (start_H == 1)
  {
    //Serial.println("start hori");
    dutyCycle_H = 75.0; // %
    digitalWrite(EN_PIN_H, LOW);      // Enable driver in hardware
  }
  else
  {
    digitalWrite(EN_PIN_H, HIGH);      // Disable driver in hardware
  }

  if (start_V == 1)
  {
    //Serial.println("start verti");
    dutyCycle_V = 75.0; // %
    digitalWrite(EN_PIN_V, LOW);      // Enable driver in hardware

  }
  else
  {
    digitalWrite(EN_PIN_V, HIGH);      // Disable driver in hardware
  }

  // send PWM to the drivers if duty cycle = 50 %
  Timer3.pwm(STEP_PIN_H, (dutyCycle_H / 100.0) * 1023.0, period_pwm_motor);
  Timer3.pwm(STEP_PIN_V, (dutyCycle_V / 100.0) * 1023.0, period_pwm_motor);
}


void set_movement_V()
{
  if (state_V == 0) // if nothing is detected by any sensor, switch off everything
  {
    start_V = 0;
  }
  else {
    if (state_V == 1)
    { // Up
      shaft_V = true;
    }
    else if (state_V == 2)
    { // Down
      shaft_V = false;
    }

    start_V = 1;
    driver_VL.shaft(shaft_V); // set the vertical sense of rotation
    driver_VR.shaft(shaft_V);
  }
}

void set_movement_H() // depending on the distances measured, the new sens and direction of movement are set
{
  if (state_H == 0) // if nothing is detected by any sensor, switch off everything
  {
    start_H = 0;
  }
  else {
    if (state_H == 1)
    { // Left
      shaft_H = true;
    }

    else if (state_H == 2)
    { // Right
      shaft_H = false;
    }

    start_H = 1;
    driver_H.shaft(shaft_H); // set the horizontal sens of rotation
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
 //Serial.print(distance_L_mm);
 
}
