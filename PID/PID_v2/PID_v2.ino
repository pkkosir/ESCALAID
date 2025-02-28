#include "HX711.h" //This library can be obtained here http://librarymanager/All#Avia_HX711
#include "PID_v2.h"
#include "AS5600.h"
#include <Wire.h>
#include <MPU6050.h>
#include <math.h> 

#define PID_input A1
#define PID_output 6
#define LOADCELL_DOUT_PIN  A4
#define LOADCELL_SCK_PIN  A3
#define BAUDRATE 9600

double setPoint = 0;
volatile double tension = 0;
volatile double error = 0;
int output = 0;
int dir = -1, lastDir = -1; //1 is cw, 0 is ccw
int braked = 0;
double Kp = 1.5, Ki = 0.00, Kd = 0.04;
int in = 0;
PID controller(&tension, &error, &setPoint, Kp, Ki, Kd, P_ON_E, DIRECT);

HX711 hx711;
double sensor_noise_threshold = 1.00; //1lb noise threshold
double upper_sensor = 8;
double lower_sensor = 4;
float calibration_factor = -20810;

const int mc_IN1 = 51;
const int mc_IN2 = 53;
const int mc_PWM = 4;

AS5600 as5600;
float temp_max = 0;
float spoolAngle = 0;
int spoolRev = 0;
float lastAngle = 0;
volatile int active = 0;

//CONTROL FLOW VARIABLES
int active_button = 27;//device init MODE
int Idle_button = 18; //18 //IDLE MODE
int Ascend_button = 19; //ASCEND MODE
int Descend_button = 3; //DESCEND MODE
int estop_button = 2 ; //emergency stop MODE

int init_angle = 0;

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PID_init(){
  setPoint = 50;
  controller.SetOutputLimits(-255, 255);
  controller.SetMode(AUTOMATIC);
}

void HX711_setup() { // HX711 Tension Sensor Init
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  hx711.set_scale();
  hx711.tare();  //Reset the scale to 0
  long zero_factor = hx711.read_average(); //Get a baseline reading
  //Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  //Serial.println(zero_factor);
  hx711.set_scale(calibration_factor); //Adjust to this calibration factor
 }

void as5600_init() {

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  //Serial.println(as5600.getAddress());

  //int b = as5600.isConnected();

}

void pin_init() {

  //initialize Motor pins
  pinMode(mc_IN1, OUTPUT);
  pinMode(mc_IN2, OUTPUT);
  pinMode(mc_PWM, OUTPUT);

  //initialize Buttons as interrupts
  pinMode(Ascend_button, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(Ascend_button),resetState,FALLING);
  pinMode(Idle_button, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(Idle_button),mode_selection,FALLING);
  pinMode(estop_button, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(estop_button),estopFlag,FALLING);

  //initialize LEDs
  // pinMode(Active_LED, OUTPUT);
  // pinMode(Idle_LED, OUTPUT );
  // pinMode(Ascend_LED, OUTPUT);
}

int reset = 0;
void resetState(){
  reset = 1;
}

void calibrate(){

  //initialize PID
  PID_init();

  //initialize Tensior Sensor
  HX711_setup();


  //initialize Hall effect Sensor
  //as5600_init();

  delay(100);
}



void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin();
  //setup deviceactive button and wait for active = 1 after button press
  pinMode(active_button, INPUT_PULLUP);

  while (digitalRead(active_button)){

  }
  calibrate();
  interrupts(); //enable interrupts
  pin_init();
  Serial.println("here");

  //Serial.println("DONE SETUP");
}

void loop() {
  if (digitalRead(Ascend_button)){
    PID_Update();
    SPI_tension();
    //Serial.println(millis());
    //Serial.println(tension);
  }
  else{
    motor_control(200,1);
    delay(1000);
    motor_control(0,0);
    reset = 0;
    delay(2000);
  }
}

void estop(){ //User manual that says after estop device must be restarted??
  Serial.println("error");
  while(spoolRev != 0){//checking the halleffect sensor reading to see if the original position is reached, which is the idle state fixed slack position
    motor_control(200, 1);
    sensors();
    Serial.println("f");
  }
  while (spoolAngle < 80){
    motor_control(200, 1);
    sensors();
    Serial.println("g");
  }

  Serial.println("out");
  motor_control(0,0);
  noInterrupts(); //disable interuppt
  while(1){ //stuck in all lights ON indicating user to restart device.
      // digitalWrite(Ascend_LED,HIGH);
      // digitalWrite(Idle_LED,HIGH);
      // digitalWrite(Active_LED,HIGH);
      active = 0;
      delay(200);
      // digitalWrite(Ascend_LED,LOW);
      // digitalWrite(Idle_LED,LOW);
      // digitalWrite(Active_LED,LOW);
  }
}

void sensors(){
  SPI_tension();
  //I2C_HE();
}

void SPI_tension() { // HX711 Tension Sensor reading
  tension = hx711.get_units();
}
/****************************************************************HALL_EFFECT_SENSOR_READINGS*********************************************************************/
void I2C_HE() {
  static uint32_t lastTime = 0;
  lastAngle = spoolAngle;
  as5600.getCumulativePosition();
  spoolAngle = fmap(as5600.readAngle(), 0, 4096, 0, 360);
  //spoolRev = (as5600.getRevolutions()*-1);
  //Serial.print("Angle: ");
  //Serial.print(spoolAngle, 2);
  //Serial.print(" | revolutions: ");
  //Serial.print(spoolRev);
  if (lastAngle > 280 && spoolAngle < 80){ //clearly reset angle, proving one revolution
    spoolRev += 1;
  }
  else if (lastAngle < 80 && spoolAngle > 280){
    spoolRev -= 1;
  }
  
}


void motor_control(int speed, bool direction){ //direction = 0 means ccw rotation and direction = 1 means cw rotation, speed = 0 means brake
  if(direction == 0 && lastDir != 0){
    mc_counterclockwise();
    lastDir = direction;
  }
  else if (direction == 1 && lastDir != 1){
    mc_clockwise();
    lastDir = direction;
  }
  mc_speed(speed);

}

void mc_clockwise() {
    digitalWrite(mc_IN1, HIGH);
    digitalWrite(mc_IN2, LOW);
    
}

void mc_counterclockwise() {
    digitalWrite(mc_IN1, LOW);
    digitalWrite(mc_IN2, HIGH);
}

void mc_speed(int mc_speed){ //Set speed=0 to brake
    analogWrite(mc_PWM, mc_speed);
}

void PID_Update(){
  controller.Compute();

  (error >= 0) ? dir = 0: dir = 1;
  float errC = constrain(error, -20, 20);
  (dir == 1) ? output = fmap(abs(errC), 0, 20, 120, 200): output = fmap(abs(errC), 0, 20, 200,250);
  (tension <= 1) ? output = 255: output = output;

  Serial.print(millis());
  Serial.print(",");
  //Serial.print(setPoint);
  //Serial.print(",");
  Serial.println(tension);
  //Serial.print(",");
  //Serial.println(errC);

  // if (dir == 1 && in == 1){
  //   setPoint = setPoint - 10;
  //   in = 0;
  // }
  // else if (dir == 0 && in == 0){
  //   setPoint = setPoint + 10;
  //   in = 1;
  // }
   if (abs(errC) < 3){
    motor_control(0,dir);
   }
   else{
    motor_control(output,dir);
   }
}

