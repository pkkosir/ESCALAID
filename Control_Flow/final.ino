#include <AS5600.h>

/***TO-DO**/
//FIGURE OUT INIT SPOOL IF STATEMENT
//FIGURE OUT MOTOR CONTROL FOR EACH STATE
//FIGURE OUT STRAIGHT LEG
//-Threads for sensor readings [CANT DO THREADS, IMPLEMENTED MICROSECOND INTERUPPTS FOR SENSOR READING INSTEAD]
//-Device active and calibration [SORTA DONE]
//-Add led functionality with modes [SORTA DONE]
//-Estop implmentation [DONE?]


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
#define INIT_ANGLE 40

/****************************************************************PID_VAR********************************************************************/
double setPoint = 0;
volatile double tension = 0;
volatile double error = 0;
int output = 0;
int dir = -1, lastDir = -1; //1 is cw, 0 is ccw
int braked = 0;
double Kp = 2, Ki = 0.0, Kd = 0.0;
int in = 0;
PID controller(&tension, &error, &setPoint, Kp, Ki, Kd, P_ON_E, DIRECT);

/************************************************************TENSION_SENSOR_VAR**************************************************************/
HX711 hx711;
double sensor_noise_threshold = 1.00; //1lb noise threshold
double upper_sensor = 8;
double lower_sensor = 4;
float calibration_factor = -20810;

/****************************************************************MOTOR_VAR*******************************************************************/
const int mc_IN1 = 51;
const int mc_IN2 = 53;
const int mc_PWM = 4;

/****************************************************************IMU_VAR*********************************************************************/

MPU6050 mpu1;
MPU6050 mpu2(0x69); // Second MPU6050 with address 0x69

#define ANGLE 145 // Defines the angle used for threshold of PID stopping

// Variables to hold the global angle data
float glob_angX1 = 65, glob_angY1 = 0; //defaults changed to take less time to find correct angle. CHECK THESE!!!!!!!!!!!
float glob_angX2 = -65, glob_angY2 = 0;
float angX1 = 0;
float angX2 = 0;
float angY1 = 0;
float angY2 = 0;
float shankThighX;
float init_shankThighX;

// Variables for the complementary filter for more accurate angle measurements
unsigned long prevTime = 0;
unsigned long currTime = 0;

unsigned long lastTime = 0;
unsigned long presTime = 0;
float dt;
float alpha = 0.7;
bool TOPRINT = 1;  // whether or not we send the spikes for state transitions
const int count = 1;  // for outputting every "count-th" sample
int counter = 0;

// STATE variable for holding which state our system is in
/*
  0 -> IDLE
  1 -> TROUGH (foot is being lifted up, not giving support)
  2 -> PEAK (foot is on downswing, used for internal tracking purposes)
  4 -> SUPPORT (from some number of samples aroudn 0, we know to start straightening the leg)
  */
int ustate;
int pstate;
int prevState;
#define PRELIFT 0
#define TROUGH 1
#define PEAK 2
#define SUPPORT 3
#define INVALID 4

// Allowance for outliers when checking for trends in the state transitions
const int allowance = 1;  // just 1 outlier for now, ensures that we don't preemptively change states
int allowed = 0;

// Variables and data structures for running average, taken from the average of X and Z gyroscope values (same trend, allows for initial smoothing)
const int sampleWindow = 2;  // averages over previous 2 samples, introduces small delay
float bufferX1[sampleWindow] = { 0 };
float bufferX2[sampleWindow] = { 0 };

// float combX = 0;
float runAvgX1 = 0;  // Running average for the previous 2 values
float runAvgX2 = 0;  // Running average for the previous 2 values

int bufferIndex = 0;

int imu1_time1;
int init_time1;
int imu2_time1;

// // Variables to hold accelerometer and gyroscope data
// float Acceleration1_X, Acceleration1_Y, Accerleration1_Z;
// float Acceleration2_X, Acceleration2_Y, Accerleration2_Z;
// float ang1_X, ang1_Y;
// float ang2_X, ang2_Y;
// float gyroX_IMU2;

// // Variables for global angle data
// float glob_angX1 = 0, glob_angY1 = 0;
// float glob_angX2 = 0, glob_angY2 = 0;

// // Variables for the complementary filter for more accurate angle measurements
// unsigned long prevTime = 0;
//unsigned long currTime = 0;
//unsigned long lastTime = 0; //sensors time var
// float dt; 
// float alpha = 0.96;


// int imu1_time1;
// int init_time1;
// int imu2_time1;

// // STATE variable for holding which state our system is in
// /*
// 0 -> PRELIFT
// 1 -> LIFT (foot is being lifted up, not giving support)
// 2 -> PRECONTACT (foot is on downswing, used for internal tracking purposes)
// 3 -> CONTACT (foot has contacted the floor)
// 4 -> SUPPORT (from tension difference, we know to start straightening the leg)
// */
// int ustate;
// int pstate; 
// int prevState;

// #define PRELIFT 0
// #define LIFT 1
// #define PRECONTACT 2
// #define CONTACT 3
// #define SUPPORT 4
// #define INVALID 5

// // Allowance for outliers when checking for trends in the state transitions
// const int allowance = 1; // just 1 outlier for now, ensures that we don't preemptively change states
// int allowed = 0;

// // Variables and data structures for running average, taken from the average of X and Z gyroscope values (same trend, allows for initial smoothing)
// const int sampleWindow = 3; // averages over previous 5 samples, introduces small delay
// float buffer[sampleWindow] = {0};
// float bufferX[sampleWindow] = {0};
// float bufferZ[sampleWindow] = {0};
// float gAvgXZ = 0; // Calculated average of the X and Z gyroscope values
// float gAvgXZ_prev = 0; // Buffer to store the previous average gXZ value
// float runAvg = 0;
// float runAvg_prev = 0; // Buffer to store the previous running average for comparison
// float shankThighX = 0;
// float runAvgX = 0; // Running average for the previous 3 values before XZ averaging
// float runAvgZ = 0;

// int bufferIndex = 0;

/****************************************************************HALL_EFFECT_VAR*********************************************************************/
AS5600 as5600;
float temp_max = 0;
float spoolAngle = 0;
int spoolRev = 0;
float lastAngle = 0;

/****************************************************************CONTROL_VAR*********************************************************************/
volatile int active = 0;
//CONTROL FLOW VARIABLES
int active_button = 27;//device init MODE
int Idle_button = 18; //18 //IDLE MODE
int Ascend_button = 19; //ASCEND MODE
int Descend_button = 3; //DESCEND MODE
int estop_button = 2 ; //emergency stop MODE

//MODE_LED
int Active_LED = 39;
int Idle_LED = 40;//40
int Ascend_LED = 41;
int Descend_LED = 42;

//TYPEDEF FOR MODE
typedef enum {
    IDLE,
    ASCEND,
    DESCEND,
    INVALID_VALUE // For error handling
} MODE;

MODE umode = INVALID_VALUE;

int idle_once = 0;
int first_step = 0;
int cal = 1;
int stop = 0;
float battery_volt = 0;
int low_power = 0;

/****************************************************************SETUP_FUNCS*********************************************************************/
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PID_init(){
  setPoint = 50;
  controller.SetSampleTime(25);
  controller.SetOutputLimits(-255, 255);
  controller.SetMode(AUTOMATIC);
}

void HX711_setup() { // HX711 Tension Sensor Init
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  hx711.set_scale();
  hx711.tare();  //Reset the scale to 0
  long zero_factor = hx711.read_average(); //Get a baseline reading
  hx711.set_scale(calibration_factor); //Adjust to this calibration factor
}

void as5600_init() {

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.println(as5600.getAddress());
  int b = as5600.isConnected();

}

void pin_init() {

  //initialize Motor pins
  pinMode(mc_IN1, OUTPUT);
  pinMode(mc_IN2, OUTPUT);
  pinMode(mc_PWM, OUTPUT);

  //initialize Buttons as interrupts
  pinMode(Ascend_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Ascend_button),mode_selection,FALLING);
  pinMode(Idle_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Idle_button),mode_selection,FALLING);
  pinMode(estop_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(estop_button),estopFlag,FALLING);
  pinMode(Descend_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Descend_button),mode_selection,FALLING);

  //initialize LEDs
  pinMode(Active_LED, OUTPUT);
  pinMode(Idle_LED, OUTPUT );
  pinMode(Ascend_LED, OUTPUT);
  pinMode(Descend_LED, OUTPUT);
}

void calibrate(){

  //initialize PID
  PID_init();

  //initialize Tensior Sensor
  HX711_setup();

  //initialize Hall effect Sensor
  as5600_init();
  Serial.println("HEDONE");
  delay(100);

  //Initialize MPU6050
  mpu1.initialize();
  mpu2.initialize();

  if (!mpu1.testConnection() || !mpu2.testConnection()) {
    if (!mpu1.testConnection()) Serial.println("MPU1 failed");
    if (!mpu1.testConnection()) Serial.println("MPU2 failed");
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  //Set default to state to idle, before walking has begun
  ustate = PRELIFT;
  prevState = ustate;

  // Identifies previous time for use in complementary filter
  prevTime = millis();
  umode = IDLE;

  //battery_volt = fmap(analogRead(A9),0,1023,0,5.2);
  Serial.println("CAL_DONE");
  delay(100);

}

void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin();
  //setup deviceactive button and wait for active = 1 after button press
  pinMode(active_button, INPUT_PULLUP);
  
  while (digitalRead(active_button)){

  }
  activate();

  //initialize pins for motor,buttons,LED
  interrupts(); //enable interrupts
  pin_init();

  Serial.println("DONE SETUP");
}

/****************************************************************LOOP_FUNC*********************************************************************/
void loop() {
  // Serial.print(runAvgX1);
  // Serial.print(",");
  // Serial.print(glob_angX1);
  // Serial.print(",");
  // Serial.print(glob_angX2);
  // Serial.print(",");
  // Serial.println(shankThighX);
  Serial.println(umode);
  //calibrate once everytime we are active
  if(cal == 1 && active == 1){
    calibrate();
    cal = 0;
    Serial.println("Calibrated");
  }
  //while active get sensor reading and execute flow
  if(active == 1){
    if (stop){
      estop();
    }
    else if (!digitalRead(active_button)){
      active = 0;
    }
    else{
      sensors(); //read_sensors
      state_detection(); //executes appropriate mode functionality
    }
  }
  else{
    while (digitalRead(active_button)){

    }
    activate();
  }
}

/****************************************************************BUTTON_MODE_FUNC*********************************************************************/
void mode_selection(){
    // Serial.print(shankThighX);
    // Serial.print(",");
    // Serial.println(tension);
   //IDLE MODE BUTTON PRESS
  if(digitalRead(Idle_button) == LOW && umode != IDLE && active == 1 && umode != DESCEND){
    if(tension<3 && shankThighX >= 120){ //check for straight leg and no load
      umode = IDLE;
      idle_once = 1;
      pstate = INVALID;
      digitalWrite(Idle_LED,HIGH);
      digitalWrite(Ascend_LED,LOW);
      digitalWrite(Descend_LED,LOW);
    }   
  } 
  //ASCEND MODE BUTTON PRESS
  if(digitalRead(Ascend_button) == LOW && umode != ASCEND && active == 1 && low_power == 0 && umode != DESCEND){//low power == 1?

    //Serial.print("ASCEND");
    if(tension<3 && shankThighX >= 120){ //check for straight leg and no load
      umode = ASCEND;
      //Set default to state to idle, before walking has begun
      ustate = PRELIFT;
      pstate = INVALID;
      digitalWrite(Ascend_LED,HIGH);
      digitalWrite(Idle_LED,LOW);
      digitalWrite(Descend_LED,LOW);
      first_step = 1;
    }
  }

  if(digitalRead(Descend_button) == LOW && umode != DESCEND){
    //Serial.println("descend");
    umode = DESCEND;
    ustate = INVALID;
    pstate = INVALID;
    digitalWrite(Descend_LED,HIGH);
    digitalWrite(Ascend_LED, LOW);
    digitalWrite(Idle_LED, LOW);
    //delay(1000);
  } 

  // if(digitalRead(Descend_button) == LOW && umode == DESCEND){
  //   //Serial.println("out");
  //   umode = IDLE;
  //   ustate = INVALID;
  //   pstate = INVALID;
  //   motor_control(0,0);
  //   digitalWrite(Descend_LED, LOW);
  //   digitalWrite(Ascend_LED, LOW);
  //   digitalWrite(Idle_LED, HIGH);
  //   delay(1000);
  // }
}

void estopFlag(){
  stop = 1;
}

void estop(){ 
  Serial.println("error");
  while(spoolRev != 0){//check if the original spool position is reached
    motor_control(255, 1);
    sensors();
  }
  while (spoolAngle < INIT_ANGLE){
    motor_control(230, 1);
    sensors();
  }
  motor_control(0,0);
  noInterrupts(); //disable interuppt
  while(1){ //stuck in all lights ON indicating user to restart device.
      digitalWrite(Ascend_LED,HIGH);
      digitalWrite(Idle_LED,HIGH);
      digitalWrite(Active_LED,HIGH);
      active = 0;
      delay(200);
      digitalWrite(Ascend_LED,LOW);
      digitalWrite(Idle_LED,LOW);
      digitalWrite(Active_LED,LOW);
      delay(200);
  }
}

void activate(){
    if (active == 0){//User turns on device
      active = 1;
      digitalWrite(Ascend_LED,LOW);
      digitalWrite(Descend_LED,LOW);
      digitalWrite(Idle_LED,HIGH);
      digitalWrite(Active_LED,HIGH);
      umode = IDLE;
      cal = 1;
    }
    else if(active == 1 ){//User tries to turn off device
      if(umode == IDLE){//ensuring we are in IDLE mode to turn off device
        active = 0;
        umode = INVALID_VALUE;
        digitalWrite(Active_LED,LOW);
        digitalWrite(Ascend_LED,LOW);
        digitalWrite(Idle_LED,LOW);
        digitalWrite(Descend_LED,LOW);
      }
    } 
}

/****************************************************************TOP_STATE_FUNC*********************************************************************/
void state_detection(){
  if(umode == IDLE){
    idle_state();
    //interrupts();
  }

  else if (umode == ASCEND){
    ascend_state();
    //interrupts();
  }

  else if (umode == DESCEND){
    descend_state();
    //noInterrupts();
  }
}
/****************************************************************IDLE_STATE_FUNC*********************************************************************/
void idle_state(){
  if(idle_once == 1){
    if(spoolRev != 0){//checking the halleffect sensor reading to see if the original position is reached, which is the idle state fixed slack position
      motor_control(255, 1);
    }
    else if (spoolAngle < INIT_ANGLE){
      motor_control(230, 1);
    }
    else{
    motor_control(0,0);
    idle_once = 0;
    }
  }
}
/****************************************************************DESCEND_STATE_FUNC*********************************************************************/
void descend_state(){
  if(!digitalRead(Ascend_button)){
    motor_control(255,1);
  }
  else if(!digitalRead(Idle_button)){
    motor_control(255,0);
  }
  else{
    motor_control(0,0);
  }
}
/****************************************************************ASCEND_STATE_FUNC*********************************************************************/
int tstart, tend;
void ascend_state(){
  
  if (ustate == PRELIFT) {
    Serial.println("PRELIFT");

    if(pstate != ustate){
      if(!first_step){
        if(spoolRev != 0){
          motor_control(255, 1);
        }
        else if (spoolAngle < INIT_ANGLE){
          motor_control(230, 1);
        }
        else{
        motor_control(0,0);
        tend = millis();
        pstate = ustate;
        }
      }
      else{
        first_step = 0;
        pstate = ustate;
      }
    }
    if (runAvgX1 <= -40 || runAvgX2 <= -40) { //trough ALWAYS below -40
      if (allowed >= allowance) {
        ustate = TROUGH; 
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }

  if (ustate == TROUGH) {
    Serial.println("TROUGH");

    if(pstate != ustate){
      pstate = ustate;
    }
    else if (runAvgX1 >= 35 && runAvgX2 <= 0) {  // peak is ALWAYS above 35 for X1 and below 0 for X2
      if (allowed >= allowance) {
        ustate = PEAK;
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  
  }


  if (ustate == PEAK) {
    Serial.println("PEAK");
    motor_control(230,0);


    if(pstate != ustate){
      pstate = ustate;
    }
    else if (runAvgX1 <= 10 ) {
      if (allowed >= 3*allowance) { //want at least 5 samples, but may not be feasible
        ustate = SUPPORT;

        //SEND SIGNAL TO PID TO START HERE

        allowed = 0;
      } else {
        allowed += 1;
      }
    }

  }

  if (ustate == SUPPORT){
    Serial.println("SUPPORT");

    if(pstate != ustate){
      init_time1 = millis();
      init_shankThighX = shankThighX;
      imu1_time1 = angX1;
      imu2_time1 = angX2;
      pstate = ustate;
    }
    //else{
      // if(((init_shankThighX+10) < shankThighX) || ((init_shankThighX-10) > shankThighX)){
      //   init_shankThighX = shankThighX;
      //   init_time1 = millis();
      // }
      // else{
      //   if ((millis() - init_time1) >= 2000 ){
      //     umode = IDLE;
      //     idle_once = 1;
      //     pstate = INVALID;
      //     digitalWrite(Idle_LED,HIGH);
      //     digitalWrite(Ascend_LED,LOW);
      //   }
      //}
    //}



    PID_Update();
    
 
    if (shankThighX >= ANGLE) {  
        if (allowed >= 5*allowance) { 
        ustate = PRELIFT;

        //SEND SIGNAL TO TURN OFF PID

        allowed = 0;
      } else {
        allowed += 1;
      }
    }


  }   
}

/****************************************************************SENSOR_TOP_LEVEL*********************************************************************/
void sensors(){
  presTime = millis();

  if (presTime - lastTime >= 10){
    if(battery_volt < -1){ //2
   
      Serial.print(battery_volt);
    }
    else if(battery_volt < -1){ //3.33
      umode = IDLE;
      low_power = 1;
    }
    else{
      //battery_volt = fmap(analogRead(A9),0,1023,0,5.2); 
      I2C_IMU();
      SPI_tension();
      I2C_HE();
      // if (tension > 80){
      //   Serial.println(tension);
      //   estop();
      // }
    }

  }
}
/****************************************************************IMU_SENSOR_READINGS*********************************************************************/

/*
Helper methods
*/
void updateRunningAverages(float sampleX1, float sampleX2) { 

  // Remove oldest sample
  runAvgX1 -= bufferX1[bufferIndex] / sampleWindow;  
  runAvgX2 -= bufferX2[bufferIndex] / sampleWindow;  

  // Adds next sample to buffer and updates sum
  bufferX1[bufferIndex] = sampleX1;
  bufferX2[bufferIndex] = sampleX2;

  runAvgX1 += sampleX1 / sampleWindow;
  runAvgX2 += sampleX2 / sampleWindow;

  // Increments buffer index and wraps around
  bufferIndex = (bufferIndex + 1) % sampleWindow;
}

// Calculation for angles of the MPUs, to use absolute/relative angles to help determine state
void calcAngles(float ax, float ay, float az, float gx, float gy, float& angX, float& angY) {

  float accelAngX = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * 180 / M_PI;
  float accelAngY = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / M_PI;

  // Gyro angles, integrated
  angX = alpha * (angX + gx * dt) + (1 - alpha) * accelAngX;  //dt is global, implicit replacement of angX w/ current angX with integrated gx
  angY = alpha * (angY + gy * dt) + (1 - alpha) * accelAngY;
}

void I2C_IMU(){ //basically the "loop" code

  currTime = millis(); // Current time for comp filter
  dt = (currTime - prevTime) / 1000.0; // Time difference for integration
  prevTime = currTime;

  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;

  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

   // Converting to physical units
  // // convert to +-8g's (converting to +-1g gave near-0 results) NOTE: THESE WERE KIND OF USELESS REGARDLESS SO I DIDN'T USE THEM AT ALL
  // converting to degrees/s
  float gyroX1 = gx1 / 131.0;
  float gyroY1 = gy1 / 131.0;
  float gyroZ1 = gz1 / 131.0;

  float gyroX2 = gx2 / 131.0;
  float gyroY2 = gy2 / 131.0;
  float gyroZ2 = gz2 / 131.0;

  // Update set "current value" and update running average for SHANK angular accleration
  updateRunningAverages(gyroX1, gyroX2); //running average on previous 2 combines samples

  // Calculates angles of MPU w/ complementary filter
  angX1 = glob_angX1, angY1 = glob_angY1;  // Localizing globals to be able to pass into function
  angX2 = glob_angX2, angY2 = glob_angY2;
  calcAngles(ax1, ay1, az1, gyroX1, gyroY1, angX1, angY1);
  calcAngles(ax2, ay2, az2, gyroX2, gyroY2, angX2, angY2);

  glob_angX1 = angX1;  // Sending newly calculated variables back to global
  glob_angY1 = angY1;
  glob_angX2 = angX2;
  glob_angY2 = angY2;

  shankThighX = abs(angX1 - angX2);  

}


/****************************************************************TENSION_SENSOR_READINGS*********************************************************************/
void SPI_tension() { // HX711 Tension Sensor reading
  tension = hx711.get_units();
}
/****************************************************************HALL_EFFECT_SENSOR_READINGS*********************************************************************/
void I2C_HE() {
  //static uint32_t lastTime = 0;
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

/****************************************************************MOTOR_CONTROL_FUNCS*********************************************************************/
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



/****************************************************************PID_FUNC*********************************************************************/

void PID_Update(){
  
  controller.Compute();

  (error >= 0) ? dir = 0: dir = 1;
  float errC = constrain(error, -20, 20);
  (dir == 1) ? output = fmap(abs(errC), 0, 20, 100, 140): output = fmap(abs(errC), 0, 20, 200,255);
  (tension <= 1) ? output = 255: output = output;

  Serial.print(millis());
  Serial.print(",");
  Serial.print(setPoint);
  Serial.print(",");
  Serial.print(tension);
  Serial.print(",");
  Serial.print(errC);
  Serial.print(",");
  Serial.println(output);

  

   if (abs(errC) < 5){
    motor_control(0,dir);
   }
   else{
    motor_control(output,dir);
   }
}




