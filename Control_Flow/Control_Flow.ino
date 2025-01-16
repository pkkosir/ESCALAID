/***TO-DO**/
//-Bring in position variables of 2 IMUs to figure out straight leg position
//-Motor control during different states of ascend
//-Threads for sensor readings [CANT DO THREADS, IMPLEMENTED MICROSECOND INTERUPPTS FOR SENSOR READING INSTEAD]
//-Device active and calibration [SORTA DONE]
//-Add led functionality with modes [SORTA DONE]
//-Estop implmentation [DONE?]



/****************************************************************PID_VAR*********************************************************************/
#include "HX711.h" //This library can be obtained here http://librarymanager/All#Avia_HX711
#include "PID_v2.h"
#include "AS5600.h"
#include "TimerOne.h"

#define PID_input A1
#define PID_output 6

double setPoint = 0;
volatile double tension = 0;
volatile double error = 0;
int output = 0;
int dir = 0, lastDir = 0; //1 is cw, 0 is ccw
int braked = 0;
double Kp = 5, Ki = 0.8, Kd = 0.15;
PID controller(&tension, &error, &setPoint, Kp, Ki, Kd, P_ON_E, DIRECT);


//HX711 Tension Sensor Codes
#define LOADCELL_DOUT_PIN  A1
#define LOADCELL_SCK_PIN  A0
HX711 hx711;
double sensor_noise_threshold = 1.00; //1lb noise threshold
double upper_sensor = 8;
double lower_sensor = 4;
float calibration_factor = -20810;

//Motor Controller
const int mc_IN1 = 5;
const int mc_IN2 = 4;
const int mc_PWM = 6;
int mc_state = 0; // 0 = brake, 1 = cw, 2 = ccw;
int prev_mc_state = -1;


int stopped = 0;
char input_mode = 'b';

// //PID Variables
// double Kp = 0.0;
// double Ki = 0.0;
// double Kd = 0.0;
// double set_point_deg = 0.0;

/****************************************************************IMU_VAR*********************************************************************/
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
MPU6050 mpu;
MPU6050_Base mpu2(0x69);


// Variables to hold accelerometer and gyroscope data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

float accelX2, accelY2, accelZ2;
float gyroX2, gyroY2, gyroZ2;

// STATE variable for holding which state our system is in
/*
0 -> PRELIFT
1 -> LIFT (foot is being lifted up, not giving support)
2 -> CONTACT (foot has contacted the floor)
3 -> SUPPORT (from some time-step/other parameter, we know to start straightening the leg)
*/
int ustate; 
int prevState;
#define PRELIFT 0
#define LIFT 1
#define PRECONTACT 2
#define CONTACT 3
#define SUPPORT 4

// Variables and data structures for running average, taken from the average of X and Z gyroscope values (same trend, allows for initial smoothing)
const int sampleWindow = 5; // averages over previous 5 samples, introduces small delay
float buffer[sampleWindow] = {0};
float gAvgXZ = 0; // Calculated average of the X and Z gyroscope values
float gAvgXZ_prev = 0; // Buffer to store the previous average gXZ value
float runAvg = 0;

int bufferIndex = 0;


/****************************************************************HALL_EFFECT_VAR*********************************************************************/

AS5600 as5600;
float temp_max = 0;
float spoolAngle = 0;
int spoolRev = 0;
/****************************************************************CONTROL_VAR*********************************************************************/
volatile int active = 0;
//CONTROL FLOW VARIABLES
int active_button = 43; //device init MODE
int Idle_button = 44; //44 //IDLE MODE
int Ascend_button = 45; //ASCEND MODE
int Descend_button = 46; //DESCEND MODE
int estop_button = 47; //emergency stop MODE

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

MODE umode = IDLE;


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/****************************************************************SETUP_FUNCS*********************************************************************/
void PID_init(){
  setPoint = 3;
  controller.SetOutputLimits(-255, 255);
  controller.SetMode(AUTOMATIC);
  delay(1000);
}

void HX711_setup() { // HX711 Tension Sensor Init
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  hx711.set_scale();
  hx711.tare();  //Reset the scale to 0
  long zero_factor = hx711.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);

  hx711.set_scale(calibration_factor); //Adjust to this calibration factor

  delay(100);
}

void as5600_init() {
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  Serial.println(as5600.getAddress());

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
}

void mc_init() {
  pinMode(mc_IN1, OUTPUT);
  pinMode(mc_IN2, OUTPUT);
  pinMode(mc_PWM, OUTPUT);
}

void calibrate(){

  //initialize PID
  PID_init();

  //initialize Tensior Sensor
  HX711_setup();

  //initialize Hall effect Sensor
  as5600_init();

  //Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
      Serial.println("1st MPU6050 connection failed!");
      while (1);
  }

  //mpu2.MPU6050_Base(MPU6050_ADDRESS_AD0_HIGH,Wire);
  mpu2.initialize();
  if (!mpu2.testConnection()) {
      Serial.println("2nd MPU6050 connection failed!");
      while (1);
  }

  Serial.println("MPU6050 initialized.");
  // Print CSV header to the serial monitor
  // Serial.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
  Serial.println("GyroX,GyroZ,MovingAvg,AngleX,AngleY"); 

  //calls sensors func every millisecond
  Timer1.initialize(1000); //default units of microseconds
  Timer1.attachInterrupt(sensors);

}

void setup() {
  //setup deviceactive button and wait for active = 1 after button press
  interrupts(); //enable interrupts
  pinMode(active_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(active_button),activate,FALLING);

  Serial.begin(9600);
  Wire.begin();
 
  //initialize motor
  mc_init();

  //initialize Buttons as interrupts
  pinMode(Ascend_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Ascend_button),mode_selection,FALLING);
  pinMode(Idle_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Idle_button),mode_selection,FALLING);
  pinMode(estop_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(estop_button),estop,FALLING);

  //initialize LEDs
  pinMode(Active_LED, OUTPUT);
  pinMode(Idle_LED, OUTPUT );
  pinMode(Ascend_LED, OUTPUT);


}

/****************************************************************LOOP_FUNC*********************************************************************/
void loop() {
  
  if(active == 1){
    delay(10);
    sensors();
    state_detection();
  }
  

}

/****************************************************************BUTTON_MODE_FUNC*********************************************************************/
void mode_selection(){
   //IDLE MODE BUTTON PRESS
  if(digitalRead(Idle_button) == LOW && umode != IDLE && active == 1){
    if(tension<3 && gAvgXZ < 10){ //check for straight leg and no load
      umode = IDLE;
      digitalWrite(Idle_LED,HIGH);
      //digitalWrite(Active_LED,LOW);
      digitalWrite(Ascend_LED,LOW);
    }   
  } 
  //ASCEND MODE BUTTON PRESS
  if(digitalRead(Ascend_button) == LOW && umode != ASCEND && active == 1){
    if(tension<3 && gAvgXZ < 10){ //check for straight leg and no load
      umode = ASCEND;
      //Set default to state to idle, before walking has begun
      ustate = PRELIFT;
      prevState = ustate;
      digitalWrite(Ascend_LED,HIGH);
      digitalWrite(Idle_LED,LOW);
      //digitalWrite(Active_LED,LOW);
    }
  } 
}

void estop(){ //User manual that says after estop device must be restarted??
  while(spoolAngle > 0 && spoolRev != 0){//checking the halleffect sensor reading to see if the original position is reached, which is the idle state fixed slack position
    motor_control(255, 0);
  }
  motor_control(0,0);
  noInterrupts(); //disable interuppt
  while(1){ //stuck in all lights ON indicating user to restart device.
      digitalWrite(Ascend_LED,HIGH);
      digitalWrite(Idle_LED,HIGH);
      digitalWrite(Active_LED,HIGH);
      active = 0;
  }
}

void activate(){
    if (active == 0){//User turns on device
      active = 1;
      calibrate();//calibrate sensors and PID everytime deviceactivates
      digitalWrite(Ascend_LED,LOW);
      digitalWrite(Idle_LED,LOW);
      digitalWrite(Active_LED,HIGH);
      //delay(200);
    }
    else if(active == 1 ){//User tries to turn off device
      if(umode == IDLE){//ensuring we are in IDLE mode to turn off device
        active = 0;
      }
    } 
}

/****************************************************************TOP_STATE_FUNC*********************************************************************/
void state_detection(){
  if(umode == IDLE){
    idle_state();
  }

  else if (umode == ASCEND){
    ascend_state();
  }

  else{

  }
}
/****************************************************************IDLE_STATE_FUNC*********************************************************************/
void idle_state(){
  //known cable slack length, pretty much spool/unspool until halleffect detects that reading.
  while(spoolAngle > 0 && spoolRev != 0){//checking the halleffect sensor reading to see if the original position is reached, which is the idle state fixed slack position
    motor_control(200, 0);
  }
  motor_control(0,0);
}
/****************************************************************ASCEND_STATE_FUNC*********************************************************************/
void ascend_state(){
  if (ustate == PRELIFT) {
    /*
    Identifies that the leg is going from IDLE to beign lifted. 
    Need the gyro X value to be much larger than the running average (basically taking derivative), and need make sure that the running average
      is above some threshold to ensure we don't switch states just due to noisy sensor data
    */
    if (gyroX >= 1.2*runAvg && runAvg >= 5) {
      ustate = LIFT;
      //printState();
    }
  }

  if (ustate == LIFT) {
    /*
    Identifies that the leg is about to make contact with the ground.
    Need to know that the previous XZ value was negative, since it will only contact after that has happened. We also need to take "derivative" of the XZ average
    to determine when it has started becoming positive since that change is when the foot has made contact
    */
    if (runAvg < 0 && gAvgXZ > gAvgXZ_prev) {
      ustate = PRECONTACT;
      //printState();
    }

  }

  if (ustate == PRECONTACT) {
    /*
    Identifies when the leg actually makes CONTACT with the ground.
    Need to see when the moving averages goes from negative to positive when in the precontact stage, which only occurs when we have the sinusoidal looking wave in movement.
    We can use either the value going above 0, or look for a peak where the previous value is higher than the current, since there's a chance of it not crossing the 0 line
    */
    if (runAvg > 0 || gAvgXZ < gAvgXZ_prev) {
      ustate = CONTACT;
      //printState();
    }
  }

  // NAIVE WAY OF CHECKING FOR WEIGHT ACCEPTANCE: USE THE HUMP THAT HAPPENS AFTER WE'VE MADE CONTACT
  // BETTER WAY OF CHECKING FOR WEIGHT ACCEPTANCE: TIGHETEN UNTIL WE FEEL TENSION IN THE STRING, THEN STOP UNTIL WE SEE THAT THERE IS NO TENSION (MEANS THE PERSON HAS STARTED STRAIGHTENING THEIR LEG)
  if (ustate == CONTACT) {
    /*
    Identifies when the leg needs SUPPORT from the system.
    Need to see seen, after making contact, we wait for a negative trend in data and for the value to be negative, since that will be the latest point we would want 
    to support (they have started adding force)
    */
    if (runAvg <= 0 && gAvgXZ < gAvgXZ_prev) {
      ustate = SUPPORT;
      //printState();
    }
  }

  // this can probably be better done with using the relative angles of the legs instead
  if (ustate == SUPPORT){
    if (!stopped){
      PID_Update();
    }
    /*
    Identifies that the leg no longer needs support.
    Need to see that the gyro value is above some threshold above 0, which has happened in every test. Resets the system into IDLE, in case they take a break, and then from IDLE if 
    they move their leg/are moving their leg it will trigger state to move into lift
    */
    //we keep supporting until we find that the value is above 10 or 15, since we will take 2 previous and divide by 2 to find an average (after the trough)
    if ((gAvgXZ + gAvgXZ_prev)/2 > 10) { // at this point we slack the wire going to the leg to allow it to bend. cycle repeats
      ustate = PRELIFT;
      //printState();
    }
  }   
}

/****************************************************************SENSOR_TOP_LEVEL*********************************************************************/
void sensors(){
  I2C_IMU();
  tension = SPI_tension();
}
/****************************************************************IMU_SENSOR_READINGS*********************************************************************/
void  I2C_IMU(){
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // // convert to +-8g's (converting to +-1g gave near-0 results) NOTE: THESE WERE KIND OF USELESS REGARDLESS SO I DIDN'T USE THEM AT ALL
  // float accelX = ax / 4096.0;
  // float accelY = ay / 4096.0;
  // float accelZ = az / 4096.0;

  // converting to degrees/s
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Update set "current value" and update running average
  gAvgXZ = (gyroX + gyroZ)/2;
  updateRunningAverage(gAvgXZ);

  // Calculates angles of MPU
  float angX, angY;
  calculateAngles(ax, ay, az, angX, angY);

}

void updateRunningAverage(float sample){
  runAvg -= buffer[bufferIndex] / sampleWindow; // Remove oldest sample
  // Adds next sample to buffer and updates sum
  buffer[bufferIndex] = sample;
  runAvg += sample / sampleWindow;
  // Incremenbts buffer index and wraps around
  bufferIndex = (bufferIndex + 1) % sampleWindow;
}

// Calculation for angles of the MPUs, to use absolute/relative angles to help determine state
void calculateAngles(float ax, float ay, float az, float& angX, float& angY) {//float& roll, float& pitch) {
  // roll = atan2(ay, az) * 180 / PI;
  // pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  angX = atan2(ay, sqrt( pow(ax, 2) + pow(az, 2))) * 180 / M_PI;
  angY = atan2(ax, sqrt( pow(ay, 2) + pow(az, 2))) * 180 / M_PI;
}

/****************************************************************TENSION_SENSOR_READINGS*********************************************************************/
double SPI_tension() { // HX711 Tension Sensor reading
  return hx711.get_units();
}
/****************************************************************HALL_EFFECT_SENSOR_READINGS*********************************************************************/
void as5600_get_angle() {
  static uint32_t lastTime = 0;

  as5600.getCumulativePosition();

  if (millis() - lastTime >= 100)
  {
    lastTime = millis();
    spoolAngle = fmap(as5600.readAngle(), 0, 4096, 0, 360);
    spoolRev = (as5600.getRevolutions()*-1);
    Serial.print("Angle: ");
    Serial.print(spoolAngle, 2);
    Serial.print(" | revolutions: ");
    Serial.print(spoolRev);
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

  (error >= 0) ? dir = 1: dir = 0;
  float errC = constrain(error, -20, 20);
  (dir == 1) ? output = fmap(abs(errC), 0, 20, 150, 180): output = fmap(abs(errC), 0, 20, 50, 100);

  Serial.print("TENSION ---- ");
  Serial.print(tension);
  Serial.print("---- DIRECTION ---- ");
  Serial.print(dir);
  Serial.print(" ---- ERROR ---- ");
  Serial.print(errC);
  Serial.print(" ---- OUTPUT ---- ");
  Serial.println(output);

   if (abs(errC) < 1.5){
    motor_control(0,dir);
   }
   else{
    motor_control(output,dir);
   }

}







