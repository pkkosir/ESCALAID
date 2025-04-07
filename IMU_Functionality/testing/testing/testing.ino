#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu1;
MPU6050 mpu2(0x69);  // Second MPU6050 with address 0x69

#define BAUDRATE 9600  // Defines baudrate for quick changes

#define ANGLE 135 // Defines the angle used for threshold of PID stopping

// Variables to hold the global angle data
float glob_angX1 = 70, glob_angY1 = 0; //defaults changed to take less time to find correct angle. CHECK THESE!!!!!!!!!!!
float glob_angX2 = -70, glob_angY2 = 0;

// Variables for the complementary filter for more accurate angle measurements
unsigned long prevTime = 0;
unsigned long currTime = 0;
float dt;
float alpha = 0.7;

// For visualization purposes only
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void printState() {
  Serial.print(100);
  Serial.print(",");
  Serial.print(100);
  Serial.print(",");
  Serial.println(100);
}

bool TOPRINT = 1;  // whether or not we send the spikes for state transitions

const int count = 1;  // for outputting every "count-th" sample
int counter = 0;
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

// STATE variable for holding which state our system is in
/*
  0 -> IDLE
  1 -> TROUGH (foot is being lifted up, not giving support)
  2 -> PEAK (foot is on downswing, used for internal tracking purposes)
  4 -> SUPPORT (from some number of samples aroudn 0, we know to start straightening the leg)
  */
int state;
int prevState;
#define IDLE 0
#define TROUGH 1
#define PEAK 2
#define SUPPORT 3

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

/*
  Initial set up of system
  */
void setup() {

  Serial.begin(BAUDRATE);
  Wire.begin();

  // Initialize MPU6050
  mpu1.initialize();
  mpu2.initialize();

  if (!mpu1.testConnection() || !mpu2.testConnection()) {
    if (!mpu1.testConnection()) Serial.println("MPU1 failed");
    if (!mpu2.testConnection()) Serial.println("MPU2 failed");
    Serial.println("MPU6050 connection failed!");
    while (1)
      ;
  }
  Serial.println("MPU6050 initialized.");

  // Print CSV header to the serial monitor
  Serial.println("X1_AVG,X2_AVG,RELATIVE_ANG");


  //Set default to state to idle, before walking has begun
  state = IDLE;
  prevState = state;

  // Identifies previous time for use in complementary filter
  prevTime = millis();
}

void loop() {

  currTime = millis();                  // Current time for comp filter
  dt = (currTime - prevTime) / 1000.0;  // Time difference for integration
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
  // combX = (gyroX1 + gyroX2)/2; //reduces subtractive noise
  updateRunningAverages(gyroX1, gyroX2); //running average on previous 2 combines samples
  // runAvg = runAvgX; //we don't need more smoothing than we already do

  // Calculates angles of MPU w/ complementary filter
  float angX1 = glob_angX1, angY1 = glob_angY1;  // Localizing globals to be able to pass into function
  float angX2 = glob_angX2, angY2 = glob_angY2;
  calcAngles(ax1, ay1, az1, gyroX1, gyroY1, angX1, angY1);
  calcAngles(ax2, ay2, az2, gyroX2, gyroY2, angX2, angY2);

  glob_angX1 = angX1;  // Sending newly calculated variables back to global
  glob_angY1 = angY1;
  glob_angX2 = angX2;
  glob_angY2 = angY2;

  float shankThighX = abs(angX1 - angX2);  // CONSISTENTLY BETWEEN 115 AND 170. STRAIGHT LEG IS ~140-160. SHOULD DO MANY SAMPLES READING ABOVE 140 TO BE SAFE

  // Identifies when to swap to next state
  if (state == IDLE) {
    /*
      Identifies that the leg is going from IDLE to beign lifted. 
      Need the gyro X value to be much larger than the running average (basically taking derivative), and need make sure that the running average
      is above some threshold to ensure we don't switch states just due to noisy sensor data
      */
    
    if (runAvgX1 <= -40 || runAvgX2 <= -40) { //trough ALWAYS below -40
      if (allowed >= allowance) {
        state = TROUGH; //moves to when data will look like a trough, needed to differentiate next state
        if (TOPRINT) printState();
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }

  if (state == TROUGH) {
    /*
      Identifies that the leg is about to make contact with the ground.
      Need to know when we go above a certain acceleration value, which happens when we swap directions
      */
    if (runAvgX1 >= 35 && runAvgX2 <= 0) {  // peak is ALWAYS above 35 for X1 and below 0 for X2
      if (allowed >= allowance) {
        state = PEAK;
        if (TOPRINT) printState();
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }


  // CONCERNS: THE WAY THAT RYAN IS WALKING MAY NOT ALLOW FOR US TO REALLY DISTINGUISH WHEN HE NEEDS SUPPORT. HE MAY NEED TO SLOW DOWN SO THAT WE CAN GET BETTER READINGS FOR THE DEMO
  if (state == PEAK) {
    /*
      Identifies when the leg actually makes CONTACT with the ground.
      This will be identified by the movement of the leg being ~0
      */
    if (runAvgX1 <= 5 ) {
      if (allowed >= 3*allowance) { //want at least 5 samples, but may not be feasible
        state = SUPPORT;

        //SEND SIGNAL TO PID TO START HERE

        if (TOPRINT) printState();
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }

  if (state == SUPPORT) {
    /*
      Identifies that the leg no longer needs support.
      Need to see that the angle is above some threshold (120), which has happened in every test. Resets the system into IDLE, in case they take a break, and then from IDLE if 
      they move their leg/are moving their leg it will trigger state to move into lift
      */
    if (shankThighX >= ANGLE) {  // straight leg is anything above ~120 degrees (for Ryan's leg)
        if (allowed >= 5*allowance) { //allows for some delay (aka straight leg) before turning off support. This should account for a few outlier readings above 120, to ensure that the leg is truly straight before releasing support 
        state = IDLE;

        //SEND SIGNAL TO TURN OFF PID

        if (TOPRINT) printState();
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }

  // runAvg_prev = runAvg;


  // Send data to the serial monitor for logging
  if ((counter % count) == 0) {  // prints every count-th sample only
    // Serial.print(gyroX1);
    // Serial.print(",");
    // Serial.print(gyroX2);
    // Serial.print(",");
    Serial.print(runAvgX1);
    Serial.print(",");
    Serial.print(runAvgX2);
    Serial.print(",");
    // Serial.print(angX2);
    // Serial.print(",");
    // Serial.print(angX1);
    // Serial.print(",");
    Serial.println(shankThighX);

  }
  counter += 1;

  delay(25);
}