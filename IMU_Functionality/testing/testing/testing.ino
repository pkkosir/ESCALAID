#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu1;
MPU6050 mpu2(0x69);  // Second MPU6050 with address 0x69

#define BAUDRATE 9600  // Defines baudrate for quick changes

// // Variables to hold accelerometer and gyroscope data (tangential and angular acceleration)
// float aX_1, aY_1, aZ_1, gX_1, gY_1, gZ_1;
// float aX_2, aY_2, aZ_2, gX_2, gY_2, gZ_2;
// Variables to hold the global angle data
float glob_angX1 = 65, glob_angY1 = 0; //defaults changed to take less time to find correct angle. CHECK THESE!!!!!!!!!!!
float glob_angX2 = -65, glob_angY2 = 0;

// Variables for the complementary filter for more accurate angle measurements
unsigned long prevTime = 0;
unsigned long currTime = 0;
float dt;
float alpha = 0.96;

// For visualization purposes only
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void printState() {

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
const int sampleWindow = 2;  // averages over previous 3 samples, introduces small delay
float bufferX[sampleWindow] = { 0 };
// float bufferZ[sampleWindow] = { 0 };

float combX = 0;
float runAvgX = 0;  // Running average for the previous 3 values before XZ averaging
// float runAvgZ = 0;

// float gAvgXZ = 0;  // Calculated average of the X and Z gyroscope values
float runAvg = 0;
float runAvg_prev = 0;  // Buffer to store the previous running average for comparison

int bufferIndex = 0;

/*
  Helper methods
  */
void updateRunningAverages(float sampleX) { //, float sampleZ) {

  runAvgX -= bufferX[bufferIndex] / sampleWindow;  // Remove oldest sample
  // runAvgZ -= bufferZ[bufferIndex] / sampleWindow;

  // Adds next sample to buffer and updates sum
  bufferX[bufferIndex] = sampleX;
  // bufferZ[bufferIndex] = sampleZ;

  runAvgX += sampleX / sampleWindow;
  // runAvgZ += sampleZ / sampleWindow;

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
  Serial.println("RUN_AVG, RELATIVE_ANG");


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
  // float accelX1 = ax1 / 4096.0; float accelY1 = ay1 / 4096.0; float accelZ1 = az1 / 4096.0;
  // converting to degrees/s
  float gyroX1 = gx1 / 131.0;
  float gyroY1 = gy1 / 131.0;
  float gyroZ1 = gz1 / 131.0;

  // float accelX2 = ax2 / 4096.0; float accelY2 = ay2 / 4096.0; float accelZ2 = az2 / 4096.0;
  float gyroX2 = gx2 / 131.0;
  float gyroY2 = gy2 / 131.0;
  float gyroZ2 = gz2 / 131.0;

  // Update set "current value" and update running average for SHANK angular accleration
  combX = (gyroX1 + gyroX2)/2; //reduces subtractive noise
  updateRunningAverages(combX); //running average on previous 2 combines samples
  runAvg = runAvgX; //we don't need more smoothing than we already do

  // gAvgXZ = (runAvgX + runAvgZ) / 2;
  // runAvg = (runAvgX + runAvg_prev) / 2;  // Gives another "running average", taking the current calc and previous average into account (better smoothing)

  // Calculates angles of MPU w/ complementary filter
  float angX1 = glob_angX1, angY1 = glob_angY1;  // Localizing globals to be able to pass into function
  float angX2 = glob_angX2, angY2 = glob_angY2;
  calcAngles(ax1, ay1, az1, gyroX1, gyroY1, angX1, angY1);
  calcAngles(ax2, ay2, az2, gyroX2, gyroY2, angX2, angY2);

  glob_angX1 = angX1;  // Sending newly calculated variables back to global
  glob_angY1 = angY1;
  glob_angX2 = angX2;
  glob_angY2 = angY2;

  /////////////// TESTING AREA FOR ANGLES ///////////////
  float shankThighX = abs(angX1 - angX2);  // CONSISTENTLY BETWEEN 115 AND 170. STRAIGHT LEG IS ~140-160. SHOULD DO MANY SAMPLES READING ABOVE 140 TO BE SAFE
  // float shankThighY = abs(angY1 - angY2);
  ///////////////////////////////////////////////////////

  // Identifies when to swap to next state
  if (state == IDLE) {
    /*
      Identifies that the leg is going from IDLE to beign lifted. 
      Need the gyro X value to be much larger than the running average (basically taking derivative), and need make sure that the running average
      is above some threshold to ensure we don't switch states just due to noisy sensor data
      */
    
    if (runAvg <= -25) { //trough ALWAYS below -25
      if (allowed >= allowance) {
        state = TROUGH; //moves to when data will look like a trough, needed to differentiate next state
        if (TOPRINT) printState();
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }

  if (state == TROUGH) {  //
    /*
      Identifies that the leg is about to make contact with the ground.
      Need to know when we go above a certain acceleration value, which happens when we swap directions
      */
    if (runAvg >= 35) {  // peak is ALWAYS above 35
      if (allowed >= allowance) {
        state = PEAK;
        if (TOPRINT) printState();
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }

  if (state == PEAK) {
    /*
      Identifies when the leg actually makes CONTACT with the ground.
      This will be identified by the movement of the leg being ~0
      */
    if (-5 <= runAvg <=5 ) {
      if (allowed >= 20*allowance) { //want at least 5 samples, but saw 20 was better for the speed at which I was moving
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
      Need to see that the angle is above some threshold (140), which has happened in every test. Resets the system into IDLE, in case they take a break, and then from IDLE if 
      they move their leg/are moving their leg it will trigger state to move into lift
      */
    if (shankThighX >= 150) {  // straight leg is anything above ~150 degrees
        if (allowed >= 10*allowance) { //allows for some delay (aka straight leg) before turning off support. This should account for a few outlier readings above 150, to ensure that the leg is truly straight before releasing support 
        state = IDLE;

        //SEND SIGNAL TO TURN OFF PID

        if (TOPRINT) printState();
        allowed = 0;
      } else {
        allowed += 1;
      }
    }
  }

  runAvg_prev = runAvg;
  // Send data to the serial monitor for logging
  if ((counter % count) == 0) {  // prints every count-th sample only
    // Serial.print(gyroX1);
    // Serial.print(",");
    // Serial.print(gyroX2);
    // Serial.print(",");
    // Serial.print(gyroY1);
    // Serial.print(",");
    // Serial.print(gyroY2);
    // Serial.print(",");
    // Serial.print(gyroZ1);
    // Serial.print(",");
    // Serial.print(gyroZ2);
    // Serial.print(",");
    Serial.print(runAvg);
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