  #include <Wire.h>
  #include <MPU6050.h>
  #include <math.h>

  MPU6050 mpu1;
  MPU6050 mpu2(0x69); // Second MPU6050 with address 0x69

  #define BAUDRATE 9600 // Defines baudrate for quick changes

  // // Variables to hold accelerometer and gyroscope data (tangential and angular acceleration)
  // float aX_1, aY_1, aZ_1, gX_1, gY_1, gZ_1;
  // float aX_2, aY_2, aZ_2, gX_2, gY_2, gZ_2;
  // Variables to hold the global angle data
  float glob_angX1 = 0, glob_angY1 = 0;
  float glob_angX2 = 0, glob_angY2 = 0;

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
    // Serial.print(100);
    // Serial.print(",");
    // Serial.print(100);
    // Serial.print(",");
    Serial.print(100);
    Serial.print(",");
    Serial.print(100);
    Serial.print(",");
    Serial.println(100);
  }

  bool TOPRINT = 1; // whether or not we send the spikes for state transitions

  const int count = 1;
  int counter = 0;
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////

  // STATE variable for holding which state our system is in
  /*
  0 -> IDLE
  1 -> LIFT (foot is being lifted up, not giving support)
  2 -> PRECONTACT (foot is on downswing, used for internal tracking purposes)
  3 -> CONTACT (foot has contacted the floor)
  4 -> SUPPORT (from some time-step/other parameter, we know to start straightening the leg)
  */
  int state; 
  int prevState;
  #define IDLE 0
  #define LIFT 1
  #define PRECONTACT 2
  #define CONTACT 3
  #define SUPPORT 4

  // Allowance for outliers when checking for trends in the state transitions
  const int allowance = 1; // just 1 outlier for now, ensures that we don't preemptively change states
  int allowed = 0;

  // Variables and data structures for running average, taken from the average of X and Z gyroscope values (same trend, allows for initial smoothing)
  const int sampleWindow = 3; // averages over previous 3 samples, introduces small delay
  float bufferX[sampleWindow] = {0};
  float bufferZ[sampleWindow] = {0};

  float runAvgX = 0; // Running average for the previous 3 values before XZ averaging
  float runAvgZ = 0;

  float gAvgXZ = 0; // Calculated average of the X and Z gyroscope values
  float runAvg = 0;
  float runAvg_prev = 0; // Buffer to store the previous running average for comparison

  int bufferIndex = 0;

  /*
  Helper methods
  */
  void updateRunningAverages(float sampleX, float sampleZ){
    
    runAvgX -= bufferX[bufferIndex] / sampleWindow; // Remove oldest sample
    runAvgZ -= bufferZ[bufferIndex] / sampleWindow;

    // Adds next sample to buffer and updates sum
    bufferX[bufferIndex] = sampleX;
    bufferZ[bufferIndex] = sampleZ;

    runAvgX += sampleX / sampleWindow;
    runAvgZ += sampleZ / sampleWindow;

    // Incremenbts buffer index and wraps around
    bufferIndex = (bufferIndex + 1) % sampleWindow;

  }

  // Calculation for angles of the MPUs, to use absolute/relative angles to help determine state
  void calcAngles(float ax, float ay, float az, float gx, float gy, float& angX, float& angY) {

    float accelAngX = atan2(ay, sqrt( pow(ax, 2) + pow(az, 2))) * 180 / M_PI;
    float accelAngY = atan2(ax, sqrt( pow(ay, 2) + pow(az, 2))) * 180 / M_PI;

    // Gyro angles, integrated
    angX = alpha*(angX + gx*dt) + (1 - alpha)*accelAngX; //dt is global, implicit replacement of angX w/ current angX with integrated gx
    angY = alpha*(angY + gy*dt) + (1 - alpha)*accelAngY;

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
      while (1);
    }
    Serial.println("MPU6050 initialized.");

    // Print CSV header to the serial monitor
    // Serial.println("GyroX,GyroZ,MovingAvg,AngleX1,AngleY1,AngleX2,AngleY2,RelativeX,RelativeY"); 
    Serial.println("ZERO,GyroX1,GyroZ1,RunAvg"); 
    // Serial.println("angX1,angY1,angX2,angY2,shankThighX,shankThighY");


    //Set default to state to idle, before walking has begun
    state = IDLE;
    prevState = state;

    // Identifies previous time for use in complementary filter
    prevTime = millis();

  }

  void loop() {

    currTime = millis(); // Current time for comp filter
    dt = (currTime - prevTime) / 1000.0; // Time difference for integration
    prevTime = currTime;

    int16_t ax1, ay1, az1, gx1, gy1, gz1;
    int16_t ax2, ay2, az2, gx2, gy2, gz2;

    mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

    // Converting to physical units

    // // convert to +-8g's (converting to +-1g gave near-0 results) NOTE: THESE WERE KIND OF USELESS REGARDLESS SO I DIDN'T USE THEM AT ALL
    // float accelX1 = ax1 / 4096.0; float accelY1 = ay1 / 4096.0; float accelZ1 = az1 / 4096.0;
    // converting to degrees/s
    float gyroX1 = gx1 / 131.0; float gyroY1 = gy1 / 131.0; float gyroZ1 = gz1 / 131.0;

    // float accelX2 = ax2 / 4096.0; float accelY2 = ay2 / 4096.0; float accelZ2 = az2 / 4096.0;
    float gyroX2 = gx2 / 131.0; float gyroY2 = gy2 / 131.0; float gyroZ2 = gz2 / 131.0;

    // Update set "current value" and update running average for SHANK angular accleration
    updateRunningAverages(-1*gyroX1, gyroZ1);
    gAvgXZ = (runAvgX + runAvgZ)/2; 
    runAvg = (gAvgXZ + runAvg_prev)/2; // Gives another "running average", taking the current calc and previous average into account (better smoothing)

    // Calculates angles of MPU w/ complementary filter
    float angX1 = glob_angX1, angY1 = glob_angY1; // Localizing globals to be able to pass into function
    float angX2 = glob_angX2, angY2 = glob_angY2;
    calcAngles(ax1, ay1, az1, gyroX1, gyroY1, angX1, angY1);
    calcAngles(ax2, ay2, az2, gyroX2, gyroY2, angX2, angY2); 

    glob_angX1 = angX1; // Sending newly calculated variables back to global
    glob_angY1 = angY1;
    glob_angX2 = angX2;
    glob_angY2 = angY2; 

    /////////////// TESTING AREA FOR ANGLES ///////////////
    float shankThighX = abs(angX1 - angX2);
    float shankThighY = abs(angY1 - angY2);
    ///////////////////////////////////////////////////////

    // Identifies when to swap to next state
    if (state == IDLE) {
      /*
      Identifies that the leg is going from IDLE to beign lifted. 
      Need the gyro X value to be much larger than the running average (basically taking derivative), and need make sure that the running average
       is above some threshold to ensure we don't switch states just due to noisy sensor data
      */
      if (runAvg >= 1.2*runAvg_prev && runAvg >= 5) {
        if (allowed >= allowance) {
          state = LIFT;
          if (TOPRINT) printState();
          allowed = 0;
        }
        else {
          allowed += 1;
        }
      }
    }

    if (state == LIFT) { // NOTE: no outlier detection for this, we want to know about this event happening ASAP and if it happens due to outlier it's non-consequential
      /*
      Identifies that the leg is about to make contact with the ground.
      Need to know that the previous XZ value was negative, since it will only contact after that has happened. We also need to take "derivative" of the XZ average
      to determine when it has started becoming positive since that change is when the foot has made contact
      */
      if (runAvg < 0 && runAvg > runAvg_prev &&  runAvg <= -15) { // -15 clause added to avoid the case where value dips after the leg is straightened
        state = PRECONTACT;
        if (TOPRINT) printState();
      }
    }

    if (state == PRECONTACT) {
      /*
      Identifies when the leg actually makes CONTACT with the ground.
      Need to see when the moving averages goes from negative to positive when in the precontact stage, which only occurs when we have the sinusoidal looking wave in movement.
      We can use either the value going above 0, or look for a peak where the previous value is higher than the current, since there's a chance of it not crossing the 0 line
      */
      if (runAvg > 0 || runAvg < runAvg_prev) {
        if (allowed >= allowance) {
          state = CONTACT;
          if (TOPRINT) printState();
          allowed = 0;
        }
        else {
          allowed += 1;
        }
      }
    }

    // NAIVE WAY OF CHECKING FOR WEIGHT ACCEPTANCE: USE THE HUMP THAT HAPPENS AFTER WE'VE MADE CONTACT
    // BETTER WAY OF CHECKING FOR WEIGHT ACCEPTANCE: TIGHETEN UNTIL WE FEEL TENSION IN THE STRING, THEN STOP UNTIL WE SEE THAT THERE IS NO TENSION (MEANS THE PERSON HAS STARTED STRAIGHTENING THEIR LEG)
    if (state == CONTACT) {
      /*
      Identifies when the leg needs SUPPORT from the system.
      Need to see seen, after making contact, we wait for a negative trend in data and for the value to be negative, since that will be the latest point we would want 
      to support (they have started adding force)
      */
      if (runAvg <= 0 && runAvg < runAvg_prev) {
        if (allowed >= allowance) {
          state = SUPPORT;
          if (TOPRINT) printState();
          allowed = 0;
        }
        else {
          allowed += 1;
        }
      }
    }

    // this can probably be better done with using the relative angles of the legs instead (above 15 degrees is def bent, below 12 degrees is def straight) <- the small window is fine
    if (state == SUPPORT){
      /*
      Identifies that the leg no longer needs support.
      Need to see that the gyro value is above some threshold above 0, which has happened in every test. Resets the system into IDLE, in case they take a break, and then from IDLE if 
      they move their leg/are moving their leg it will trigger state to move into lift
      */
      //we keep supporting until we find that the value is above 10 or 15, since we will take 2 previous and divide by 2 to find an average (after the trough)
      // if ((gAvgXZ + gAvgXZ_prev)/2 > 10) { 
        if (shankThighX <= 15) { // want to check for angles to be near 0, when it's bent it will be 20+ degrees 
        if (allowed >= allowance) { 
          state = IDLE;
          if (TOPRINT) printState();
          allowed = 0;
        }
        else {
          allowed += 1;
        }
      }
    }
    
    runAvg_prev = runAvg;
    // Send data to the serial monitor for logging
    if ((counter % count) == 0) { // prints every count-th sample only
    // Serial.print(0);
    // Serial.print(",");
    Serial.print(gyroX1); 
    Serial.print(",");
    // Serial.print(gyroX2);
    // Serial.print(",");
    // Serial.print(gyroY1); 
    // Serial.print(",");
    // Serial.print(gyroY2); 
    // Serial.print(",");
    Serial.print(gyroZ1); 
    Serial.print(",");
    // Serial.println(gyroZ2); 
    // Serial.print(",");
    Serial.println(runAvg);
    // Serial.print(",");
    // Serial.println(shankThighX);
    // Serial.print(",");
    // Serial.println(shankThighY);

    }
    counter += 1;

    delay(25);
  }