  #include <Wire.h>
  #include <MPU6050.h>
  #include <math.h>

  MPU6050 mpu1;
  MPU6050 mpu2(0x69); // Second MPU6050 with address 0x69

  #define BAUDRATE 9600 // Defines baudrate for quick changes


  // Variables to hold accelerometer and gyroscope data (tangential and angular acceleration)
  float aX_1, aY_1, aZ_1, gX_1, gY_1, gZ_1;
  float aX_2, aY_2, aZ_2, gX_2, gY_2, gZ_2;


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
  const int sampleWindow = 5; // averages over previous 5 samples, introduces small delay
  float buffer[sampleWindow] = {0};
  float gAvgXZ = 0; // Calculated average of the X and Z gyroscope values
  float gAvgXZ_prev = 0; // Buffer to store the previous average gXZ value
  float runAvg = 0;

  int bufferIndex = 0;

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


  /*
  Initial set up of system
  */
  void setup() {

    Serial.begin(BAUDRATE);
    Wire.begin();

    // Initialize MPU6050
    mpu1.initialize();
    mpu2.initialize();

    // if (!mpu1.testConnection() || !mpu2.testConnection()) {
      if (!mpu2.testConnection()){
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 initialized.");

    // Print CSV header to the serial monitor
    // Serial.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    Serial.println("GyroX,GyroZ,MovingAvg,AngleX1,AngleY1,AngleX2,AngleY2,RelativeX,RelativeY"); 


    //Set default to state to idle, before walking has begun
    state = IDLE;
    prevState = state;

  }

  // For visualization purposes only
  //////////////////////////////////////////////////////////////////////
  void printState() {
    Serial.print(100);
    Serial.print(",");
    Serial.print(100);
    Serial.print(",");
    Serial.print(100);
    Serial.print(",");
    Serial.print(100);
    Serial.print(",");
    Serial.println(100);
  }

  bool TOPRINT = 0; // whether or not we send the spikes for state transitions


  const int count = 5;
  int counter = 0;
  //////////////////////////////////////////////////////////////////////


  void loop() {

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
    gAvgXZ = (gyroX1 + gyroZ1)/2;
    updateRunningAverage(gAvgXZ);

    // Calculates angles of MPU
    float angX1, angY1;
    float angX2, angY2;
    calculateAngles(ax1, ay1, az1, angX1, angY1);
    calculateAngles(ax2, ay2, az2, angX2, angY2);

    /////////////// TESTING AREA FOR ANGLES ///////////////
    float shankThighX = abs(angX1 - angX2);
    float shankThighY = abs(angY1 - angY2);

    float compX1 = 0.96*gyroX1 + 0.04*angX1;
    float compX2 = 0.96*gyroX2 + 0.04*angX2;  
  /////////////////////////////////////////////////////////

    // Identifies when to swap to next state
    if (state == IDLE) {
      /*
      Identifies that the leg is going from IDLE to beign lifted. 
      Need the gyro X value to be much larger than the running average (basically taking derivative), and need make sure that the running average
       is above some threshold to ensure we don't switch states just due to noisy sensor data
      */
      if (gyroX1 >= 1.2*runAvg && runAvg >= 5) {
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
      if (runAvg < 0 && gAvgXZ > gAvgXZ_prev) {
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
      if (runAvg > 0 || gAvgXZ < gAvgXZ_prev) {
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
      if (runAvg <= 0 && gAvgXZ < gAvgXZ_prev) {
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
      if ((gAvgXZ + gAvgXZ_prev)/2 > 10) { // at this point we slack the wire going to the leg to allow it to bend. cycle repeats
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
    
    // Send data to the serial monitor for logging
    /*  
    Serial.print(accelX);
    Serial.print(",");
    Serial.print(accelY);
    Serial.print(",");
    Serial.print(accelZ);
    Serial.print(",");
    */
    if ((counter % count) == 0) { // prints every 10th sample only
    // Serial.print(gyroX1);
    // Serial.print(",");
    // Serial.print(gyroZ1);
    // Serial.print(",");
    // Serial.print(runAvg);
    // Serial.print(",");
    Serial.print(angX1);
    Serial.print(",");
    // Serial.print(angY1);
    // Serial.print(",");
    Serial.print(angX2);
    Serial.print(",");
    // Serial.print(angY2);
    // Serial.print(",");
    Serial.print(shankThighX);
    Serial.print(",");
    // Serial.println(shankThighY);

    Serial.println(sTX2);
    }
    counter += 1;

    delay(25);
  }