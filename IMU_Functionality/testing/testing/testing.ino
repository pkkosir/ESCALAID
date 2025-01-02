  #include <Wire.h>
  #include <MPU6050.h>
  #include <math.h>

  MPU6050 mpu;

  // Variables to hold accelerometer and gyroscope data
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;

  // STATE variable for holding which state our system is in
  /*
  0 -> IDLE
  1 -> LIFT (foot is being lifted up, not giving support)
  2 -> CONTACT (foot has contacted the floor)
  3 -> SUPPORT (from some time-step/other parameter, we know to start straightening the leg)
  */
  int state; 
  int prevState;
  #define IDLE 0
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

    Serial.begin(9600);
    // Serial.begin(600);
    Wire.begin();

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 initialized.");

    // Print CSV header to the serial monitor
    // Serial.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    Serial.println("GyroX,GyroZ,MovingAvg,AngleX,AngleY"); 


    //Set default to state to idle, before walking has begun
    state = IDLE;
    prevState = state;

  }

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


  void loop() {

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

    // Identifies when to swap to next state
    if (state == IDLE) {
      /*
      Identifies that the leg is going from IDLE to beign lifted. 
      Need the gyro X value to be much larger than the running average (basically taking derivative), and need make sure that the running average
       is above some threshold to ensure we don't switch states just due to noisy sensor data
      */
      if (gyroX >= 1.2*runAvg && runAvg >= 5) {
        state = LIFT;
        printState();
      }
    }

    if (state == LIFT) {
      /*
      Identifies that the leg is about to make contact with the ground.
      Need to know that the previous XZ value was negative, since it will only contact after that has happened. We also need to take "derivative" of the XZ average
      to determine when it has started becoming positive since that change is when the foot has made contact
      */
      if (runAvg < 0 && gAvgXZ > gAvgXZ_prev) {
        state = PRECONTACT;
        printState();
      }

    }

    if (state == PRECONTACT) {
      /*
      Identifies when the leg actually makes CONTACT with the ground.
      Need to see when the moving averages goes from negative to positive when in the precontact stage, which only occurs when we have the sinusoidal looking wave in movement.
      We can use either the value going above 0, or look for a peak where the previous value is higher than the current, since there's a chance of it not crossing the 0 line
      */
      if (runAvg > 0 || gAvgXZ < gAvgXZ_prev) {
        state = CONTACT;
        printState();
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
        state = SUPPORT;
        printState();
      }
    }

    // this can probably be better done with using the relative angles of the legs instead
    if (state == SUPPORT){
      /*
      Identifies that the leg no longer needs support.
      Need to see that the gyro value is above some threshold above 0, which has happened in every test. Resets the system into IDLE, in case they take a break, and then from IDLE if 
      they move their leg/are moving their leg it will trigger state to move into lift
      */
      //we keep supporting until we find that the value is above 10 or 15, since we will take 2 previous and divide by 2 to find an average (after the trough)
      if ((gAvgXZ + gAvgXZ_prev)/2 > 10) { // at this point we slack the wire going to the leg to allow it to bend. cycle repeats
        state = IDLE;
        printState();
      }
    }
    
    // Send data to the serial monitor for logging
    // Serial.print(accelX);
    // Serial.print(",");
    // Serial.print(accelY);
    // Serial.print(",");
    // Serial.print(accelZ);
    // Serial.print(",");

    Serial.print(gyroX);
    Serial.print(",");
    // Serial.print(gyroY);
    // Serial.print(",");
    Serial.print(gyroZ);
    Serial.print(",");
    Serial.print(runAvg);
    Serial.print(",");
    Serial.print(angX);
    Serial.print(",");
    Serial.println(angY);

    delay(25);
  }