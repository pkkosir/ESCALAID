#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variables to hold accelerometer and gyroscope data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Variables for gait detection
bool footOnGround = false;
unsigned long lastTransitionTime = 0;
const unsigned long debounceTime = 200; // milliseconds

void setup() {

  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
      while (1);
  }
  Serial.println("MPU6050 initialized.");

  // Print CSV header to the serial monitor
  // Serial.println("Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,FootState");
  Serial.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");

}

void loop() {

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // convert to g's
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  // converting to degrees/s
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Send data to the serial monitor for logging
  // Serial.print(millis());
  // Serial.print(",");
  Serial.print(accelX);
  Serial.print(",");
  Serial.print(accelY);
  Serial.print(",");
  Serial.print(accelZ);
  Serial.print(",");
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.println(gyroZ);

  delay(100); // THIS DETERMINES HOW MANY SAMPLES WE GET PER SECOND. TURNING THIS DOWN WILL GIVE US MORE SAMPLES TO USE PER SECOND
}
