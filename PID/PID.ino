#include "HX711.h" //This library can be obtained here http://librarymanager/All#Avia_HX711
#include "PID_v1.h"

#define PID_input A1
#define PID_output 6

double setPoint = 0;
double input = 0;
double error = 0;
int output = 0;
int dir = 0, lastDir = 0; //0 is cw, 1 is ccw
int braked = 0;
double Kp = 5, Ki = 0.8, Kd = 0.15;
PID controller(&input, &error, &setPoint, Kp, Ki, Kd, P_ON_E, DIRECT);


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


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

void setup() {
  Serial.begin(9600);
  HX711_setup();
  mc_init();

  //initialize PID
  input = HX711_reading();
  setPoint = 3;
  controller.SetOutputLimits(-255, 255);
  controller.SetMode(AUTOMATIC);
  delay(1000);
  
  
  digitalWrite(mc_IN1, HIGH);
  digitalWrite(mc_IN2, LOW);
  analogWrite(mc_PWM,100);
  delay(50);
  analogWrite(mc_PWM,0);
  
}

void loop() {
  delay(10);
//  if (HX711_reading() > 10){
//    stopped = 1;
//    digitalWrite(mc_IN1, LOW);
//    digitalWrite(mc_IN2, HIGH);
//    analogWrite(mc_PWM, 200);
//    delay(1000);
//  }
  if (!stopped){
  PID_Update();}


  //spool accordingly to computation
  //spool();
  //schmidty();

}

void PID_Update(){
  input = HX711_reading();
  controller.Compute();

  (error >= 0) ? dir = 0: dir = 1;
  float errC = constrain(error, -20, 20);
  (dir == 0) ? output = fmap(abs(errC), 0, 20, 150, 180): output = fmap(abs(errC), 0, 20, 50, 100);

  Serial.print("TENSION ---- ");
  Serial.print(input);
  Serial.print("---- DIRECTION ---- ");
  Serial.print(dir);
  Serial.print(" ---- ERROR ---- ");
  Serial.print(errC);
  Serial.print(" ---- OUTPUT ---- ");
  Serial.println(output);

   if (dir == 0 && lastDir != 0){
     digitalWrite(mc_IN1, HIGH);
     digitalWrite(mc_IN2, LOW);
     lastDir = dir;
   }
   else if (dir == 1 && lastDir != 1){
     digitalWrite(mc_IN1, LOW);
     digitalWrite(mc_IN2, HIGH);
     lastDir = dir;
   }
   if (abs(errC) < 1.5){
     analogWrite(mc_PWM, 0);
     braked = 1;
   }
   else{
     analogWrite(mc_PWM, output);
     braked = 0;
   }
}

void HX711_setup() { // HX711 Tension Sensor Init
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  hx711.set_scale();
  hx711.tare();  //Reset the scale to 0
  long zero_factor = hx711.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
  delay(100);
}

double HX711_reading() { // HX711 Tension Sensor reading
  hx711.set_scale(calibration_factor); //Adjust to this calibration factor
  return hx711.get_units();
}

void mc_init() {
  pinMode(mc_IN1, OUTPUT);
  pinMode(mc_IN2, OUTPUT);
  pinMode(mc_PWM, OUTPUT);
}

void mc_clockwise(int mc_speed) {
  if (prev_mc_state != 1 && mc_state == 1) {
    digitalWrite(mc_IN1, HIGH);
    digitalWrite(mc_IN2, LOW);
    analogWrite(mc_PWM, mc_speed);
    prev_mc_state = 1;
  }
}

void mc_counterclockwise(int mc_speed) {
  if (prev_mc_state != 2 && mc_state == 2) {
    digitalWrite(mc_IN1, LOW);
    digitalWrite(mc_IN2, HIGH);
    analogWrite(mc_PWM, mc_speed);
    prev_mc_state = 2;
  }
}

void mc_brake() {
  if (prev_mc_state != 0 && mc_state == 0) {
    digitalWrite(mc_IN1, LOW);
    digitalWrite(mc_IN2, LOW);
    analogWrite(mc_PWM, 0);
    prev_mc_state = 0;
  }
}


void schmidty(){
  if (HX711_reading() >= upper_sensor){
    mc_state = 2;
    mc_counterclockwise(255);
    delay(10);
  }
  else if (HX711_reading() <= lower_sensor){
    mc_state = 1;
    mc_clockwise(255);
    delay(10);
  }
  else{
    mc_state = 0;
    mc_brake();
  }
}
