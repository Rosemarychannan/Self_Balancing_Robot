#include <math.h>
#include <Arduino_BMI270_BMM150.h>

//defining arduino pins for readability
#define BIN1 21
#define BIN2 23
#define AIN2 24
#define AIN1 27
#define A 1
#define B 2

float old_theta = 0; 
float gyro_theta = 0;
unsigned long start_time = 0;

//Functions for motor controll
void motorOff(int num){
  
  if(num == A){
    analogWrite(AIN1,255);
    analogWrite(AIN2,255);
  }
  else if(num == B){
    analogWrite(BIN1,255);
    analogWrite(BIN2,255);
  }
}
void forward(int num, int pwm_percent){
  int duty_cycle = 255.0f * pwm_percent / 100.0f;
  int 
  if(num == A){
    analogWrite(AIN1,255);
    analogWrite(AIN2,duty_cycle);
  }
  else if(num == B){
    analogWrite(BIN1,255);
    analogWrite(BIN2,duty_cycle);
  }
}
void reverse(int num, int pwm_percent){
  int duty_cycle = 255.0f * pwm_percent / 100.0f;
  if(num == A){
    analogWrite(AIN1,duty_cycle);
    analogWrite(AIN2,255);
  }
  else if(num == B){
    analogWrite(BIN1,duty_cycle);
    analogWrite(BIN2,255);
  }
}
void both_motorOff(){
  analogWrite(AIN1,255);
  analogWrite(AIN2,255);
  analogWrite(BIN1,255);
  analogWrite(BIN2,255);
}
void both_forward(int pwm_percent){
  forward(A,pwm_percent);
  forward(B,pwm_percent);
}
void both_reverse(int pwm_percent){
  reverse(A,pwm_percent);
  reverse(B,pwm_percent);
}
void opposite_A(int pwm_percent){
  int re_pwm_percent = 100 - pwm_percent;
  forward(A,pwm_percent);
  reverse(B,re_pwm_percent);
}
void opposite_B(int pwm_percent){
  int re_pwm_percent = 100 - pwm_percent;
  forward(B,pwm_percent);
  reverse(A,re_pwm_percent);
}
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);

  Serial.println("Setup Finished");
  start_time = micros();
}

void loop() {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float acc_theta, theta;
  float k = 0.6;

  int pwm_percent;

  //sensor measurement and angle calculation
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    // Read accelerometer
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    acc_theta = atan2(acc_y, acc_z) * 180.0 / M_PI;

    // Read gyroscope
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    gyro_x = -gyro_x;  // Axis alignment

    // Calculate delta time in seconds
    unsigned long now = micros();
    float dt = (now - start_time) / 1000000.0;
    start_time = now;

    // Integrate gyro to get angle
    gyro_theta += gyro_x * dt;

    // Complementary filter
    theta = (1 - k) * acc_theta + k * (old_theta + gyro_x * dt);
    old_theta = theta;

    // Print values
    Serial.print(acc_theta);
    Serial.print(" ");
    Serial.print(gyro_theta);
    Serial.print(" ");
    Serial.println(theta);
  }
  //MAYBE WE SHOULD PUT ALL THE FOLLOWING IN THE IF STATEMENT ABOVE SO IT ONLY WORKS ON VALID ANGLES, NOT TOO SURE HERE
  //calculating poritive percentage of PWM needed based on tilt angle
  pwm_percent = 100.0 - 100.0*(90.0 - fabs(theta))/90.0; //fabs() finds absolute value
  Serial.println(pwm_percent); //print value for debugging

  // assuming positive means needs to go in reverse to bring back to zero here, and vise versa
  if(pwm_percent <= 100 && pwm_percent >= 0){     //make sure not to send invalid data
    if(pwm_percent == 0){
        both_motorOff();
      }
    else{
      if(theta < 0) both_forward(pwm_percent);  //use theta to determine sign (direction) since pwm_percent is always positive
      else if(theta > 0) both_reverse(pwm_percent);
      else both_motorOff();
    }
  }
  else{
    Serial.println("Invalid Data");
  }
}
