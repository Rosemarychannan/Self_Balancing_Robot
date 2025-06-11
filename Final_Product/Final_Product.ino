#include <math.h>
#include <Arduino_BMI270_BMM150.h>

//defining arduino pins for readability
#define BIN1 21
#define BIN2 23
#define AIN2 24
#define AIN1 27
#define A 1
#define B 2

float k = 0.6;
float kp = 1;
float ki = 1;
float kd = 1;
float old_theta = 0; 
float gyro_theta = 0;
unsigned long start_time = 0;
float desired_angle, error, ki, kp, kd;
float old_error = 0;

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
void forward(int num, int pwm){
  if(num == A){
    analogWrite(AIN1,255);
    analogWrite(AIN2,pwm);
  }
  else if(num == B){
    analogWrite(BIN1,255);
    analogWrite(BIN2,pwm);
  }
}
void reverse(int num, int pwm){
  if(num == A){
    analogWrite(AIN1,pwm);
    analogWrite(AIN2,255);
  }
  else if(num == B){
    analogWrite(BIN1,pwm);
    analogWrite(BIN2,255);
  }
}
void both_motorOff(){
  analogWrite(AIN1,255);
  analogWrite(AIN2,255);
  analogWrite(BIN1,255);
  analogWrite(BIN2,255);
}
void both_forward(int pwm){
  forward(A,pwm);
  forward(B,pwm);
}
void both_reverse(int pwm){
  reverse(A,pwm);
  reverse(B,pwm);
}
void opposite_A(int pwm){
  int re_pwm = 100 - pwm;
  forward(A,pwm);
  reverse(B,re_pwm);
}
void opposite_B(int pwm){
  int re_pwm = 100 - pwm;
  forward(B,pwm);
  reverse(A,re_pwm);
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
  int pwm;

  while(1){
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

      // Print values
      Serial.print(acc_theta);
      Serial.print(" ");
      Serial.print(gyro_theta);
      Serial.print(" ");
      Serial.println(theta);
    }
    //MAYBE WE SHOULD PUT ALL THE FOLLOWING IN THE IF STATEMENT ABOVE SO IT ONLY WORKS ON VALID ANGLES, NOT TOO SURE HERE
    //calculating poritive percentage of PWM needed based on tilt angle
    error = desired_angle - theta;
    d_theta = (old_theta - theta)/dt;
    i_theta += (error + old_error)/2*dt;
    pid_out = (kp * error) + (ki * i_theta) + (kd * d_theta);
    if(pid_out < -255 or pid_out > 255) pwm = 255;
    Serial.println(pwm); //print value for debugging
    
    // assuming positive means needs to go in reverse to bring back to zero here, and vise versa
    if(pwm <= 255 && pwm >= 0){     //make sure not to send invalid data
      if(pwm == 0){
          both_motorOff();
        }
      else{
        if(pid_out < 0) both_forward(pwm);  //use theta to determine sign (direction) since pwm is always positive
        else if(pid_out > 0) both_reverse(pwm);
        else both_motorOff();
      }
    }
    else{
      Serial.println("Invalid Data");
    }
    old_theta = theta;
  }
}
