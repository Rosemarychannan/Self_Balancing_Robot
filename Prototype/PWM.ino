#include <math.h>
#include <Arduino_BMI270_BMM150.h>

//defining arduino pins for readability
#define BIN1 D3
#define BIN2 D5
#define AIN2 D6
#define AIN1 D9
#define A 1
#define B 2

//Variable PWM, initially set to 100%, not used in this code
float pwm = 100; //by percentage %

//Functions for motor controll
void motorOff(int num){
  
  if(num == A){
    analogWrite(AIN1,0);
    analogWrite(AIN2,0);
  }
  else if(num == B){
    analogWrite(BIN1,0);
    analogWrite(BIN2,0);
  }
}
void forward(int num, int pwm_percent){
  int duty_cycle = 255.0f * pwm_percent / 100.0f;
  if(num == A){
    analogWrite(AIN1,duty_cycle);
    analogWrite(AIN2,0);
  }
  else if(num == B){
    analogWrite(BIN1,duty_cycle);
    analogWrite(BIN2,0);
  }
}
void reverse(int num, int pwm_percent){
  int duty_cycle = 255.0f * pwm_percent / 100.0f;
  if(num == A){
    analogWrite(AIN1,0);
    analogWrite(AIN2,duty_cycle);
  }
  else if(num == B){
    analogWrite(BIN1,0);
    analogWrite(BIN2,duty_cycle);
  }
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
}

void loop() {
  Serial.println("Demo starting");
  delay(5000);


  // Drive motors at 25%, 50%, 75%, 100% of maximum rpm in one direction
  // 20 second delay to measure and show PWM signal
  for(int i = 25; i <= 100; i+=25){
    Serial.println("Both driving forward(%): ");
    Serial.print(i);
    both_forward(i);
    delay(20000);
  }

  // Drive motors at 25% and 75% of maximum rpm in the other direction
  // 20 second delay to measure and show PWM signal
  for(int i = 25; i <= 75; i+=50){
    Serial.println("Both driving in reverse(%): ");
    Serial.print(i);
    both_reverse(i);
    delay(20000);
  }

  // Drive motors at 25% and 75% of maximum rpm in opposite directions of each other - A
  for(int i = 25; i <= 75; i+=50){
    Serial.println("Driving in opposing direction(%) - A: ");
    Serial.print(i);
    opposite_A(i);
    delay(20000);
  }

  // Drive motors at 25% and 75% of maximum rpm in opposite directions of each other - B
  for(int i = 25; i <= 75; i+=50){
    Serial.println("Driving in opposing direction(%) - B: ");
    Serial.print(i);
    opposite_B(i);
    delay(20000);
  }

}
