// A is left wheel and B is right wheel
// AIN1 and BIN1 used for reverse
// AIN2 and BIN2 used for forward
//yy


#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <AS5600.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include "TCA9548A.h"

#define BUFFER_SIZE 20

TCA9548A I2CMux;
AS5600 encoderLeft;
AS5600 encoderRight;

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6ED");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6ED", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

#include <math.h>
#include <Arduino_BMI270_BMM150.h>
#include <string.h>

// Define motor pins
#define BIN1 D3
#define BIN2 D5
#define AIN2 D6
#define AIN1 D9
#define A 1
#define B 2

float vel, angV0,angV1;
// Constants and state variables
float k = 0.9;
float kp = 10;
float ki = 140;
float kd = 0.3;

float offset_angle = 0.4; // balance setpoint
float desired_angle = 0; // balance setpoint
float desired_vel = 0;
float turn_L = 1;
float turn_R = 1;
float old_theta = 0;
float gyro_theta = 0;
float old_error = 0;
float i_theta = 0;

int task = 0;
String input;
unsigned long start_time = 0;

// Motor control functions
void motorOff(int num){
  if(num == A){
    analogWrite(AIN1,255);
    analogWrite(AIN2,255);
  } else if(num == B){
    analogWrite(BIN1,255);
    analogWrite(BIN2,255);
  }
}

void forward(int num, int pwm){
  if(num == A){
    analogWrite(AIN1,255);
    analogWrite(AIN2,constrain(255 - pwm*turn_L, 0, 255));
  } else if(num == B){
    analogWrite(BIN1,255);
    analogWrite(BIN2,constrain(255-pwm*turn_R, 0, 255));
  }
}
void reverse(int num, int pwm){
  if(num == A){
    analogWrite(AIN1,constrain(255- pwm, 0, 255));
    analogWrite(AIN2,255);
  } else if(num == B){
    analogWrite(BIN1,constrain(255- pwm, 0, 255));
    analogWrite(BIN2,255);
  }
}

void both_motorOff(){
  motorOff(A);
  motorOff(B);
}

void both_forward(int pwm){
  forward(A, pwm);
  forward(B, pwm*0.95);
}

void both_reverse(int pwm){
  reverse(A, pwm);
  reverse(B, pwm*0.95);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
    // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("Lance");
  BLE.setDeviceName("Lance");

  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);

  // Add the service
  BLE.addService(customService);

  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");

  // Start advertising the service
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
  Serial.println("Setup Finished");
  start_time = micros();
}

void keyboard_test (void) {
    if (Serial.available() > 0) {
        input = Serial.readString();
        input.trim();
        input.toLowerCase();
    }

    if (input == "kp") {
        task = 1;
    }
    else if (input == "ki") {
        task = 2;
    }
    else if (input == "kd") {
        task = 3;
    }
    else if (input == "reset") {
        i_theta = 0;
    }
    else if (input == "c") {
      task = 4;
    }
     else if (input == "angle") {
      task = 5;
    }


    switch (task) {
        case 1:
            if(input == "0" || input.toFloat() > 0) kp = input.toFloat();
        break;
        case 2:
            if(input == "0" || input.toFloat() > 0) ki = input.toFloat();
        break;
        case 3:
            if(input == "0" || input.toFloat() > 0) kd = input.toFloat();
        break;
        case 5:
            if(input == "0" || input.toFloat() > 0) offset_angle = input.toFloat();
        break;
    }
}

void loop() {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float acc_theta, theta, vel;
  float error, d_theta, pid_out;
  int pwm;
  //Bluetooth setup
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      keyboard_test();
      //Serial.println("Connected from central.");
      //Finish bluetooth setup
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
          // Read accelerometer
          IMU.readAcceleration(acc_x, acc_y, acc_z);
          acc_theta = atan2(acc_y, acc_z) * 180.0 / M_PI;

          // Read gyroscope
          IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
          gyro_x = -gyro_x;

          // Calculate delta time
          unsigned long now = micros();
          float dt = (now - start_time) / 1000000.0;
          start_time = now;

          // Integrate gyro
          gyro_theta += gyro_x * dt;

          // Complementary filter
          theta = (1 - k) * acc_theta + k * (old_theta + gyro_x * dt);
          
          // PID calculations
          error = (offset_angle + desired_angle) - theta;
          d_theta = (old_theta - theta) / dt;
          i_theta += ki *(error + old_error) / 2.0 * dt;
          i_theta = constrain(i_theta,-50,50);
          pid_out = (kp * error) + (i_theta) + (kd * d_theta); 
          
          if(pid_out < -255 or pid_out > 255) {
            pwm = 255;
          }
          else pwm = int(abs(pid_out));

          //Serial.print("offset_angle: "); Serial.print(offset_angle);
          //Serial.print("\t");
          //Serial.print("Kp: "); Serial.print(kp);
          //Serial.print("\t");
          //Serial.print("Ki: "); Serial.print(ki);
          //Serial.print("\t");
          //Serial.print("Kd: "); Serial.print(kd);
          //Serial.println("\t");
          //Serial.print(" i_theta: "); Serial.print(i_theta);
          //Serial.print("\t");
          //Serial.print(" kd*d_theta: "); Serial.println(kd*d_theta);

          // Apply motor control
          if (pid_out < 0)
            both_forward(pwm);
          else if (pid_out > 0)
            both_reverse(pwm);
          else
            both_motorOff();

          old_error = error;
          old_theta = theta;
      // Check if the characteristic was written
      if (customCharacteristic.written()) {
       // Get the length of the received data
        int length = customCharacteristic.valueLength();

        // Read the received data
        const unsigned char* receivedData = customCharacteristic.value();

        // Create a properly terminated string
        char receivedString[length + 1]; // +1 for null terminator
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0'; // Null-terminate the string

        // Print the received data to the Serial Monitor
        Serial.print("Received data: ");
        Serial.println(receivedString);

        if (strcmp((const char*)receivedString, "r") == 0) {
          turn_L = 1.5;
          turn_R = 1;
          desired_angle = -0.5;
        }   
        else if (strcmp((const char*)receivedString, "l") == 0) {
          turn_L = 1.5;
          turn_R = 1;
          desired_angle = 0.5;
        }
        else if (strcmp((const char*)receivedString, "f") == 0) {
          turn_L = 1;
          turn_R = 1;
          desired_angle = -0.5;
        }
        else if (strcmp((const char*)receivedString, "b") == 0) {
          turn_L = 1;
          turn_R = 1;
          desired_angle = 0.5;
        }
        else if (strcmp((const char*)receivedString, "s") == 0) {
          desired_angle = 0;
          turn_L = 1;
          turn_R = 1;
        }
        else if (strcmp((const char*)receivedString, "p") == 0) {
          desired_angle +=0.1;
        }
        else if (strcmp((const char*)receivedString, "n") == 0) {
          desired_angle -=0.1;
        }

        // Optionally, respond by updating the characteristic's value
        //customCharacteristic.writeValue("Data received");
        Serial.println(desired_angle);
      }
    }
  }
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");
  }
}
