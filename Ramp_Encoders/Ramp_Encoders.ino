#include <AS5600.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include "TCA9548A.h"
#include <math.h>
#include <Arduino_BMI270_BMM150.h>

// Motor control pin definitions
#define LEFT_MOTOR_REVERSE_PIN D3
#define LEFT_MOTOR_FORWARD_PIN D5
#define RIGHT_MOTOR_FORWARD_PIN D6
#define RIGHT_MOTOR_REVERSE_PIN D9
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

// Constants
const float WHEEL_RADIUS_CM = 4.0; // adjust based on actual wheel radius
const float FILTER_ALPHA = 0.2; // low-pass filter factor for velocity

// Global objects
TCA9548A I2CMux;
AS5600 encoderLeft;
AS5600 encoderRight;

BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6ED");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6ED", BLERead | BLEWrite | BLENotify, 20, false);

// PID tuning parameters
float complementary_filter_k = 0.9;
float balance_kp = 7;
float balance_ki = 120;
float balance_kd = 0.4;
float velocity_kp = 5;
float velocity_ki = velocity_kp / 200.0;

// Control state
float setpoint_offset_angle = 0.35;
float target_angle = 0;
float target_velocity = 0;
float left_turn_factor = 1;
float right_turn_factor = 1;

// IMU and encoder PID state
float previous_angle = 0;
float integrated_gyro_angle = 0;
float previous_balance_error = 0;
float previous_velocity_error = 0;
float integral_balance_error = 0;
float integral_velocity_error = 0;

// Velocity filtering
float filtered_velocity = 0;

// Tilt calibration
float tilt_accumulator = 0;
int tilt_sample_count = 0;
unsigned long last_tilt_update_time = 0;
unsigned long last_loop_time = 0;

// Motor control utility functions
void motorOff(int motor) {
  if (motor == MOTOR_LEFT) {
    analogWrite(RIGHT_MOTOR_REVERSE_PIN, 255);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 255);
  } else if (motor == MOTOR_RIGHT) {
    analogWrite(LEFT_MOTOR_REVERSE_PIN, 255);
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 255);
  }
}

void forward(int motor, int pwm) {
  if (motor == MOTOR_LEFT) {
    analogWrite(RIGHT_MOTOR_REVERSE_PIN, 255);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, constrain(255 - pwm * left_turn_factor, 0, 255));
  } else if (motor == MOTOR_RIGHT) {
    analogWrite(LEFT_MOTOR_REVERSE_PIN, 255);
    analogWrite(LEFT_MOTOR_FORWARD_PIN, constrain(255 - pwm * right_turn_factor, 0, 255));
  }
}

void reverse(int motor, int pwm) {
  if (motor == MOTOR_LEFT) {
    analogWrite(RIGHT_MOTOR_REVERSE_PIN, constrain(255 - pwm * left_turn_factor, 0, 255));
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 255);
  } else if (motor == MOTOR_RIGHT) {
    analogWrite(LEFT_MOTOR_REVERSE_PIN, constrain(255 - pwm * right_turn_factor, 0, 255));
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 255);
  }
}

void stopMotors() {
  motorOff(MOTOR_LEFT);
  motorOff(MOTOR_RIGHT);
}

void driveForward(int pwm) {
  forward(MOTOR_LEFT, pwm);
  forward(MOTOR_RIGHT, int(pwm * 0.97));
}

void driveReverse(int pwm) {
  reverse(MOTOR_LEFT, pwm);
  reverse(MOTOR_RIGHT, int(pwm * 0.97));
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  I2CMux.begin(Wire);
  I2CMux.closeAll();
  I2CMux.openChannel(0);
  delay(10);

  encoderLeft.begin(4);
  encoderLeft.setDirection(AS5600_CLOCK_WISE);
  encoderRight.begin(4);
  encoderRight.setDirection(AS5600_CLOCK_WISE);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  pinMode(LEFT_MOTOR_REVERSE_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Lance");
  BLE.setDeviceName("Lance");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("Waiting for data");
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");

  last_tilt_update_time = millis();
  last_loop_time = micros();
}

void loop() {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float acc_angle, fused_angle;
  float velocity_left, velocity_right, average_velocity;
  float balance_error, velocity_error, angle_derivative, pid_output, velocity_pid_output;
  int pwm;

  BLEDevice central = BLE.central();

  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(acc_x, acc_y, acc_z);
        acc_angle = atan2(acc_y, acc_z) * 180.0 / M_PI;

        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        gyro_x = -gyro_x; // Correct gyro direction

        unsigned long current_time = micros();
        float dt = (current_time - last_loop_time) / 1000000.0;
        last_loop_time = current_time;

        integrated_gyro_angle += gyro_x * dt;
        fused_angle = (1 - complementary_filter_k) * acc_angle + complementary_filter_k * (previous_angle + gyro_x * dt);

        // Tilt recalibration every second
        /*tilt_accumulator += acc_angle;
        tilt_sample_count++;
        if (millis() - last_tilt_update_time >= 500) {
          setpoint_offset_angle = tilt_accumulator / tilt_sample_count;
          tilt_accumulator = 0;
          tilt_sample_count = 0;
          last_tilt_update_time = millis();
        }
        */
        // Read both encoders
        velocity_left = -encoderLeft.getAngularSpeed(AS5600_MODE_RPM);
        I2CMux.closeChannel(0);
        I2CMux.openChannel(1);
        velocity_right = encoderRight.getAngularSpeed(AS5600_MODE_RPM);
        I2CMux.closeChannel(1);
        I2CMux.openChannel(0);

        average_velocity = (velocity_left + velocity_right) / 2.0 * PI / 180 * WHEEL_RADIUS_CM;

        // Apply low-pass filter
        filtered_velocity = FILTER_ALPHA * average_velocity + (1 - FILTER_ALPHA) * filtered_velocity;
        if (abs(filtered_velocity) < 0.5) filtered_velocity = 0.0;

        // Velocity PID
        velocity_error = target_velocity - filtered_velocity;
        integral_velocity_error += velocity_ki * (velocity_error + previous_velocity_error) / 2.0 * dt;
        velocity_pid_output = (velocity_kp * velocity_error) + constrain(integral_velocity_error, -10, 10);
        target_angle = constrain(velocity_pid_output, -15, 15);

        // Balance PID
        balance_error = (setpoint_offset_angle + target_angle) - fused_angle;
        angle_derivative = (previous_angle - fused_angle) / dt;
        integral_balance_error += balance_ki * (balance_error + previous_balance_error) / 2.0 * dt;
        integral_balance_error = constrain(integral_balance_error, -50, 50);

        pid_output = (balance_kp * balance_error) + integral_balance_error + (balance_kd * angle_derivative);
        pwm = int(min(abs(pid_output), 255));

        if (pid_output < 0) {
          driveForward(pwm);
        } else if (pid_output > 0) {
          driveReverse(pwm);
        } else {
          stopMotors();
        }

        previous_balance_error = balance_error;
        previous_velocity_error = velocity_error;
        previous_angle = fused_angle;
      }

      if (customCharacteristic.written()) {
        int length = customCharacteristic.valueLength();
        const unsigned char* receivedData = customCharacteristic.value();
        char command[length + 1];
        memcpy(command, receivedData, length);
        command[length] = '\0';

        Serial.print("Received command: ");
        Serial.println(command);

        if (strcmp(command, "f") == 0) {
          target_velocity = 5;
        } else if (strcmp(command, "b") == 0) {
          target_velocity = -5;
        } else if (strcmp(command, "s") == 0) {
          target_velocity = 0;
        }
      }
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Disconnected from central.");
  }
}
