#include <AS5600.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <math.h>
#include <Arduino_BMI270_BMM150.h>

#define BIN1 D3
#define BIN2 D5
#define AIN2 D6
#define AIN1 D9
#define A 1
#define B 2

TCA9548A I2CMux;
AS5600 encoderLeft;
AS5600 encoderRight;

BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6ED");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6ED", BLERead | BLEWrite | BLENotify, 20, false);

float k = 0.9;
float kp = 7;
float ki = 120;
float kd = 0.4;
float kp_en = 5;
float ki_en = kp_en / 200.0;

float offset_angle = 0.35;
float desired_angle = 0;
float desired_vel = 0;
float turn_L = 1;
float turn_R = 1;
float old_theta = 0;
float gyro_theta = 0;
float old_error = 0;
float old_error_en = 0;
float i_theta = 0;
float i_theta_en = 0;

float tilt_accumulator = 0;
int tilt_samples = 0;
unsigned long tilt_timer = 0;
unsigned long start_time = 0;

String input;

void motorOff(int num) {
  if (num == A) {
    analogWrite(AIN1, 255);
    analogWrite(AIN2, 255);
  } else if (num == B) {
    analogWrite(BIN1, 255);
    analogWrite(BIN2, 255);
  }
}

void forward(int num, int pwm) {
  if (num == A) {
    analogWrite(AIN1, 255);
    analogWrite(AIN2, constrain(255 - pwm * turn_L, 0, 255));
  } else if (num == B) {
    analogWrite(BIN1, 255);
    analogWrite(BIN2, constrain(255 - pwm * turn_R, 0, 255));
  }
}

void reverse(int num, int pwm) {
  if (num == A) {
    analogWrite(AIN1, constrain(255 - pwm * turn_L, 0, 255));
    analogWrite(AIN2, 255);
  } else if (num == B) {
    analogWrite(BIN1, constrain(255 - pwm * turn_R, 0, 255));
    analogWrite(BIN2, 255);
  }
}

void both_motorOff() {
  motorOff(A);
  motorOff(B);
}

void both_forward(int pwm) {
  forward(A, pwm);
  forward(B, int(pwm * 0.97));
}

void both_reverse(int pwm) {
  reverse(A, pwm);
  reverse(B, int(pwm * 0.97));
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

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
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
  tilt_timer = millis();
  start_time = micros();
}

void loop() {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float acc_theta, theta, vel;
  float error, error_en, d_theta, pid_out, pid_out_en;
  int pwm;

  BLEDevice central = BLE.central();

  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(acc_x, acc_y, acc_z);
        acc_theta = atan2(acc_y, acc_z) * 180.0 / M_PI;

        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        gyro_x = -gyro_x;

        unsigned long now = micros();
        float dt = (now - start_time) / 1000000.0;
        start_time = now;

        gyro_theta += gyro_x * dt;
        theta = (1 - k) * acc_theta + k * (old_theta + gyro_x * dt);

        tilt_accumulator += acc_theta;
        tilt_samples++;
        if (millis() - tilt_timer >= 1000) {
          offset_angle = tilt_accumulator / tilt_samples;
          tilt_accumulator = 0;
          tilt_samples = 0;
          tilt_timer = millis();
        }

        vel = -encoderLeft.getAngularSpeed(AS5600_MODE_RPM);
        vel = vel * PI / 180 * 4;

        if (abs(vel) < 0.5) vel = 0.0;

        error_en = desired_vel - vel;
        i_theta_en += ki_en * (error_en + old_error_en) / 2.0 * dt;
        pid_out_en = (kp_en * error_en) + constrain(i_theta_en, -10, 10);
        desired_angle = constrain(pid_out_en, -15, 15);

        error = (offset_angle + desired_angle) - theta;
        d_theta = (old_theta - theta) / dt;
        i_theta += ki * (error + old_error) / 2.0 * dt;
        i_theta = constrain(i_theta, -50, 50);
        pid_out = (kp * error) + i_theta + (kd * d_theta);

        pwm = int(min(abs(pid_out), 255));

        if (pid_out < 0)
          both_forward(pwm);
        else if (pid_out > 0)
          both_reverse(pwm);
        else
          both_motorOff();

        old_error = error;
        old_error_en = error_en;
        old_theta = theta;
      }

      if (customCharacteristic.written()) {
        int length = customCharacteristic.valueLength();
        const unsigned char* receivedData = customCharacteristic.value();
        char receivedString[length + 1];
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0';

        Serial.print("Received data: ");
        Serial.println(receivedString);

        if (strcmp((const char*)receivedString, "f") == 0) {
          desired_vel = 10;
        } else if (strcmp((const char*)receivedString, "b") == 0) {
          desired_vel = -10;
        } else if (strcmp((const char*)receivedString, "s") == 0) {
          desired_vel = 0;
        }
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Disconnected from central.");
  }
}
