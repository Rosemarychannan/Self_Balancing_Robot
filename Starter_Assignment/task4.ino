#include <math.h>
#include <Arduino_BMI270_BMM150.h>

float old_theta = 0; 
float gyro_theta = 0;
unsigned long start_time = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("acc_theta gyro_theta filter_theta");
  start_time = micros();
}

void loop() {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float acc_theta, theta;
  float k = 0.6;

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
}