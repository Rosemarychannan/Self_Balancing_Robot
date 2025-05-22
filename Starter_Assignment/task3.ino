#include "Arduino_BMI270_BMM150.h"

float angle = 0.0;
long start_time = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    x = -x;

    long now = micros();
    float dt = (now - start_time) / 1000000.0;
    start_time = now;

    angle += x * dt;
    Serial.print(angle);
    Serial.print('\n');

  }
}