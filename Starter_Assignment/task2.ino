#include <math.h>
#include <Arduino_BMI270_BMM150.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z, angle;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    angle = - atan2(y, z) * 180 / M_PI;
    Serial.print(angle); 
    Serial.println();

  }  
}