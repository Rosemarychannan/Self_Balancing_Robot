#include "TCA9548.h"
#include <Wire.h>

#define EN_ADDR 0x36       // AS5600 I2C address
#define ANGLE_REG 0x0E     // MSB of angle register

TCA9548 MP(0x70);          // Multiplexer at I2C address 0x70

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!MP.begin()) {
    Serial.println("COULD NOT CONNECT TO MULTIPLEXER");
    while (1);
  }

  // Just for confirmation
  MP.enableChannel(0);
  Serial.print("Channel 0 enabled: ");
  Serial.println(MP.isEnabled(0));

  MP.enableChannel(1);
  Serial.print("Channel 1 enabled: ");
  Serial.println(MP.isEnabled(1));
}

int read_angle() {
  int angle = 0;

  Wire.beginTransmission(EN_ADDR);     // Start transmission to AS5600
  Wire.write(ANGLE_REG);               // Request ANGLE (MSB register)
  if (Wire.endTransmission(false) != 0) {  // Send repeated start (false) for combined read
    Serial.println("Transmission error");
    return -1;
  }

  Wire.requestFrom(EN_ADDR, 2);        // Request 2 bytes: MSB, LSB
  if (Wire.available() == 2) {
    angle = Wire.read() << 8;          // MSB
    angle |= Wire.read();              // LSB
  } else {
    Serial.println("Read error");
    return -1;
  }

  return angle;
}

void loop() {
  int angle_0, angle_1;

  MP.selectChannel(0);
  angle_0 = read_angle();
  Serial.print("Angle from channel 0: ");
  Serial.println(angle_0);

  MP.selectChannel(1);
  angle_1 = read_angle();
  Serial.print("Angle from channel 1: ");
  Serial.println(angle_1);

  delay(500); // Adjust as needed
}
