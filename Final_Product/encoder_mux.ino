//
//    FILE: TCA9548_demo.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo TCA9548 I2C multiplexer
//     URL: https://github.com/RobTillaart/TCA9548


#include <Wire.h>
#define EN_ADDR 0x36
#define ANGLE_REG 0x0E
#define MP 0x70
#define CH0 0x01
#define CH1 0x02


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
}
int read_angle(int channel){
  Wire.beginTransmission(MP); // transmit to device #4
  Serial.print("transmission begin...or not: ");
  Wire.write(channel);              // sends one byte
  int a = Wire.endTransmission();    // stop transmitting
  Serial.println("error code: hoping for 0. ");
  Serial.println(a);

  Wire.beginTransmission(EN_ADDR);
  int b = Wire.write(ANGLE_REG);
  Serial.print("bytes written: ");
  Serial.print(b);
  int c = Wire.endTransmission(false);    // stop transmitting
  Serial.print(c);

  int d = Wire.requestFrom(EN_ADDR, 2);
  Serial.print(c);

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    Serial.print(msb);
    uint8_t lsb = Wire.read();
    Serial.print(lsb);
    return ((msb << 8) | lsb); // 12-bit result is in lower bits
  } else {
    return 0xFFFF;  // error code
    Serial.print("ERROR");
  }
}
void loop()
{
  int angle_0,angle_1;
  angle_0 = read_angle(CH0);
  Serial.print(angle_0);
  Serial.print("\t");
  angle_1 = read_angle(CH1);
  Serial.println(angle_1);
}
/*
  velocity_error = target_velocity - current_velocity;
  d_velocity = (velocity_error - old_velocity_error) / dt;
  i_velocity += ki_vel * (velocity_error + old_velocity_error) / 2.0 * dt;

  // Optional: constrain integral to avoid windup
  i_velocity = constrain(i_velocity, -10.0, 10.0); // degrees

  tilt_setpoint = kp_vel * velocity_error + i_velocity + kd_vel * d_velocity;

  // Optional: limit tilt output
  tilt_setpoint = constrain(tilt_setpoint, -10.0, 10.0); // degrees

  old_velocity_error = velocity_error;
*/


//  -- END OF FILE --
