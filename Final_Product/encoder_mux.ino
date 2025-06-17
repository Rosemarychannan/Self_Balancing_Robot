#include <Wire.h>
#define EN_ADDR 0x36
#define ANGLE_REG 0x0E
#define MP 0x70
#define CH0 0x01
#define CH1 0x02

int old_angle_0, old_angle_1;
unsigned long start_time_1, start_time_0;
int angle_0,angle_1;
double ang_v_0, ang_v_1,dt_0,dt_1;
unsigned long now_1,now_0;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  start_time_0 = micros();
  old_angle_0 = read_angle(CH0);
  Serial.print("angle_L: ");
  Serial.print(old_angle_0);
  Serial.print("\t");
  start_time_1 = micros();
  old_angle_1 = read_angle(CH1);
  Serial.print("angle_R: ");
  Serial.println(old_angle_1);
  
}
int read_angle(int channel){
  Wire.beginTransmission(MP); // transmit to device #4
  //Serial.println("transmission begin...or not: ");
  Wire.write(channel);              // sends one byte
  int a = Wire.endTransmission();    // stop transmitting
  //Serial.println("error code: hoping for 0: ");
  //Serial.print(a);

  Wire.beginTransmission(EN_ADDR);
  int b = Wire.write(ANGLE_REG);
  //Serial.print("bytes written: ");
  //Serial.println(b);

  int c = Wire.endTransmission(false);    // stop transmitting
  //Serial.println("error code: hoping for 0: ");
  //Serial.println(c);
  
  int d = Wire.requestFrom(EN_ADDR, 2);
  //Serial.println(d);

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    //Serial.println("msb: ");
    //Serial.print(msb);
    uint8_t lsb = Wire.read();
    //Serial.println("lsb: ");
    //Serial.print(lsb);
    return ((msb << 8) | lsb); // 12-bit result is in lower bits
  } else {
    return 0xFFFF;  // error code
    //Serial.print("ERROR");
  }
}
void loop()
{
  angle_0 = read_angle(CH0);
  now_0  = micros();
  dt_0 = (now_0-start_time_0)/1000000.0;
  //Serial.print("angle_L: ");
  //Serial.print(angle_0);
  //Serial.print("\t");
  angle_1 = read_angle(CH1);
  now_1  = micros();
  dt_1 = (now_1-start_time_1)/1000000.0;
  //Serial.print("angle_R: ");
  //Serial.println(angle_1);
  //Serial.print("time: ");
  //Serial.println(now);
  start_time_0 = now_0;
  start_time_1 = now_1;
  //Serial.print("dt: ");
  //Serial.println(dt);
  ang_v_0 = (angle_0 - old_angle_0)/4096.0*360.0/dt_0;
  Serial.print("angle_v_L: ");
  Serial.print(angle_0 - old_angle_0);
  Serial.print("\t");
  //Serial.print("old_angle_v_L: ");
  //Serial.print(old_angle_0);
  ang_v_1 = (angle_1 - old_angle_1)/4096.0*360.0/dt_1;
  Serial.print("angle_v_R: ");
  Serial.println(angle_1 - old_angle_1);
  old_angle_1 = angle_1;
  old_angle_0 = angle_0;
  delay(200);
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
