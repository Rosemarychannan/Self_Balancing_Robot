//
//    FILE: TCA9548_demo.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo TCA9548 I2C multiplexer
//     URL: https://github.com/RobTillaart/TCA9548


#include "TCA9548.h"
#include "AS5600.h"
#define EN_ADDR 0x36
#define ANGLE_REG 0x0E
#define MUX_RST (reset_pin)
TCA9548 MP(0x70);
AS5600L as5600;   //  use default Wire

// Define motor pins
#define BIN1 D3
#define BIN2 D5
#define AIN2 D6
#define AIN1 D9
#define A 1
#define B 2

uint8_t channels = 0;
int angle = 0;

void forward(int num, int pwm){
  if(num == A){
    analogWrite(AIN1,255);
    analogWrite(AIN2,constrain(255-pwm, 0, 255));
  } else if(num == B){
    analogWrite(BIN1,255);
    analogWrite(BIN2,constrain(255-pwm, 0, 255));
  }
}
void both_forward(int pwm){
  forward(A, pwm);
  forward(B, pwm);
}
void setup()
{
  Serial.begin(115200);

  Wire.begin();
  if (MP.begin() == false)
  {
    Serial.println("COULD NOT CONNECT");
  }
  MP.setResetPin(MUX_RST);
  MP.reset();
  MP.enableChannel(0);
  Serial.print(MP.isEnabled(0));
  MP.enableChannel(1);
  Serial.print(MP.isEnabled(1));
  
//Encoder setup
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);

}

void loop()
{
  both_forward(255);
  int angle_0,angle_1;
  MP.selectChannel(0);
  angle_0 = as5600.read_angle();
  MP.selectChannel(1);
  angle_1 = as5600.read_angle();
  delay(1000);
}
