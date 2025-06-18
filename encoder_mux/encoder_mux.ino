#include "Arduino_BMI270_BMM150.h"
#include <AS5600.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include "TCA9548A.h"

TCA9548A I2CMux;
AS5600 encoderLeft;
AS5600 encoderRight;

float angV, angV0,angV1;
void setup() 
{
    Serial.begin(9600);
    Serial.setTimeout(10);

    Wire.begin();
    I2CMux.begin(Wire);
    I2CMux.closeAll();

    I2CMux.openChannel(0);
    delay(10);

    encoderLeft.begin(4);  //  set direction pin.
    encoderLeft.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    encoderRight.begin(4);  //  set direction pin.
    encoderRight.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

    int b = encoderLeft.isConnected();
    Serial.print("Connect: ");
    Serial.println(b);

    int c = encoderRight.isConnected();
    Serial.print("Connect: ");
    Serial.println(c);

    delay(500);
     analogWrite(D6,255);    // left wheel
     analogWrite(D5,254);    // right wheel
}

void loop()
{
    angV0 = encoderLeft.getAngularSpeed(AS5600_MODE_RPM);
    Serial.print(-angV0);
    Serial.print('\t');
    I2CMux.closeChannel(0);
    I2CMux.openChannel(1);
    angV1 = encoderRight.getAngularSpeed(AS5600_MODE_RPM);
    Serial.print(angV1);
    Serial.print('\t');
    I2CMux.closeChannel(1);
    I2CMux.openChannel(0);
    angV = (-angV0 + angV1)/2;
    Serial.println(angV);
    Serial.print('\t');
    delay(100);
}
