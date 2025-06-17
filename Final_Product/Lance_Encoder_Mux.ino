#include "TCA9548.h"
#include "AS5600.h"
#include <Wire.h>

// Multiplexer setup
TCA9548 mp(0x70);  // TCA9548 default address

// AS5600 setup (default Wire)
AS5600L as5600;  // Use AS5600 for AS5600L if needed

const int NUM_CHANNELS = 4;  // Number of AS5600s on different mux channels (adjust as needed)
const uint8_t DIRECTION_PIN = 4;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  // Initialize multiplexer
  if (!mp.begin()) {
    Serial.println("TCA9548 not found.");
    while (1);
  }
  
  Serial.println("TCA9548 initialized.");

  // Test each AS5600 sensor on its channel
  for (int chan = 0; chan < NUM_CHANNELS; chan++)
  {
    mp.setChannel(chan);  // Select channel
    delay(10);  // Small delay to allow switching

    Serial.print("Testing channel: ");
    Serial.println(chan);

    as5600.begin(DIRECTION_PIN);
    as5600.setDirection(AS5600_CLOCK_WISE);

    if (as5600.isConnected()) {
      Serial.print("AS5600 connected at channel ");
      Serial.println(chan);
    } else {
      Serial.print("AS5600 NOT found at channel ");
      Serial.println(chan);
    }

    delay(100);
  }

  Serial.println("Setup complete.\n");
}

void loop()
{
  // Loop over each AS5600 sensor on different channels
  for (int chan = 0; chan < NUM_CHANNELS; chan++)
  {
    mp.setChannel(chan);  // Select the correct multiplexer channel
    delay(5);  // Ensure I2C switch completes

    as5600.readAngle();  // Initiate new reading

    int pos = as5600.getCumulativePosition(false);
    float speed = as5600.getAngularSpeed(AS5600_MODE_DEGREES, false);

    Serial.print("Channel ");
    Serial.print(chan);
    Serial.print(" - Position: ");
    Serial.print(pos);
    Serial.print(" | Speed (deg/s): ");
    Serial.println(speed);

    delay(100);  // Optional delay per channel
  }

  Serial.println();
  delay(300);  // Wait before reading again
}
