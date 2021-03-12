#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setTimeout(1);
}

void loop() {
  Serial.println("I2C scanner");
  uint8_t error, address;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(true);
    if (error == 0)
    {
      Serial.println(address, HEX);
    }
  }
  Serial.println("Done");
  Serial.println();
  delay(3000);
}
