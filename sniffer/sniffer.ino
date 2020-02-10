// This sketch is a sniffer for serial port
// This is valid for any esc with serial output. Other kind of protocols like castlelink are no supported
// Circuit:
// 1. esc serial data to pin 7 on the arduino
// 2. esc gnd to arduino gnd
// 3. usb-ttl to an arduino (or usb to an arduino with usb)
// Read the output in the serial monitor. First column is timestamp, the rest is the serial packet in hex
// Adjust BAUD_RATE_SNIFFER if needed (300,1200,2400,4800,9600,19200,38400,57600,115200,etc)

#define PIN_SNIFFER 7
#define BAUD_RATE_SNIFFER 19200
#define TIMEOUT_SNIFFER 5

#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial snifferSerial(PIN_SNIFFER, PIN_SNIFFER);  // Rx Tx
uint8_t data[64];

void setup() {
  Serial.begin(115200);
  Serial.println("INIT");
  snifferSerial.begin(BAUD_RATE_SNIFFER);
  snifferSerial.setTimeout(TIMEOUT_SNIFFER);
}

void loop() {
  if (snifferSerial.available()) {
    uint8_t cont = snifferSerial.readBytes(data, 64);
    Serial.print(millis());
    Serial.print(" ");
    for (uint8_t i = 0; i < cont; i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

  }
}
