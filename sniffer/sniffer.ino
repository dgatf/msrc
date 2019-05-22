#define PIN_SNIFFER 8
#define BAUD_RATE_SNIFFER 19200
#define TIMEOUT_SNIFFER 5

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
