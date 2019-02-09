/*
 * rpm_volt_sensor v0.3
 *    09/02/2019
 *
 * Arduino sketch to send to Frsky SmartPort ESC RPM and battery voltage:
 *
 * - Hobywing digitaL ESC RPM or any ESC with RPM PWM out
 * - Battery voltage with voltage divider or lipo cells
 *
 * Adjusting POLES in the sketch is optional and this can be adjusted in the
 * Opentx. If you leave POLES 1:
 *
 * - Blades/poles: number of pair of poles * main gear teeth
 * - Multiplies: pinion gear teeth
 *
 * Pinout
 * ------
 *
 * - ESC Vcc to Arduino Vcc
 * - ESC Gnd to Arduino Gnd
 * - Receiver smartport to Arduino 8
 * - ESC Data to Arduino 2
 * - Voltage divider + to A4
 * - Voltage divider - to GND
 *
 * author: Daniel Gorbea <danielgorbea@hotmail.com>
 *
 */

#define POLES 1
#define PIN_SMARTPORT 8
#define PIN_ESC 2
#define PIN_CELL1 A1
#define PIN_CELL2 A2
#define PIN_CELL3 A3
#define PIN_BATT A4
#define CELLS 3
#define ESC_DIGITAL
//#define BATT_SENSOR_CELLS
#define BATT_SENSOR_VOLT
#define RPM_QUEUE_SIZE 20
#define VOLT_QUEUE_SIZE 20

#include <Arduino.h>
#include <Queue.h>
#include <SoftwareSerial.h>

void queueInit();
void sendByte(uint8_t c, uint16_t *crcp);
void sendByte(uint8_t c, uint16_t *crcp);
void sendData(uint16_t id, int32_t val);
uint32_t lipoCell(uint8_t id, float val1, float val2);
uint32_t lipoCell(uint8_t id, float val);
void sendVolt(float volt);
void sendCell(float cell1, float cell2, float cell3);
void sendRpm(float rpm);
float escDigitalRead();
void escPwmRead(float &rpm);
void readCell(float &cell1, float &cell2, float &cell3);
float readVolt();
void setup();
void loop();
