/*
 * esc_smartport
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Arduino sketch to send to Frsky SmartPort ESC telemetry:
 *
 * - Hobywing V3
 * - Hobywing V4/V5
 * - RPM PWM signal supported
 * - Battery voltage with voltage divider
 * - PWM output (for HW V5 Flyfun)
 *
 * Adjust RPM sensor in OpenTx:
 *
 * - Blades/pair pof poles: number of pair of poles * main gear teeth
 * - Multiplies: pinion gear teeth
 *
 * Wiring
 * ------
 *
 * - SmartPort Vcc to Arduino RAW
 * - SmartPort Gnd to Arduino Gnd
 * - Smartport Signal to Arduino PIN_SMARTPORT_RX (7)
 * - Smartport Signal to R3 (4.7k)
 * - R3 (4.7k) to Arduino PIN_SMARTPORT_TX (12)
 * - If using ESC serial: ESC serial signal to Arduino Rx
 * - If using ESC PWM: ESC PWM signal to Arduino PIN_PWM_ESC (8)
 * - If PWM output is required (for HobbyWing Flyfun V5): Flybarless PWM signal
 * input to Arduino PIN_PWM_OUT (9)
 * - Voltage divider + to PIN_BATT (A1)
 * - Voltage divider - to Gnd
 *
 */

// Version

#define VERSION_MAJOR 0
#define VERSION_MINOR 3
#define VERSION_PATCH 1

// Pins

#define PIN_SMARTPORT_RX 7
#define PIN_SMARTPORT_TX 12

// Analog inputs

#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A4
#define PIN_PWM_OUT_ICRA 9 // TIMER1 PWM PIN 9
#define PIN_PWM_OUT_OCR 10 // TIMER1 PWM PIN 10

// Config type

#define CONFIG_LUA

// Board Vcc

#if F_CPU == 16000000UL
#define BOARD_VCC 5
#else
#define BOARD_VCC 3.3
#endif

// Thermistors (NTC 100k, R1 10k)

#define NTC_R_REF 100000UL
#define NTC_R1 10000
#define NTC_BETA 4190
//#define NTC_A1 3.35E-03
//#define NTC_B1 2.46E-04
//#define NTC_C1 3.41E-06
//#define NTC_D1 1.03E-07

// PWM out (OCR pin 10, ICR pin 9)

#define MODE_PWM_OUT OCR // ICR
#define DUTY 0.5  // 0.5 = 50%

// Config bitmask

// packet 1

// byte 1: version patch
// byte 2: version minor
// byte 3: version major

// packet 2

// byte 1
#define BM_PROTOCOL(VALUE) VALUE & 0B00000011
#define BM_VOLTAGE1(VALUE) VALUE >> 2 & 0B00000001
#define BM_VOLTAGE2(VALUE) VALUE >> 3 & 0B00000001
#define BM_CURRENT(VALUE) VALUE >> 4 & 0B00000001
#define BM_NTC1(VALUE) VALUE >> 5 & 0B00000001
#define BM_NTC2(VALUE) VALUE >> 6 & 0B00000001
#define BM_PWM(VALUE) VALUE >> 7 & 0B00000001

// byte 2
#define BM_REFRESH_RPM(VALUE) VALUE >> 8 & 0B00001111
#define BM_REFRESH_VOLT(VALUE) VALUE >> 12 & 0B00001111

// byte 3
#define BM_REFRESH_CURR(VALUE) VALUE >> 16 & 0B00001111
#define BM_REFRESH_TEMP(VALUE) VALUE >> 20 & 0B00001111

// packet 3

// byte 1
#define BM_AVG_ELEM_RPM(VALUE) VALUE & 0B00001111
#define BM_AVG_ELEM_VOLT(VALUE) VALUE >> 4 & 0B00001111

// byte 2
#define BM_AVG_ELEM_CURR(VALUE) VALUE >> 8 & 0B00001111
#define BM_AVG_ELEM_TEMP(VALUE) VALUE >> 12 & 0B00001111

// byte 3
#define BM_AVG_ELEM_PWM(VALUE) VALUE >> 16 & 0B00001111

#define ESCSERIAL_TIMEOUT 3
#define escSerial Serial

// Debug. Uncommnent for debugging
// Disconnect Vcc from the RC model to the Arduino
// Do not connect at the same time Vcc from the model and usb (FTDI)
// Telemetry may not work properly in debug mode
// Connect arduino Rx to FTDI Tx for flashing, then connect arduino Rx to esc

//#define DEBUG
//#define DEBUG_PLOTTER rpm/60
//#define DEBUG_TELEMETRY

#include "Esc.h"
#include "Smartport.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// Default config

struct Config {
  uint8_t protocol = PROTOCOL_HW_V3;  // protocol (PROTOCOL_HW_V3, PROTOCOL_HW_V4, PWM)
  bool voltage1 = false;              // enable/disable voltage1 analog reading
  bool voltage2 = false;              // enable/disable voltage2 analog reading
  bool current = false;               // enable/disable current analog reading
  bool ntc1 = false;                  // enable/disable ntc1 analog reading
  bool ntc2 = false;                  // enable/disable ntc2 analog reading
  bool pwmOut = false;                // enable/disable pwm out generation (governor)
  // max refresh rate 15 (1.5s)
  uint8_t refreshRpm = 2;             // telemetry rpm refresh rate (ms / 100)
  uint8_t refreshVolt = 10;           // telemetry voltage refresh rate (ms / 100)
  uint8_t refreshCurr = 10;           // telemetry current refresh rate (ms / 100)
  uint8_t refreshTemp = 10;           // telemetry temperature refresh rate (ms / 100)
  // max queue size 16
  float alphaRpm = 0.3;               // rpm averaging elements
  float alphaVolt = 0.3;              // voltage averaging elements
  float alphaCurr = 0.3;              // current averaging elements
  float alphaTemp = 0.3;              // temperature averaging elements
  float alphaPwm = 0.3;               // pwm out averaging elements (governor)
};

struct Telemetry {
  float *escRpmConsP = NULL;
  float *escPowerP = NULL;
  float *cellP = NULL;
  float *temp1P = NULL;
  float *temp2P = NULL;
  float *voltageAnalog1P = NULL;
  float *voltageAnalog2P = NULL;
  float *currentAnalogP = NULL;
  float *ntc1P = NULL;
  float *ntc2P = NULL;
  float rpm = 0;
  float voltage = 0;
  float current = 0;
  float temp1 = 0;
  float temp2 = 0;
  float voltageAnalog1 = 0;
  float voltageAnalog2 = 0;
  float currentAnalog = 0;
  float ntc1 = 0;
  float ntc2 = 0;
  float pwm = 0;
  uint8_t cellCount = 0;
};

void readConfig();
void writeConfig();
void initConfig();
float calcAlpha(uint8_t elements);
void setPwmOut();
uint8_t setCellCount();
float readVoltageAnalog(uint8_t pin);
float readNtc(uint8_t pin);
void setup();
void loop();

Config config;
Telemetry telemetry;

SoftwareSerial smartportSerial(PIN_SMARTPORT_RX, PIN_SMARTPORT_TX, true);
Smartport smartport(smartportSerial);
Esc esc(escSerial);
