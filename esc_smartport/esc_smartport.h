/*
 * esc_smartport v0.1
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Arduino sketch to send to Frsky SmartPort ESC telemetry:
 *
 * - Hobywing V3 and V4/V5 serial telemetry supported (ESC_DIGITAL_V3 or
 * ESC_DIGITAL_V4)
 * - RPM PWM signal supported (PIN_PWM_ESC)
 * - Battery voltage with voltage divider (PIN_BATT)
 * - PWM output for HW V5 Flyfun (PIN_PWM_OUT)
 *
 * Adjust RPM sensor in OpenTx:
 *
 * - Blades/pair pof poles: number of pair of poles * main gear teeth
 * - Multiplies: pinion gear teeth
 *
 * Wiring
 * ------
 *
 * - SmartPort Vcc to Arduino Vcc
 * - SmartPort Gnd to Arduino Gnd
 * - Smartport Signal to Arduino PIN_SMARTPORT (11)
 * - If using ESC serial: ESC serial signal to Arduino Rx
 * - If using ESC PWM: ESC PWM signal to Arduino PIN_PWM_ESC (2)
 * - If PWM output is required (for HobbyWing Flyfun V5): Flybarless PWM signal
 * input to Arduino PIN_PWM_OUT (4)
 * - Voltage divider + to PIN_BATT (A1)
 * - Voltage divider - to Gnd
 *
 */

// Version

#define VERSION_MAJOR 0
#define VERSION_MINOR 1

// Pins

#define PIN_SMARTPORT_RX 8  // only pins 8,9,10,11
#define PIN_SMARTPORT_TX 11
#define PIN_BATT A1

// Telemetry refresh rate in ms

#define REFRESH_RPM 200
#define REFRESH_VOLT 1000
#define REFRESH_TEMP 1000
#define REFRESH_CURR 1000

// Config bitmask

#define BITMASK_PROTOCOL B00000011
#define BITMASK_BATTERY B00000100
#define BITMASK_PWM B00001000
#define BITMASK_VERSION_MAJOR B11110000
#define BITMASK_VERSION_MINOR B00001111

#define ESCSERIAL_TIMEOUT 2

#define escSerial Serial

// Debug. Uncommnent for debugging
// Disconnect Vcc from the RC model to the Arduino
// Do not connect at the same time Vcc from the model and usb (FTDI)
// Telemetry may not work properly in debug mode
// Connect arduino Rx to FTDI Tx for flashing, then connect arduino Rx to esc

//#define DEBUG

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "Esc.h"
#include "Smartport.h"

// Default config values

struct Config {
  uint8_t versionMajor = VERSION_MAJOR;
  uint8_t versionMinor = VERSION_MINOR;
  uint8_t protocol = PROTOCOL_HW_V3;
  bool battery = false; // Read voltage from voltage divider. Adjust VFAS sensor
                        // ratio on Tx
  bool pwmOut = false; // Generate PWM output from esc serial (for HW Flyfun V5)
};

struct Telemetry {
  float *rpmP = NULL;
  float *voltageP = NULL;
  float *rippleVoltageP = NULL;
  float *currentP = NULL;
  float *becCurrentP = NULL;
  float *becVoltageP = NULL;
  float *temp1P = NULL;
  float *temp2P = NULL;
  float *voltageAnalogP = NULL;
};

void readConfig();
void writeConfig();
void initConfig();
float readVoltageAnalog();
void setup();
void loop();

Config config;
Telemetry telemetry;

SoftwareSerial smartportSerial(PIN_SMARTPORT_RX, PIN_SMARTPORT_TX, true);
Smartport smartport(smartportSerial);
Esc esc(escSerial);
