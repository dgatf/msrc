/*
 *            Multi Sensor RC - MSRC
 *
 * License https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 *           Daniel.GeA.1000@gmail.com
 *
 */

// Version
#define VERSION_MAJOR 0
#define VERSION_MINOR 5
#define VERSION_PATCH 0

// pins
#define PIN_SMARTPORT_RX 7
#define PIN_SMARTPORT_TX 7 //12
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6

// opentx
#define DATA_ID 0x5100            // DataId (sensor type)
#define CONFIG_LUA                // Uncomment if using lua script for configuration

// esc protocol
#define PROTOCOL_NONE 0
#define PROTOCOL_HW_V3 1
#define PROTOCOL_HW_V4 2
#define PROTOCOL_PWM 3
#define PROTOCOL_CASTLE 4

// pwm out
#define PIN_PWM_OUT_OCR 10
#define DUTY 0.5 // 0.5 = 50%

// i2c
#define I2C_NONE 0
#define I2C_BMP180 1
#define I2C_BMP280 2
#define I2C_MS5611 3
#define WIRE_TIMEOUT 3

#define ESCSERIAL_TIMEOUT 3
#define escSerial Serial

/* Debug
   Disconnect Vcc from the RC model to the Arduino
   Do not connect at the same time Vcc from the model and usb (TTL)
   Telemetry may not work properly in debug mode
   Connect arduino Rx to TTL Tx for flashing, then connect arduino Rx to esc
*/
//#define DEBUG
//#define DEBUG_PLOTTER rpm/60

// Config bitmask

// byte 1: command

// packet 1: byte 2 version patch, byte 3 version minor, byte 4 version major

// packet 2
// byte 2
#define BM_VOLTAGE1(VALUE) VALUE >> 10 & 0B00000001
#define BM_VOLTAGE2(VALUE) VALUE >> 11 & 0B00000001
#define BM_CURRENT(VALUE) VALUE >> 12 & 0B00000001
#define BM_NTC1(VALUE) VALUE >> 13 & 0B00000001
#define BM_NTC2(VALUE) VALUE >> 14 & 0B00000001
#define BM_PWM(VALUE) VALUE >> 15 & 0B00000001
// byte 3
#define BM_REFRESH_RPM(VALUE) VALUE >> 16 & 0B00001111
#define BM_REFRESH_VOLT(VALUE) VALUE >> 20 & 0B00001111
// byte 4
#define BM_REFRESH_CURR(VALUE) VALUE >> 24 & 0B00001111
#define BM_REFRESH_TEMP(VALUE) VALUE >> 28 & 0B00001111

// packet 3
// byte 2
#define BM_AVG_ELEM_RPM(VALUE) VALUE >> 8& 0B00001111
#define BM_AVG_ELEM_VOLT(VALUE) VALUE >> 12 & 0B00001111
// byte 3
#define BM_AVG_ELEM_CURR(VALUE) VALUE >> 16 & 0B00001111
#define BM_AVG_ELEM_TEMP(VALUE) VALUE >> 20 & 0B00001111
// byte 4
#define BM_PROTOCOL(VALUE) VALUE >> 24

// packet 4
// byte 2
#define BM_I2C1(VALUE) VALUE >> 8 & 0B00001111
#define BM_I2C2(VALUE) VALUE >> 12 & 0B11110000
// byte 3
#define BM_I2C1_ADDRESS(VALUE) VALUE >> 16
#define BM_I2C2_ADDRESS(VALUE) VALUE >> 24

#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include "smartport.h"
#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "voltage.h"
#include "ntc.h"
#include "bmp180.h"
#include "bmp280.h"
#include "ms5611.h"

// Default config

struct Refresh
{
    // max refresh rate 15 (1.5s)
    uint8_t rpm = 2;   // telemetry rpm refresh rate (ms / 100)
    uint8_t volt = 10; // telemetry voltage refresh rate (ms / 100)
    uint8_t curr = 10; // telemetry current refresh rate (ms / 100)
    uint8_t temp = 10; // telemetry temperature refresh rate (ms / 100)
    uint8_t def = 10;  // telemetry default refresh rate (ms / 100)
};

struct Alpha
{
    // equivalent EMA to SMA elements (alpha=2/(N+1))
    uint8_t rpm = 33;  // rpm averaging elements (alpha * 100)
    uint8_t volt = 33; // voltage averaging elements (alpha * 100)
    uint8_t curr = 33; // current averaging elements (alpha * 100)
    uint8_t temp = 33; // temperature averaging elements (alpha * 100)
    uint8_t def = 33;  // (alpha * 100)
};

struct DeviceI2C
{
    uint8_t type = 0;
    uint8_t address = 0;
};

struct Config
{
    uint8_t sensorId = 10;
    uint8_t protocol = PROTOCOL_CASTLE; // protocol
    bool voltage1 = false;             // enable/disable voltage1 analog reading
    bool voltage2 = false;             // enable/disable voltage2 analog reading
    bool current = false;              // enable/disable current analog reading
    bool ntc1 = false;                 // enable/disable ntc1 analog reading
    bool ntc2 = false;                 // enable/disable ntc2 analog reading
    bool pwmOut = false;               // enable/disable pwm out generation (governor)
    Refresh refresh;
    Alpha alpha;
    DeviceI2C deviceI2C[2];
};

bool pwmOut = false;
Sensor *rpmSensor;
Config readConfig();
void writeConfig(Config &config);
void initConfig(Config &config);
uint8_t calcAlpha(uint8_t elements);
void setPwmOut(bool pwmOut);
void updatePwmOut(bool pwmOut);
void processPacket(uint8_t frameId, uint16_t dataId, uint32_t value);
void setup();
void loop();

SoftwareSerial smartportSerial(PIN_SMARTPORT_RX, PIN_SMARTPORT_TX, true);
Smartport smartport(smartportSerial);