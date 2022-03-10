#ifndef CONSTANTS_H
#define CONSTANTS_H

/* Enums */
#define RX_SMARTPORT 0
#define RX_XBUS 1
#define RX_SRXL 2
#define RX_FRSKY 3
#define RX_IBUS 4
#define RX_SBUS 5
#define RX_MULTIPLEX 6
#define RX_JETIEX 7
#define RX_HITEC 8

#define PROTOCOL_NONE 0
#define PROTOCOL_HW_V3 1
#define PROTOCOL_HW_V4 2
#define PROTOCOL_PWM 3
#define PROTOCOL_CASTLE 4
#define PROTOCOL_KONTRONIK 5
#define PROTOCOL_APD_F 6
#define PROTOCOL_APD_HV 7
#define PROTOCOL_APD_F_DSHOT 8

#include "config.h"

#if (defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)) \
    && defined(I2C_T3_TEENSY) \
    && (RX_PROTOCOL == RX_XBUS || RX_PROTOCOL == RX_HITEC)
#include <i2c_t3.h>
#define I2C_SENSOR Wire1
#else
#include <Wire.h>
#define I2C_SENSOR Wire
#endif

#if (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB) || defined(__AVR_ATmega32U4__)) || RX_PROTOCOL != RX_SMARTPORT
#define ESC_PROTOCOL CONFIG_ESC_PROTOCOL
#else
#define ESC_PROTOCOL config.protocol
#endif

/* Version */
#define VERSION_MAJOR 0
#define VERSION_MINOR 9
#define VERSION_PATCH 0

/* Init debug port */
#if (defined(DEBUG) || defined(DEBUG_PACKET) || defined(DEBUG_SBUS_MS) || defined(DEBUG_EEPROM_WRITE) || defined(DEBUG_EEPROM_READ) || defined(DEBUG_GPS) || defined(DEBUG_HW3) || defined(DEBUG_HW4) || defined(DEBUG_PWM) || defined(DEBUG_CASTLE) || defined(DEBUG_CASTLE_RX) || defined(DEBUG_KONTRONIK) || defined(DEBUG_APDF) || defined(DEBUG_APDHV))
#define DEBUG_INIT DEBUG_SERIAL.begin(115200);
#endif

/* RPM multiplier */
#define RPM_MULTIPLIER (RPM_PINION_TEETH / (1.0 * RPM_MAIN_TEETH * RPM_PAIR_OF_POLES))

/* Pins & Serial ports */

// ATmega328P
#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB) 
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
#define PIN_SWITCH_XBUS 13 // PB5
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK || CONFIG_GPS
#define SRXL_IBUS_SERIAL softSerial
#else
#define SRXL_IBUS_SERIAL hardSerial0
#endif
#define ESC_SERIAL hardSerial0
#define GPS_SERIAL hardSerial0
#define DEBUG_SERIAL hardSerial0
#endif

// ATmega328PB
#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
#define PIN_SWITCH_XBUS 13 // PB5
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
// ESC serial & GPS
#if (CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK) && CONFIG_GPS
#define SRXL_IBUS_SERIAL softserial
#endif
// ESC serial
#if (CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK) && !CONFIG_GPS
#define SRXL_IBUS_SERIAL hardSerial1
#endif
// GPS
#if !(CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK) && CONFIG_GPS
#define SRXL_IBUS_SERIAL hardSerial0
#endif
// No serial sensor
#if !(CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK) && CONFIG_GPS
#define SRXL_IBUS_SERIAL hardSerial1
#endif
#define ESC_SERIAL hardSerial0  
#define GPS_SERIAL hardSerial1
#define DEBUG_SERIAL hardSerial0
#endif

// ATmega2560
#if defined(__AVR_ATmega2560__)
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
#define PIN_SWITCH_XBUS 13 // PB7
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
#define SRXL_IBUS_SERIAL hardSerial3
#define ESC_SERIAL hardSerial1
#define GPS_SERIAL hardSerial2
#define DEBUG_SERIAL hardSerial0
#endif

// ATmega32U4 (Teensy 2.0 / Arduino Micro)
#if defined(__AVR_ATmega32U4__)
#define PIN_NTC1 A0             // PF7
#define PIN_NTC2 A1             // PF6
#define PIN_VOLTAGE1 A2         // PF5
#define PIN_VOLTAGE2 A3         // PF4
#define PIN_CURRENT A9          // PB5
#define PIN_PRESSURE A7         // PD7
#define PIN_SWITCH_XBUS PB5
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
#if CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V3 || CONFIG_ESC_PROTOCOL == PROTOCOL_HW_V4 || CONFIG_ESC_PROTOCOL == PROTOCOL_KONTRONIK || CONFIG_GPS
#define SRXL_IBUS_SERIAL softSerial
#else
#define SRXL_IBUS_SERIAL hardSerial1
#endif
#define ESC_SERIAL hardSerial1
#define GPS_SERIAL hardSerial1
#define DEBUG_SERIAL Serial
#endif

// Teensy LC/3.x
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
#define PIN_NTC1 14
#define PIN_NTC2 15
#define PIN_VOLTAGE1 16
#define PIN_VOLTAGE2 17
#define PIN_CURRENT 20
#define PIN_PRESSURE 26
#define PIN_SWITCH_XBUS 13
#define SMARTPORT_FRSKY_SBUS_SERIAL hardSerial0
#define SRXL_IBUS_SERIAL hardSerial0
#define ESC_SERIAL hardSerial1
#define GPS_SERIAL hardSerial2
#define DEBUG_SERIAL Serial
#endif

#define N_TO_ALPHA(VALUE) (2.0 / (1 + VALUE) * 100)

// i2c
#define I2C_NONE 0
#define I2C_BMP280 1
#define I2C_MS5611 2
#define WIRE_TIMEOUT 3

#define ALPHA(ELEMENTS) (uint8_t)(2.0 / (ELEMENTS + 1) * 100)
#define MS_TO_COMP(SCALER) (F_CPU / (SCALER * 1000.0))
#define US_TO_COMP(SCALER) (F_CPU / (SCALER * 1000000.0))
#define COMP_TO_MS(SCALER) ((SCALER * 1000.0) / F_CPU)

#define DEBUG_PRINT(VALUE) DEBUG_SERIAL.print(VALUE)
#define DEBUG_PRINT_HEX(VALUE) DEBUG_SERIAL.print(VALUE, HEX)
#define DEBUG_PRINTLN() DEBUG_SERIAL.print("\n")

#endif