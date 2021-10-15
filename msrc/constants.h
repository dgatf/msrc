#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "config.h"

/* Version */
#define VERSION_MAJOR 0
#define VERSION_MINOR 8
#define VERSION_PATCH 0

/* Init debug port */
#if (defined(DEBUG) || defined(DEBUG_PACKET) || defined(DEBUG_EEPROM_WRITE) || defined(DEBUG_EEPROM_READ) || defined(DEBUG_GPS) || defined(DEBUG_HW3) || defined(DEBUG_HW4) || defined(DEBUG_PWM) || defined(DEBUG_CASTLE) || defined(DEBUG_CASTLE_RX) || defined(DEBUG_KONTRONIK))
#define DEBUG_INIT DEBUG_SERIAL.begin(115200);
#endif

/* Pins & Serial ports */

// ATmega328P
#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB) 
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
#define SRXL_IBUS_SERIAL hardSerial0
#define ESC_SERIAL hardSerial0   // if using with srxl, ibus or multiplex, use softSerial. If there is no reading, use a 3.3v board
#define GPS_SERIAL hardSerial0   // same as above. Only one softserial is feasible
#define DEBUG_SERIAL hardSerial0
#if defined(DEBUG_INIT) && !defined(DEBUG_GPS) 
#define DISABLE_GPS
#endif
#if defined(DEBUG_INIT) && !defined(DEBUG_HW4) 
#define DISABLE_HW4
#endif
#endif

// ATmega328PB
#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
#define SRXL_IBUS_SERIAL hardSerial1
#define ESC_SERIAL hardSerial0  
#define GPS_SERIAL hardSerial1   // if using with esc and srxl or ibus, use softSerial. If there is no reading, use 3.3v board
#define DEBUG_SERIAL hardSerial0
#if defined(DEBUG_INIT) && !defined(DEBUG_GPS) 
#define DISABLE_GPS
#endif
#if defined(DEBUG_INIT) && !defined(DEBUG_HW4) 
#define DISABLE_HW4
#endif
#endif

// ATmega2560
#if defined(__AVR_ATmega2560__)
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
#define SRXL_IBUS_SERIAL hardSerial3
#define ESC_SERIAL hardSerial1
#define GPS_SERIAL hardSerial2
#define DEBUG_SERIAL hardSerial0
#endif

// ATmega32U4 (Teensy 2.0)
#if defined(__AVR_ATmega32U4__)
#define PIN_NTC1 A0             // PF7
#define PIN_NTC2 A1             // PF6
#define PIN_VOLTAGE1 A2         // PF5
#define PIN_VOLTAGE2 A3         // PF4
#define PIN_CURRENT A9          // PB5
#define PIN_PRESSURE A7         // PD7
#define SMARTPORT_FRSKY_SBUS_SERIAL softSerial
#define SRXL_IBUS_SERIAL hardSerial1
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
#define PIN_CURRENT 18
#define PIN_PRESSURE 19
#define SMARTPORT_FRSKY_SBUS_SERIAL hardSerial0
#define SRXL_IBUS_SERIAL hardSerial0
#define ESC_SERIAL hardSerial1
#define GPS_SERIAL hardSerial2
#define DEBUG_SERIAL Serial
#endif

#define N_TO_ALPHA(VALUE) (2.0 / (1 + VALUE) * 100)

#define RX_SMARTPORT 0
#define RX_XBUS 1
#define RX_SRXL 2
#define RX_FRSKY 3
#define RX_IBUS 4
#define RX_SBUS 5
#define RX_MULTIPLEX 6

#define PROTOCOL_NONE 0
#define PROTOCOL_HW_V3 1
#define PROTOCOL_HW_V4_LV 2
#define PROTOCOL_HW_V4_HV 3
#define PROTOCOL_HW_V5_LV 4
#define PROTOCOL_HW_V5_HV 5
#define PROTOCOL_PWM 6
#define PROTOCOL_CASTLE 7
#define PROTOCOL_KONTRONIK 8

// i2c
#define I2C_NONE 0
#define I2C_BMP280 1
#define WIRE_TIMEOUT 3

#define ALPHA(ELEMENTS) (uint8_t)(2.0 / (ELEMENTS + 1) * 100)
#define MS_TO_COMP(SCALER) (F_CPU / (SCALER * 1000.0))
#define COMP_TO_MS(SCALER) ((SCALER * 1000.0) / F_CPU)

#define DEBUG_PRINT(VALUE) DEBUG_SERIAL.print(VALUE)
#define DEBUG_PRINT_HEX(VALUE) DEBUG_SERIAL.print(VALUE, HEX)
#define DEBUG_PRINTLN() DEBUG_SERIAL.print("\n")

#endif