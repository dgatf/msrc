#ifndef CONSTANTS_H
#define CONSTANTS_H

// Version
#define VERSION_MAJOR 0
#define VERSION_MINOR 8
#define VERSION_PATCH 0

// Pins & Serial ports
#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB) // ATmega328P
#define PIN_SOFTSERIAL_RX 7
#define PIN_SOFTSERIAL_TX 12
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
//#define SMARTPORT_FRSKY_HARDWARE_SERIAL
#define SMARTPORT_FRSKY_SERIAL softSerial
#define SRXL_IBUS_SBUS_SERIAL Serial
//#define SOFTWARE_SERIAL
#define ESC_SERIAL Serial
#define GPS_SERIAL Serial
#endif

#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB) // ATmega328PB
#define PIN_SOFTSERIAL_RX 4
#define PIN_SOFTSERIAL_TX 23
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
//#define SMARTPORT_FRSKY_HARDWARE_SERIAL
#define SMARTPORT_FRSKY_SERIAL softSerial
#define SRXL_IBUS_SBUS_SERIAL Serial1
//#define SOFTWARE_SERIAL
#define ESC_SERIAL Serial
#define GPS_SERIAL Serial1
#endif

#if defined(__AVR_ATmega2560__) // ATmega2560
#define PIN_SOFTSERIAL_RX 10
#define PIN_SOFTSERIAL_TX 15
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7
//#define SMARTPORT_FRSKY_HARDWARE_SERIAL
#define SMARTPORT_FRSKY_SERIAL softSerial
#define SRXL_IBUS_SBUS_SERIAL Serial3
//#define SOFTWARE_SERIAL
#define ESC_SERIAL Serial1
#define GPS_SERIAL Serial2
#endif

#if defined(__AVR_ATmega32U4__) // ATmega32U4 (Teensy 2.0 / Arduino Pro Micro)
#define PIN_SOFTSERIAL_RX PB2 // 16
#define PIN_SOFTSERIAL_TX PB4 // 8
#define PIN_NTC1 A0 // PF7
#define PIN_NTC2 A1 // PF6
#define PIN_VOLTAGE1 A2 // PF5
#define PIN_VOLTAGE2 A3 // PF4
#define PIN_CURRENT A9 // PB5
#define PIN_PRESSURE A7 // PD7
//#define SMARTPORT_FRSKY_HARDWARE_SERIAL
#define SMARTPORT_FRSKY_SERIAL softSerial
#define SRXL_IBUS_SBUS_SERIAL Serial
//#define SOFTWARE_SERIAL
#define ESC_SERIAL Serial1
#define GPS_SERIAL Serial1
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define PIN_NTC1 14
#define PIN_NTC2 15
#define PIN_VOLTAGE1 16
#define PIN_VOLTAGE2 17
#define PIN_CURRENT 18
#define PIN_PRESSURE 19
#define SMARTPORT_FRSKY_SERIAL Serial1
#define SRXL_IBUS_SBUS_SERIAL Serial3
//#define SOFTWARE_SERIAL
#define ESC_SERIAL Serial2
#define GPS_SERIAL Serial3
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

#endif