#ifndef CONFIG_H
#define CONFIG_H

// Version
#define VERSION_MAJOR 0
#define VERSION_MINOR 8
#define VERSION_PATCH 0


// Select RX protocol
#define RX_PROTOCOL RX_SMARTPORT // RX_SMARTPORT, RX_XBUS, RX_SRXL, RX_FRSKY, RX_IBUS

// Select SRLX valriant (only for SRXL)
#define SRXL_VARIANT SRXL_V5 // Only implemented SRXL_V5 (SPEKTRUM)

// Sensor Id (only smartport)
#define SENSOR_ID 10

// Pwm out
#define PWMOUT_DUTY 0.5 // 0.5 = 50%

// GPS serial
#define GPS_BAUD_RATE 9600

// HW V4 signature (only smartport). This outputs esc signature and raw current to sensors 5100, 5101 and 5102
#define ESC_SIGNATURE

// Add init delay for FlyFun ESC. Uncomment if the ESC doesn't arm
//#define ESC_INIT_DELAY 10000

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
#define SRXL_IBUS_SERIAL Serial

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
#define SRXL_IBUS_SERIAL Serial1

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
#define SRXL_IBUS_SERIAL Serial3

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
#define SRXL_IBUS_SERIAL Serial

#define SOFTWARE_SERIAL
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
#define SRXL_IBUS_SERIAL Serial3

//#define SOFTWARE_SERIAL
#define ESC_SERIAL Serial2
#define GPS_SERIAL Serial3
#endif

// Lua config (only opentx)
#define CONFIG_LUA // Comment if not using lua script for configuration (only smartport)

// Force eeprom write
//#define FORCE_EEPROM_WRITE // Uncomment to force write eeprom as defined in config.h. Useful when using lua and eeprom is messed up. Reflash againg with line commented or config will be reset at power up 

/* Debug
   Disconnect Vcc from the RC model to the Arduino
   Do not connect at the same time Vcc from the model and usb (TTL)
   Telemetry may not work properly in debug mode
   Connect arduino Rx to TTL Tx for flashing, then if applicabe connect arduino Rx to esc or gps
*/

//#define DEBUG
//#define DEBUG_P
//#define DEBUG_ESC
//#define DEBUG_ESC_RX
//#define DEBUG_EEPROM_WRITE
//#define DEBUG_EEPROM_READ
//#define DEBUG_GPS

//#define DEBUG_ESC_HW_V3
//#define DEBUG_ESC_HW_V4
//#define DEBUG_ESC_PWM
//#define DEBUG_ESC_CASTLE
//#define DEBUG_ESC_KONTRONIK

//#define SIM_RX
//#define SIM_SENSORS
//#define SIM_LUA_SEND
//#define SIM_LUA_RECEIVE

// --------------------------- Do not change config below if using lua script ---------------------------

// Select sensors
#define CONFIG_ESC_PROTOCOL PROTOCOL_NONE // PROTOCOL_NONE, PROTOCOL_HW_V3, PROTOCOL_HW_V4_LV, PROTOCOL_HW_V4_HV, PROTOCOL_HW_V5_LV, PROTOCOL_HW_V5_HV, PROTOCOL_PWM, PROTOCOL_CASTLE, PROTOCOL_KONTRONIK
#define CONFIG_AIRSPEED false
#define CONFIG_GPS false
#define CONFIG_VOLTAGE1 false
#define CONFIG_VOLTAGE2 false
#define CONFIG_CURRENT false
#define CONFIG_NTC1 false
#define CONFIG_NTC2 false
#define CONFIG_PWMOUT false
#define CONFIG_I2C1_TYPE I2C_NONE // I2C_NONE, I2C_BMP280
#define CONFIG_I2C1_ADDRESS 0

// Refresh rate in 0.1s (1 = 100ms)
#define CONFIG_REFRESH_RPM 10
#define CONFIG_REFRESH_VOLT 10
#define CONFIG_REFRESH_CURR 10
#define CONFIG_REFRESH_TEMP 10
#define CONFIG_REFRESH_DEF 10

// Averaging elements
#define CONFIG_AVERAGING_ELEMENTS_RPM 3
#define CONFIG_AVERAGING_ELEMENTS_VOLT 3
#define CONFIG_AVERAGING_ELEMENTS_CURR 3
#define CONFIG_AVERAGING_ELEMENTS_TEMP 3
#define CONFIG_AVERAGING_ELEMENTS_DEF 3

//------------------------- END OF CONFIG -----------------------------//

#define DEBUG_SERIAL Serial

#define N_TO_ALPHA(VALUE) (2.0 / (1 + VALUE) * 100)

#define RX_SMARTPORT 0
#define RX_XBUS 1
#define RX_SRXL 2
#define RX_IBUS 3
#define RX_FRSKY 4
#define RX_BST 5

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

// opentx
#define DATA_ID 0x5000 // DataId (sensor type)

//#define SRXL_V1 0x01
//#define SRXL_V2 0x02
#define SRXL_V5 0x05

#define ALPHA(ELEMENTS) (uint8_t)(2.0 / (ELEMENTS + 1) * 100)
#define MS_TO_COMP(SCALER) (F_CPU / (SCALER * 1000.0))
#define COMP_TO_MS(SCALER) ((SCALER * 1000.0) / F_CPU)

#endif