#ifndef CONFIG_H
#define CONFIG_H

// Select RX protocol
#define RX_PROTOCOL RX_SMARTPORT // RX_SMARTPORT, RX_XBUS, RX_SRXL

// Select SRLX valriant (only for SRXL)
#define SRXL_VARIANT SRXL_V5 // Only implemented SRXL_V5 (SPEKTRUM)

// Select sensors
#define CONFIG_ESC_PROTOCOL PROTOCOL_NONE // PROTOCOL_NONE PROTOCOL_HW_V3, PROTOCOL_HW_V4_LV, PROTOCOL_HW_V4_HV, PROTOCOL_HW_V5_LV, PROTOCOL_HW_V5_HV, PROTOCOL_PWM, PROTOCOL_CASTLE
#define CONFIG_AIRSPEED false
#define CONFIG_GPS false
#define CONFIG_VOLTAGE1 false
#define CONFIG_VOLTAGE2 false
#define CONFIG_CURRENT false
#define CONFIG_NTC1 false
#define CONFIG_NTC2 false
#define CONFIG_PWMOUT false

// Refresh rate in 0.1s (1 = 100ms)
#define CONFIG_REFRESH_RPM 2
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

// Sensor Id (smartport)
#define SENSOR_ID 10

// Lua config
//#define CONFIG_LUA // Comment if not using lua script for configuration (only smartport)

// Pwm out
#define PWMOUT_DUTY 0.5 // 0.5 = 50%

// Serial ports
#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB)
#define ESC_SERIAL Serial
#define GPS_SERIAL Serial
#define DEBUG_SERIAL Serial
#endif
#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
#define ESC_SERIAL Serial
#define GPS_SERIAL Serial1
#define DEBUG_SERIAL Serial
#endif
#if defined(__AVR_ATmega2560__)
#define ESC_SERIAL Serial1
#define GPS_SERIAL Serial2
#define DEBUG_SERIAL Serial
#endif

/* Debug
   Disconnect Vcc from the RC model to the Arduino
   Do not connect at the same time Vcc from the model and usb (TTL)
   Telemetry may not work properly in debug mode
   Connect arduino Rx to TTL Tx for flashing, then if applicabe connect arduino Rx to esc or gps
*/

//#define DEBUG
//#define DEBUG_PLOTTER rpm/60
//#define DEBUG_ESC

//#define SIM_RX
//#define SIM_SENSORS

//-------------------------END OF CONFIG-----------------------------//

//#define SRXL_V1 0x01
//#define SRXL_V2 0x02
#define SRXL_V5 0x05

#define CONFIG_ALPHA_RPM N_TO_ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM)
#define CONFIG_ALPHA_VOLT N_TO_ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM)
#define CONFIG_ALPHA_CURR N_TO_ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM)
#define CONFIG_ALPHA_TEMP N_TO_ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM)
#define CONFIG_ALPHA_DEF N_TO_ALPHA(CONFIG_AVERAGING_ELEMENTS_RPM)

#define N_TO_ALPHA(VALUE) 2 / (1 + VALUE) * 100

#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_A_STAR_328PB)
#define PIN_SMARTPORT_RX 7
#define PIN_SMARTPORT_TX 12
#endif
#if defined(__AVR_ATmega328PB__) || defined(ARDUINO_AVR_A_STAR_328PB)
#define PIN_SMARTPORT_RX 4
#define PIN_SMARTPORT_TX 23
#endif
#if defined(__AVR_ATmega2560__)
#define PIN_SMARTPORT_RX 4
#define PIN_SMARTPORT_TX 12
#endif
#define PIN_NTC1 A0
#define PIN_NTC2 A1
#define PIN_VOLTAGE1 A2
#define PIN_VOLTAGE2 A3
#define PIN_CURRENT A6
#define PIN_PRESSURE A7

#define RX_SMARTPORT 1
#define RX_XBUS 2
#define RX_SRXL 3
#define RX_IBUS 4

#define PROTOCOL_NONE 0
#define PROTOCOL_HW_V3 1
#define PROTOCOL_HW_V4_LV 2
#define PROTOCOL_HW_V4_HV 3
#define PROTOCOL_HW_V5_LV 4
#define PROTOCOL_HW_V5_HV 5
#define PROTOCOL_PWM 6
#define PROTOCOL_CASTLE 7

#endif