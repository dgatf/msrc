#ifndef CONFIG_H
#define CONFIG_H

/* Receiver protocol */
#define RX_PROTOCOL RX_SMARTPORT // RX_SMARTPORT, RX_XBUS, RX_SRXL, RX_FRSKY, RX_IBUS, RX_SBUS, RX_MULTIPLEX, RX_JETIEX

/* Sensors */
#define CONFIG_ESC_PROTOCOL PROTOCOL_NONE // PROTOCOL_NONE, PROTOCOL_HW_V3, PROTOCOL_HW_V4_LV, PROTOCOL_HW_V4_HV, PROTOCOL_HW_V5_LV, PROTOCOL_HW_V5_HV, PROTOCOL_PWM, PROTOCOL_CASTLE, PROTOCOL_KONTRONIK
#define CONFIG_GPS false
#define GPS_BAUD_RATE 9600
#define CONFIG_VOLTAGE1 false
#define CONFIG_VOLTAGE2 false
#define CONFIG_NTC1 false
#define CONFIG_NTC2 false
#define CONFIG_CURRENT false
#define CONFIG_AIRSPEED false
#define CONFIG_I2C1_TYPE I2C_NONE // I2C_NONE, I2C_BMP280
#define CONFIG_I2C1_ADDRESS 0
/* Refresh rate in 0.1s (1 = 100ms) */
#define CONFIG_REFRESH_RPM 1
#define CONFIG_REFRESH_VOLT 1
#define CONFIG_REFRESH_CURR 1
#define CONFIG_REFRESH_TEMP 1
#define CONFIG_REFRESH_DEF 1
/* Averaging elements */
#define CONFIG_AVERAGING_ELEMENTS_RPM 3
#define CONFIG_AVERAGING_ELEMENTS_VOLT 3
#define CONFIG_AVERAGING_ELEMENTS_CURR 3
#define CONFIG_AVERAGING_ELEMENTS_TEMP 3
#define CONFIG_AVERAGING_ELEMENTS_DEF 3

/* Pwm out */
#define CONFIG_PWMOUT false
#define PWMOUT_DUTY 0.5 // 0.5 = 50%

/* Only smartport and opentx */
#define SENSOR_ID 10 // Sensor Id 
#define DATA_ID 0x5000 // DataId (sensor type)
//#define ESC_SIGNATURE // HW V4 signature (only smartport). This outputs esc signature and raw current to sensors 5100, 5101 and 5102
#define CONFIG_LUA // Comment if not using lua script for configuration (only smartport)

/* Use library I2C_T3 for Teensy LC/3.X */
#define I2C_T3_TEENSY

/* Add init delay for FlyFun ESC. Uncomment if the ESC doesn't arm */
//#define ESC_INIT_DELAY 10000

/* Force eeprom write */
//#define FORCE_EEPROM_WRITE // Uncomment to force write eeprom as defined in config.h. Useful when using lua and eeprom is messed up. Reflash againg with line commented or config will be reset at power up 

/* Debug
   Disconnect Vcc from the RC model to the Arduino
   Do not connect at the same time Vcc from the model and usb (TTL)
   Telemetry may not work properly in debug mode
   Connect arduino Rx to TTL Tx for flashing, then if applicabe connect arduino Rx to esc or gps
*/

//#define DEBUG
//#define DEBUG_PACKET
//#define DEBUG_EEPROM_WRITE
//#define DEBUG_EEPROM_READ

//#define DEBUG_HW3
//#define DEBUG_HW4
//#define DEBUG_KONTRONIK
//#define DEBUG_GPS
//#define DEBUG_PWM
//#define DEBUG_CASTLE
//#define DEBUG_CASTLE_RX

//#define SIM_RX
//#define SIM_SENSORS
//#define SIM_LUA_SEND
//#define SIM_LUA_RECEIVE

#endif