#include "hott.h"

#include <hardware/flash.h>
#include <hardware/sync.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "airspeed.h"
#include "bmp180.h"
#include "bmp280.h"
#include "common.h"
#include "config.h"
#include "current.h"
#include "esc_apd_f.h"
#include "esc_apd_hv.h"
#include "esc_castle.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
#include "esc_hw5.h"
#include "esc_kontronik.h"
#include "esc_omp_m4.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "gps.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "ibus.h"
#include "ina3221.h"
#include "ms5611.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "smart_esc.h"
#include "stdlib.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "xgzp68xxd.h"

#define HOTT_VARIO_MODULE_ID 0x89
#define HOTT_GPS_MODULE_ID 0x8A
#define HOTT_ESC_MODULE_ID 0x8C
#define HOTT_GENERAL_AIR_MODULE_ID 0x8D
#define HOTT_ELECTRIC_AIR_MODULE_ID 0x8E  // not used, for internal Hott telemetry

#define HOTT_VARIO_TEXT_ID ((HOTT_VARIO_MODULE_ID << 4) & 0xFF)
#define HOTT_GPS_TEXT_ID ((HOTT_GPS_MODULE_ID << 4) & 0xFF)
#define HOTT_ESC_TEXT_ID ((HOTT_ESC_MODULE_ID << 4) & 0xFF)
#define HOTT_GENERAL_AIR_TEXT_ID ((HOTT_GENERAL_AIR_MODULE_ID << 4) & 0xFF)
#define HOTT_ELECTRIC_AIR_TEXT_ID ((HOTT_ELECTRIC_AIR_MODULE_ID << 4) & 0xFF)

#define HOTT_BINARY_MODE_REQUEST_ID 0x80
#define HOTT_TEXT_MODE_REQUEST_ID 0x7F

#define HOTT_TIMEOUT_US 5000
#define HOTT_INTERBYTE_DELAY_US 500
#define HOTT_PACKET_LENGHT 2

#define HOTT_START_BYTE 0x7C
#define HOTT_END_BYTE 0x7D

// TYPE
#define HOTT_TYPE_VARIO 0
#define HOTT_TYPE_ESC 1
#define HOTT_TYPE_GENERAL 2
#define HOTT_TYPE_GPS 3
#define HOTT_TYPE_ELECTRIC 4

// VARIO
#define HOTT_VARIO_ALTITUDE 0
#define HOTT_VARIO_M1S 1
#define HOTT_VARIO_M3S 2
#define HOTT_VARIO_M10S 3

//  ESC
#define HOTT_ESC_VOLTAGE 0
#define HOTT_ESC_CONSUMPTION 1
#define HOTT_ESC_TEMPERATURE 2
#define HOTT_ESC_CURRENT 3
#define HOTT_ESC_RPM 4
#define HOTT_ESC_THROTTLE 5  // 0-100%
#define HOTT_ESC_SPEED 6
#define HOTT_ESC_BEC_VOLTAGE 7
#define HOTT_ESC_BEC_CURRENT 8
#define HOTT_ESC_BEC_TEMPERATURE 9
#define HOTT_ESC_EXT_TEMPERATURE 10

// ELECTRIC - used by internal receiver vario (shouldnt be used)
#define HOTT_ELECTRIC_EXT_TEMPERATURE 0
#define HOTT_ELECTRIC_CELL_BAT_1_VOLTAGE 1
#define HOTT_ELECTRIC_CELL_BAT_2_VOLTAGE 2
#define HOTT_ELECTRIC_BAT_1_VOLTAGE 3
#define HOTT_ELECTRIC_BAT_2_VOLTAGE 4
#define HOTT_ELECTRIC_TEMPERATURE_1 5
#define HOTT_ELECTRIC_TEMPERATURE_2 6
#define HOTT_ELECTRIC_HEIGHT 7
#define HOTT_ELECTRIC_CURRENT 8
#define HOTT_ELECTRIC_CAPACITY 9
#define HOTT_ELECTRIC_M2S 10
#define HOTT_ELECTRIC_M3S 11
#define HOTT_ELECTRIC_RPM 12
#define HOTT_ELECTRIC_MINUTES 13
#define HOTT_ELECTRIC_SECONDS 14
#define HOTT_ELECTRIC_SPEED 15

// GPS
#define HOTT_GPS_DIRECTION 0
#define HOTT_GPS_SPEED 1
#define HOTT_GPS_LATITUDE 2
#define HOTT_GPS_LONGITUDE 3
#define HOTT_GPS_DISTANCE 4
#define HOTT_GPS_ALTITUDE 5
#define HOTT_GPS_CLIMBRATE 6
#define HOTT_GPS_CLIMBRATE3S 7
#define HOTT_GPS_SATS 8
#define HOTT_GPS_FIX 9
#define HOTT_GPS_TIME 10

// GENERAL
#define HOTT_GENERAL_CELL_1 0
#define HOTT_GENERAL_CELL_2 1
#define HOTT_GENERAL_CELL_3 2
#define HOTT_GENERAL_CELL_4 3
#define HOTT_GENERAL_CELL_5 4
#define HOTT_GENERAL_CELL_6 5
#define HOTT_GENERAL_BATTERY_1 6
#define HOTT_GENERAL_BATTERY_2 7
#define HOTT_GENERAL_TEMP_1 8
#define HOTT_GENERAL_TEMP_2 9
#define HOTT_GENERAL_FUEL 10  // ml
#define HOTT_GENERAL_RPM_1 11
#define HOTT_GENERAL_ALTITUDE 12
#define HOTT_GENERAL_CLIMBRATE 13
#define HOTT_GENERAL_CLIMBRATE3S 14
#define HOTT_GENERAL_CURRENT 15
#define HOTT_GENERAL_VOLTAGE 16
#define HOTT_GENERAL_CAPACITY 17
#define HOTT_GENERAL_SPEED 18
#define HOTT_GENERAL_RPM_2 19
#define HOTT_GENERAL_PRESSURE 20

#define HOTT_KEY_RIGHT 0xE
#define HOTT_KEY_DOWN 0xB
#define HOTT_KEY_UP 0xD
#define HOTT_KEY_SET 0x9
#define HOTT_KEY_LEFT 0x7

#define ALARMS_FLASH_TARGET_OFFSET (CONFIG_FLASH_TARGET_OFFSET + 4096)  // after program + config

// Missing: 0x01, 0x04, 0x05, 0x0C, 0x15
#define ALARM_VOICE_NO_ALARM 0x00
#define ALARM_VOICE_UNKNOWN1 0x01
#define ALARM_VOICE_NEGATIVE_DIFFERENCE_2 0x02
#define ALARM_VOICE_NEGATIVE_DIFFERENCE_1 0x03
#define ALARM_VOICE_UNKNOWN2 0x04
#define ALARM_VOICE_UNKNOWN3 0x05
#define ALARM_VOICE_MIN_SENSOR_1_TEMP 0x06
#define ALARM_VOICE_MIN_SENSOR_2_TEMP 0x07
#define ALARM_VOICE_MAX_SENSOR_1_TEMP 0x08
#define ALARM_VOICE_MAX_SENSOR_2_TEMP 0x09
#define ALARM_VOICE_MAX_SENSOR_1_VOLTAGE 0x0A
#define ALARM_VOICE_MAX_SENSOR_2_VOLTAGE 0x0B
#define ALARM_VOICE_UNKNOWN4 0x0C
#define ALARM_VOICE_POSITIVE_DIFFERENCE_2 0x0D
#define ALARM_VOICE_POSITIVE_DIFFERENCE_1 0x0E
#define ALARM_VOICE_MIN_ALTITUDE 0x0F
#define ALARM_VOICE_MIN_POWER_VOLTAGE 0x10
#define ALARM_VOICE_MIN_CELL_VOLTAGE 0x11
#define ALARM_VOICE_MIN_SENSOR_1_VOLTAGE 0x12
#define ALARM_VOICE_MIN_SENSOR_2_VOLTAGE 0x13
#define ALARM_VOICE_MIN_RPM 0x14
#define ALARM_VOICE_UNKNOWN5 0x15
#define ALARM_VOICE_MAX_CAPACITY 0x16
#define ALARM_VOICE_MAX_CURRENT 0x17
#define ALARM_VOICE_MAX_POWER_VOLTAGE 0x18
#define ALARM_VOICE_MAX_RPM 0x19
#define ALARM_VOICE_MAX_ALTITUDE 0x1A

typedef enum alarm_vario_t {
    ALARM_BITMASK_VARIO_ALTITUDE = 0,
    ALARM_BITMASK_VARIO_MAX_ALTITUDE,
    ALARM_BITMASK_VARIO_MIN_ALTITUDE,
    ALARM_BITMASK_VARIO_M1S,
    ALARM_BITMASK_VARIO_M3S,
    ALARM_BITMASK_VARIO_M10S,
} alarm_vario_t;

typedef enum alarm_airesc_t {
    ALARM_BITMASK_AIRESC_VOLTAGE = 0,
    ALARM_BITMASK_AIRESC_MIN_VOLTAGE,
    ALARM_BITMASK_AIRESC_CAPACITY,
    ALARM_BITMASK_AIRESC_TEMPERATURE,
    ALARM_BITMASK_AIRESC_MAX_TEMPERATURE,
    ALARM_BITMASK_AIRESC_CURRENT,
    ALARM_BITMASK_AIRESC_MAX_CURRENT,
    ALARM_BITMASK_AIRESC_RPM,
    ALARM_BITMASK_AIRESC_MAX_RPM,
    ALARM_BITMASK_AIRESC_THROTTLE,
    ALARM_BITMASK_AIRESC_SPEED,
    ALARM_BITMASK_AIRESC_MAX_SPEED,
    ALARM_BITMASK_AIRESC_BEC_VOLTAGE,
    ALARM_BITMASK_AIRESC_MIN_BEC_VOLTAGE,
    ALARM_BITMASK_AIRESC_BEC_CURRENT,
    ALARM_BITMASK_AIRESC_MIN_BEC_CURRENT,
    ALARM_BITMASK_AIRESC_MAX_BEC_CURRENT,
    ALARM_BITMASK_AIRESC_BEC_TEMPERATURE,
    ALARM_BITMASK_AIRESC_MAX_BEC_TEMPERATURE,
    ALARM_BITMASK_AIRESC_EXT_TEMPERATURE,
    ALARM_BITMASK_AIRESC_MAX_EXT_TEMPERATURE,
    ALARM_BITMASK_AIRESC_RPM_WITHOUT_GEAR,
} alarm_airesc_t;

typedef enum alarm_general_air_t {
    ALARM_BITMASK_GENERAL_AIR_CELL_VOLTAGE = 0,
    ALARM_BITMASK_GENERAL_AIR_BATTERY_1,
    ALARM_BITMASK_GENERAL_AIR_BATTERY_2,
    ALARM_BITMASK_GENERAL_AIR_TEMPERATURE_1,
    ALARM_BITMASK_GENERAL_AIR_TEMPERATURE_2,
    ALARM_BITMASK_GENERAL_AIR_FUEL_PERCENT,
    ALARM_BITMASK_GENERAL_AIR_FUEL_ML,
    ALARM_BITMASK_GENERAL_AIR_RPM,
    ALARM_BITMASK_GENERAL_AIR_ALTITUDE,
    ALARM_BITMASK_GENERAL_AIR_CLIMBRATE,
    ALARM_BITMASK_GENERAL_AIR_CLIMBRATE3S,
    ALARM_BITMASK_GENERAL_AIR_CURRENT,
    ALARM_BITMASK_GENERAL_AIR_VOLTAGE,
    ALARM_BITMASK_GENERAL_AIR_CAPACITY,
    ALARM_BITMASK_GENERAL_AIR_SPEED,
    ALARM_BITMASK_GENERAL_AIR_MIN_CELL_VOLTAGE,
    ALARM_BITMASK_GENERAL_AIR_MIN_CELL_VOLTAGE_NUM,
    ALARM_BITMASK_GENERAL_AIR_RPM_2,
} alarm_general_air_t;

typedef enum alarm_gps_t {
    ALARM_BITMASK_GPS_FLIGHT_DIRECTION = 0,
    ALARM_BITMASK_GPS_SPEED,
    ALARM_BITMASK_GPS_LATITUDE,
    ALARM_BITMASK_GPS_LONGITUDE,
    ALARM_BITMASK_GPS_DISTANCE,
    ALARM_BITMASK_GPS_ALTITUDE,
    ALARM_BITMASK_GPS_CLIMBRATE,
    ALARM_BITMASK_GPS_CLIMBRATE3S,
    ALARM_BITMASK_GPS_SATS,
} alarm_general_gps_t;

typedef struct hott_sensor_vario_t {
    uint8_t startByte;  // 1
    uint8_t sensorID;
    uint8_t warningID;
    uint8_t sensorTextID;
    uint8_t alarmInverse;
    uint16_t altitude;     // 6-7 value + 500 (e.g. 0m = 500)
    uint16_t maxAltitude;  // 8-9
    uint16_t minAltitude;
    uint16_t m1s;   // 12-13 (value * 100) + 30000 (e.g. 10m = 31000)
    uint16_t m3s;   // ?? idem
    uint16_t m10s;  // ?? idem
    uint8_t text[24];
    uint8_t version;
    uint8_t empty;
    uint8_t endByte;
    uint8_t checksum;
} __attribute__((packed)) hott_sensor_vario_t;

typedef struct hott_sensor_airesc_t {
    uint8_t startByte;                  // 1
    uint8_t sensorID;                   // 2
    uint8_t warningID;                  // 3
    uint8_t sensorTextID;               // Byte 4
    uint16_t alarmInverse;              // Byte 5, 6
    uint16_t inputVolt;                 // Byte 7,8
    uint16_t minInputVolt;              // Byte 9,10
    uint16_t capacity;                  // Byte 11,12
    uint8_t escTemperature;             // Byte 13
    uint8_t maxEscTemperature;          // Byte 14
    uint16_t current;                   // Byte 15,16
    uint16_t maxCurrent;                // Byte 17,18
    uint16_t RPM;                       // Byte 19,20
    uint16_t maxRPM;                    // Byte 21,22
    uint8_t throttlePercent;            // Byte 23
    uint16_t speed;                     // Byte 24,25
    uint16_t maxSpeed;                  // Byte 26,27
    uint8_t BECVoltage;                 // Byte 28
    uint8_t minBECVoltage;              // Byte 29
    uint8_t BECCurrent;                 // Byte 30
    uint8_t minBECCurrent;              // Byte 31
    uint8_t maxBECCurrent;              // Byte 32
    uint8_t PWM;                        // Byte 33
    uint8_t BECTemperature;             // Byte 34
    uint8_t maxBECTemperature;          // Byte 35
    uint8_t motorOrExtTemperature;      // Byte 36
    uint8_t maxMotorOrExtTemperature;   // Byte 37
    uint16_t RPMWithoutGearOrExt;       // Byte 38,39
    uint8_t timing;                     // Byte 40
    uint8_t advancedTiming;             // Byte 41
    uint8_t highestCurrentMotorNumber;  // Byte 42
    uint8_t version;                    /* Byte 43: 00 version number */
    uint8_t endByte;                    /* Byte 44: 0x7D Ende byte */
    uint8_t checksum;                   /* Byte 45: Parity Byte */
} __attribute__((packed)) hott_sensor_airesc_t;

// used by internal receiver vario (shouldnt be used)
typedef struct hott_sensor_electric_air_t {
    uint8_t startByte;      // 1
    uint8_t sensorID;       // 2
    uint8_t warningID;      // 3: Alarm
    uint8_t sensorTextID;   // 4:
    uint16_t alarmInverse;  // 5 6: alarm bitmask. Value is displayed inverted
    uint8_t cell1L;         // 7: Low Voltage Cell 1 in 0,02 V steps
    uint8_t cell2L;         // 8: Low Voltage Cell 2 in 0,02 V steps
    uint8_t cell3L;         // 9: Low Voltage Cell 3 in 0,02 V steps
    uint8_t cell4L;         // 10: Low Voltage Cell 4 in 0,02 V steps
    uint8_t cell5L;         // 11: Low Voltage Cell 5 in 0,02 V steps
    uint8_t cell6L;         // 12: Low Voltage Cell 6 in 0,02 V steps
    uint8_t cell7L;         // 13: Low Voltage Cell 7 in 0,02 V steps
    uint8_t cell1H;         // 14: High Voltage Cell 1 in 0.02 V steps
    uint8_t cell2H;         // 15
    uint8_t cell3H;         // 16
    uint8_t cell4H;         // 17
    uint8_t cell5H;         // 18
    uint8_t cell6H;         // 19
    uint8_t cell7H;         // 20
    uint16_t battery1;      // 21 Battery 1 in 100mv steps; 50 == 5V
    uint16_t battery2;      // 23 Battery 2 in 100mv steps; 50 == 5V
    uint8_t temp1;          // 25 Temp 1; Offset of 20. 20 == 0C
    uint8_t temp2;          // 26 Temp 2; Offset of 20. 20 == 0C
    uint16_t height;        // 27 28 Height. Offset -500. 500 == 0
    uint16_t current;       // 29 30 1 = 0.1A
    uint16_t driveVoltage;  // 31
    uint16_t capacity;      // 33 34 mAh
    uint16_t m2s;           // 35 36  /* Steigrate m2s; 0x48 == 0
    uint8_t m3s;            // 37  /* Steigrate m3s; 0x78 == 0
    uint16_t rpm;           // 38 39 /* RPM. 10er steps; 300 == 3000rpm
    uint8_t minutes;        // 40
    uint8_t seconds;        // 41
    uint8_t speed;          // 42
    uint8_t version;        // 43
    uint8_t endByte;        // 44
    uint8_t checksum;       // 45
} __attribute__((packed)) hott_sensor_electric_air_t;

typedef struct hott_sensor_general_air_t {
    uint8_t startByte;             //#01 start byte constant value 0x7c
    uint8_t sensorID;              //#02 EAM sensort id. constat value 0x8d=GENRAL AIR MODULE
    uint8_t warningID;             //#03 1=A 2=B ... 0x1a=Z 0 = no alarm
                                   /* VOICE OR BIP WARNINGS
                           Alarme sonore A.. Z, octet correspondant 1 à 26
                           0x00 00 0 No alarm
                           0x01 01 A
                           0x02 02 B Negative Difference 2 B
                           0x03 03 C Negative Difference 1 C
                           0x04 04 D
                           0x05 05 E
                           0x06 06 F Min. Sensor 1 temp. F
                           0x07 07 G Min. Sensor 2 temp. G
                           0x08 08 H Max. Sensor 1 temp. H
                           0x09 09 I Max. Sensor 2 temp. I
                           0xA 10 J Max. Sens. 1 voltage J
                           0xB 11 K Max. Sens. 2 voltage K
                           0xC 12 L
                           0xD 13 M Positive Difference 2 M
                           0xE 14 N Positive Difference 1 N
                           0xF 15 O Min. Altitude O
                           0x10 16 P Min. Power Voltage P // We use this one for Battery Warning
                           0x11 17 Q Min. Cell voltage Q
                           0x12 18 R Min. Sens. 1 voltage R
                           0x13 19 S Min. Sens. 2 voltage S
                           0x14 20 T Minimum RPM T
                           0x15 21 U
                           0x16 22 V Max. used capacity V
                           0x17 23 W Max. Current W
                           0x18 24 X Max. Power Voltage X
                           0x19 25 Y Maximum RPM Y
                           0x1A 26 Z Max. Altitude Z
                           */
    uint8_t sensorTextID;          //#04 constant value 0xd0
    uint16_t alarmInverse;         //#05-06 alarm bitmask. Value is displayed inverted
                                   // 0 all cell voltage
                                   // 1 Battery 1
                                   // 2 Battery 2
                                   // 3 Temperature 1
                                   // 4 Temperature 2
                                   // 5 Fuel
                                   // 6 mAh
                                   // 7 Altitude
                                   // 8 main power current
                                   // 9 main power voltage
                                   // 10 Altitude
                                   // 11 m/s
                                   // 12 m/3s
                                   // 13 unknown
                                   // 14 unknown
                                   // 15 "ON" sign/text msg active
    uint8_t cell[6];               //#7 Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
                                   //#8 Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
                                   //#9 Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
                                   //#10 Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
                                   //#11 Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
                                   //#12 Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
    uint16_t battery1;             //#13 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                                   //#14 MSB
    uint16_t battery2;             //#15 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                                   //#16 MSB
    uint8_t temperature1;          //#17 Temperature 1. Offset of 20. a value of 20 = 0°C
    uint8_t temperature2;          //#18 Temperature 2. Offset of 20. a value of 20 = 0°C
    uint8_t fuel_procent;          //#19 Fuel capacity in %. Values 0--100
                                   // graphical display ranges: 0-100% with new firmwares of the radios MX12/MX20/...
    uint16_t fuel_ml;              //#20 LSB Fuel in ml scale. Full = 65535!
                                   //#21 MSB
    uint16_t rpm;                  //#22 RPM in 10 RPM steps. 300 = 3000rpm
                                   //#23 MSB
    uint16_t altitude;             //#24 altitude in meters. offset of 500, 500 = 0m
                                   //#25 MSB
    uint16_t climbrate;            //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                                   //#27 MSB
    uint8_t climbrate3s;           //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
    uint16_t current;              //#29 current in 0.1A steps 100 == 10,0A
                                   //#30 MSB current display only goes up to 99.9 A (continuous)
    uint16_t main_voltage;         //#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
                                   //#32 MSB (Appears in GAM display right as alternate display.)
    uint16_t batt_cap;             //#33 LSB used battery capacity in 10mAh steps
                                   //#34 MSB
    uint16_t speed;                //#35 LSB (air?) speed in km/h(?) we are using ground speed here per default
                                   //#36 MSB speed
    uint8_t min_cell_volt;         //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
    uint8_t min_cell_volt_num;     //#38 number of the cell with the lowest voltage
    uint16_t rpm2;                 //#39 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
                                   //#40 MSB
    uint8_t general_error_number;  //#41 General Error Number (Voice Error == 12) TODO: more documentation
    uint8_t pressure;              //#42 High pressure up to 16bar. 0,1bar scale. 20 == 2.0 bar
                                   // 1 bar = 10 hoch 5 Pa
    uint8_t version;               //#43 version number (Bytes 35 .43 new but not yet in the record in the display!)
    uint8_t endByte;               //#44 stop byte 0x7D
    uint8_t checksum;              //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
} __attribute__((packed)) hott_sensor_general_air_t;

typedef struct hott_sensor_gps_t {
    uint8_t startByte;     /* Byte 1: 0x7C = Start byte data */
    uint8_t sensorID;      /* Byte 2: 0x8A = GPS Sensor */
    uint8_t warningID;     /* Byte 3: 0…= warning beeps */
    uint8_t sensorTextID;  /* Byte 4: 160 0xA0 Sensor ID Neu! */
    uint16_t alarmInverse; /* Byte 5-6: 01 inverse status */
    uint8_t
        flightDirection; /* Byte 7: 119 = Flugricht./dir. 1 = 2°; 0° (North), 9 0° (East), 180° (South), 270° (West) */
    uint16_t GPSSpeed;        /* Byte 8: 8 = Geschwindigkeit/GPS speed low byte 8km/h */
    uint8_t LatitudeNS;       /* Byte 10: 000 = N = 48°39’988 */
    uint16_t LatitudeDegMin;  /* Byte 11: 231 0xE7 = 0x12E7 = 4839 */
    uint16_t LatitudeSec;     /* Byte 13: 171 220 = 0xDC = 0x03DC =0988 */
    uint8_t longitudeEW;      /* Byte 15: 000  = E= 9° 25’9360 */
    uint16_t longitudeDegMin; /* Byte 16: 150 157 = 0x9D = 0x039D = 0925 */
    uint16_t longitudeSec;    /* Byte 18: 056 144 = 0x90 0x2490 = 9360*/
    uint16_t distance;        /* Byte 20: 027 123 = Entfernung/distance low byte 6 = 6 m */
    uint16_t altitude;        /* Byte 22: 243 244 = Höhe/Altitude low byte 500 = 0m */
    uint16_t climbrate;       /* Byte 24: 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) */
    uint8_t climbrate3s;      /* Byte 26: climbrate in m/3s resolution, value of 120 = 0 m/3s*/
    uint8_t GPSNumSat;        /* Byte 27: GPS.Satelites (number of satelites) (1 byte) */
    uint8_t GPSFixChar;       /* Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
    uint8_t homeDirection;    /* Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) */
    uint8_t angleXdirection;  /* Byte 30: angle x-direction (1 byte) */
    uint8_t angleYdirection;  /* Byte 31: angle y-direction (1 byte) */
    uint8_t angleZdirection;  /* Byte 32: angle z-direction (1 byte) */
    uint8_t gps_time_h;       //#33 UTC time hours
    uint8_t gps_time_m;       //#34 UTC time minutes
    uint8_t gps_time_s;       //#35 UTC time seconds
    uint8_t gps_time_sss;     //#36 UTC time milliseconds
    uint16_t msl_altitude;    //#37 mean sea level altitude
    uint8_t vibration;        /* Byte 39: vibration (1 bytes) */
    uint8_t Ascii4;           /* Byte 40: 00 ASCII Free Character [4] appears right to home distance */
    uint8_t Ascii5;           /* Byte 41: 00 ASCII Free Character [5] appears right to home direction*/
    uint8_t GPS_fix;          /* Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX */
    uint8_t version;          /* Byte 43: 00 version number */
    uint8_t endByte;          /* Byte 44: 0x7D Ende byte */
    uint8_t checksum;         /* Byte 45: Parity Byte */
} __attribute__((packed)) hott_sensor_gps_t;

typedef struct hott_text_msg_t {
    uint8_t start_byte;  //#01 Starting constant value == 0x7b
    uint8_t esc;         //#02 Escape (higher-ranking menu in text mode or Text mode leave)
                         // 0x00 to stay normal
                         // 0x01 to exit
                         // I will send 2 times, so the ESCAPE works really well, so two data frames with 0x01 in byte 2
    uint8_t warning_beeps;  //#03 1=A 2=B ...
    char text[8][21];       //#04...#171 168 ASCII text to display to
                            // Bit 7 = 1 -> Inverse character display
                            // Display 21x8
    uint8_t stop_byte;      //#172 constant value 0x7d
    uint8_t checksum;       //#173 Checksum / parity
} __attribute__((packed)) hott_text_msg_t;

typedef struct hott_sensors_t {
    bool is_enabled[4];
    float *gps[11];
    float *vario[4];
    float *esc[11];
    float *general_air[21];
} hott_sensors_t;

typedef struct trigger_t {
    float value;
    float max;
    float incr;
    char str[21];
} trigger_t;

typedef enum gps_triggers_t {
    TRIGGER_GPS_MIN_SPEED,
    TRIGGER_GPS_MAX_SPEED,
    TRIGGER_GPS_MIN_ALTITUDE,
    TRIGGER_GPS_MAX_ALTITUDE,
    TRIGGER_GPS_MAX_CLIMB,
    TRIGGER_GPS_MIN_SATS,
    TRIGGER_GPS_MAX_DISTANCE,
    TRIGGERS_GPS
} gps_triggers_t;

typedef enum vario_triggers_t {
    TRIGGER_VARIO_MIN_ALTITUDE,
    TRIGGER_VARIO_MAX_ALTITUDE,
    TRIGGER_VARIO_VSPD,
    TRIGGERS_VARIO
} vario_triggers_t;

typedef enum esc_triggers_t {
    TRIGGER_ESC_CONSUMPTION,
    TRIGGER_ESC_TEMPERATURE,
    TRIGGER_ESC_MIN_RPM,
    TRIGGER_ESC_MAX_RPM,
    TRIGGER_ESC_VOLTAGE,
    TRIGGER_ESC_CURRENT,
    TRIGGERS_ESC
} esc_triggers_t;

typedef enum general_triggers_t {
    TRIGGER_GENERAL_BATTERY,
    TRIGGER_GENERAL_CAPACITY,
    TRIGGER_GENERAL_TEMPERATURE,
    TRIGGER_GENERAL_MIN_ALTITUDE,
    TRIGGER_GENERAL_MAX_ALTITUDE,
    TRIGGER_GENERAL_CURRENT,
    TRIGGERS_GENERAL
} general_triggers_t;

typedef struct triggers_value_t {
    float gps[TRIGGERS_GPS];
    float vario[TRIGGERS_VARIO];
    float esc[TRIGGERS_ESC];
    float general[TRIGGERS_GENERAL];
} triggers_value_t;

typedef struct triggers_menu_t {
    trigger_t gps[TRIGGERS_GPS];
    trigger_t vario[TRIGGERS_VARIO];
    trigger_t esc[TRIGGERS_ESC];
    trigger_t general[TRIGGERS_GENERAL];
} triggers_menu_t;

typedef struct triggers_t {
    triggers_value_t *triggers;
    triggers_menu_t *pages;
} triggers_t;

typedef struct vario_alarm_parameters_t {
    float *altitude;
    float *vspd;
    float m1s;
    float m3s;
    float m10s;
} vario_alarm_parameters_t;

vario_alarm_parameters_t vario_alarm_parameters;
float *baro_temp = NULL, *baro_pressure = NULL;

static void process(hott_sensors_t *sensors, triggers_t *alarms);
static void format_binary_packet(triggers_t *alarms, hott_sensors_t *sensors, uint8_t address);
static void format_text_packet(triggers_t *alarms, hott_sensors_t *sensors, uint8_t sensor_id, uint8_t key);
static void send_packet(uint8_t *buffer, uint len);
static void module_alarms_save(triggers_value_t *module_alarms);
static triggers_value_t *alarms_read(void);
static uint8_t get_crc(const uint8_t *buffer, uint len);
static void set_config(hott_sensors_t *sensors);
static int64_t interval_1000_callback(alarm_id_t id, void *parameters);
static int64_t interval_3000_callback(alarm_id_t id, void *parameters);
static int64_t interval_10000_callback(alarm_id_t id, void *parameters);

void hott_task(void *parameters) {
    hott_sensors_t sensors = {0};
    triggers_menu_t pages = {.gps = {{.max = 200, .incr = 1, .str = "Speed Min"},
                                     {.max = 200, .incr = 1, .str = "Speed Max"},
                                     {.max = 5000, .incr = 1, .str = "Alt Min"},
                                     {.max = 5000, .incr = 1, .str = "Alt Max"},
                                     {.max = 200, .incr = 1, .str = "Vspd Max"},
                                     {.max = 20, .incr = 1, .str = "Sats Min"},
                                     {.max = 20000, .incr = 10, .str = "Dist Max"}},
                             .vario = {{.max = 5000, .incr = 1, .str = "Alt Min"},
                                       {.max = 5000, .incr = 1, .str = "Alt Max"},
                                       {.max = 200, .incr = 1, .str = "Vspd Max"}},
                             .esc = {{.max = 30000, .incr = 10, .str = "Cons Max"},
                                     {.max = 100, .incr = 1, .str = "Temp Max"},
                                     {.max = 20000, .incr = 10, .str = "RPM Min"},
                                     {.max = 20000, .incr = 10, .str = "RPM Max"},
                                     {.max = 100, .incr = 0.1, .str = "Volt Min"},
                                     {.max = 300, .incr = 1, .str = "Curr Max"}},
                             .general = {{.max = 100, .incr = 0.1, .str = "Volt Min"},
                                         {.max = 100, .incr = 10, .str = "Cons Max"},
                                         {.max = 100, .incr = 1, .str = "Temp Max"},
                                         {.max = 5000, .incr = 1, .str = "Alt Min"},
                                         {.max = 5000, .incr = 1, .str = "Alt Max"},
                                         {.max = 300, .incr = 0.1, .str = "Curr Max"}}};
    triggers_value_t triggers_value;
    memcpy(&triggers_value, (uint8_t *)alarms_read(), sizeof(triggers_value_t));
    triggers_t hott_alarms = {.triggers = &triggers_value, .pages = &pages};
    set_config(&sensors);
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(19200, UART_RECEIVER_TX, UART_RECEIVER_RX, HOTT_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    debug("\nHOTT init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&sensors, &hott_alarms);
    }
}

static void process(hott_sensors_t *sensors, triggers_t *alarms) {
    uint8_t len = uart0_available();
    if (len == HOTT_PACKET_LENGHT || len == HOTT_PACKET_LENGHT + 1) {
        uint8_t buffer[len];
        uart0_read_bytes(buffer, len);
        debug("\nHOTT (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, len, "0x%X ");
        if (buffer[0] == HOTT_BINARY_MODE_REQUEST_ID)
            format_binary_packet(alarms, sensors, buffer[1]);
        else if (buffer[0] == HOTT_TEXT_MODE_REQUEST_ID)
            format_text_packet(alarms, sensors, buffer[1] >> 4, buffer[1] & 0x0F);
    }
}

static void format_text_packet(triggers_t *alarms, hott_sensors_t *sensors, uint8_t sensor_id, uint8_t key) {
    static uint8_t item = 1;
    static uint8_t last_sensor = 0;
    static bool is_selected = false;
    hott_text_msg_t packet = {0};
    packet.start_byte = 0x7B;
    packet.esc = 0x00;
    packet.warning_beeps = 0x00;
    uint size = 0;
    trigger_t *module_alarms_pages = NULL;
    float *module_alarms_triggers = NULL;
    if (sensor_id != last_sensor) {
        item = 1;
        is_selected = false;
    }
    last_sensor = sensor_id;

    // Select module alarms
    switch (0x80 | sensor_id) {
        case HOTT_GPS_MODULE_ID:
            if (!sensors->is_enabled[HOTT_TYPE_GPS]) return;
            size = TRIGGERS_GPS;
            module_alarms_pages = alarms->pages->gps;
            module_alarms_triggers = alarms->triggers->gps;
            strcpy(packet.text[0], "GPS sensor");
            break;
        case HOTT_VARIO_MODULE_ID:
            if (!sensors->is_enabled[HOTT_TYPE_VARIO]) return;
            size = TRIGGERS_VARIO;
            module_alarms_pages = alarms->pages->vario;
            module_alarms_triggers = alarms->triggers->vario;
            strcpy(packet.text[0], "Vario sensor");
            break;
        case HOTT_ESC_MODULE_ID:
            if (!sensors->is_enabled[HOTT_TYPE_ESC]) return;
            size = TRIGGERS_ESC;
            module_alarms_pages = alarms->pages->esc;
            module_alarms_triggers = alarms->triggers->esc;
            strcpy(packet.text[0], "ESC sensor");
            break;
        case HOTT_GENERAL_AIR_MODULE_ID:
            if (!sensors->is_enabled[HOTT_TYPE_GENERAL]) return;
            size = TRIGGERS_GENERAL;
            module_alarms_pages = alarms->pages->general;
            module_alarms_triggers = alarms->triggers->general;
            strcpy(packet.text[0], "General sensor");
            break;
        default:
            break;
    }
    if (!module_alarms_triggers) return;

    // Handle events
    if (key == HOTT_KEY_LEFT) {
        if (is_selected) {
            module_alarms_triggers[item] -= 10 * module_alarms_pages[item].incr;
            if (module_alarms_triggers[item] < 0) module_alarms_triggers[item] = 0;
        } else {
            packet.esc = 0x01;  // exit
            strcat(packet.text[0], " (Exit)");
            is_selected = false;
            item = 1;
        }
    } else if (key == HOTT_KEY_RIGHT && is_selected) {
        module_alarms_triggers[item] += 10 * module_alarms_pages[item].incr;
        if (module_alarms_triggers[item] > module_alarms_pages[item].max)
            module_alarms_triggers[item] = module_alarms_pages[item].max;
    } else if (key == HOTT_KEY_UP) {
        if (!is_selected) {
            item++;
            if (item > size - 1) item = size - 1;
        } else {
            module_alarms_triggers[item] += module_alarms_pages[item].incr;
            if (module_alarms_triggers[item] > module_alarms_pages[item].max)
                module_alarms_triggers[item] = module_alarms_pages[item].max;
        }
    } else if (key == HOTT_KEY_DOWN) {
        if (!is_selected) {
            item--;
            if (item < 0) item = 0;
        } else {
            module_alarms_triggers[item] -= module_alarms_pages[item].incr;
            if (module_alarms_triggers[item] < 0) module_alarms_triggers[item] = 0;
        }
    } else if (key == HOTT_KEY_SET) {
        if (is_selected) {
            module_alarms_save(alarms->triggers);
            strcat(packet.text[0], " (Saved)");
            debug("\nHOTT (%u). Saved sensor config.", uxTaskGetStackHighWaterMark(NULL));
            is_selected = false;
        } else {
            is_selected = true;
        }
    } else {
        debug("\nHOTT (%u). Unknown key 0x%X.", uxTaskGetStackHighWaterMark(NULL), key);
    }

    // Draw page
    // 123456789012345678901
    // <str          > 12345
    if (size > 7) size = 7;
    for (int i = 0; i < size; i++) {
        if (module_alarms_triggers[i] < 0) module_alarms_triggers[i] = 0;
        if (module_alarms_triggers[i] > module_alarms_pages[i].max)
            module_alarms_triggers[i] = module_alarms_pages[i].max;
        if (isinf(module_alarms_triggers[i])) module_alarms_triggers[i] = 0;
        if (isnan(module_alarms_triggers[i])) module_alarms_triggers[i] = 0;
        if (module_alarms_pages[i].incr == 1)
            snprintf(packet.text[i + 1], 21, " %-13s %5.0f", module_alarms_pages[i].str, module_alarms_triggers[i]);
        else
            snprintf(packet.text[i + 1], 21, " %-13s %5.1f", module_alarms_pages[i].str, module_alarms_triggers[i]);
    }
    packet.text[item + 1][0] |= '>';
    if (is_selected) {
        for (int i = 16; i < 21; i++) packet.text[item + 1][i] |= 0x80;
    }

    // Send packet
    packet.stop_byte = 0x7D;
    packet.checksum = get_crc((uint8_t *)&packet, sizeof(packet) - 1);
    debug("\nHOTT (%u). Send sensor menu. Item %d Selected %s Key 0x%X Exit %d Value %.1f",
          uxTaskGetStackHighWaterMark(NULL), item, is_selected ? "Yes" : "No", key, packet.esc,
          module_alarms_triggers[item]);
    for (uint i = 0; i < size + 1; i++) {
        debug("\n%.21s", packet.text[i]);
    }
    send_packet((uint8_t *)&packet, sizeof(packet));
}

static void format_binary_packet(triggers_t *alarms, hott_sensors_t *sensors, uint8_t address) {
    // packet in little endian
    switch (address) {
        case HOTT_VARIO_MODULE_ID: {
            static uint16_t max_altitude = 0, min_altitude = 0xFFFF;
            if (!sensors->is_enabled[HOTT_TYPE_VARIO]) return;
            hott_sensor_vario_t packet = {0};
            packet.startByte = HOTT_START_BYTE;
            packet.sensorID = HOTT_VARIO_MODULE_ID;
            packet.sensorTextID = HOTT_VARIO_TEXT_ID;
            if (*sensors->vario[HOTT_VARIO_ALTITUDE] < alarms->triggers->vario[TRIGGER_VARIO_MIN_ALTITUDE]) {
                packet.warningID = ALARM_VOICE_MIN_ALTITUDE;
                packet.alarmInverse |= 1 << ALARM_BITMASK_VARIO_ALTITUDE;
                packet.alarmInverse |= 1 << ALARM_BITMASK_VARIO_MIN_ALTITUDE;
            }
            if (*sensors->vario[HOTT_VARIO_ALTITUDE] > alarms->triggers->vario[TRIGGER_VARIO_MAX_ALTITUDE]) {
                packet.warningID = ALARM_VOICE_MAX_ALTITUDE;
                packet.alarmInverse |= 1 << ALARM_BITMASK_VARIO_ALTITUDE;
                packet.alarmInverse |= 1 << ALARM_BITMASK_VARIO_MAX_ALTITUDE;
            }
            if (*sensors->vario[HOTT_VARIO_M1S] < alarms->triggers->vario[TRIGGER_VARIO_VSPD]) {
                packet.alarmInverse |= 1 << ALARM_BITMASK_VARIO_M1S;
            }
            packet.altitude = *sensors->vario[HOTT_VARIO_ALTITUDE] + 500;
            if (max_altitude < packet.altitude) max_altitude = packet.altitude;
            if (min_altitude > packet.altitude) min_altitude = packet.altitude;
            packet.maxAltitude = max_altitude;
            packet.minAltitude = min_altitude;
            packet.m1s = *sensors->vario[HOTT_VARIO_M1S] * 100 + 30000;
            packet.m3s = vario_alarm_parameters.m3s;
            packet.m10s = vario_alarm_parameters.m10s;
            packet.endByte = HOTT_END_BYTE;
            packet.checksum = get_crc((uint8_t *)&packet, sizeof(packet) - 1);
            send_packet((uint8_t *)&packet, sizeof(packet));
            break;
        }
        case HOTT_ESC_MODULE_ID: {
            if (!sensors->is_enabled[HOTT_TYPE_ESC]) return;
            hott_sensor_airesc_t packet = {0};
            static uint16_t minInputVolt = 0xFFFF;
            static uint8_t maxEscTemperature = 0;
            static uint16_t maxCurrent = 0;
            static uint16_t maxRPM = 0;
            static uint16_t maxSpeed = 0;
            static uint16_t minBECCurrent = 0xFFFF;
            static uint16_t maxBECCurrent = 0;
            static uint16_t minBECVoltage = 0xFFFF;
            static uint16_t maxBECTemperature = 0;
            static uint16_t maxMotorOrExtTemperature = 0;
            packet.startByte = HOTT_START_BYTE;
            packet.sensorID = HOTT_ESC_MODULE_ID;
            packet.sensorTextID = HOTT_ESC_TEXT_ID;
            if (sensors->esc[HOTT_ESC_VOLTAGE]) {
                packet.inputVolt = *sensors->esc[HOTT_ESC_VOLTAGE] * 10;
                if (packet.inputVolt < minInputVolt) packet.minInputVolt = packet.inputVolt;
                if (*sensors->esc[HOTT_ESC_VOLTAGE] < alarms->triggers->esc[TRIGGER_ESC_VOLTAGE]) {
                    packet.warningID = ALARM_VOICE_MIN_POWER_VOLTAGE;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_MIN_VOLTAGE;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_VOLTAGE;
                }
            }
            if (sensors->esc[HOTT_ESC_CONSUMPTION]) {
                packet.capacity = *sensors->esc[HOTT_ESC_CONSUMPTION] / 10;
                if (*sensors->esc[HOTT_ESC_CONSUMPTION] > alarms->triggers->esc[TRIGGER_ESC_CONSUMPTION]) {
                    packet.warningID = ALARM_VOICE_MAX_CAPACITY;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_CAPACITY;
                }
            }
            if (sensors->esc[HOTT_ESC_TEMPERATURE]) {
                packet.escTemperature = *sensors->esc[HOTT_ESC_TEMPERATURE] + 20;
                if (packet.escTemperature > maxEscTemperature) packet.maxEscTemperature = packet.escTemperature;
                if (*sensors->esc[HOTT_ESC_TEMPERATURE] > alarms->triggers->esc[TRIGGER_ESC_TEMPERATURE]) {
                    packet.warningID = ALARM_VOICE_MAX_SENSOR_1_TEMP;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_TEMPERATURE;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_MAX_TEMPERATURE;
                }
            }
            if (sensors->esc[HOTT_ESC_CURRENT]) {
                packet.current = *sensors->esc[HOTT_ESC_CURRENT] * 10;
                if (packet.current > maxCurrent) packet.maxCurrent = packet.current;
                if (*sensors->esc[HOTT_ESC_CURRENT] > alarms->triggers->esc[TRIGGER_ESC_CURRENT]) {
                    packet.warningID = ALARM_VOICE_MAX_CURRENT;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_MAX_CURRENT;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_CURRENT;
                }
            }
            if (sensors->esc[HOTT_ESC_RPM]) {
                packet.RPM = *sensors->esc[HOTT_ESC_RPM] / 10;
                if (packet.RPM > maxRPM) packet.maxRPM = packet.RPM;
                if (*sensors->esc[HOTT_ESC_RPM] < alarms->triggers->esc[TRIGGER_ESC_MIN_RPM]) {
                    packet.warningID = ALARM_VOICE_MIN_RPM;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_RPM;
                }
                if (*sensors->esc[HOTT_ESC_RPM] > alarms->triggers->esc[TRIGGER_ESC_MAX_RPM]) {
                    packet.warningID = ALARM_VOICE_MAX_RPM;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_RPM;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_AIRESC_MAX_RPM;
                }
            }
            // uint8_t throttlePercent;            // Byte 22
            if (sensors->esc[HOTT_ESC_SPEED]) {
                packet.speed = *sensors->esc[HOTT_ESC_SPEED];
                if (packet.speed > maxSpeed) packet.maxSpeed = packet.speed;
            }
            if (sensors->esc[HOTT_ESC_BEC_VOLTAGE]) {
                packet.BECVoltage = *sensors->esc[HOTT_ESC_BEC_VOLTAGE] * 10;
                if (packet.minBECVoltage < minBECVoltage) packet.minBECVoltage = packet.BECVoltage;
            }
            if (sensors->esc[HOTT_ESC_BEC_CURRENT]) {
                packet.BECCurrent = *sensors->esc[HOTT_ESC_BEC_CURRENT] * 10;
                if (packet.BECCurrent < minBECCurrent) packet.minBECCurrent = packet.BECCurrent;
                if (packet.BECCurrent > maxBECCurrent) packet.maxBECCurrent = packet.BECCurrent;
            }
            // uint8_t PWM;                        // Byte 32
            if (sensors->esc[HOTT_ESC_BEC_TEMPERATURE]) {
                packet.BECTemperature = *sensors->esc[HOTT_ESC_BEC_TEMPERATURE] + 20;
                if (packet.BECTemperature > maxBECTemperature) packet.maxBECTemperature = packet.BECTemperature;
            }
            if (sensors->esc[HOTT_ESC_EXT_TEMPERATURE]) {
                packet.motorOrExtTemperature = *sensors->esc[HOTT_ESC_EXT_TEMPERATURE] + 20;
                if (packet.motorOrExtTemperature > maxMotorOrExtTemperature)
                    packet.maxMotorOrExtTemperature = packet.motorOrExtTemperature;
            }
            // uint16_t RPMWithoutGearOrExt;       // Byte 37
            // uint8_t timing;                     // Byte 39
            // uint8_t advancedTiming;             // Byte 40
            // uint8_t highestCurrentMotorNumber;  // Byte 41
            packet.endByte = HOTT_END_BYTE;
            packet.checksum = get_crc((uint8_t *)&packet, sizeof(packet) - 1);
            send_packet((uint8_t *)&packet, sizeof(packet));
            break;
        }
        case HOTT_GENERAL_AIR_MODULE_ID: {
            if (!sensors->is_enabled[HOTT_TYPE_GENERAL]) return;
            hott_sensor_general_air_t packet = {0};
            packet.startByte = HOTT_START_BYTE;
            packet.sensorID = HOTT_GENERAL_AIR_MODULE_ID;
            packet.sensorTextID = HOTT_GENERAL_AIR_TEXT_ID;
            if (sensors->general_air[HOTT_GENERAL_BATTERY_1]) {
                packet.battery1 = *sensors->general_air[HOTT_GENERAL_BATTERY_1] * 10;
                if (*sensors->general_air[HOTT_GENERAL_BATTERY_1] <
                    alarms->triggers->general[TRIGGER_GENERAL_BATTERY]) {
                    packet.warningID = ALARM_VOICE_MIN_POWER_VOLTAGE;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_GENERAL_AIR_BATTERY_1;
                }
            }
            if (sensors->general_air[HOTT_GENERAL_CURRENT]) {
                packet.current = *sensors->general_air[HOTT_GENERAL_CURRENT] * 10;
                if (*sensors->general_air[HOTT_GENERAL_CURRENT] < alarms->triggers->general[TRIGGER_GENERAL_CURRENT]) {
                    packet.warningID = ALARM_VOICE_MAX_CURRENT;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_GENERAL_AIR_CURRENT;
                }
            }
            if (sensors->general_air[HOTT_GENERAL_CAPACITY]) {
                packet.batt_cap = *sensors->general_air[HOTT_GENERAL_CAPACITY] / 10;
                if (*sensors->general_air[HOTT_GENERAL_CAPACITY] >
                    alarms->triggers->general[TRIGGER_GENERAL_CAPACITY]) {
                    packet.warningID = ALARM_VOICE_MAX_CAPACITY;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_GENERAL_AIR_CAPACITY;
                }
            }
            if (sensors->general_air[HOTT_GENERAL_PRESSURE]) {
                packet.pressure =
                    *sensors->general_air[HOTT_GENERAL_PRESSURE] * 1e-5 * 10;  // Pa -> bar (in steps of 0.1 bar)
            }
            if (sensors->general_air[HOTT_GENERAL_ALTITUDE]) {
                if (*sensors->vario[HOTT_GENERAL_ALTITUDE] < alarms->triggers->vario[TRIGGER_GENERAL_MIN_ALTITUDE]) {
                    packet.warningID = ALARM_VOICE_MIN_ALTITUDE;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_GENERAL_AIR_ALTITUDE;
                }
                if (*sensors->vario[HOTT_GENERAL_ALTITUDE] > alarms->triggers->vario[TRIGGER_GENERAL_MAX_ALTITUDE]) {
                    packet.warningID = ALARM_VOICE_MAX_ALTITUDE;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_GENERAL_AIR_ALTITUDE;
                }
                packet.altitude = *sensors->general_air[HOTT_GENERAL_ALTITUDE] + 500;
            }
            if (sensors->general_air[HOTT_GENERAL_CLIMBRATE]) {
                packet.climbrate = *sensors->general_air[HOTT_GENERAL_CLIMBRATE] * 100 + 30000;
            }
            if (sensors->general_air[HOTT_GENERAL_CELL_1]) {
                packet.cell[0] = *sensors->general_air[HOTT_GENERAL_CELL_1] * 50;
            }
            if (sensors->general_air[HOTT_GENERAL_CELL_2]) {
                packet.cell[1] = *sensors->general_air[HOTT_GENERAL_CELL_2] * 50;
            }
            if (sensors->general_air[HOTT_GENERAL_CELL_3]) {
                packet.cell[2] = *sensors->general_air[HOTT_GENERAL_CELL_3] * 50;
            }
            if (sensors->general_air[HOTT_GENERAL_CELL_4]) {
                packet.cell[3] = *sensors->general_air[HOTT_GENERAL_CELL_4] * 50;
            }
            if (sensors->general_air[HOTT_GENERAL_CELL_5]) {
                packet.cell[4] = *sensors->general_air[HOTT_GENERAL_CELL_5] * 50;
            }
            if (sensors->general_air[HOTT_GENERAL_CELL_6]) {
                packet.cell[5] = *sensors->general_air[HOTT_GENERAL_CELL_6] * 50;
            }
            if (sensors->general_air[HOTT_GENERAL_TEMP_1]) {
                packet.temperature1 = *sensors->general_air[HOTT_GENERAL_TEMP_1] + 20;
                if (*sensors->general_air[HOTT_GENERAL_TEMP_1] >
                    alarms->triggers->general[TRIGGER_GENERAL_TEMPERATURE]) {
                    packet.warningID = ALARM_VOICE_MAX_SENSOR_1_TEMP;
                    packet.alarmInverse |= 1 << ALARM_BITMASK_GENERAL_AIR_TEMPERATURE_1;
                }
            }
            packet.endByte = HOTT_END_BYTE;
            packet.checksum = get_crc((uint8_t *)&packet, sizeof(packet) - 1);
            send_packet((uint8_t *)&packet, sizeof(packet));
            break;
        }
        case HOTT_GPS_MODULE_ID: {
            if (!sensors->is_enabled[HOTT_TYPE_GPS]) return;
            hott_sensor_gps_t packet = {0};
            if (*sensors->vario[HOTT_GPS_SPEED] < alarms->triggers->vario[TRIGGER_GPS_MIN_SPEED]) {
                packet.alarmInverse |= 1 << ALARM_BITMASK_GPS_SPEED;
            }
            if (*sensors->vario[HOTT_GPS_SPEED] > alarms->triggers->vario[TRIGGER_GPS_MAX_SPEED]) {
                packet.alarmInverse |= 1 << ALARM_BITMASK_GPS_SPEED;
            }
            if (*sensors->vario[HOTT_GPS_ALTITUDE] < alarms->triggers->vario[TRIGGER_GPS_MIN_ALTITUDE]) {
                packet.warningID = ALARM_VOICE_MIN_ALTITUDE;
                packet.alarmInverse |= 1 << ALARM_BITMASK_GPS_ALTITUDE;
            }
            if (*sensors->vario[HOTT_GPS_ALTITUDE] > alarms->triggers->vario[TRIGGER_GPS_MAX_ALTITUDE]) {
                packet.warningID = ALARM_VOICE_MAX_ALTITUDE;
                packet.alarmInverse |= 1 << ALARM_BITMASK_GPS_ALTITUDE;
            }
            if (*sensors->vario[HOTT_GPS_CLIMBRATE] < alarms->triggers->vario[TRIGGER_GPS_MAX_CLIMB]) {
                packet.alarmInverse |= 1 << ALARM_BITMASK_GPS_CLIMBRATE;
            }
            if (*sensors->vario[HOTT_GPS_SATS] < alarms->triggers->vario[TRIGGER_GPS_MIN_SATS]) {
                packet.alarmInverse |= 1 << ALARM_BITMASK_GPS_SATS;
            }
            packet.startByte = HOTT_START_BYTE;
            packet.sensorID = HOTT_GPS_MODULE_ID;
            packet.sensorTextID = HOTT_GPS_TEXT_ID;
            packet.flightDirection = *sensors->gps[HOTT_GPS_DIRECTION] / 2;  // 0.5°
            packet.GPSSpeed = *sensors->gps[HOTT_GPS_SPEED];                 // km/h
            packet.LatitudeNS = sensors->gps[HOTT_GPS_LATITUDE] > 0 ? 0 : 1;
            float latitude = fabs(*sensors->gps[HOTT_GPS_LATITUDE]);
            packet.LatitudeDegMin = (uint)latitude * 100 + (latitude - (uint)latitude) * 60;
            packet.LatitudeSec = (latitude * 60 - (uint)(latitude * 60)) * 60;
            packet.longitudeEW = sensors->gps[HOTT_GPS_LATITUDE] > 0 ? 0 : 1;
            float longitude = fabs(*sensors->gps[HOTT_GPS_LONGITUDE]);
            packet.longitudeDegMin = (uint)longitude * 100 + (longitude - (uint)longitude) * 60;
            packet.longitudeSec = (longitude * 60 - (uint)(longitude * 60)) * 60;
            packet.distance = *sensors->gps[HOTT_GPS_DISTANCE];
            packet.altitude = *sensors->gps[HOTT_GPS_ALTITUDE] + 500;
            float climbrate = *sensors->gps[HOTT_GPS_CLIMBRATE] * 100 + 30000;
            if (climbrate < 0) climbrate = 0;
            packet.climbrate = climbrate;  // 30000, 0.00
            // packet.climbrate3s = *sensors->gps[HOTT_GPS_ALTITUDE];  // 120, 0
            packet.GPSNumSat = *sensors->gps[HOTT_GPS_SATS];
            packet.GPSFixChar = *sensors->gps[HOTT_GPS_FIX];  // (GPS fix character. display, if DGPS, 2D oder 3D)
            // (1 byte) uint8_t homeDirection;   // Byte 29: HomeDirection (direction from starting point to Model
            // position) (1 byte)
            uint hour = (uint)(*sensors->gps[HOTT_GPS_TIME]) / 10000;
            uint min = (uint)(*sensors->gps[HOTT_GPS_TIME]) / 100 - hour * 100;
            uint sec = (uint)(*sensors->gps[HOTT_GPS_TIME]) - hour * 10000 - sec * 100;
            packet.gps_time_h = hour;
            packet.gps_time_m = min;
            packet.gps_time_s = sec;
            // uint8_t gps_time_sss;//#36 UTC time milliseconds
            packet.msl_altitude = *sensors->gps[HOTT_GPS_ALTITUDE] + 500;
            // uint8_t vibration; // Byte 39 vibrations level in %
            // uint8_t Ascii4;    // Byte 40: 00 ASCII Free Character [4] appears right to home distance
            // uint8_t Ascii5;    // Byte 41: 00 ASCII Free Character [5] appears right to home direction
            // uint8_t GPS_fix;   // Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX
            // uint8_t version;   // Byte 43: 00 version number

            packet.endByte = HOTT_END_BYTE;
            packet.checksum = get_crc((uint8_t *)&packet, sizeof(packet) - 1);
            send_packet((uint8_t *)&packet, sizeof(packet));
            break;
        }
    }
}

static void send_packet(uint8_t *buffer, uint len) {
    for (uint i = 0; i < len; i++) {
        uart0_write(*(buffer + i));
        sleep_us(HOTT_INTERBYTE_DELAY_US);
    }
    vTaskResume(context.led_task_handle);
    if (len < 100) {
        debug("\nHOTT (%u) %u > ", uxTaskGetStackHighWaterMark(NULL), len);
        debug_buffer(buffer, len, "0x%X ");
    }
}

static triggers_value_t *alarms_read(void) {
    triggers_value_t *alarms = (triggers_value_t *)(XIP_BASE + ALARMS_FLASH_TARGET_OFFSET);
    return (triggers_value_t *)(XIP_BASE + ALARMS_FLASH_TARGET_OFFSET);
}

static void module_alarms_save(triggers_value_t *triggers_value) {
    uint8_t flash[FLASH_PAGE_SIZE] = {0};
    memcpy(flash, (uint8_t *)triggers_value, sizeof(triggers_value_t));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(ALARMS_FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(ALARMS_FLASH_TARGET_OFFSET, flash, sizeof(triggers_value_t));
    restore_interrupts(ints);
    debug("\nHOTT Alarms saved");
}

static uint8_t get_crc(const uint8_t *buffer, uint len) {
    uint16_t crc = 0;
    for (uint i = 0; i < len; i++) {
        crc += buffer[i];
    }
    // debug("\n>CRC: 0x%X", crc & 0xFF);
    return crc;
}

static void set_config(hott_sensors_t *sensors) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
    }
    if (config->esc_protocol == ESC_HW4) {
        esc_hw4_parameters_t parameter = {config->rpm_multiplier,
                                          config->enable_pwm_out,
                                          config->enable_esc_hw4_init_delay,
                                          config->alpha_rpm,
                                          config->alpha_voltage,
                                          config->alpha_current,
                                          config->alpha_temperature,
                                          config->esc_hw4_voltage_multiplier,
                                          config->esc_hw4_current_multiplier,
                                          config->esc_hw4_current_thresold,
                                          config->esc_hw4_current_max,
                                          config->esc_hw4_is_manual_offset,
                                          config->esc_hw4_auto_detect,
                                          config->esc_hw4_offset,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw4_task, "esc_hw4_task", STACK_ESC_HW4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (config->enable_pwm_out) {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            context.pwm_out_task_handle = task_handle;
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temperature_fet;
        sensors->esc[HOTT_ESC_BEC_TEMPERATURE] = parameter.temperature_bec;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
    }
    if (config->esc_protocol == ESC_HW5) {
        esc_hw5_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw5_task, "esc_hw5_task", STACK_ESC_HW5, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temperature_fet;
        sensors->esc[HOTT_ESC_BEC_TEMPERATURE] = parameter.temperature_bec;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_BEC_VOLTAGE] = parameter.voltage_bec;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_BEC_CURRENT] = parameter.current_bec;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
        sensors->esc[HOTT_ESC_EXT_TEMPERATURE] = parameter.temperature_motor;
    }
    if (config->esc_protocol == ESC_CASTLE) {
        esc_castle_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                             config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temperature;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_BEC_VOLTAGE] = parameter.voltage_bec;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_BEC_CURRENT] = parameter.current_bec;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
        sensors->esc[HOTT_ESC_EXT_TEMPERATURE] = parameter.consumption;
    }
    if (config->esc_protocol == ESC_KONTRONIK) {
        esc_kontronik_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage,  config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_kontronik_task, "esc_kontronik_task", STACK_ESC_KONTRONIK, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temperature_fet;
        sensors->esc[HOTT_ESC_BEC_TEMPERATURE] = parameter.temperature_bec;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_BEC_VOLTAGE] = parameter.voltage_bec;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_BEC_CURRENT] = parameter.current_bec;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
    }
    if (config->esc_protocol == ESC_APD_F) {
        esc_apd_f_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                            config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_f_task, "esc_apd_f_task", STACK_ESC_APD_F, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temperature;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
    }
    if (config->esc_protocol == ESC_APD_HV) {
        esc_apd_hv_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_hv_task, "esc_apd_hv_task", STACK_ESC_APD_HV, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temperature;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
    }
    if (config->esc_protocol == ESC_SMART) {
        smart_esc_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temperature_fet = malloc(sizeof(float));
        parameter.temperature_bec = malloc(sizeof(float));
        parameter.voltage_bec = malloc(sizeof(float));
        parameter.current_bec = malloc(sizeof(float));
        parameter.temperature_bat = malloc(sizeof(float));
        parameter.current_bat = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        for (uint i = 0; i < 18; i++) parameter.cell[i] = malloc(sizeof(float));
        parameter.cells = malloc(sizeof(uint8_t));
        parameter.cycles = malloc(sizeof(uint16_t));
        xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 4, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temperature_fet;
        sensors->esc[HOTT_ESC_BEC_TEMPERATURE] = parameter.temperature_bec;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_BEC_VOLTAGE] = parameter.voltage_bec;
        sensors->esc[HOTT_ESC_BEC_CURRENT] = parameter.current_bec;

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_TEMP_1] = parameter.temperature_bat;
        sensors->general_air[HOTT_GENERAL_CURRENT] = parameter.current_bat;
        sensors->general_air[HOTT_GENERAL_CAPACITY] = parameter.consumption;
        sensors->general_air[HOTT_GENERAL_CELL_1] = parameter.cell[0];
        sensors->general_air[HOTT_GENERAL_CELL_2] = parameter.cell[1];
        sensors->general_air[HOTT_GENERAL_CELL_3] = parameter.cell[2];
        sensors->general_air[HOTT_GENERAL_CELL_4] = parameter.cell[3];
        sensors->general_air[HOTT_GENERAL_CELL_5] = parameter.cell[4];
        sensors->general_air[HOTT_GENERAL_CELL_6] = parameter.cell[5];
    }
    if (config->esc_protocol == ESC_OMP_M4) {
        esc_omp_m4_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temp_esc = malloc(sizeof(float));
        parameter.temp_motor = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_omp_m4_task, "esc_omp_m4_task", STACK_SMART_ESC, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temp_esc;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
        sensors->esc[HOTT_ESC_EXT_TEMPERATURE] = parameter.temp_motor;
    }
    if (config->esc_protocol == ESC_ZTW) {
        esc_ztw_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temp_esc = malloc(sizeof(float));
        parameter.temp_motor = malloc(sizeof(float));
        parameter.bec_voltage = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_omp_m4_task, "esc_ztw_task", STACK_ESC_ZTW, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_RPM] = parameter.rpm;
        sensors->esc[HOTT_ESC_TEMPERATURE] = parameter.temp_esc;
        sensors->esc[HOTT_ESC_VOLTAGE] = parameter.voltage;
        sensors->esc[HOTT_ESC_CURRENT] = parameter.current;
        sensors->esc[HOTT_ESC_CONSUMPTION] = parameter.consumption;
        sensors->esc[HOTT_ESC_EXT_TEMPERATURE] = parameter.temp_motor;
    }
    if (config->enable_gps) {
        gps_parameters_t parameter;
        parameter.protocol = config->gps_protocol;
        parameter.baudrate = config->gps_baudrate;
        parameter.rate = config->gps_rate;
        parameter.lat = malloc(sizeof(double));
        parameter.lon = malloc(sizeof(double));
        parameter.alt = malloc(sizeof(float));
        parameter.spd = malloc(sizeof(float));
        parameter.cog = malloc(sizeof(float));
        parameter.hdop = malloc(sizeof(float));
        parameter.sat = malloc(sizeof(float));
        parameter.time = malloc(sizeof(float));
        parameter.date = malloc(sizeof(float));
        parameter.vspeed = malloc(sizeof(float));
        parameter.dist = malloc(sizeof(float));
        parameter.spd_kmh = malloc(sizeof(float));
        parameter.fix = malloc(sizeof(float));
        parameter.vdop = malloc(sizeof(float));
        parameter.speed_acc = malloc(sizeof(float));
        parameter.h_acc = malloc(sizeof(float));
        parameter.v_acc = malloc(sizeof(float));
        parameter.track_acc = malloc(sizeof(float));
        parameter.n_vel = malloc(sizeof(float));
        parameter.e_vel = malloc(sizeof(float));
        parameter.v_vel = malloc(sizeof(float));
        parameter.alt_elipsiod = malloc(sizeof(float));
        parameter.pdop = malloc(sizeof(float));
        xTaskCreate(gps_task, "gps_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_GPS] = true;
        sensors->gps[HOTT_GPS_LATITUDE] = parameter.lat;
        sensors->gps[HOTT_GPS_LONGITUDE] = parameter.lon;
        sensors->gps[HOTT_GPS_SATS] = parameter.sat;
        sensors->gps[HOTT_GPS_FIX] = parameter.fix;
        sensors->gps[HOTT_GPS_ALTITUDE] = parameter.alt;
        sensors->gps[HOTT_GPS_SPEED] = parameter.spd_kmh;
        sensors->gps[HOTT_GPS_DIRECTION] = parameter.cog;
        sensors->gps[HOTT_GPS_DISTANCE] = parameter.dist;
        sensors->gps[HOTT_GPS_CLIMBRATE] = parameter.vspeed;
        sensors->gps[HOTT_GPS_TIME] = parameter.time;
        sensors->gps[HOTT_GPS_TIME] = parameter.time;
        sensors->gps[HOTT_GPS_TIME] = parameter.time;
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_BATTERY_1] = parameter.voltage;
    }
    if (config->enable_analog_current) {
        current_parameters_t parameter = {1,
                                          config->analog_rate,
                                          config->alpha_current,
                                          config->analog_current_multiplier,
                                          config->analog_current_offset,
                                          config->analog_current_autoffset,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_CURRENT] = parameter.current;
        sensors->general_air[HOTT_GENERAL_CAPACITY] = parameter.consumption;
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_TEMP_1] = parameter.ntc;
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3,
                                           config->analog_rate,
                                           config->alpha_airspeed,
                                           (float)config->airspeed_offset / 1000,
                                           (float)config->airspeed_vcc / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_ESC] = true;
        sensors->esc[HOTT_ESC_SPEED] = parameter.airspeed;
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensors->is_enabled[HOTT_TYPE_VARIO] = true;
        sensors->vario[HOTT_VARIO_ALTITUDE] = parameter.altitude;
        sensors->vario[HOTT_VARIO_M1S] = parameter.vspeed;

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_ALTITUDE] = parameter.altitude;
        sensors->general_air[HOTT_GENERAL_CLIMBRATE] = parameter.vspeed;

        vario_alarm_parameters.altitude = parameter.altitude;

        if (config->enable_gps) {
            sensors->gps[HOTT_GPS_ALTITUDE] = parameter.altitude;
            sensors->gps[HOTT_GPS_CLIMBRATE] = parameter.vspeed;
        }

        add_alarm_in_ms(1000, interval_1000_callback, &vario_alarm_parameters, false);
        add_alarm_in_ms(3000, interval_3000_callback, &vario_alarm_parameters, false);
        add_alarm_in_ms(10000, interval_10000_callback, &vario_alarm_parameters, false);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensors->is_enabled[HOTT_TYPE_VARIO] = true;
        sensors->vario[HOTT_VARIO_ALTITUDE] = parameter.altitude;
        sensors->vario[HOTT_VARIO_M1S] = parameter.vspeed;

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_ALTITUDE] = parameter.altitude;
        sensors->general_air[HOTT_GENERAL_CLIMBRATE] = parameter.vspeed;

        vario_alarm_parameters.altitude = parameter.altitude;

        if (config->enable_gps) {
            sensors->gps[HOTT_GPS_ALTITUDE] = parameter.altitude;
            sensors->gps[HOTT_GPS_CLIMBRATE] = parameter.vspeed;
        }

        add_alarm_in_ms(1000, interval_1000_callback, &vario_alarm_parameters, false);
        add_alarm_in_ms(3000, interval_3000_callback, &vario_alarm_parameters, false);
        add_alarm_in_ms(10000, interval_10000_callback, &vario_alarm_parameters, false);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensors->is_enabled[HOTT_TYPE_VARIO] = true;
        sensors->vario[HOTT_VARIO_ALTITUDE] = parameter.altitude;
        sensors->vario[HOTT_VARIO_M1S] = parameter.vspeed;

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_ALTITUDE] = parameter.altitude;
        sensors->general_air[HOTT_GENERAL_CLIMBRATE] = parameter.vspeed;

        vario_alarm_parameters.altitude = parameter.altitude;

        if (config->enable_gps) {
            sensors->gps[HOTT_GPS_ALTITUDE] = parameter.altitude;
            sensors->gps[HOTT_GPS_CLIMBRATE] = parameter.vspeed;
        }

        add_alarm_in_ms(1000, interval_1000_callback, &vario_alarm_parameters, false);
        add_alarm_in_ms(3000, interval_3000_callback, &vario_alarm_parameters, false);
        add_alarm_in_ms(10000, interval_10000_callback, &vario_alarm_parameters, false);
    }
    if (config->enable_fuel_flow) {
        fuel_meter_parameters_t parameter = {config->fuel_flow_ml_per_pulse, malloc(sizeof(float)),
                                             malloc(sizeof(float))};
        xTaskCreate(fuel_meter_task, "fuel_meter_task", STACK_FUEL_METER, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_FUEL] = parameter.consumption_total;
    }
    if (config->enable_fuel_pressure) {
        xgzp68xxd_parameters_t parameter = {config->xgzp68xxd_k, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(xgzp68xxd_task, "fuel_pressure_task", STACK_FUEL_PRESSURE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        sensors->general_air[HOTT_GENERAL_PRESSURE] = parameter.pressure;
    }
    if (config->enable_lipo) {
        sensors->is_enabled[HOTT_TYPE_GENERAL] = true;
        if (config->lipo_cells > 0) {
            ina3221_parameters_t parameter = {
                .i2c_address = 0x40,
                .filter = config->ina3221_filter,
                .cell_count = config->lipo_cells,
                .cell[0] = malloc(sizeof(float)),
                .cell[1] = malloc(sizeof(float)),
                .cell[2] = malloc(sizeof(float)),
            };
            xTaskCreate(ina3221_task, "ina3221_1_task", STACK_INA3221, (void *)&parameter, 2, &task_handle);
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            sensors->general_air[HOTT_GENERAL_CELL_1] = parameter.cell[0];
            sensors->general_air[HOTT_GENERAL_CELL_2] = parameter.cell[1];
            sensors->general_air[HOTT_GENERAL_CELL_3] = parameter.cell[2];
        }
        if (config->lipo_cells > 3) {
            ina3221_parameters_t parameter = {
                .i2c_address = 0x41,
                .filter = config->ina3221_filter,
                .cell_count = config->lipo_cells - 3,
                .cell[0] = malloc(sizeof(float)),
                .cell[1] = malloc(sizeof(float)),
                .cell[2] = malloc(sizeof(float)),
            };
            xTaskCreate(ina3221_task, "ina3221_2_task", STACK_INA3221, (void *)&parameter, 2, &task_handle);
            xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            sensors->general_air[HOTT_GENERAL_CELL_4] = parameter.cell[0];
            sensors->general_air[HOTT_GENERAL_CELL_5] = parameter.cell[1];
            sensors->general_air[HOTT_GENERAL_CELL_6] = parameter.cell[2];
        }
    }
}

static int64_t interval_1000_callback(alarm_id_t id, void *parameters) {
    vario_alarm_parameters_t *parameter = (vario_alarm_parameters_t *)parameters;
    static float prev = 0;
    parameter->m1s = (*parameter->altitude - prev) * 100 + 30000;
#ifdef SIM_SENSORS
    vario_alarm_parameters.m1s = 12 * 100 + 30000;
#endif
    prev = *parameter->altitude;
    return 1000000L;
}

static int64_t interval_3000_callback(alarm_id_t id, void *parameters) {
    vario_alarm_parameters_t *parameter = (vario_alarm_parameters_t *)parameters;
    static float prev = 0;
    parameter->m3s = (*parameter->altitude - prev) * 100 + 30000;
    *parameter->vspd = parameter->m3s / 3.0F;
#ifdef SIM_SENSORS
    vario_alarm_parameters.m3s = 34 * 100 + 30000;
#endif
    prev = *parameter->altitude;
    return 3000000L;
}

static int64_t interval_10000_callback(alarm_id_t id, void *parameters) {
    vario_alarm_parameters_t *parameter = (vario_alarm_parameters_t *)parameters;
    static float prev = 0;
    parameter->m10s = (*parameter->altitude - prev) * 100 + 30000;
#ifdef SIM_SENSORS
    vario_alarm_parameters.m10s = 56 * 100 + 30000;
#endif
    prev = *parameter->altitude;
    return 10000000L;
}
