#include "hott.h"

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
#include "esc_pwm.h"
#include "ibus.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pwm_out.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"

#define swap_16(value) (((value & 0xFF) << 8) | (value & 0xFF00) >> 8)

#define HOTT_ELECTRIC_AIR_MODULE_ID 0x8E
#define HOTT_ELECTRIC_AIR_SENSOR_ID 0xE0
#define HOTT_TEXT_MODE_REQUEST_ELECTRIC_AIR 0xEF
#define HOTT_GENERAL_AIR_MODULE_ID 0x8D
#define HOTT_GENERAL_AIR_SENSOR_ID 0xD0
#define HOTT_TEXT_MODE_REQUEST_GENERAL_AIR 0xDF
#define HOTT_GPS_MODULE_ID 0x8A
#define HOTT_GPS_SENSOR_ID 0xA0
#define HOTT_TEXT_MODE_REQUEST_GPS 0xAF
#define HOTT_VARIO_MODULE_ID 0x89
#define HOTT_VARIO_SENSOR_ID 0x90

#define HOTT_BINARY_MODE_REQUEST_ID 0x80
#define HOTT_TEXT_MODE_REQUEST_ID 0x7F

#define HOTT_TIMEOUT_US 1000
#define HOTT_PACKET_LENGHT 2

typedef struct hott_sensor_vario_t {  // ko
    uint8_t startByte;                //  0 1
    uint8_t sensorID;                 //  1 2
    uint8_t warningId;                //  2 3
    uint8_t sensorTextID;             //  3 4
    uint8_t alarmInverse;             //  4 5
    int16_t altitude;                 //  6 7 5??
    int16_t maxAltitude;
    int16_t minAltitude;
    uint16_t m1s;
    uint16_t m3s;
    uint16_t m10s;
    uint8_t text[24];
    uint8_t empty;
    uint8_t version;
    uint8_t endByte;
    uint8_t checksum;
} __attribute__((packed)) hott_sensor_vario_t;

typedef struct hott_sensor_airesc_t {   // ko
    uint8_t startByte;                  // 1
    uint8_t sensorID;                   // 2
    uint8_t sensorTextID;               // Byte 3
    uint8_t Inverse;                    // Byte 4
    uint8_t InverseStatusI;             // Byte 5
    uint16_t InputVolt;                 // Byte 6
    uint16_t MinInputVolt;              // Byte 8
    uint16_t Capacity;                  // Byte 10 10
    uint8_t EscTemperature;             // Byte 12
    uint8_t MaxEscTemperature;          // Byte 13
    uint16_t Current;                   // Byte 14
    uint16_t MaxCurrent;                // Byte 16
    uint16_t RPM;                       // Byte 18
    uint16_t MaxRPM;                    // Byte 20
    uint8_t ThrottlePercent;            // Byte 22
    uint16_t Speed;                     // Byte 23
    uint16_t MaxSpeed;                  // Byte 25
    uint8_t BECVoltage;                 // Byte 27
    uint8_t MinBECVoltage;              // Byte 28
    uint8_t BECCurrent;                 // Byte 29
    uint8_t MinBECCurrent;              // Byte 30
    uint8_t MaxBECCurrent;              // Byte 31
    uint8_t PWM;                        // Byte 32
    uint8_t BECTemperature;             // Byte 33
    uint8_t MaxBECTemperature;          // Byte 34
    uint8_t MotorOrExtTemperature;      // Byte 35
    uint8_t MaxMotorOrExtTemperature;   // Byte 36
    uint16_t RPMWithoutGearOrExt;       // Byte 37
    uint8_t Timing;                     // Byte 39
    uint8_t AdvancedTiming;             // Byte 40
    uint8_t HighestCurrentMotorNumber;  // Byte 41
    uint8_t VersionNumber;              // Byte 42
    uint8_t version;                    /* Byte 43: 00 version number */
    uint8_t endByte;                    /* Byte 44: 0x7D Ende byte */
    uint8_t checksum;                   /* Byte 45: Parity Byte */
} __attribute__((packed)) hott_sensor_airesc_t;

typedef struct hott_sensor_electric_air_t {  // ok
    uint8_t startByte;                       // 1
    uint8_t sensorID;                        // 2
    uint8_t alarmTone;                       // 3: Alarm
    uint8_t sensorTextID;                    // 4:
    uint8_t alarmInverse1;                   // 5:
    uint8_t alarmInverse2;                   // 6:
    uint8_t cell1L;                          // 7: Low Voltage Cell 1 in 0,02 V steps
    uint8_t cell2L;                          // 8: Low Voltage Cell 2 in 0,02 V steps
    uint8_t cell3L;                          // 9: Low Voltage Cell 3 in 0,02 V steps
    uint8_t cell4L;                          // 10: Low Voltage Cell 4 in 0,02 V steps
    uint8_t cell5L;                          // 11: Low Voltage Cell 5 in 0,02 V steps
    uint8_t cell6L;                          // 12: Low Voltage Cell 6 in 0,02 V steps
    uint8_t cell7L;                          // 13: Low Voltage Cell 7 in 0,02 V steps
    uint8_t cell1H;                          // 14: High Voltage Cell 1 in 0.02 V steps
    uint8_t cell2H;                          // 15
    uint8_t cell3H;                          // 16
    uint8_t cell4H;                          // 17
    uint8_t cell5H;                          // 18
    uint8_t cell6H;                          // 19
    uint8_t cell7H;                          // 20
    uint8_t battery1Low;                     // 21 Battery 1 LSB/MSB in 100mv steps; 50 == 5V
    uint8_t battery1High;                    // 22Battery 1 LSB/MSB in 100mv steps; 50 == 5V
    uint8_t battery2Low;                     // 23 Battery 2 LSB/MSB in 100mv steps; 50 == 5V
    uint8_t battery2High;                    // 24 Battery 2 LSB/MSB in 100mv steps; 50 == 5V
    uint8_t temp1;                           // 25 Temp 1; Offset of 20. 20 == 0C
    uint8_t temp2;                           // 26 Temp 2; Offset of 20. 20 == 0C
    uint16_t height;                         // 27 28 Height. Offset -500. 500 == 0
    uint16_t current;                        // 29 30 1 = 0.1A
    uint8_t driveVoltageLow;                 // 31
    uint8_t driveVoltageHigh;                // 32
    uint16_t capacity;                       // 33 34 mAh
    uint16_t m2s;                            // 35 36  /* Steigrate m2s; 0x48 == 0
    uint8_t m3s;                             // 37  /* Steigrate m3s; 0x78 == 0
    uint16_t rpm;                            // 38 39 /* RPM. 10er steps; 300 == 3000rpm
    uint8_t minutes;                         // 40
    uint8_t seconds;                         // 41
    uint8_t speed;                           // 42
    uint8_t version;                         // 43
    uint8_t endByte;                         // 44
    uint8_t checksum;                        // 45
} __attribute__((packed)) hott_sensor_electric_air_t;

typedef struct hott_sensor_general_air_t {  // ok
    uint8_t startByte;                      //#01 start byte constant value 0x7c
    uint8_t sensorID;                       //#02 EAM sensort id. constat value 0x8d=GENRAL AIR MODULE
    uint8_t alarmTone;                      //#03 1=A 2=B ... 0x1a=Z 0 = no alarm
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
    uint8_t sensorTextID;                   //#04 constant value 0xd0
    uint8_t alarmInverse1;                  //#05 alarm bitmask. Value is displayed inverted
                                            // Bit# Alarm field
                                            // 0 all cell voltage
                                            // 1 Battery 1
                                            // 2 Battery 2
                                            // 3 Temperature 1
                                            // 4 Temperature 2
                                            // 5 Fuel
                                            // 6 mAh
                                            // 7 Altitude
    uint8_t alarm_invers2;                  //#06 alarm bitmask. Value is displayed inverted
                                            // Bit# Alarm Field
                                            // 0 main power current
                                            // 1 main power voltage
                                            // 2 Altitude
                                            // 3 m/s
                                            // 4 m/3s
                                            // 5 unknown
                                            // 6 unknown
                                            // 7 "ON" sign/text msg active
    uint8_t cell[6];                        //#7 Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
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
    uint8_t parity;                //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
} __attribute__((packed)) hott_sensor_general_air_t;

typedef struct hott_sensor_gps_t {  // ok
    uint8_t startByte;              /* Byte 1: 0x7C = Start byte data */
    uint8_t sensorID;               /* Byte 2: 0x8A = GPS Sensor */
    uint8_t alarmTone;              /* Byte 3: 0…= warning beeps */
    uint8_t sensorTextID;           /* Byte 4: 160 0xA0 Sensor ID Neu! */

    uint8_t alarmInverse1; /* Byte 5: 01 inverse status */
    uint8_t alarmInverse2; /* Byte 6: 00 inverse status status 1 = kein GPS Signal */

    uint8_t
        flightDirection; /* Byte 7: 119 = Flugricht./dir. 1 = 2°; 0° (North), 9 0° (East), 180° (South), 270° (West) */
    uint8_t GPSSpeedLow;  /* Byte 8: 8 = Geschwindigkeit/GPS speed low byte 8km/h */
    uint8_t GPSSpeedHigh; /* Byte 9: 0 = Geschwindigkeit/GPS speed high byte */

    uint8_t LatitudeNS;      /* Byte 10: 000 = N = 48°39’988 */
    uint8_t LatitudeMinLow;  /* Byte 11: 231 0xE7 = 0x12E7 = 4839 */
    uint8_t LatitudeMinHigh; /* Byte 12: 018 18 = 0x12 */
    uint8_t LatitudeSecLow;  /* Byte 13: 171 220 = 0xDC = 0x03DC =0988 */
    uint8_t LatitudeSecHigh; /* Byte 14: 016 3 = 0x03 */

    uint8_t longitudeEW;      /* Byte 15: 000  = E= 9° 25’9360 */
    uint8_t longitudeMinLow;  /* Byte 16: 150 157 = 0x9D = 0x039D = 0925 */
    uint8_t longitudeMinHigh; /* Byte 17: 003 3 = 0x03 */
    uint8_t longitudeSecLow;  /* Byte 18: 056 144 = 0x90 0x2490 = 9360*/
    uint8_t longitudeSecHigh; /* Byte 19: 004 36 = 0x24 */

    uint8_t distanceLow;     /* Byte 20: 027 123 = Entfernung/distance low byte 6 = 6 m */
    uint8_t distanceHigh;    /* Byte 21: 036 35 = Entfernung/distance high byte */
    uint8_t altitudeLow;     /* Byte 22: 243 244 = Höhe/Altitude low byte 500 = 0m */
    uint8_t altitudeHigh;    /* Byte 23: 001 1 = Höhe/Altitude high byte */
    uint8_t climbrateLow;    /* Byte 24: 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) */
    uint8_t climbrateHigh;   /* Byte 25: 117 = High Byte m/s resolution 0.01m */
    uint8_t climbrate3s;     /* Byte 26: climbrate in m/3s resolution, value of 120 = 0 m/3s*/
    uint8_t GPSNumSat;       /* Byte 27: GPS.Satelites (number of satelites) (1 byte) */
    uint8_t GPSFixChar;      /* Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
    uint8_t HomeDirection;   /* Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) */
    uint8_t angleXdirection; /* Byte 30: angle x-direction (1 byte) */
    uint8_t angleYdirection; /* Byte 31: angle y-direction (1 byte) */
    uint8_t angleZdirection; /* Byte 32: angle z-direction (1 byte) */

    //  int8_t gps_time_h;  //#33 UTC time hours
    //  int8_t gps_time_m;  //#34 UTC time minutes
    //  int8_t gps_time_s;  //#35 UTC time seconds
    //  int8_t gps_time_sss;//#36 UTC time milliseconds
    //  int8_t msl_altitude_L;  //#37 mean sea level altitude
    //  int8_t msl_altitude_H;  //#38

    uint8_t gyroXLow;  /* Byte 33: gyro x low byte (2 bytes) */
    uint8_t gyroXHigh; /* Byte 34: gyro x high byte */
    uint8_t gyroYLow;  /* Byte 35: gyro y low byte (2 bytes) */
    uint8_t gyroYHigh; /* Byte 36: gyro y high byte */
    uint8_t gyroZLow;  /* Byte 37: gyro z low byte (2 bytes) */
    uint8_t gyroZHigh; /* Byte 38: gyro z high byte */

    uint8_t vibration; /* Byte 39: vibration (1 bytes) */
    uint8_t Ascii4;    /* Byte 40: 00 ASCII Free Character [4] appears right to home distance */
    uint8_t Ascii5;    /* Byte 41: 00 ASCII Free Character [5] appears right to home direction*/
    uint8_t GPS_fix;   /* Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX */
    uint8_t version;   /* Byte 43: 00 version number */
    uint8_t endByte;   /* Byte 44: 0x7D Ende byte */
    uint8_t checksum;  /* Byte 45: Parity Byte */
} __attribute__((packed)) hott_sensor_gps_t;

typedef struct hott_sensors_t {
    hott_sensor_gps_t gps;
    hott_sensor_vario_t vario;
    hott_sensor_airesc_t airesc;
    hott_sensor_electric_air_t electric_air;
} hott_sensors_t;

static hott_sensors_t sensors;

static void process(void);
static void send_packet(uint8_t address);
static uint8_t crc8(uint8_t crc, unsigned char a);
static uint8_t get_crc(uint8_t *buffer, uint8_t len);

void hott_task(void *parameters) {
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(19200, UART_RECEIVER_TX, UART_RECEIVER_RX, HOTT_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    debug("\nHOTT init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

static uint8_t get_crc(uint8_t *buffer, uint8_t len) {
    uint16_t crc = 0;
    for (uint8_t i = 0; i < len; i++) crc += buffer[i];
    return crc;
}

static void process(void) {
    if (uart0_available() == HOTT_PACKET_LENGHT) {
        uint8_t buffer[HOTT_PACKET_LENGHT];
        uart0_read_bytes(buffer, HOTT_PACKET_LENGHT);
        debug("\nHOTT (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, HOTT_PACKET_LENGHT, "0x%X ");
        if (buffer[0] == HOTT_BINARY_MODE_REQUEST_ID) send_packet(buffer[1]);
    }
}

static void send_packet(uint8_t address) {
    switch (address) {
        case HOTT_ELECTRIC_AIR_MODULE_ID: {
            uint8_t buffer[] = {0x7C, 0x8E, 0x0, 0xE0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  0x0,
                                0x0,  0x0,  0x0, 0x0,  0x0, 0x5, 0x0, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  0x0,
                                0x0,  0x0,  0x0, 0x0,  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7D, 0x72};
            for (uint i = 0; i < sizeof(buffer); i++) {
                uart0_write(buffer[i]);
                sleep_us(1000);
            }
            debug("\nHOTT (%u) %u > ", uxTaskGetStackHighWaterMark(NULL), sizeof(buffer));
            debug_buffer(buffer, sizeof(buffer), "0x%X ");

            // blink led
            vTaskResume(context.led_task_handle);
            break;
        }
    }
}