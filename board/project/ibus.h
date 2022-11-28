#ifndef IBUS_H
#define IBUS_H

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"

#include "common.h"
#include "uart.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
#include "voltage.h"
#include "current.h"
#include "ntc.h"
#include "airspeed.h"
#include "esc_kontronik.h"
#include "esc_apd_f.h"
#include "esc_apd_hv.h"
#include "esc_castle.h"
#include "nmea.h"
#include "esc_pwm.h"
#include "bmp280.h"
#include "ms5611.h"
#include "bmp180.h"
#include "pwm_out.h"


/* Flysky IBUS Data Id */
#define AFHDS2A_ID_VOLTAGE 0x00        // Internal Voltage
#define AFHDS2A_ID_TEMPERATURE 0x01    // Temperature
#define AFHDS2A_ID_MOT 0x02            // RPM
#define AFHDS2A_ID_EXTV 0x03           // External Voltage
#define AFHDS2A_ID_CELL_VOLTAGE 0x04   // Avg Cell voltage
#define AFHDS2A_ID_BAT_CURR 0x05       // battery current A * 100
#define AFHDS2A_ID_FUEL 0x06           // remaining battery percentage / mah drawn otherwise or fuel level no unit!
#define AFHDS2A_ID_RPM 0x07            // throttle value / battery capacity
#define AFHDS2A_ID_CMP_HEAD 0x08       // Heading  0..360 deg 0north 2bytes
#define AFHDS2A_ID_CLIMB_RATE 0x09     // 2 bytes m/s *100 signed
#define AFHDS2A_ID_COG 0x0A            // 2 bytes  Course over ground(NOT heading but direction of movement) in degrees * 100 0.0..359.99 degrees. unknown max uint
#define AFHDS2A_ID_GPS_STATUS 0x0B     // 2 bytes
#define AFHDS2A_ID_ACC_X 0x0C          // 2 bytes m/s *100 signed
#define AFHDS2A_ID_ACC_Y 0x0D          // 2 bytes m/s *100 signed
#define AFHDS2A_ID_ACC_Z 0x0E          // 2 bytes m/s *100 signed
#define AFHDS2A_ID_ROLL 0x0F           // 2 bytes deg *100 signed
#define AFHDS2A_ID_PITCH 0x10          // 2 bytes deg *100 signed
#define AFHDS2A_ID_YAW 0x11            // 2 bytes deg *100 signed
#define AFHDS2A_ID_VERTICAL_SPEED 0x12 // 2 bytes m/s *100 signed
#define AFHDS2A_ID_GROUND_SPEED 0x13   // 2 bytes m/s *100 different unit than build-in sensor
#define AFHDS2A_ID_GPS_DIST 0x14       // 2 bytes distance from home m unsigned
#define AFHDS2A_ID_ARMED 0x15          // 2 bytes
#define AFHDS2A_ID_FLIGHT_MODE 0x16    // 2 bytes
#define AFHDS2A_ID_PRES 0x41           // Pressure
#define AFHDS2A_ID_ODO1 0x7C           // Odometer1
#define AFHDS2A_ID_ODO2 0x7D           // Odometer2
#define AFHDS2A_ID_SPE 0x7E            // Speed 2 bytes km/h * 100
#define AFHDS2A_ID_TX_V 0x7F           // TX Voltage
#define AFHDS2A_ID_GPS_LAT 0x80 // 4bytes signed WGS84 in degrees * 1E7
#define AFHDS2A_ID_GPS_LON 0x81 // 4bytes signed WGS84 in degrees * 1E7
#define AFHDS2A_ID_GPS_ALT 0x82 // 4bytes signed!!! GPS alt m*100
#define AFHDS2A_ID_ALT 0x83     // 4bytes signed!!! Alt m*100
#define AFHDS2A_ID_S84 0x84
#define AFHDS2A_ID_S85 0x85
#define AFHDS2A_ID_S86 0x86
#define AFHDS2A_ID_S87 0x87
#define AFHDS2A_ID_S88 0x88
#define AFHDS2A_ID_S89 0x89
#define AFHDS2A_ID_S8a 0x8A
#define AFHDS2A_ID_RX_SIG_AFHDS3 0xF7 // SIG
#define AFHDS2A_ID_RX_SNR_AFHDS3 0xF8 // SNR
#define AFHDS2A_ID_ALT_FLYSKY 0xF9    // Altitude 2 bytes signed in m - used in FlySky native TX
#define AFHDS2A_ID_RX_SNR 0xFA        // SNR
#define AFHDS2A_ID_RX_NOISE 0xFB      // Noise
#define AFHDS2A_ID_RX_RSSI 0xFC       // RSSI
#define AFHDS2A_ID_RX_ERR_RATE 0xFE   // Error rate
#define AFHDS2A_ID_END 0xFF
// AC type telemetry with multiple values in one packet
#define AFHDS2A_ID_GPS_FULL 0xFD
#define AFHDS2A_ID_VOLT_FULL 0xF0
#define AFHDS2A_ID_ACC_FULL 0xEF
#define AFHDS2A_ID_TX_RSSI 0x200 // Pseudo id outside 1 byte range of FlySky sensors

#define IBUS_TYPE_U16 0
#define IBUS_TYPE_S16 1
#define IBUS_TYPE_U32 2
#define IBUS_TYPE_S32 3
#define IBUS_TYPE_GPS 4

#define IBUS_RECEIVED_NONE 0
#define IBUS_RECEIVED_POLL 1

#define IBUS_COMMAND_DISCOVER 0x8
#define IBUS_COMMAND_TYPE 0x9
#define IBUS_COMMAND_MEASURE 0xA

#define IBUS_TIMEOUT_US 10000
#define IBUS_PACKET_LENGHT 4

typedef struct sensor_ibus_t
{
    uint8_t data_id;
    uint8_t type;
    float *value;
} sensor_ibus_t;

extern QueueHandle_t sensors_queue_handle, tasks_queue_handle;
extern TaskHandle_t pwm_out_task_handle, led_task_handle;
extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;

void ibus_task(void *parameters);

#endif