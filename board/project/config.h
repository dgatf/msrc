#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <string.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "config.h"

#include "constants.h"

#define CONFIG_FORZE_WRITE false
#define CONFIG_FLASH_TARGET_OFFSET (512 * 1024)
#define CONFIG_VERSION 1

/* Receiver protocol */
#define RX_PROTOCOL RX_SMARTPORT // RX_SMARTPORT, RX_XBUS, RX_SRXL, RX_FRSKY_D, RX_IBUS, RX_SBUS, RX_MULTIPLEX, RX_JETIEX, RX_HITEC
/* Sensors */
#define ESC_PROTOCOL ESC_NONE // ESC_NONE, ESC_HW3, ESC_HW4, ESC_PWM, ESC_CASTLE, ESC_KONTRONIK, ESC_APD_F, ESC_APD_HV
#define ENABLE_GPS false
#define GPS_BAUD_RATE 9600
#define ENABLE_ANALOG_VOLTAGE false
#define ENABLE_ANALOG_CURRENT false
#define ENABLE_ANALOG_NTC false
#define ENABLE_ANALOG_AIRSPEED false
#define I2C1_TYPE I2C_NONE // I2C_NONE, I2C_BMP280, I2C_MS5611, I2C_BMP180
#define I2C1_ADDRESS 0x77  // 0x76 (BMP280), 0x77 (BMP180, MS5611)
/* Pwm out */
#define ENABLE_PWM_OUT false
/* Refresh rate in ms (only Frsky) */
#define REFRESH_RATE_RPM 1000
#define REFRESH_RATE_VOLTAGE 1000
#define REFRESH_RATE_CURRENT 1000
#define REFRESH_RATE_TEMPERATURE 1000
#define REFRESH_RATE_GPS 1000
#define REFRESH_RATE_CONSUMPTION 1000
#define REFRESH_RATE_VARIO 1000
#define REFRESH_RATE_AIRSPEED 1000
#define REFRESH_RATE_DEFAULT 1000
/* Averaging elements (1 = no averaging) */
#define AVERAGING_ELEMENTS_RPM 1
#define AVERAGING_ELEMENTS_VOLTAGE 1
#define AVERAGING_ELEMENTS_CURRENT 1
#define AVERAGING_ELEMENTS_TEMPERATURE 1
#define AVERAGING_ELEMENTS_VARIO 1
#define AVERAGING_ELEMENTS_AIRSPEED 1

/* Analog voltage sensors */
#define ANALOG_VOLTAGE_MULTIPLIER 7.8 // 7.8 if using 68k and 10k as proposed in the docs
/* Analog current sensor */
#define ANALOG_CURRENT_MULTIPLIER 1 // current_multiplier = 1000 / sensitivity(mV/A)
/* Zero current output voltage offset or quiescent voltage (voltage offset when I = 0, Viout)
 - All hall effect core sensors (Amploc) are bidirectional. Viout=Vs/2
 - Hall effect coreless sensors (IC) (ACS758) can be bidirectional or directional. Recommended to use directional for higher sensitivity. Viout defined in datasheet
 - If CURRENT_AUTO_OFFSET is true, then after 5 seconds, voltage read is set as offset. It is recommended to use auto offset
*/
#define ANALOG_CURRENT_OFFSET 0
#define ANALOG_CURRENT_AUTO_OFFSET true
#define ANALOG_CURRENT_SENSITIVITY 1000

/* RPM multipliers (optional, this may be done in transmitter*/
#define RPM_PAIR_OF_POLES 1
#define RPM_PINION_TEETH 1 // For helis
#define RPM_MAIN_TEETH 1   // For helis

/* Vario */
#define VARIO_AUTO_OFFSET false
#define BMP280_FILTER 3 // BMP Filter. Higher filter = lower noise: 1 - low, 2 - medium, 3 - high 

/* Only smartport and opentx */
#define SMARTPORT_SENSOR_ID 18   // Sensor Id
#define SMARTPORT_DATA_ID 0x5000 // DataId (sensor type)

/* Ibus */
#define IBUS_GPS_ALTERNATIVE_COORDINATES false

/* XBus */
#define XBUS_CLOCK_STRECH_SWITCH true

/* Jeti Ex */
#define JETI_GPS_SPEED_UNITS_KMH true

/* Add init delay for FlyFun ESC. Enable if the ESC doesn't arm */
#define ENABLE_ESC_INIT_DELAY false
#define ESC_INIT_DELAY_DURATION 10000

/* HW V4/V5 parameters */
#define ESC_HW4_CURRENT_THRESHOLD 25
#define ESC_HW4_DIVISOR 11
#define ESC_HW4_AMPGAIN 10
#define ESC_HW4_CURRENT_MAX 250

/* 
   Debug
   Disconnect Vcc from the RC model to the board before connecting USB
   Telemetry may not work properly in debug mode
*/

#define MSRC_DEBUG 0  // 0 = no debug, 1 = debug level 1, 2 = debug level 2

//#define SIM_RX
//#define SIM_SENSORS

//#define SIM_SMARTPORT_SEND_CONFIG_LUA
//#define SIM_SMARTPORT_RECEIVE_CONFIG_LUA
//#define SIM_SMARTPORT_SEND_SENSOR_ID
//#define SIM_SMARTPORT_RECEIVE_SENSOR_ID

typedef enum rx_protocol_t
{
    RX_SMARTPORT,
    RX_FRSKY_D,
    RX_XBUS,
    RX_SRXL,
    RX_IBUS,
    RX_SBUS,
    RX_MULTIPLEX,
    RX_JETIEX,
    RX_HITEC
} rx_protocol_t;

typedef enum esc_protocol_t
{
    ESC_NONE,
    ESC_HW3,
    ESC_HW4,
    ESC_PWM,
    ESC_CASTLE,
    ESC_KONTRONIK,
    ESC_APD_F,
    ESC_APD_HV
} esc_protocol_t;

typedef enum i2c_module_t
{
    I2C_NONE,
    I2C_BMP280,
    I2C_MS5611,
    I2C_BMP180
} i2c_module_t;

typedef enum analog_current_type_t
{
    CURRENT_TYPE_HALL,
    CURRENT_TYPE_SHUNT
} analog_current_type_t;

typedef struct config_t
{
    uint16_t version;
    enum rx_protocol_t rx_protocol;
    enum esc_protocol_t esc_protocol;
    bool enable_gps;
    uint32_t gps_baudrate;
    bool enable_analog_voltage;
    bool enable_analog_current;
    bool enable_analog_ntc;
    bool enable_analog_airspeed;
    enum i2c_module_t i2c_module;
    uint8_t i2c_address;
    float alpha_rpm;
    float alpha_voltage;
    float alpha_current;
    float alpha_temperature;
    float alpha_vario;
    float alpha_airspeed;
    uint16_t refresh_rate_rpm;
    uint16_t refresh_rate_voltage;
    uint16_t refresh_rate_current;
    uint16_t refresh_rate_temperature;
    uint16_t refresh_rate_gps;
    uint16_t refresh_rate_consumption;
    uint16_t refresh_rate_vario;
    uint16_t refresh_rate_airspeed;
    uint16_t refresh_rate_default;
    float analog_voltage_multiplier;
    enum analog_current_type_t analog_current_type;
    uint16_t analog_current_sensitivity;
    float analog_current_quiescent_voltage;
    float analog_current_multiplier;
    float analog_current_offset;
    bool analog_current_autoffset;
    uint8_t pairOfPoles;
    uint8_t mainTeeth;
    uint8_t pinionTeeth;
    float rpm_multiplier;
    uint8_t bmp280_filter;
    bool enable_pwm_out;
    uint8_t smartport_sensor_id;
    uint16_t smartport_data_id;
    bool vario_auto_offset;
    bool xbus_clock_stretch;
    bool jeti_gps_speed_units_kmh;
    bool enable_esc_hw4_init_delay;
    uint16_t esc_hw4_init_delay_duration;
    uint8_t esc_hw4_current_thresold;
    uint16_t esc_hw4_current_max;
    float esc_hw4_divisor;
    float esc_hw4_ampgain;
    bool ibus_alternative_coordinates;
    uint8_t debug;
    uint32_t spare1;
    uint32_t spare2;
    uint32_t spare3;
    uint32_t spare4;
    uint32_t spare5;
} config_t;

config_t *config_read();
void config_write(config_t *config);
void config_forze_write();
void config_get(config_t *config);

#endif