#include "config.h"

#include <hardware/flash.h>
#include <hardware/sync.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#define CONFIG_FLASH_TARGET_OFFSET (512 * 1024)

/* Receiver protocol */
#define RX_PROTOCOL \
    RX_SMARTPORT  // RX_SMARTPORT, RX_XBUS, RX_SRXL, RX_FRSKY_D, RX_IBUS, RX_SBUS, RX_MULTIPLEX, RX_JETIEX, RX_HITEC
/* Sensors */
#define ESC_PROTOCOL ESC_NONE  // ESC_NONE, ESC_HW3, ESC_HW4, ESC_PWM, ESC_CASTLE, ESC_KONTRONIK, ESC_APD_F, ESC_APD_HV
#define ENABLE_GPS false
#define GPS_BAUD_RATE 9600
#define ENABLE_ANALOG_VOLTAGE false
#define ENABLE_ANALOG_CURRENT false
#define ENABLE_ANALOG_NTC false
#define ENABLE_ANALOG_AIRSPEED false
#define I2C1_TYPE I2C_NONE  // I2C_NONE, I2C_BMP280, I2C_MS5611, I2C_BMP180
#define I2C1_ADDRESS 0x77   // 0x76 (BMP280), 0x77 (BMP180, MS5611)
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

/* Analog rate*/
#define ANALOG_RATE 10
/* Analog voltage sensors */
#define ANALOG_VOLTAGE_MULTIPLIER 7.8  // 7.8 if using 68k and 10k as proposed in the docs
/* Analog current sensor */
#define ANALOG_CURRENT_MULTIPLIER 1  // current_multiplier = 1000 / sensitivity(mV/A)
/* Zero current output voltage offset or quiescent voltage (voltage offset when I = 0, Viout)
 - All hall effect core sensors (Amploc) are bidirectional. Viout=Vs/2
 - Hall effect coreless sensors (IC) (ACS758) can be bidirectional or directional. Recommended to use directional for
 higher sensitivity. Viout defined in datasheet
 - If CURRENT_AUTO_OFFSET is true, then after 5 seconds, voltage read is set as offset. It is recommended to use auto
 offset
*/
#define ANALOG_CURRENT_OFFSET 0
#define ANALOG_CURRENT_AUTO_OFFSET true
/* Analog airspeed */
#define AIRSPEED_OFFSET 0
#define AIRSPEED_SLOPE 1

/* RPM multipliers (optional, this may be done in transmitter*/
#define RPM_PAIR_OF_POLES 1
#define RPM_PINION_TEETH 1  // For helis
#define RPM_MAIN_TEETH 1    // For helis

/* Vario */
#define VARIO_AUTO_OFFSET false
#define BMP280_FILTER 3  // BMP Filter. Higher filter = lower noise: 1 - low, 2 - medium, 3 - high

/* Only smartport and opentx */
#define SMARTPORT_SENSOR_ID 18    // Sensor Id
#define SMARTPORT_DATA_ID 0x5000  // DataId (sensor type)

/* Ibus */
#define IBUS_GPS_ALTERNATIVE_COORDINATES false

/* XBus */
#define XBUS_CLOCK_STRECH_SWITCH false
#define XBUS_ALTERNATIVE_VOLT_TEMP false

/* Jeti Ex */
#define JETI_GPS_SPEED_UNITS_KMH true

/* Serial monitor */
#define SERIAL_MONITOR_BAUDRATE 19200
#define SERIAL_MONITOR_STOPBITS 1
#define SERIAL_MONITOR_PARITY 0
#define SERIAL_MONITOR_TIMEOUT_MS 1
#define SERIAL_MONITOR_INVERTED false

/* Add init delay for FlyFun ESC. Enable if the ESC doesn't arm */
#define ENABLE_ESC_INIT_DELAY false
#define ESC_INIT_DELAY_DURATION 10000

/* HW V4/V5 parameters */
#define ESC_HW4_MANUAL_OFFSET 1
#define ESC_HW4_CURRENT_OFFSET 0
#define ESC_HW4_CURRENT_THRESHOLD 25
#define ESC_HW4_DIVISOR 11
#define ESC_HW4_CURRENT_MULTIPLIER 10
#define ESC_HW4_CURRENT_MAX 250

/* Fuel flow sensor */
#define ENABLE_FUEL_FLOW false
#define FUEL_FLOW_ML_PER_MINUTE 0.01

/* Fuel pressure */
#define XGZP68XXD_K 64

config_t *config_read() {
    uint16_t *version = (uint16_t *)(XIP_BASE + CONFIG_FLASH_TARGET_OFFSET);
    if (*version != CONFIG_VERSION) {
        config_forze_write();
    }
    return (config_t *)(XIP_BASE + CONFIG_FLASH_TARGET_OFFSET);
}

void config_write(config_t *config) {
    uint8_t flash[FLASH_PAGE_SIZE] = {0};
    memcpy(flash, (uint8_t *)config, sizeof(config_t));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_TARGET_OFFSET, flash, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

void config_get(config_t *config) {
    memcpy(config, (config_t *)(XIP_BASE + CONFIG_FLASH_TARGET_OFFSET), sizeof(config_t));
    config->debug = MSRC_DEBUG;
}

void config_forze_write() {
    config_t config;
    config.version = CONFIG_VERSION;
    config.rx_protocol = RX_PROTOCOL;
    config.esc_protocol = ESC_PROTOCOL;
    config.enable_gps = ENABLE_GPS;
    config.gps_baudrate = GPS_BAUD_RATE;
    config.enable_analog_voltage = ENABLE_ANALOG_VOLTAGE;
    config.enable_analog_current = ENABLE_ANALOG_CURRENT;
    config.enable_analog_ntc = ENABLE_ANALOG_NTC;
    config.enable_analog_airspeed = ENABLE_ANALOG_AIRSPEED;
    config.i2c_module = I2C1_TYPE;
    config.i2c_address = I2C1_ADDRESS;
    config.alpha_rpm = ALPHA(AVERAGING_ELEMENTS_RPM);
    config.alpha_voltage = ALPHA(AVERAGING_ELEMENTS_VOLTAGE);
    config.alpha_current = ALPHA(AVERAGING_ELEMENTS_CURRENT);
    config.alpha_temperature = ALPHA(AVERAGING_ELEMENTS_TEMPERATURE);
    config.alpha_vario = ALPHA(AVERAGING_ELEMENTS_VARIO);
    config.alpha_airspeed = ALPHA(AVERAGING_ELEMENTS_AIRSPEED);
    config.refresh_rate_rpm = REFRESH_RATE_RPM;
    config.refresh_rate_voltage = REFRESH_RATE_VOLTAGE;
    config.refresh_rate_current = REFRESH_RATE_CURRENT;
    config.refresh_rate_temperature = REFRESH_RATE_TEMPERATURE;
    config.refresh_rate_gps = REFRESH_RATE_GPS;
    config.refresh_rate_consumption = REFRESH_RATE_CONSUMPTION;
    config.refresh_rate_vario = REFRESH_RATE_VARIO;
    config.refresh_rate_airspeed = REFRESH_RATE_AIRSPEED;
    config.refresh_rate_default = REFRESH_RATE_DEFAULT;
    config.analog_voltage_multiplier = ANALOG_VOLTAGE_MULTIPLIER;
    config.analog_current_multiplier = ANALOG_CURRENT_MULTIPLIER;
    config.analog_current_offset = ANALOG_CURRENT_OFFSET;
    config.analog_current_autoffset = ANALOG_CURRENT_AUTO_OFFSET;
    config.pairOfPoles = RPM_PAIR_OF_POLES;
    config.mainTeeth = RPM_MAIN_TEETH;
    config.pinionTeeth = RPM_PINION_TEETH;
    config.rpm_multiplier = RPM_PINION_TEETH / (1.0 * RPM_MAIN_TEETH * RPM_PAIR_OF_POLES);
    config.bmp280_filter = BMP280_FILTER;
    config.enable_pwm_out = ENABLE_PWM_OUT;
    config.smartport_sensor_id = SMARTPORT_SENSOR_ID;
    config.smartport_data_id = SMARTPORT_DATA_ID;
    config.vario_auto_offset = VARIO_AUTO_OFFSET;
    config.xbus_clock_stretch = XBUS_CLOCK_STRECH_SWITCH;
    config.jeti_gps_speed_units_kmh = JETI_GPS_SPEED_UNITS_KMH;
    config.enable_esc_hw4_init_delay = ENABLE_ESC_INIT_DELAY;
    config.esc_hw4_init_delay_duration = ESC_INIT_DELAY_DURATION;
    config.esc_hw4_current_thresold = ESC_HW4_CURRENT_THRESHOLD;
    config.esc_hw4_current_max = ESC_HW4_CURRENT_MAX;
    config.esc_hw4_divisor = ESC_HW4_DIVISOR;
    config.esc_hw4_current_multiplier = ESC_HW4_CURRENT_MULTIPLIER;
    config.esc_hw4_current_max = ESC_HW4_CURRENT_MAX;
    config.ibus_alternative_coordinates = IBUS_GPS_ALTERNATIVE_COORDINATES;
    config.debug = MSRC_DEBUG;
    config.esc_hw4_is_manual_offset = ESC_HW4_MANUAL_OFFSET;
    config.esc_hw4_offset = ESC_HW4_CURRENT_OFFSET;
    config.xbus_use_alternative_volt_temp = XBUS_ALTERNATIVE_VOLT_TEMP;
    config.serial_monitor_baudrate = SERIAL_MONITOR_BAUDRATE;
    config.serial_monitor_stop_bits = SERIAL_MONITOR_STOPBITS;
    config.serial_monitor_parity = SERIAL_MONITOR_PARITY;
    config.serial_monitor_timeout_ms = SERIAL_MONITOR_TIMEOUT_MS;
    config.serial_monitor_inverted = SERIAL_MONITOR_INVERTED;
    config.airspeed_offset = AIRSPEED_OFFSET * 100;
    config.airspeed_slope = AIRSPEED_SLOPE * 100;
    config.fuel_flow_ml_per_pulse = FUEL_FLOW_ML_PER_MINUTE;
    config.enable_fuel_flow = ENABLE_FUEL_FLOW;
    config.enable_fuel_pressure = false;
    config.xgzp68xxd_k = XGZP68XXD_K;
    config_write(&config);
}
