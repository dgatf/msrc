#include "config.h"

config_t *config_read()
{
    uint16_t *version = (uint16_t *)(XIP_BASE + CONFIG_FLASH_TARGET_OFFSET);
    if (*version != CONFIG_VERSION)
    {
        config_forze_write();
    }
    return (config_t *)(XIP_BASE + CONFIG_FLASH_TARGET_OFFSET);
}

void config_write(config_t *config)
{
    uint8_t flash[FLASH_PAGE_SIZE] = {0};
    memcpy(flash, (uint8_t *)config, sizeof(config_t));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_TARGET_OFFSET, flash, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

void config_get(config_t *config)
{
    memcpy(config, (config_t *)(XIP_BASE + CONFIG_FLASH_TARGET_OFFSET), sizeof(config_t));
    config->debug = MSRC_DEBUG;
}

void config_forze_write()
{
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
    config.analog_current_sensitivity = ANALOG_CURRENT_SENSITIVITY;
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
    config.esc_hw4_ampgain = ESC_HW4_AMPGAIN;
    config.esc_hw4_current_max = ESC_HW4_CURRENT_MAX;
    config.ibus_alternative_coordinates = IBUS_GPS_ALTERNATIVE_COORDINATES;
    config.debug = MSRC_DEBUG;
    config.spare1 = 0;
    config.spare2 = 0;
    config.spare3 = 0;
    config.spare4 = 0;
    config.spare5 = 0;

    config_write(&config);
}
