#include "xbus.h"

#include <math.h>
#include <stdio.h>

#include "airspeed.h"
#include "bmp180.h"
#include "bmp280.h"
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
#include "esc_openyge.h"
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "gps.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ina3221.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "stdlib.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "xgzp68xxd.h"

#define XBUS_GPS_FIX (1 << 0)
#define XBUS_GPS_3D (1 << 1)
#define XBUS_GPS_HOME (1 << 4)

static void i2c_request_handler(uint8_t address);
// static void set_config(void);
static uint8_t bcd8(float value, uint8_t precision);
static uint16_t bcd16(float value, uint8_t precision);
static uint32_t bcd32(float value, uint8_t precision);
static int64_t interval_250_callback(alarm_id_t id, void *parameters);
static int64_t interval_500_callback(alarm_id_t id, void *parameters);
static int64_t interval_1000_callback(alarm_id_t id, void *parameters);
static int64_t interval_1500_callback(alarm_id_t id, void *parameters);
static int64_t interval_2000_callback(alarm_id_t id, void *parameters);
static int64_t interval_3000_callback(alarm_id_t id, void *parameters);

xbus_sensor_t sensor;
static volatile int16_t delta_0250ms, delta_0500ms, delta_1000ms, delta_1500ms, delta_2000ms, delta_3000ms;

void xbus_i2c_handler(uint8_t address) { i2c_request_handler(address); }

void xbus_task(void *parameters) {
    context.led_cycle_duration = 6;
    context.led_cycles = 1;

    PIO pio = pio1;
    uint pin = I2C1_SDA_GPIO;

    i2c_multi_init(pio, pin);
    i2c_multi_set_request_handler(i2c_request_handler);

    xbus_set_config();

    debug("\nXBUS init");

    vTaskSuspend(NULL);
}

void xbus_format_sensor(uint8_t address, uint8_t *buffer) {
    static float alt = 0;
    switch (address) {
        case XBUS_AIRSPEED_ID: {
            xbus_airspeed_t airspeed;
            airspeed.identifier = XBUS_AIRSPEED_ID;
            airspeed.airspeed = swap_16((uint16_t)(*sensor.airspeed[XBUS_AIRSPEED_AIRSPEED]));
            if (swap_16((uint16_t)(airspeed.airspeed)) > swap_16((uint16_t)(airspeed.max_airspeed)))
                airspeed.max_airspeed = airspeed.airspeed;
            memcpy(buffer, &airspeed, sizeof(xbus_airspeed_t));
            break;
        }
        case XBUS_GPS_LOC_ID: {
            xbus_gps_loc_t gps_loc;
            gps_loc.identifier = XBUS_GPS_LOC_ID;
            uint8_t gps_flags = 0;
            float lat = *sensor.gps_loc->latitude;
            if (lat < 0)  // N=1,+, S=0,-
                lat *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_NORTH_BIT;
            uint deg = lat;
            float min = (lat - deg) * 60;
            gps_loc.latitude = ((uint32_t)bcd8(deg, 0) << 24) | bcd32(min, 4);
            float lon = *sensor.gps_loc->longitude;
            if (lon < 0)  // E=1,+, W=0,-
                lon *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_EAST_BIT;
            if (lon >= 100) {
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
                lon -= 100;
            }
            deg = lon;
            min = (lon - deg) * 60;
            gps_loc.longitude = ((uint32_t)bcd8(deg, 0) << 24) | bcd32(min, 4);
            gps_loc.course = bcd16(*sensor.gps_loc->course, 1);
            gps_loc.hdop = bcd8(*sensor.gps_loc->hdop, 1);
            alt = *sensor.gps_loc->altitude_low;
            if (alt < 0) {
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_NEGATIVE_ALT_BIT;
                alt *= -1;
            }
            gps_loc.gps_flags = gps_flags;
            gps_loc.altitude_low = bcd16(fmod(alt, 1000), 1);
            gps_loc.gps_flags |= (*sensor.gps_stat[XBUS_GPS_STAT_FIX_TYPE] > 0 ? 1 : 0) << XBUS_GPS_FIX;
            gps_loc.gps_flags |= (*sensor.gps_stat[XBUS_GPS_STAT_HOME_SET] > 0 ? 1 : 0) << XBUS_GPS_HOME;
            gps_loc.gps_flags |= (*sensor.gps_stat[XBUS_GPS_STAT_FIX_TYPE] == 2 ? 1 : 0) << XBUS_GPS_3D;
            memcpy(buffer, &gps_loc, sizeof(xbus_gps_loc_t));
            break;
        }
        case XBUS_GPS_STAT_ID: {
            xbus_gps_stat_t gps_stat;
            gps_stat.identifier = XBUS_GPS_STAT_ID;
            gps_stat.speed = bcd16(*sensor.gps_stat[XBUS_GPS_STAT_SPEED], 1);
            gps_stat.utc = bcd32(*sensor.gps_stat[XBUS_GPS_STAT_TIME], 0) << 8;
            gps_stat.num_sats = bcd8(*sensor.gps_stat[XBUS_GPS_STAT_SATS], 0);
            gps_stat.altitude_high = bcd8((uint8_t)(alt / 1000), 0);
            memcpy(buffer, &gps_stat, sizeof(xbus_gps_stat_t));
            break;
        }
        case XBUS_ENERGY_ID: {
            xbus_energy_t energy;
            energy.identifier = XBUS_ENERGY_ID;
            if (sensor.energy[XBUS_ENERGY_CURRENT1])
                energy.current_a = swap_16((int16_t)(*sensor.energy[XBUS_ENERGY_CURRENT1] * 100));
            if (sensor.energy[XBUS_ENERGY_CONSUMPTION1])
                energy.charge_used_a = swap_16((int16_t)(*sensor.energy[XBUS_ENERGY_CONSUMPTION1] * 10));
            if (sensor.energy[XBUS_ENERGY_VOLTAGE1])
                energy.volts_a = swap_16((uint16_t)(*sensor.energy[XBUS_ENERGY_VOLTAGE1] * 100));
            if (sensor.energy[XBUS_ENERGY_CURRENT2])
                energy.current_b = swap_16((int16_t)(*sensor.energy[XBUS_ENERGY_CURRENT2] * 10));
            if (sensor.energy[XBUS_ENERGY_CONSUMPTION2])
                energy.charge_used_b = swap_16((int16_t)(*sensor.energy[XBUS_ENERGY_CONSUMPTION2]));
            if (sensor.energy[XBUS_ENERGY_VOLTAGE2])
                energy.volts_b = swap_16((uint16_t)(*sensor.energy[XBUS_ENERGY_VOLTAGE2] * 100));
            memcpy(buffer, &energy, sizeof(xbus_energy_t));
            break;
        }
        case XBUS_ESC_ID: {
            xbus_esc_t esc;
            esc.identifier = XBUS_ESC_ID;
            if (sensor.esc[XBUS_ESC_RPM]) esc.rpm = swap_16((uint16_t)(*sensor.esc[XBUS_ESC_RPM] / 10));
            if (sensor.esc[XBUS_ESC_VOLTAGE])
                esc.volts_input = swap_16((uint16_t)(*sensor.esc[XBUS_ESC_VOLTAGE] * 100));
            if (sensor.esc[XBUS_ESC_TEMPERATURE_FET])
                esc.temp_fet = swap_16((uint16_t)(*sensor.esc[XBUS_ESC_TEMPERATURE_FET] * 10));
            if (sensor.esc[XBUS_ESC_CURRENT])
                esc.current_motor = swap_16((uint16_t)(*sensor.esc[XBUS_ESC_CURRENT] * 100));
            if (sensor.esc[XBUS_ESC_TEMPERATURE_BEC])
                esc.temp_bec = swap_16((uint16_t)(*sensor.esc[XBUS_ESC_TEMPERATURE_BEC] * 10));
            if (sensor.esc[XBUS_ESC_CURRENT_BEC]) esc.current_bec = *sensor.esc[XBUS_ESC_CURRENT_BEC] * 10;
            if (sensor.esc[XBUS_ESC_VOLTAGE_BEC]) esc.voltage_bec = *sensor.esc[XBUS_ESC_VOLTAGE_BEC] * 20;
            memcpy(buffer, &esc, sizeof(xbus_esc_t));
            break;
        }
        case XBUS_BATTERY_ID: {
            xbus_battery_t battery;
            battery.identifier = XBUS_BATTERY_ID;
            if (sensor.battery[XBUS_BATTERY_CURRENT1])
                battery.current_a = swap_16((int16_t)(*sensor.battery[XBUS_BATTERY_CURRENT1] * 10));
            if (sensor.battery[XBUS_BATTERY_CONSUMPTION1])
                battery.charge_used_a = swap_16((int16_t)(*sensor.battery[XBUS_BATTERY_CONSUMPTION1]));
            if (sensor.battery[XBUS_BATTERY_TEMP1])
                battery.temp_a = swap_16((uint16_t)(*sensor.battery[XBUS_BATTERY_TEMP1] * 10));
            if (sensor.battery[XBUS_BATTERY_CURRENT2])
                battery.current_b = swap_16((int16_t)(*sensor.battery[XBUS_BATTERY_CURRENT2] * 10));
            if (sensor.battery[XBUS_BATTERY_CONSUMPTION2])
                battery.charge_used_b = swap_16((int16_t)(*sensor.battery[XBUS_BATTERY_CONSUMPTION2]));
            if (sensor.battery[XBUS_BATTERY_TEMP2])
                battery.temp_b = swap_16((uint16_t)(*sensor.battery[XBUS_BATTERY_TEMP2] * 10));
            memcpy(buffer, &battery, sizeof(xbus_battery_t));
            break;
        }
        case XBUS_VARIO_ID: {
            xbus_vario_t vario;
            vario.identifier = XBUS_VARIO_ID;
            float altitude = *sensor.vario[XBUS_VARIO_ALTITUDE];
            vario.altitude = swap_16((int16_t)(altitude * 10));
#ifdef SIM_SENSORS
            vario.delta_0250ms = swap_16((int16_t)(-10));
            vario.delta_0500ms = swap_16((int16_t)(20));
            vario.delta_1000ms = swap_16((int16_t)(-12.32));
            vario.delta_1500ms = swap_16((int16_t)(15));
            vario.delta_2000ms = swap_16((int16_t)(20));
            vario.delta_3000ms = swap_16((int16_t)(-300));
#endif
            memcpy(buffer, &vario, sizeof(xbus_vario_t));
            break;
        }
        case XBUS_RPMVOLTTEMP_ID: {
            xbus_rpm_volt_temp_t rpm_volt_temp;
            rpm_volt_temp.identifier = XBUS_RPMVOLTTEMP_ID;
            if (sensor.rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT])
                rpm_volt_temp.volts = swap_16((uint16_t)(*sensor.rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT] * 100));
            if (sensor.rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP])
                rpm_volt_temp.temperature = swap_16((int16_t)(*sensor.rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP]));
            memcpy(buffer, &rpm_volt_temp, sizeof(xbus_rpm_volt_temp_t));
            break;
        }
        case XBUS_FUEL_FLOW_ID: {
            xbus_fuel_flow_t fuel_flow;
            fuel_flow.identifier = XBUS_FUEL_FLOW_ID;
            if (sensor.fuel_flow[XBUS_FUEL_FLOW_CONSUMED])
                fuel_flow.fuel_consumed_A = swap_16((uint16_t)(*sensor.fuel_flow[XBUS_FUEL_FLOW_CONSUMED] * 10));
            if (sensor.fuel_flow[XBUS_FUEL_FLOW_RATE])
                fuel_flow.flow_rate_A = swap_16((uint16_t)(*sensor.fuel_flow[XBUS_FUEL_FLOW_RATE] * 10));
            memcpy(buffer, &fuel_flow, sizeof(xbus_fuel_flow_t));
            break;
        }
        case XBUS_STRU_TELE_DIGITAL_AIR_ID: {
            xbus_stru_tele_digital_air_t stru_tele_digital_air;
            stru_tele_digital_air.identifier = XBUS_STRU_TELE_DIGITAL_AIR_ID;
            if (sensor.stru_tele_digital_air[XBUS_DIGITAL_AIR_FUEL_PRESSURE])
                stru_tele_digital_air.pressure =
                    swap_16((uint16_t)(*sensor.stru_tele_digital_air[XBUS_DIGITAL_AIR_FUEL_PRESSURE] * 0.000145038 *
                                       10));  // Pa to psi, precision 0.1 psi
            memcpy(buffer, &stru_tele_digital_air, sizeof(xbus_stru_tele_digital_air_t));
            break;
        }
        case XBUS_TELE_LIPOMON_ID: {
            xbus_tele_lipomon_t tele_lipomon;
            tele_lipomon.identifier = XBUS_TELE_LIPOMON_ID;
            for (uint i = 0; i < 6; i++) {
                if (*sensor.tele_lipomon[i] < 1 || !sensor.tele_lipomon[i])
                    tele_lipomon.cell[i] = 0xFFFF;
                else
                    tele_lipomon.cell[i] = swap_16((uint16_t)(*sensor.tele_lipomon[i] * 100));
            }
            memcpy(buffer, &tele_lipomon, sizeof(xbus_tele_lipomon_t));
            break;
        }
        case XBUS_TELE_G_METER_ID: {
            xbus_tele_g_meter_t tele_g_meter;
            tele_g_meter.identifier = XBUS_TELE_G_METER_ID;
            if (sensor.tele_g_meter[XBUS_TELE_G_METER_X]) {
                tele_g_meter.GForceX = swap_16((int16_t)(*sensor.tele_g_meter[XBUS_TELE_G_METER_X] * 100));
                if (swap_16((int16_t)(fabs(tele_g_meter.GForceX))) > swap_16((int16_t)(tele_g_meter.maxGForceX))) {
                    tele_g_meter.maxGForceX = tele_g_meter.GForceX;
                }
            }
            if (sensor.tele_g_meter[XBUS_TELE_G_METER_Y]) {
                tele_g_meter.GForceY = swap_16((int16_t)(*sensor.tele_g_meter[XBUS_TELE_G_METER_Y] * 100));
                if (swap_16((int16_t)(fabs(tele_g_meter.GForceY))) > swap_16((uint16_t)(tele_g_meter.maxGForceY))) {
                    tele_g_meter.maxGForceY = tele_g_meter.GForceY;
                }
            }
            if (sensor.tele_g_meter[XBUS_TELE_G_METER_Z]) {
                tele_g_meter.GForceZ = swap_16((int16_t)(*sensor.tele_g_meter[XBUS_TELE_G_METER_Z] * 100));
                if (swap_16((int16_t)(tele_g_meter.GForceZ)) > swap_16((int16_t)(tele_g_meter.maxGForceZ))) {
                    tele_g_meter.maxGForceZ = tele_g_meter.GForceZ;
                }
                if (swap_16((int16_t)(tele_g_meter.GForceZ)) < swap_16((int16_t)(tele_g_meter.minGForceZ))) {
                    tele_g_meter.minGForceZ = tele_g_meter.GForceZ;
                }
            }
            memcpy(buffer, &tele_g_meter, sizeof(xbus_tele_g_meter_t));
            break;
        }
        case XBUS_TELE_GYRO_ID: {
            xbus_tele_gyro_t tele_gyro;
            tele_gyro.identifier = XBUS_TELE_GYRO_ID;
            if (sensor.tele_gyro[XBUS_TELE_GYRO_ROLL])
                tele_gyro.gyroX = swap_16((int16_t)(*sensor.tele_gyro[XBUS_TELE_GYRO_ROLL] * 10));
            if (swap_16((int16_t)(fabs(tele_gyro.gyroX))) > swap_16((int16_t)(tele_gyro.maxGyroX))) {
                tele_gyro.maxGyroX = tele_gyro.gyroX;
            }
            if (sensor.tele_gyro[XBUS_TELE_GYRO_PITCH])
                tele_gyro.gyroY = swap_16((int16_t)(*sensor.tele_gyro[XBUS_TELE_GYRO_PITCH] * 10));
            if (swap_16((int16_t)(fabs(tele_gyro.gyroY))) > swap_16((int16_t)(tele_gyro.maxGyroY))) {
                tele_gyro.maxGyroY = tele_gyro.gyroY;
            }
            if (sensor.tele_gyro[XBUS_TELE_GYRO_YAW])
                tele_gyro.gyroZ = swap_16((int16_t)(*sensor.tele_gyro[XBUS_TELE_GYRO_YAW] * 10));
            if (swap_16((int16_t)(fabs(tele_gyro.gyroZ))) > swap_16((int16_t)(tele_gyro.maxGyroZ))) {
                tele_gyro.maxGyroZ = tele_gyro.gyroZ;
            }
            memcpy(buffer, &tele_gyro, sizeof(xbus_tele_gyro_t));
            break;
        }
    }
}

static void i2c_request_handler(uint8_t address) {
    debug("\nXBUS (%u) Address: %X Packet: ", uxTaskGetStackHighWaterMark(context.receiver_task_handle), address);
    uint8_t buffer[16] = {0};
    switch (address) {
        case XBUS_AIRSPEED_ID:
            if (!sensor.is_enabled[XBUS_AIRSPEED]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_airspeed_t), "0x%X ");
            break;
        case XBUS_ALTIMETER_ID:
            if (!sensor.is_enabled[XBUS_ALTIMETER]) break;
            break;
        case XBUS_GPS_LOC_ID:
            if (!sensor.is_enabled[XBUS_GPS_LOC]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_gps_loc_t), "0x%X ");
            break;
        case XBUS_GPS_STAT_ID:
            if (!sensor.is_enabled[XBUS_GPS_STAT]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_gps_stat_t), "0x%X ");
            break;
        case XBUS_ENERGY_ID:
            if (!sensor.is_enabled[XBUS_ENERGY]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_energy_t), "0x%X ");
            break;
        case XBUS_ESC_ID:
            if (!sensor.is_enabled[XBUS_ESC]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_esc_t), "0x%X ");
            break;
        case XBUS_BATTERY_ID:
            if (!sensor.is_enabled[XBUS_BATTERY]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_battery_t), "0x%X ");
            break;
        case XBUS_VARIO_ID:
            if (!sensor.is_enabled[XBUS_VARIO]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_vario_t), "0x%X ");
            break;
        case XBUS_RPMVOLTTEMP_ID:
            if (!sensor.is_enabled[XBUS_RPMVOLTTEMP]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_rpm_volt_temp_t), "0x%X ");
            break;
        case XBUS_FUEL_FLOW_ID:
            if (!sensor.is_enabled[XBUS_FUEL_FLOW]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_fuel_flow_t), "0x%X ");
            break;
        case XBUS_STRU_TELE_DIGITAL_AIR_ID:
            if (!sensor.is_enabled[XBUS_STRU_TELE_DIGITAL_AIR]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_stru_tele_digital_air_t), "0x%X ");
            break;
        case XBUS_TELE_LIPOMON_ID:
            if (!sensor.is_enabled[XBUS_TELE_LIPOMON]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_tele_lipomon_t), "0x%X ");
            break;
    }
}

void xbus_set_config(void) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (config->enable_pwm_out) {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            context.pwm_out_task_handle = task_handle;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor.esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor.is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);
    }
    if (config->esc_protocol == ESC_HW5) {
        esc_hw5_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw5_task, "esc_hw5_task", STACK_ESC_HW5, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor.esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);
    }
    if (config->esc_protocol == ESC_CASTLE) {
        esc_castle_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                             config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                             malloc(sizeof(float)),  malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor.esc[XBUS_ESC_CURRENT_BEC] = parameter.current_bec;
        sensor.esc[XBUS_ESC_VOLTAGE_BEC] = parameter.voltage_bec;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_KONTRONIK) {
        esc_kontronik_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage,  config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)),  malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_kontronik_task, "esc_kontronik_task", STACK_ESC_KONTRONIK, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor.esc[XBUS_ESC_TEMPERATURE_BEC] = parameter.temperature_bec;
        sensor.esc[XBUS_ESC_CURRENT_BEC] = parameter.current_bec;
        sensor.esc[XBUS_ESC_VOLTAGE_BEC] = parameter.voltage_bec;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor.battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor.battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor.is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_F) {
        esc_apd_f_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm,         config->alpha_voltage,
                                            config->alpha_current,  config->alpha_temperature, malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(float)),
                                            malloc(sizeof(float)),  malloc(sizeof(float)),     malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_f_task, "esc_apd_f_task", STACK_ESC_APD_F, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor.battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor.battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor.is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_HV) {
        esc_apd_hv_parameters_t parameter = {
            config->rpm_multiplier,    config->alpha_rpm,     config->alpha_voltage, config->alpha_current,
            config->alpha_temperature, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
            malloc(sizeof(float)),     malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_hv_task, "esc_apd_hv_task", STACK_ESC_APD_HV, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor->battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor->battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        xTaskCreate(esc_omp_m4_task, "esc_omp_m4_task", STACK_ESC_OMP_M4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temp_esc;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor.battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor.battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor.is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_ztw_task, "esc_ztw_task", STACK_ESC_ZTW, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temp_esc;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensor.battery[XBUS_BATTERY_CURRENT1] = parameter.current;
        // sensor.battery[XBUS_BATTERY_CONSUMPTION1] = parameter.consumption;
        // sensor.is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_OPENYGE) {
        esc_openyge_parameters_t parameter;
        parameter.rpm_multiplier = config->rpm_multiplier;
        parameter.pwm_out = config->enable_pwm_out;
        parameter.alpha_rpm = config->alpha_rpm;
        parameter.alpha_voltage = config->alpha_voltage;
        parameter.alpha_current = config->alpha_current;
        parameter.alpha_temperature = config->alpha_temperature;
        parameter.rpm = malloc(sizeof(float));
        parameter.voltage = malloc(sizeof(float));
        parameter.current = malloc(sizeof(float));
        parameter.temperature_fet = malloc(sizeof(float));
        parameter.temperature_bec = malloc(sizeof(float));
        parameter.cell_voltage = malloc(sizeof(float));
        parameter.consumption = malloc(sizeof(float));
        parameter.voltage_bec = malloc(sizeof(float));
        parameter.current_bec = malloc(sizeof(float));
        parameter.throttle = malloc(sizeof(float));
        parameter.pwm_percent = malloc(sizeof(float));
        parameter.cell_count = malloc(sizeof(uint8_t));
        xTaskCreate(esc_openyge_task, "esc_openyge_task", STACK_ESC_OPENYGE, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;

        sensor.esc[XBUS_ESC_RPM] = parameter.rpm;
        sensor.esc[XBUS_ESC_VOLTAGE] = parameter.voltage;
        sensor.esc[XBUS_ESC_CURRENT] = parameter.current;
        sensor.esc[XBUS_ESC_TEMPERATURE_FET] = parameter.temperature_fet;
        sensor.is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps) {
        gps_parameters_t parameter;
        parameter.protocol = config->gps_protocol;
        parameter.baudrate = config->gps_baudrate;
        parameter.rate = config->gps_rate;
        parameter.lat = malloc(sizeof(float));
        parameter.lon = malloc(sizeof(float));
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
        parameter.fix_type = malloc(sizeof(uint8_t));
        parameter.home_set = malloc(sizeof(uint8_t));
        xTaskCreate(gps_task, "gps_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;

        sensor.gps_loc = malloc(sizeof(xbus_sensor_gps_loc_t));
        sensor.gps_loc->altitude_low = parameter.alt;
        sensor.gps_loc->latitude = parameter.lat;
        sensor.gps_loc->longitude = parameter.lon;
        sensor.gps_loc->course = parameter.cog;
        sensor.gps_loc->hdop = parameter.hdop;
        sensor.gps_loc->fix_type = parameter.fix_type;
        sensor.gps_loc->home_set = parameter.home_set;
        sensor.gps_stat[XBUS_GPS_STAT_SPEED] = parameter.spd;
        sensor.gps_stat[XBUS_GPS_STAT_TIME] = parameter.time;
        sensor.gps_stat[XBUS_GPS_STAT_SATS] = parameter.sat;
        sensor.gps_stat[XBUS_GPS_STAT_ALTITUDE] = parameter.alt;

        sensor.is_enabled[XBUS_GPS_LOC] = true;
        sensor.is_enabled[XBUS_GPS_STAT] = true;
        i2c_multi_enable_address(XBUS_GPS_LOC_ID);
        i2c_multi_enable_address(XBUS_GPS_STAT_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        if (!config->xbus_use_alternative_volt_temp) {
            sensor.rpm_volt_temp[XBUS_RPMVOLTTEMP_VOLT] = parameter.voltage;
            sensor.is_enabled[XBUS_RPMVOLTTEMP] = true;
            i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);
        } else {
            sensor.energy[XBUS_ENERGY_VOLTAGE1] = parameter.voltage;
            sensor.is_enabled[XBUS_ENERGY] = true;
            i2c_multi_enable_address(XBUS_ENERGY_ID);
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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

        sensor.energy[XBUS_ENERGY_CURRENT1] = parameter.current;
        sensor.energy[XBUS_ENERGY_CONSUMPTION1] = parameter.consumption;
        if (!sensor.is_enabled[XBUS_ENERGY]) {
            sensor.is_enabled[XBUS_ENERGY] = true;
        }
        i2c_multi_enable_address(XBUS_ENERGY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        if (!config->xbus_use_alternative_volt_temp) {
            sensor.rpm_volt_temp[XBUS_RPMVOLTTEMP_TEMP] = parameter.ntc;
            sensor.is_enabled[XBUS_RPMVOLTTEMP] = true;
            i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);
        } else {
            sensor.battery[XBUS_BATTERY_TEMP1] = parameter.ntc;
            sensor.is_enabled[XBUS_BATTERY] = true;
            i2c_multi_enable_address(XBUS_BATTERY_ID);
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        add_alarm_in_ms(250, interval_250_callback, NULL, false);
        add_alarm_in_ms(500, interval_500_callback, NULL, false);
        add_alarm_in_ms(1000, interval_1000_callback, NULL, false);
        add_alarm_in_ms(1500, interval_1500_callback, NULL, false);
        add_alarm_in_ms(2000, interval_2000_callback, NULL, false);
        add_alarm_in_ms(3000, interval_3000_callback, NULL, false);
        sensor.vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor.is_enabled[XBUS_VARIO] = true;
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, 0,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        add_alarm_in_ms(250, interval_250_callback, NULL, false);
        add_alarm_in_ms(500, interval_500_callback, NULL, false);
        add_alarm_in_ms(1000, interval_1000_callback, NULL, false);
        add_alarm_in_ms(1500, interval_1500_callback, NULL, false);
        add_alarm_in_ms(2000, interval_2000_callback, NULL, false);
        add_alarm_in_ms(3000, interval_3000_callback, NULL, false);
        sensor.vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor.is_enabled[XBUS_VARIO] = true;
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        add_alarm_in_ms(250, interval_250_callback, NULL, false);
        add_alarm_in_ms(500, interval_500_callback, NULL, false);
        add_alarm_in_ms(1000, interval_1000_callback, NULL, false);
        add_alarm_in_ms(1500, interval_1500_callback, NULL, false);
        add_alarm_in_ms(2000, interval_2000_callback, NULL, false);
        add_alarm_in_ms(3000, interval_3000_callback, NULL, false);

        if (config->enable_analog_airspeed) {
            baro_temp = parameter.temperature;
            baro_pressure = parameter.pressure;
        }

        sensor.vario[XBUS_VARIO_ALTITUDE] = parameter.altitude;
        sensor.is_enabled[XBUS_VARIO] = true;
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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

        sensor.airspeed[XBUS_AIRSPEED_AIRSPEED] = parameter.airspeed;
        sensor.is_enabled[XBUS_AIRSPEED] = true;
        i2c_multi_enable_address(XBUS_AIRSPEED_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_flow) {
        fuel_meter_parameters_t parameter = {config->fuel_flow_ml_per_pulse, malloc(sizeof(float)),
                                             malloc(sizeof(float))};
        xTaskCreate(fuel_meter_task, "fuel_meter_task", STACK_FUEL_METER, (void *)&parameter, 2, &task_handle);

        sensor.fuel_flow[XBUS_FUEL_FLOW_RATE] = parameter.consumption_instant;
        sensor.fuel_flow[XBUS_FUEL_FLOW_CONSUMED] = parameter.consumption_total;
        sensor.is_enabled[XBUS_FUEL_FLOW] = true;
        i2c_multi_enable_address(XBUS_FUEL_FLOW_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_pressure) {
        xgzp68xxd_parameters_t parameter = {config->xgzp68xxd_k, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(xgzp68xxd_task, "fuel_pressure_task", STACK_FUEL_PRESSURE, (void *)&parameter, 2, &task_handle);

        sensor.stru_tele_digital_air[XBUS_DIGITAL_AIR_FUEL_PRESSURE] = parameter.pressure;
        sensor.is_enabled[XBUS_STRU_TELE_DIGITAL_AIR] = true;
        i2c_multi_enable_address(XBUS_STRU_TELE_DIGITAL_AIR_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->xbus_clock_stretch) {
        gpio_set_dir(CLOCK_STRETCH_GPIO, true);
        gpio_put(CLOCK_STRETCH_GPIO, false);
    }
    if (config->enable_gyro) {
        mpu6050_parameters_t parameter = {1,
                                          0,
                                          config->mpu6050_acc_scale,
                                          config->mpu6050_gyro_scale,
                                          config->mpu6050_gyro_weighting,
                                          config->mpu6050_filter,
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float)),
                                          malloc(sizeof(float))};
        xTaskCreate(mpu6050_task, "mpu6050_task", STACK_MPU6050, (void *)&parameter, 2, &task_handle);
        sensor.is_enabled[XBUS_TELE_G_METER] = true;
        sensor.tele_g_meter[XBUS_TELE_G_METER_X] = parameter.acc_x;
        sensor.tele_g_meter[XBUS_TELE_G_METER_Y] = parameter.acc_y;
        sensor.tele_g_meter[XBUS_TELE_G_METER_Z] = parameter.acc_z;
        i2c_multi_enable_address(XBUS_TELE_G_METER_ID);
        sensor.is_enabled[XBUS_TELE_GYRO] = true;
        sensor.tele_gyro[XBUS_TELE_GYRO_PITCH] = parameter.pitch;
        sensor.tele_gyro[XBUS_TELE_GYRO_ROLL] = parameter.roll;
        sensor.tele_gyro[XBUS_TELE_GYRO_YAW] = parameter.yaw;
        i2c_multi_enable_address(XBUS_TELE_GYRO_ID);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_lipo) {
        float *cell_prev = 0;
        sensor.is_enabled[XBUS_TELE_LIPOMON] = true;
        i2c_multi_enable_address(XBUS_TELE_LIPOMON_ID);
        if (config->lipo_cells > 0) {
            ina3221_parameters_t parameter = {
                .i2c_address = 0x40,
                .filter = config->ina3221_filter,
                .cell_count = MIN(config->lipo_cells, 3),
                .cell[0] = malloc(sizeof(float)),
                .cell[1] = malloc(sizeof(float)),
                .cell[2] = malloc(sizeof(float)),
                .cell_prev = malloc(sizeof(float)),
            };
            *parameter.cell_prev = 0;
            cell_prev = parameter.cell[2];
            xTaskCreate(ina3221_task, "ina3221_task", STACK_INA3221, (void *)&parameter, 2, &task_handle);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            for (uint8_t i = 0; i < parameter.cell_count; i++) {
                sensor.tele_lipomon[XBUS_TELE_LIPOMON_CELL1 + i] = parameter.cell[i];
            }
            for (uint8_t i = parameter.cell_count; i < 3; i++) {
                *sensor.tele_lipomon[XBUS_TELE_LIPOMON_CELL1 + i] = 0x7FFF;
            }
        }
        if (config->lipo_cells > 3) {
            ina3221_parameters_t parameter = {
                .i2c_address = 0x41,
                .filter = config->ina3221_filter,
                .cell_count = MIN(config->lipo_cells - 3, 3),
                .cell[0] = malloc(sizeof(float)),
                .cell[1] = malloc(sizeof(float)),
                .cell[2] = malloc(sizeof(float)),
                .cell_prev = malloc(sizeof(float)),
            };
            parameter.cell_prev = cell_prev;
            cell_prev = parameter.cell[2];
            xTaskCreate(ina3221_task, "ina3221_task", STACK_INA3221, (void *)&parameter, 2, &task_handle);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            for (uint8_t i = 0; i < parameter.cell_count; i++) {
                sensor.tele_lipomon[XBUS_TELE_LIPOMON_CELL4 + i] = parameter.cell[i];
            }
            for (uint8_t i = parameter.cell_count; i < 3; i++) {
                *sensor.tele_lipomon[XBUS_TELE_LIPOMON_CELL4 + i] = 0x7FFF;
            }
        }
    }
}

static uint8_t bcd8(float value, uint8_t precision) {
    char buf[10] = {0};
    uint8_t output = 0;
    for (int i = 0; i < precision; i++) value = value * 10;
    sprintf(buf, "%02i", (uint8_t)value);
    for (int i = 0; i < 2; i++) output |= (buf[i] - 48) << ((1 - i) * 4);
    return output;
}

static uint16_t bcd16(float value, uint8_t precision) {
    char buf[10] = {0};
    uint16_t output = 0;
    for (int i = 0; i < precision; i++) value = value * 10;
    sprintf(buf, "%04i", (uint16_t)value);
    for (int i = 0; i < 4; i++) output |= (uint16_t)(buf[i] - 48) << ((3 - i) * 4);
    return output;
}

static uint32_t bcd32(float value, uint8_t precision) {
    char buf[10] = {0};
    uint32_t output = 0;
    for (int i = 0; i < precision; i++) value = value * 10;
    sprintf(buf, "%08li", (uint32_t)value);
    for (int i = 0; i < 8; i++) output |= (uint32_t)(buf[i] - 48) << ((7 - i) * 4);
    return output;
}

static int64_t interval_250_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    delta_0250ms = swap_16((int16_t)(round(*sensor.vario[XBUS_VARIO_ALTITUDE] - prev) * 10));
    prev = *sensor.vario[XBUS_VARIO_ALTITUDE];
    return 250000L;
}

static int64_t interval_500_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    delta_0500ms = swap_16((int16_t)(round(*sensor.vario[XBUS_VARIO_ALTITUDE] - prev) * 10));
    prev = *sensor.vario[XBUS_VARIO_ALTITUDE];
    return 500000L;
}

static int64_t interval_1000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    delta_1000ms = swap_16((int16_t)round(*sensor.vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor.vario[XBUS_VARIO_ALTITUDE];
    return 1000000L;
}

static int64_t interval_1500_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    delta_1500ms = swap_16((int16_t)round(*sensor.vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor.vario[XBUS_VARIO_ALTITUDE];
    return 1500000L;
}

static int64_t interval_2000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    delta_2000ms = swap_16((int16_t)round(*sensor.vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor.vario[XBUS_VARIO_ALTITUDE];
    return 2000000L;
}

static int64_t interval_3000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    delta_3000ms = swap_16((int16_t)round(*sensor.vario[XBUS_VARIO_ALTITUDE] - prev));
    prev = *sensor.vario[XBUS_VARIO_ALTITUDE];
    return 3000000L;
}
