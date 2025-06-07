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
#include "esc_pwm.h"
#include "esc_ztw.h"
#include "fuel_meter.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_multi.h"
#include "ms5611.h"
#include "gps.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "stdlib.h"
#include "string.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
#include "xgzp68xxd.h"

volatile xbus_sensors_t *sensors;

static void i2c_request_handler(uint8_t address);
static uint8_t bcd8(float value, uint8_t precision);
static uint16_t bcd16(float value, uint8_t precision);
static uint32_t bcd32(float value, uint8_t precision);
static int64_t interval_250_callback(alarm_id_t id, void *parameters);
static int64_t interval_500_callback(alarm_id_t id, void *parameters);
static int64_t interval_1000_callback(alarm_id_t id, void *parameters);
static int64_t interval_1500_callback(alarm_id_t id, void *parameters);
static int64_t interval_2000_callback(alarm_id_t id, void *parameters);
static int64_t interval_3000_callback(alarm_id_t id, void *parameters);

void xbus_i2c_handler(uint8_t address) { i2c_request_handler(address); }

void xbus_task(void *parameters) {
    sensors = calloc(1, sizeof(xbus_sensors_t));

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
            static float max_airspeed = 0;
            xbus_airspeed_t *airspeed;
            airspeed = (xbus_airspeed_t *)buffer;
            airspeed->identifier = XBUS_AIRSPEED_ID;
            airspeed->airspeed = swap_16((uint16_t)*sensors->airspeed.speed);
            if (*sensors->airspeed.speed > max_airspeed) max_airspeed = *sensors->airspeed.speed;
            airspeed->max_airspeed = swap_16((uint16_t)max_airspeed);
            break;
        }
        case XBUS_GPS_LOC_ID: {
            xbus_gps_loc_t *gps_loc;
            gps_loc = (xbus_gps_loc_t *)buffer;
            gps_loc->identifier = XBUS_GPS_LOC_ID;
            uint8_t gps_flags = 0;
            double lat = *sensors->gps_loc.latitude;
            if (lat < 0)  // N=1,+, S=0,-
                lat *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_NORTH_BIT;
            uint deg = lat;
            float min = (lat - deg) * 60;
            gps_loc->latitude = ((uint32_t)bcd8(deg, 0) << 24) | bcd32(min, 4);
            double lon = *sensors->gps_loc.longitude;
            if (lon < 0)  // E=1,+, W=0,-
                lon *= -1;
            else
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_IS_EAST_BIT;
            if (lon >= 6000) {
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_LONG_GREATER_99_BIT;
                lon -= 6000;
            }
            deg = lon;
            min = (lon - deg) * 60;
            gps_loc->longitude = ((uint32_t)bcd8(deg, 0) << 24) | bcd32(min, 4);
            gps_loc->course = bcd16(*sensors->gps_loc.course, 1);
            gps_loc->hdop = bcd8(*sensors->gps_loc.hdop, 1);
            alt = *sensors->gps_loc.altitude;
            if (alt < 0) {
                gps_flags |= 1 << XBUS_GPS_INFO_FLAGS_NEGATIVE_ALT_BIT;
                alt *= -1;
            }
            gps_loc->gps_flags = gps_flags;
            gps_loc->altitude_low = bcd16(fmod(alt, 1000), 1);
            break;
        }
        case XBUS_GPS_STAT_ID: {
            xbus_gps_stat_t *gps_stat;
            gps_stat = (xbus_gps_stat_t *)buffer;
            gps_stat->identifier = XBUS_GPS_STAT_ID;
            gps_stat->speed = bcd16(*sensors->gps_stat.speed, 1);
            gps_stat->utc = bcd32(*sensors->gps_stat.time, 0) << 8;
            gps_stat->num_sats = bcd8(*sensors->gps_stat.sats, 0);
            gps_stat->altitude_high = bcd8((uint8_t)(alt / 1000), 0);
            break;
        }
        case XBUS_ENERGY_ID: {
            xbus_energy_t *energy;
            energy = (xbus_energy_t *)buffer;
            energy->identifier = XBUS_ENERGY_ID;
            energy->current_a = swap_16((int16_t)(*sensors->energy.current1 * 100));
            energy->charge_used_a = swap_16((int16_t)(*sensors->energy.consumption1 * 10));
            energy->volts_a = swap_16((uint16_t)(*sensors->energy.voltage1 * 100));
            energy->current_b = swap_16((int16_t)(*sensors->energy.current2 * 10));
            energy->charge_used_b = swap_16((int16_t)(*sensors->energy.consumption2));
            energy->volts_b = swap_16((uint16_t)(*sensors->energy.voltage2 * 100));
            break;
        }
        case XBUS_ESC_ID: {
            xbus_esc_t *esc;
            esc = (xbus_esc_t *)buffer;
            esc->identifier = XBUS_ESC_ID;
            esc->rpm = swap_16((uint16_t)(*sensors->esc.rpm / 10));
            esc->volts_input = swap_16((uint16_t)(*sensors->esc.voltage * 100));
            esc->temp_fet = swap_16((uint16_t)(*sensors->esc.temp_fet * 10));
            esc->current_motor = swap_16((uint16_t)(*sensors->esc.current * 100));
            esc->temp_bec = swap_16((uint16_t)(*sensors->esc.temp_bec * 10));
            esc->current_bec = *sensors->esc.current_bec * 10;
            esc->voltage_bec = *sensors->esc.voltage_bec * 20;
            break;
        }
        case XBUS_BATTERY_ID: {
            xbus_battery_t *battery;
            battery = (xbus_battery_t *)buffer;
            battery->identifier = XBUS_BATTERY_ID;
            battery->current_a = swap_16((int16_t)(*sensors->battery.current1 * 10));
            battery->charge_used_a = swap_16((int16_t)(*sensors->battery.consumption1));
            battery->temp_a = swap_16((uint16_t)(*sensors->battery.temp1 * 10));
            battery->current_b = swap_16((int16_t)(*sensors->battery.current2 * 10));
            battery->charge_used_b = swap_16((int16_t)(*sensors->battery.consumption2));
            battery->temp_b = swap_16((uint16_t)(*sensors->battery.temp2 * 10));
            break;
        }
        case XBUS_VARIO_ID: {
            xbus_vario_t *vario;
            vario = (xbus_vario_t *)buffer;
            vario->identifier = XBUS_BATTERY_ID;
            float altitude = *sensors->vario.altitude;
            vario->altitude = swap_16((int16_t)(altitude * 10));
#ifdef SIM_SENSORS
            vario->delta_0250ms = swap_16((int16_t)(-10));
            vario->delta_0500ms = swap_16((int16_t)(20));
            vario->delta_1000ms = swap_16((int16_t)(-12.32));
            vario->delta_1500ms = swap_16((int16_t)(15));
            vario->delta_2000ms = swap_16((int16_t)(20));
            vario->delta_3000ms = swap_16((int16_t)(-300));
#endif
            break;
        }
        case XBUS_RPMVOLTTEMP_ID: {
            xbus_rpm_volt_temp_t *rpm_volt_temp;
            rpm_volt_temp = (xbus_rpm_volt_temp_t *)buffer;
            rpm_volt_temp->identifier = XBUS_RPMVOLTTEMP_ID;
            rpm_volt_temp->volts = swap_16((uint16_t)(*sensors->rpm_volt_temp.volt * 100));
            rpm_volt_temp->temperature = swap_16((int16_t)(*sensors->rpm_volt_temp.temp));
            break;
        }
        case XBUS_FUEL_FLOW_ID: {
            xbus_fuel_flow_t *fuel_flow;
            fuel_flow = (xbus_fuel_flow_t *)buffer;
            fuel_flow->identifier = XBUS_FUEL_FLOW_ID;
            fuel_flow->fuel_consumed_A = swap_16((uint16_t)(*sensors->fuel_flow.consumed * 10));
            fuel_flow->flow_rate_A = swap_16((uint16_t)(*sensors->fuel_flow.flow_rate * 10));
            break;
        }
        case XBUS_STRU_TELE_DIGITAL_AIR_ID: {
            xbus_stru_tele_digital_air_t *stru_tele_digital_air;
            stru_tele_digital_air = (xbus_stru_tele_digital_air_t *)buffer;
            stru_tele_digital_air->identifier = XBUS_STRU_TELE_DIGITAL_AIR_ID;
            if (sensors->stru_tele_digital_air.fuel_pressure)
                stru_tele_digital_air->pressure =
                    swap_16((uint16_t)(*sensors->stru_tele_digital_air.fuel_pressure * 0.000145038 *
                                       10));  // Pa to psi, precision 0.1 psi
            break;
        }
    }
}

void xbus_set_config(void) {
    config_t *config = config_read();
    TaskHandle_t task_handle;
    float *baro_temp = NULL, *baro_pressure = NULL;
    if (config->esc_protocol == ESC_PWM) {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensors->esc.rpm = parameter.rpm;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3) {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensors->esc.rpm = parameter.rpm;
        sensors->is_enabled[XBUS_ESC] = true;
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
                                          config->esc_hw4_divisor,
                                          config->esc_hw4_current_multiplier,
                                          config->esc_hw4_current_thresold,
                                          config->esc_hw4_current_max,
                                          config->esc_hw4_is_manual_offset,
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
        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temperature_fet;
        sensors->esc.temp_bec = parameter.temperature_bec;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temperature_fet;
        sensors->esc.temp_bec = parameter.temperature_bec;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temperature;
        sensors->esc.current_bec = parameter.current_bec;
        sensors->esc.voltage_bec = parameter.voltage_bec;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temperature_fet;
        sensors->esc.temp_bec = parameter.temperature_bec;
        sensors->esc.current_bec = parameter.current_bec;
        sensors->esc.voltage_bec = parameter.voltage_bec;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temperature;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temperature;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temp_esc;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
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
        xTaskCreate(esc_ztw_task, "esc_ztw_task", STACK_ESC_OMP_M4, (void *)&parameter, 2, &task_handle);
        context.uart1_notify_task_handle = task_handle;
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->esc.rpm = parameter.rpm;
        sensors->esc.voltage = parameter.voltage;
        sensors->esc.current = parameter.current;
        sensors->esc.temp_fet = parameter.temp_esc;
        sensors->is_enabled[XBUS_ESC] = true;
        i2c_multi_enable_address(XBUS_ESC_ID);
        // sensors->battery.current1 = parameter.current;
        // sensors->battery.consumption1 = parameter.consumption;
        // sensors->is_enabled[XBUS_BATTERY] = true;
        // sensor_formatted->battery = malloc(sizeof(xbus_battery_t));
        //*sensor_formatted->battery = (xbus_battery_t){XBUS_BATTERY_ID, 0, 0, 0, 0, 0, 0};
        // i2c_multi_enable_address(XBUS_BATTERY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        parameter.dist = malloc(sizeof(float));
        xTaskCreate(gps_task, "gps_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        context.uart_pio_notify_task_handle = task_handle;

        sensors->gps_loc.altitude = parameter.alt;
        sensors->gps_loc.latitude = parameter.lat;
        sensors->gps_loc.longitude = parameter.lon;
        sensors->gps_loc.course = parameter.cog;
        sensors->gps_loc.hdop = parameter.hdop;
        sensors->gps_stat.speed = parameter.spd;
        sensors->gps_stat.time = parameter.time;
        sensors->gps_stat.sats = parameter.sat;
        sensors->gps_stat.altitude = parameter.alt;
        sensors->is_enabled[XBUS_GPS_LOC] = true;
        sensors->is_enabled[XBUS_GPS_STAT] = true;
        i2c_multi_enable_address(XBUS_GPS_LOC_ID);
        i2c_multi_enable_address(XBUS_GPS_STAT_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (false/*config->enable_analog_voltage*/) {
        voltage_parameters_t parameter = {0, config->analog_rate, config->alpha_voltage,
                                          config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        if (!config->xbus_use_alternative_volt_temp) {
            sensors->rpm_volt_temp.volt = parameter.voltage;
            sensors->is_enabled[XBUS_RPMVOLTTEMP] = true;
            i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);
        } else {
            sensors->energy.voltage1 = parameter.voltage;
            sensors->is_enabled[XBUS_ENERGY] = true;
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
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->energy.current1 = parameter.current;
        sensors->energy.consumption1 = parameter.consumption;
        if (!sensors->is_enabled[XBUS_ENERGY]) {
            sensors->is_enabled[XBUS_ENERGY] = true;
        }
        i2c_multi_enable_address(XBUS_ENERGY_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc) {
        ntc_parameters_t parameter = {2, config->analog_rate, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
        if (!config->xbus_use_alternative_volt_temp) {
            sensors->rpm_volt_temp.temp = parameter.ntc;
            sensors->is_enabled[XBUS_RPMVOLTTEMP] = true;
            i2c_multi_enable_address(XBUS_RPMVOLTTEMP_ID);
        } else {
            sensors->battery.temp1 = parameter.ntc;
            sensors->is_enabled[XBUS_BATTERY] = true;
            i2c_multi_enable_address(XBUS_BATTERY_ID);
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280) {
        bmp280_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         config->bmp280_filter, malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

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
        sensors->vario.altitude = parameter.altitude;
        sensors->is_enabled[XBUS_VARIO] = true;
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611) {
        ms5611_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

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
        sensors->vario.altitude = parameter.altitude;
        sensors->is_enabled[XBUS_VARIO] = true;
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180) {
        bmp180_parameters_t parameter = {config->alpha_vario,   config->vario_auto_offset, config->i2c_address,
                                         malloc(sizeof(float)), malloc(sizeof(float)),     malloc(sizeof(float)),
                                         malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);
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

        sensors->vario.altitude = parameter.altitude;
        sensors->is_enabled[XBUS_VARIO] = true;
        i2c_multi_enable_address(XBUS_VARIO_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed) {
        airspeed_parameters_t parameter = {3,
                                           config->analog_rate,
                                           config->alpha_airspeed,
                                           (float)config->airspeed_offset / 100,
                                           (float)config->airspeed_slope / 100,
                                           baro_temp,
                                           baro_pressure,
                                           malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->airspeed.speed = parameter.airspeed;
        sensors->is_enabled[XBUS_AIRSPEED] = true;
        i2c_multi_enable_address(XBUS_AIRSPEED_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_flow) {
        fuel_meter_parameters_t parameter = {config->fuel_flow_ml_per_pulse, malloc(sizeof(float)),
                                             malloc(sizeof(float))};
        xTaskCreate(fuel_meter_task, "fuel_meter_task", STACK_FUEL_METER, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->fuel_flow.flow_rate = parameter.consumption_instant;
        sensors->fuel_flow.consumed = parameter.consumption_total;
        sensors->is_enabled[XBUS_FUEL_FLOW] = true;
        i2c_multi_enable_address(XBUS_FUEL_FLOW_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_fuel_pressure) {
        xgzp68xxd_parameters_t parameter = {config->xgzp68xxd_k, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(xgzp68xxd_task, "fuel_pressure_task", STACK_FUEL_PRESSURE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

        sensors->stru_tele_digital_air.fuel_pressure = parameter.pressure;
        sensors->is_enabled[XBUS_STRU_TELE_DIGITAL_AIR] = true;
        i2c_multi_enable_address(XBUS_STRU_TELE_DIGITAL_AIR_ID);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    if (config->xbus_clock_stretch) {
        gpio_set_dir(CLOCK_STRETCH_GPIO, true);
        gpio_put(CLOCK_STRETCH_GPIO, false);
    }
}

static void i2c_request_handler(uint8_t address) {
    uint8_t buffer[16] = {0};
    debug("\nXBUS (%u) Address: %X Packet: ", uxTaskGetStackHighWaterMark(context.receiver_task_handle), address);
    switch (address) {
        case XBUS_AIRSPEED_ID:
            if (!sensors->is_enabled[XBUS_AIRSPEED]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_airspeed_t), "0x%X ");
            break;
        case XBUS_ALTIMETER_ID:
            if (!sensors->is_enabled[XBUS_ALTIMETER]) break;
            break;
        case XBUS_GPS_LOC_ID:
            if (!sensors->is_enabled[XBUS_GPS_LOC]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_gps_loc_t), "0x%X ");
            break;
        case XBUS_GPS_STAT_ID:
            if (!sensors->is_enabled[XBUS_GPS_STAT]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_gps_stat_t), "0x%X ");
            break;
        case XBUS_ENERGY_ID:
            if (!sensors->is_enabled[XBUS_ENERGY]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_energy_t), "0x%X ");
            break;
        case XBUS_ESC_ID:
            if (!sensors->is_enabled[XBUS_ESC]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_esc_t), "0x%X ");
            break;
        case XBUS_BATTERY_ID:
            if (!sensors->is_enabled[XBUS_BATTERY]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_battery_t), "0x%X ");
            break;
        case XBUS_VARIO_ID:
            if (!sensors->is_enabled[XBUS_VARIO]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_vario_t), "0x%X ");
            break;
        case XBUS_RPMVOLTTEMP_ID:
            if (!sensors->is_enabled[XBUS_RPMVOLTTEMP]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_rpm_volt_temp_t), "0x%X ");
            break;
        case XBUS_FUEL_FLOW_ID:
            if (!sensors->is_enabled[XBUS_FUEL_FLOW]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_fuel_flow_t), "0x%X ");
            break;
        case XBUS_STRU_TELE_DIGITAL_AIR_ID:
            if (!sensors->is_enabled[XBUS_STRU_TELE_DIGITAL_AIR]) break;
            xbus_format_sensor(address, buffer);
            i2c_multi_set_write_buffer(buffer);
            vTaskResume(context.led_task_handle);
            debug_buffer(buffer, sizeof(xbus_stru_tele_digital_air_t), "0x%X ");
            break;
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
    sensors->vario.delta_0250ms = swap_16((int16_t)(round(*sensors->vario.altitude - prev) * 10));
    prev = *sensors->vario.altitude;
    return 250000L;
}

static int64_t interval_500_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensors->vario.delta_0500ms = swap_16((int16_t)(round(*sensors->vario.altitude - prev) * 10));
    prev = *sensors->vario.altitude;
    return 500000L;
}

static int64_t interval_1000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensors->vario.delta_1000ms = swap_16((int16_t)round(*sensors->vario.altitude - prev));
    prev = *sensors->vario.altitude;
    return 1000000L;
}

static int64_t interval_1500_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensors->vario.delta_1500ms = swap_16((int16_t)round(*sensors->vario.altitude - prev));
    prev = *sensors->vario.altitude;
    return 1500000L;
}

static int64_t interval_2000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensors->vario.delta_2000ms = swap_16((int16_t)round(*sensors->vario.altitude - prev));
    prev = *sensors->vario.altitude;
    return 2000000L;
}

static int64_t interval_3000_callback(alarm_id_t id, void *parameters) {
    static float prev = 0;
    sensors->vario.delta_3000ms = swap_16((int16_t)round(*sensors->vario.altitude - prev));
    prev = *sensors->vario.altitude;
    return 3000000L;
}
