#include "multiplex.h"

static void process(sensor_multiplex_t **sensor);
static void send_packet(uint8_t address, sensor_multiplex_t *sensor);
static int16_t format(uint8_t data_id, float value);
static void add_sensor(sensor_multiplex_t *new_sensor, sensor_multiplex_t **sensors);
static void set_config(sensor_multiplex_t **sensors);

void multiplex_task(void *parameters)
{
    
    sensor_multiplex_t *sensor[16] = {NULL};
    led_cycle_duration = 6;
    led_cycles = 1;
    uart0_begin(38400, UART_RECEIVER_TX, UART_RECEIVER_RX, MULTIPLEX_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    set_config(sensor);
    if (debug)
        printf("\nMultiplex init");
    while (1)
    {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(sensor);
    }
}

static void process(sensor_multiplex_t **sensor)
{
    uint8_t address = 0;
    if (uart0_available() == MULTIPLEX_PACKET_LENGHT)
    {
        uart0_read_bytes(&address, MULTIPLEX_PACKET_LENGHT);
        if (debug)
            printf("\nMultiplex (%u) < %X", uxTaskGetStackHighWaterMark(NULL), address);

        if (address < 16)
        {
            send_packet(address, sensor[address]);
        }
    }
}

static void send_packet(uint8_t address, sensor_multiplex_t *sensor)
{
    if (!sensor)
        return;
    uint8_t sensor_id = address << 4 | sensor->data_id;
    uart0_write(sensor_id);
    int16_t value = format(sensor->data_id, *sensor->value);
    uart0_write_bytes((uint8_t *)&value, 2);

    vTaskResume(led_task_handle);

    if (debug)
        printf("\nMultiplex (%u) > %X %X", uxTaskGetStackHighWaterMark(NULL), sensor_id, value);
}

static void add_sensor(sensor_multiplex_t *new_sensor, sensor_multiplex_t **sensors)
{
    static uint8_t sensor_count = 0;
    if (sensor_count < 16)
    {
        sensors[sensor_count] = new_sensor;
        sensor_count++;
    }
}

static int16_t format(uint8_t data_id, float value)
{
    int16_t formatted;
    if (data_id == FHSS_VOLTAGE ||
        data_id == FHSS_CURRENT ||
        data_id == FHSS_VARIO ||
        data_id == FHSS_SPEED ||
        data_id == FHSS_TEMP ||
        data_id == FHSS_COURSE ||
        data_id == FHSS_DISTANCE)
        formatted = round(value * 10);
    else if (data_id == FHSS_RPM)
        formatted = round(value / 10);
    else
        formatted = round(value);
    if (formatted > 16383)
        formatted = 16383;
    if (formatted < -16383)
        formatted = -16383;
    bool isNegative = false;
    if (formatted < 0)
        isNegative = true;
    formatted <<= 1;
    if (isNegative)
        formatted |= 1 << 15;
    return formatted;
}

static void set_config(sensor_multiplex_t **sensors)
{
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_multiplex_t *new_sensor;
    if (config->esc_protocol == ESC_PWM)
    {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3)
    {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;

        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW4)
    {
        esc_hw4_parameters_t parameter = {config->rpm_multiplier, config->enable_pwm_out,
                                          config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature, config->esc_hw4_divisor, config->esc_hw4_ampgain, config->esc_hw4_current_thresold, config->esc_hw4_current_max,
                                          malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw4_task, "esc_hw4_task", STACK_ESC_HW4, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (config->enable_pwm_out)
        {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            pwm_out_task_handle = task_handle;
            xQueueSendToBack(tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature_fet};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
    }
    if (config->esc_protocol == ESC_CASTLE)
    {
        esc_castle_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_KONTRONIK)
    {
        esc_kontronik_parameters_t parameter = {config->rpm_multiplier,
                                                config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                                malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_kontronik_task, "esc_kontronik_task", STACK_ESC_KONTRONIK, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.voltage_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CURRENT, parameter.current_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature_fet};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature_bec};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_F)
    {
        esc_apd_f_parameters_t parameter = {config->rpm_multiplier,
                                            config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                            malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_f_task, "esc_apd_f_task", STACK_ESC_APD_F, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_APD_HV)
    {
        esc_apd_hv_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_apd_hv_task, "esc_apd_hv_task", STACK_ESC_APD_HV, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_RPM, parameter.rpm};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.cell_voltage};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps)
    {
        nmea_parameters_t parameter = {config->gps_baudrate,
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        uart_pio_notify_task_handle = task_handle;
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_ALTITUDE, parameter.alt};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_SPEED, parameter.spd};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_DISTANCE, parameter.dist};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage)
    {
        voltage_parameters_t parameter = {0, config->alpha_voltage, config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VOLTAGE, parameter.voltage};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current)
    {
        current_parameters_t parameter = {1, config->alpha_current, config->analog_current_multiplier, config->analog_current_offset, config->analog_current_autoffset, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_CURRENT, parameter.current};
        add_sensor(new_sensor, sensors);
        *new_sensor = (sensor_multiplex_t){FHSS_CONSUMPTION, parameter.consumption};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc)
    {
        ntc_parameters_t parameter = {2, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.ntc};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed)
    {
        airspeed_parameters_t parameter = {3, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_SPEED, parameter.airspeed};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280)
    {
        bmp280_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, config->bmp280_filter, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_ALTITUDE, parameter.altitude};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611)
    {
        ms5611_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_ALTITUDE, parameter.altitude};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180)
    {
        bmp180_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_TEMP, parameter.temperature};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_ALTITUDE, parameter.altitude};
        add_sensor(new_sensor, sensors);
        new_sensor = malloc(sizeof(sensor_multiplex_t));
        *new_sensor = (sensor_multiplex_t){FHSS_VARIO, parameter.vspeed};
        add_sensor(new_sensor, sensors);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
