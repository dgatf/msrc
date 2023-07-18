#include "smartport.h"

#define AIRCR_Register (*((volatile uint32_t *)(PPB_BASE + 0x0ED0C)))

static SemaphoreHandle_t semaphore_sensor = NULL;
static bool is_maintenance_mode = false;
static const uint8_t sensor_id_matrix[29] = {0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0xD, 0x8E, 0x2F, 0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7, 0x98, 0x39, 0xBA, 0x1B, 0x0};
static TaskHandle_t packet_task_handle;
static QueueHandle_t packet_queue_handle;

static void sensor_task(void *parameters);
static void sensor_void_task(void *parameters);
static void sensor_double_task(void *parameters);
static void sensor_coordinates_task(void *parameters);
static void sensor_datetime_task(void *parameters);
static void sensor_cell_task(void *parameters);
static void packet_task(void *parameters);
static void process(smartport_parameters_t *parameter);
static void process_packet(smartport_parameters_t *parameter, uint8_t type_id, uint16_t data_id, uint32_t value);
static void add_packet(uint8_t type_id, uint16_t data_id, uint32_t value);
static uint32_t format(uint16_t data_id, float value);
static uint32_t format_double(uint16_t data_id, float value_l, float value_h);
static uint32_t format_coordinate(coordinate_type_t type, float value);
static uint32_t format_datetime(uint8_t type, uint32_t value);
static uint32_t format_cell(uint8_t cell_index, float value);
static void send_packet(uint8_t type_id, uint16_t data_id, uint32_t value);
static void send_byte(uint8_t c, uint16_t *crcp);
static void set_config(smartport_parameters_t *parameter);
static uint8_t sensor_id_to_crc(uint8_t sensor_id);
static uint8_t sensor_crc_to_id(uint8_t sensor_id_crc);
static uint8_t get_crc(uint8_t *data);
static int64_t reboot_callback(alarm_id_t id, void *user_data);

void smartport_task(void *parameters)
{
    smartport_parameters_t parameter;
    led_cycle_duration = 6;
    led_cycles = 1;
    uart0_begin(57600, UART_RECEIVER_TX, UART_RECEIVER_RX, SMARTPORT_TIMEOUT_US, 8, 1, UART_PARITY_NONE, true);
    semaphore_sensor = xSemaphoreCreateBinary();
    xSemaphoreTake(semaphore_sensor, 0);
    set_config(&parameter);
    packet_queue_handle = xQueueCreate(32, sizeof(smartport_packet_t));
    xTaskCreate(packet_task, "packet_task", STACK_SMARTPORT_PACKET_TASK, (void *)&parameter.data_id, 3, &packet_task_handle);
    xQueueSendToBack(tasks_queue_handle, packet_task_handle, 0);
    // TaskHandle_t task_handle;
    // xTaskCreate(sensor_void_task, "sensor_void_task", STACK_SMARTPORT_SENSOR_VOID_TASK, NULL, 2, &task_handle);
    // xQueueSendToBack(tasks_queue_handle, task_handle, 0);
    if (debug)
        printf("\nSmartport init");
    while (1)
    {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(smartport_parameters_t *parameter)
{
    uint lenght = uart0_available();
    if (lenght)
    {
        uint8_t data[lenght];
        if (debug == 2)
        {
            if (lenght != 2)
            {
                printf("\n");
                for (int i = 0; i < lenght; i++)
                    printf("%X ", data[i]);
            }
        }
        uart0_read_bytes(data, lenght);
        if (data[0] == 0x7E && data[1] == sensor_id_to_crc(parameter->sensor_id))
        {
            if (lenght == SMARTPORT_PACKET_LENGHT)
            {
                if (debug)
                {
                    printf("\nSmartport (%u) < ", uxTaskGetStackHighWaterMark(NULL));
                    for (uint8_t i = 0; i < SMARTPORT_PACKET_LENGHT; i++)
                    {
                        printf("%X ", data[i]);
                    }
                }
                if (is_maintenance_mode && uxQueueMessagesWaiting(packet_queue_handle))
                {
                    xTaskNotifyGive(packet_task_handle);
                }
                else if (!is_maintenance_mode)
                {
                    xSemaphoreGive(semaphore_sensor);
                    vTaskDelay(4 / portTICK_PERIOD_MS);
                    xSemaphoreTake(semaphore_sensor, 0);
                }
            }
            else if (lenght >= 10)
            {
                uint8_t i;
                uint8_t delta = 0;
                for (i = 0; i < lenght; i++)
                {
                    data[i] = data[i + delta];
                    if (data[i] == 0x7D)
                    {
                        delta++;
                        data[i] = data[i + delta] ^ 0x20;
                    }
                }
                if (i == 10)
                {
                    uint8_t crc = get_crc(data);
                    if (crc == data[9] && data[2] != 0x00 && data[2] != 0x10)
                    {
                        uint8_t type_id = data[2];
                        uint16_t data_id = (uint16_t)data[4] << 8 | data[3];
                        uint value = (uint32_t)data[8] << 24 | (uint32_t)data[7] << 16 |
                                     (uint16_t)data[6] << 8 | data[5];
                        if (debug)
                        {
                            printf("\nSmartport. Received packet (%u) < ", uxTaskGetStackHighWaterMark(NULL));
                            for (uint8_t i = 0; i < 10; i++)
                            {
                                printf("%X ", data[i]);
                            }
                        }
                        process_packet(parameter, type_id, data_id, value);
                    }
                }
            }
        }
    }
}

static void process_packet(smartport_parameters_t *parameter, uint8_t type_id, uint16_t data_id, uint32_t value)
{
    // set maintenance mode on
    if (type_id == 0x21 && data_id == 0xFFFF && value == 0x80)
    {
        is_maintenance_mode = true;
        if (debug)
            printf("\nSmartport (%u). Maintenance mode ON ", uxTaskGetStackHighWaterMark(NULL));
        return;
    }

    // set maintenance mode off
    if (type_id == 0x20 && data_id == 0xFFFF && value == 0x80)
    {
        is_maintenance_mode = false;
        if (debug)
            printf("\nSmartport (%u). Maintenance mode OFF ", uxTaskGetStackHighWaterMark(NULL));
        return;
    }

    // send sensor id
    if (is_maintenance_mode && type_id == 0x30 && data_id == parameter->data_id && value == 0x01)
    {
        smartport_packet_t packet;
        packet.data = (parameter->sensor_id - 1) << 8 | 0x01;
        packet.type_id = 0x32;
        packet.data_id = parameter->data_id;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        if (debug)
            printf("\nSmartport (%u). Send sensor Id %i (0x%X)", uxTaskGetStackHighWaterMark(NULL), parameter->sensor_id, sensor_id_to_crc(parameter->sensor_id));
        return;
    }

    // change sensor id
    if (is_maintenance_mode && type_id == 0x31 && data_id == parameter->data_id && (uint8_t)value == 0x01)
    {
        uint8_t sensor_id = (value >> 8) + 1;
        config_t *config = config_read();
        config->smartport_sensor_id = sensor_id;
        config_write(config);
        if (debug)
            printf("\nSmartport (%u). Change sensor Id %i (0x%X)", uxTaskGetStackHighWaterMark(NULL), sensor_id, sensor_id_to_crc(sensor_id));
        return;
    }

    // send config
    if (is_maintenance_mode && type_id == 0x30 && data_id == parameter->data_id && value == 0)
    {
        uint32_t value = 0;
        smartport_packet_t packet;
        config_t *config = config_read();
        // packet 1
        value = 0xF1;
        value |= (uint32_t)VERSION_PATCH << 8;
        value |= (uint32_t)VERSION_MINOR << 16;
        value |= (uint32_t)VERSION_MAJOR << 24;
        packet.type_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.data = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        // packet 2
        value = 0xF2;
        value |= config->enable_analog_airspeed << 8;
        value |= config->enable_gps << 9;
        value |= config->enable_analog_voltage << 10;
        value |= config->enable_analog_current << 12;
        value |= config->enable_analog_ntc << 13;
        value |= config->enable_pwm_out << 15;
        value |= ((uint8_t)(config->refresh_rate_rpm / 100) & 0b1111) << 16;
        value |= ((uint8_t)(config->refresh_rate_voltage / 100) & 0b1111) << 20;
        value |= ((uint8_t)(config->refresh_rate_current / 100) & 0b1111) << 24;
        value |= ((uint8_t)(config->refresh_rate_temperature / 100) & 0b1111) << 28;
        packet.type_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.data = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        // packet 3
        value = 0xF3;
        value |= (ELEMENTS(config->alpha_rpm) & 0b1111) << 8;
        value |= (ELEMENTS(config->alpha_voltage) & 0b1111) << 12;
        value |= (ELEMENTS(config->alpha_current) & 0b1111) << 16;
        value |= (ELEMENTS(config->alpha_temperature) & 0b1111) << 20;
        value |= config->esc_protocol << 24;
        packet.type_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.data = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        // packet 4
        value = 0xF4;
        value |= config->i2c_module << 8;
        value |= config->i2c_address << 12;
        packet.type_id = 0x32;
        packet.data_id = parameter->data_id;
        packet.data = value;
        xQueueSendToBack(packet_queue_handle, &packet, 0);
        if (debug)
            printf("\nSmartport (%u). Send config...", uxTaskGetStackHighWaterMark(NULL));
        return;
    }

    // receive config
    if (is_maintenance_mode && type_id == 0x31 && data_id == parameter->data_id && ((uint8_t)(value) == 0xF1 || (uint8_t)(value) == 0xF2 || (uint8_t)(value) == 0xF3))
    {
        static config_t config;

        if ((uint8_t)value == 0xF1)
        {
            config_get(&config);
            config.enable_analog_airspeed = (value >> 8) & 1;
            config.enable_gps = (value >> 9) & 1;
            config.enable_analog_voltage = (value >> 10) & 1;
            config.enable_analog_current = (value >> 12) & 1;
            config.enable_analog_ntc = (value >> 13) & 1;
            config.enable_pwm_out = (value >> 15) & 1;
            config.refresh_rate_rpm = ((uint8_t)(value >> 16) & 0b1111) * 100;
            config.refresh_rate_voltage = ((uint8_t)(value >> 20) & 0b1111) * 100;
            config.refresh_rate_current = ((uint8_t)(value >> 24) & 0b1111) * 100;
            config.refresh_rate_temperature = ((uint8_t)(value >> 28) & 0b1111) * 100;
            if (debug)
                printf("\nSmartport (%u). Received config 1/3", uxTaskGetStackHighWaterMark(NULL));
        }
        else if ((uint8_t)value == 0xF2)
        {
            config.alpha_rpm = ALPHA((value >> 8) & 0b1111);
            config.alpha_voltage = ALPHA((value >> 12) & 0b1111);
            config.alpha_current = ALPHA((value >> 16) & 0b1111);
            config.alpha_temperature = ALPHA((value >> 20) & 0b1111);
            config.esc_protocol = (value >> 24) & 0b11111111;
            if (debug)
                printf("\nSmartport (%u). Received config 2/3", uxTaskGetStackHighWaterMark(NULL));
        }
        else if ((uint8_t)value == 0xF3)
        {
            smartport_packet_t packet;
            config.i2c_module = (value >> 8) & 0b1111;
            config.i2c_address = (value >> 12) & 0b11111111;
            config_write(&config);

            packet.type_id = 0x32;
            packet.data_id = parameter->data_id;
            packet.data = 0xFF;
            xQueueSendToBack(packet_queue_handle, &packet, 0);
            add_alarm_in_ms(6000, reboot_callback, NULL, true);
            if (debug)
                printf("\nSmartport (%u). Received config 3/3. Write config. Reboot...\n", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

static int64_t reboot_callback(alarm_id_t id, void *user_data)
{
    AIRCR_Register = 0x5FA0004;
}

static void sensor_task(void *parameters)
{
    smartport_sensor_parameters_t parameter = *(smartport_sensor_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    while (1)
    {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted = format(parameter.data_id, *parameter.value);
        if (debug)
            printf("\nSmartport. Sensor (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, parameter.data_id, data_formatted);
    }
}

static void sensor_void_task(void *parameters)
{
    while (1)
    {
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        if (debug == 2)
            printf("\nSmartport. Sensor void (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0, 0, 0);
    }
}

static void sensor_double_task(void *parameters)
{
    smartport_sensor_double_parameters_t parameter = *(smartport_sensor_double_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    while (1)
    {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted = format_double(parameter.data_id, *parameter.value_l, *parameter.value_h);
        if (debug)
            printf("\nSmartport. Sensor double (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, parameter.data_id, data_formatted);
    }
}

static void sensor_coordinates_task(void *parameters)
{
    smartport_sensor_coordinate_parameters_t parameter = *(smartport_sensor_coordinate_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    while (1)
    {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted;
        if (parameter.type == SMARTPORT_LATITUDE)
            data_formatted = format_coordinate(parameter.type, *parameter.latitude);
        else
            data_formatted = format_coordinate(parameter.type, *parameter.longitude);
        parameter.type = !parameter.type;
        if (debug)
            printf("\nSmartport. Sensor coordinates (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, GPS_LONG_LATI_FIRST_ID, data_formatted);
    }
}

static void sensor_datetime_task(void *parameters)
{
    smartport_sensor_datetime_parameters_t parameter = *(smartport_sensor_datetime_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    while (1)
    {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted;
        if (parameter.type == SMARTPORT_DATE)
            data_formatted = format_datetime(parameter.type, *parameter.date);
        else
            data_formatted = format_datetime(parameter.type, *parameter.time);
        parameter.type = !parameter.type;
        if (debug)
            printf("\nSmartport. Sensor datetime (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, GPS_TIME_DATE_FIRST_ID, data_formatted);
    }
}

static void sensor_cell_task(void *parameters)
{
    smartport_sensor_cell_parameters_t parameter = *(smartport_sensor_cell_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    uint8_t cell_index = 0;
    while (1)
    {
        vTaskDelay(parameter.rate / portTICK_PERIOD_MS);
        xSemaphoreTake(semaphore_sensor, portMAX_DELAY);
        uint32_t data_formatted = format_cell(cell_index, *parameter.cell_voltage);
        cell_index++;
        if (cell_index > *parameter.cell_count - 1)
            cell_index = 0;
        if (debug)
            printf("\nSmartport. Sensor cell (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(0x10, CELLS_FIRST_ID, data_formatted);
    }
}

static void packet_task(void *parameters)
{
    uint16_t data_id = *(uint16_t *)parameters;
    smartport_packet_t packet;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xQueueReceive(packet_queue_handle, &packet, 0);
        if (debug)
            printf("\nSmartport. Packet (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        send_packet(packet.type_id, packet.data_id, packet.data);
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}

static uint32_t format(uint16_t data_id, float value)
{
    if ((data_id >= GPS_SPEED_FIRST_ID && data_id <= GPS_SPEED_LAST_ID) ||
        (data_id >= RBOX_BATT1_FIRST_ID && data_id <= RBOX_BATT2_FIRST_ID))
        return round(value * 1000);

    if ((data_id >= ALT_FIRST_ID && data_id <= VARIO_LAST_ID) ||
        (data_id >= VFAS_FIRST_ID && data_id <= VFAS_LAST_ID) ||
        (data_id >= ACCX_FIRST_ID && data_id <= GPS_ALT_LAST_ID) ||
        (data_id >= GPS_COURS_FIRST_ID && data_id <= GPS_COURS_LAST_ID) ||
        (data_id >= A3_FIRST_ID && data_id <= A4_LAST_ID))
        return round(value * 100);

    if ((data_id >= CURR_FIRST_ID && data_id <= CURR_LAST_ID) ||
        (data_id >= AIR_SPEED_FIRST_ID && data_id <= AIR_SPEED_LAST_ID) ||
        data_id == A1_ID || data_id == A2_ID || data_id == RXBT_ID)
        return round(value * 10);

    return round(value);
}

static uint32_t format_double(uint16_t data_id, float value_l, float value_h)
{
    if ((data_id >= ESC_POWER_FIRST_ID && data_id <= ESC_POWER_LAST_ID) ||
        (data_id >= SBEC_POWER_FIRST_ID && data_id <= SBEC_POWER_LAST_ID))
        return (uint32_t)round(value_h * 100) << 16 | (uint16_t)round(value_l * 100);

    if (data_id >= ESC_RPM_CONS_FIRST_ID && data_id <= ESC_RPM_CONS_LAST_ID)
    {
        return (uint32_t)round(value_h) << 16 | (uint16_t)round((value_l) / 100);
    }

    return (uint16_t)round(value_h * 500) << 8 | (uint16_t)value_l;
}

static uint32_t format_coordinate(coordinate_type_t type, float value)
{
    uint32_t data = 0;
    if (value < 0)
        data |= (uint32_t)1 << 30;
    if (type == SMARTPORT_LONGITUDE)
    {
        data |= (uint32_t)1 << 31;
    }
    data |= (uint32_t)abs(round(value * 10000));
    return data;
}

static uint32_t format_datetime(uint8_t type, uint32_t value)
{
    uint8_t dayHour = value / 10000;
    uint8_t monthMin = value / 100 - dayHour * 100;
    uint8_t yearSec = value - (value / 100) * 100;
    if (type == SMARTPORT_DATE)
    {
        return (uint32_t)yearSec << 24 | (uint32_t)monthMin << 16 | dayHour << 8 | 0xFF;
    }
    return (uint32_t)dayHour << 24 | (uint32_t)monthMin << 16 | yearSec << 8;
}

static uint32_t format_cell(uint8_t cell_index, float value)
{
    return cell_index | (uint16_t)round(value * 500) << 8;
}

static void send_byte(uint8_t c, uint16_t *crcp)
{
    if (crcp != NULL)
    {
        uint16_t crc = *crcp;
        crc += c;
        crc += crc >> 8;
        crc &= 0x00FF;
        *crcp = crc;
    }
    if (c == 0x7D || c == 0x7E)
    {
        uart0_write(c);
        c ^= 0x20;
    }
    uart0_write(c);
    if (debug)
        printf("%X ", c);
}

static void send_packet(uint8_t type_id, uint16_t data_id, uint32_t value)
{
    uint16_t crc = 0;
    uint8_t *u8p;
    // type_id
    send_byte(type_id, &crc);
    // data_id
    u8p = (uint8_t *)&data_id;
    send_byte(u8p[0], &crc);
    send_byte(u8p[1], &crc);
    // value
    u8p = (uint8_t *)&value;
    send_byte(u8p[0], &crc);
    send_byte(u8p[1], &crc);
    send_byte(u8p[2], &crc);
    send_byte(u8p[3], &crc);
    // crc
    send_byte(0xFF - (uint8_t)crc, NULL);
    // blink
    vTaskResume(led_task_handle);
}

static void set_config(smartport_parameters_t *parameter)
{
    config_t *config = config_read();
    TaskHandle_t task_handle;
    parameter->sensor_id = config->smartport_sensor_id;
    parameter->data_id = config->smartport_data_id;
    if (config->esc_protocol == ESC_PWM)
    {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = NULL;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3)
    {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = NULL;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
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

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_CASTLE)
    {
        esc_castle_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID + 1;
        parameter_sensor_double.value_l = parameter.voltage_bec;
        parameter_sensor_double.value_h = parameter.current_bec;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature_fet;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID + 1;
        parameter_sensor.value = parameter.temperature_bec;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        smartport_sensor_double_parameters_t parameter_sensor_double;
        smartport_sensor_cell_parameters_t parameter_sensor_cell;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = parameter.rpm;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_rpm;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_double.data_id = ESC_POWER_FIRST_ID;
        parameter_sensor_double.value_l = parameter.voltage;
        parameter_sensor_double.value_h = parameter.current;
        parameter_sensor_double.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_double_task, "sensor_double_task", STACK_SENSOR_SMARTPORT_DOUBLE, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.temperature;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor_cell.cell_count = parameter.cell_count;
        parameter_sensor_cell.cell_voltage = parameter.cell_voltage;
        parameter_sensor_cell.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_cell_task, "sensor_cell_task", STACK_SENSOR_SMARTPORT_CELL, (void *)&parameter_sensor_cell, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps)
    {
        nmea_parameters_t parameter = {config->gps_baudrate,
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        uart_pio_notify_task_handle = task_handle;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_coordinate_parameters_t parameter_sensor_coordinate;
        parameter_sensor_coordinate.type = SMARTPORT_LATITUDE;
        parameter_sensor_coordinate.latitude = parameter.lat;
        parameter_sensor_coordinate.longitude = parameter.lon;
        parameter_sensor_coordinate.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_coordinates_task, "sensor_coordinates_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor_coordinate, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_datetime_parameters_t parameter_sensor_datetime;
        parameter_sensor_datetime.type = SMARTPORT_DATE;
        parameter_sensor_datetime.date = parameter.date;
        parameter_sensor_datetime.time = parameter.time;
        parameter_sensor_datetime.rate = 1000;
        xTaskCreate(sensor_datetime_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor_datetime, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = GPS_ALT_FIRST_ID;
        parameter_sensor.value = parameter.alt;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GPS_SPEED_FIRST_ID;
        parameter_sensor.value = parameter.spd;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = GPS_COURS_FIRST_ID;
        parameter_sensor.value = parameter.cog;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID + 1;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 3;
        parameter_sensor.value = parameter.sat;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = DIY_FIRST_ID + 4;
        parameter_sensor.value = parameter.dist;
        parameter_sensor.rate = config->refresh_rate_gps;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage)
    {
        voltage_parameters_t parameter = {0, config->alpha_voltage, config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = A3_FIRST_ID;
        parameter_sensor.value = parameter.voltage;
        parameter_sensor.rate = config->refresh_rate_voltage;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current)
    {
        current_parameters_t parameter = {1, config->alpha_current, config->analog_current_multiplier, config->analog_current_offset, config->analog_current_autoffset, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = CURR_FIRST_ID;
        parameter_sensor.value = parameter.current;
        parameter_sensor.rate = config->refresh_rate_current;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_FRSKY_D, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        smartport_sensor_double_parameters_t parameter_sensor_double;
        parameter_sensor_double.data_id = ESC_RPM_CONS_FIRST_ID;
        parameter_sensor_double.value_l = NULL;
        parameter_sensor_double.value_h = parameter.consumption;
        parameter_sensor_double.rate = config->refresh_rate_current;
        xTaskCreate(sensor_double_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor_double, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc)
    {
        ntc_parameters_t parameter = {2, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ESC_TEMPERATURE_FIRST_ID;
        parameter_sensor.value = parameter.ntc;
        parameter_sensor.rate = config->refresh_rate_temperature;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed)
    {
        airspeed_parameters_t parameter = {3, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = AIR_SPEED_FIRST_ID;
        parameter_sensor.value = parameter.airspeed;
        parameter_sensor.rate = config->refresh_rate_airspeed;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280)
    {
        bmp280_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, config->bmp280_filter, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611)
    {
        ms5611_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180)
    {
        bmp180_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        smartport_sensor_parameters_t parameter_sensor;
        parameter_sensor.data_id = ALT_FIRST_ID;
        parameter_sensor.value = parameter.altitude;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        parameter_sensor.data_id = VARIO_FIRST_ID;
        parameter_sensor.value = parameter.vspeed;
        parameter_sensor.rate = config->refresh_rate_vario;
        xTaskCreate(sensor_task, "sensor_task", STACK_SENSOR_SMARTPORT, (void *)&parameter_sensor, 3, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

static uint8_t sensor_id_to_crc(uint8_t sensor_id)
{
    if (sensor_id < 1 || sensor_id > 28)
    {
        return 0;
    }
    return sensor_id_matrix[sensor_id - 1];
}

static uint8_t sensor_crc_to_id(uint8_t sensor_id_crc)
{
    uint8_t cont = 0;
    while (sensor_id_crc != sensor_id_matrix[cont] && cont < 28)
    {
        cont++;
    }
    if (cont == 28)
        return 0;
    return cont + 1;
}

static uint8_t get_crc(uint8_t *data)
{
    uint16_t crc = 0;
    for (uint8_t i = 2; i < 9; i++)
    {
        crc += data[i];
        crc += crc >> 8;
        crc &= 0x00FF;
    }
    return 0xFF - (uint8_t)crc;
}
