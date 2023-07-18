#include "jetiex.h"

static void process(uint *baudrate, sensor_jetiex_t **sensor);
static void send_packet(uint8_t packet_id, sensor_jetiex_t **sensor);
static uint8_t create_telemetry_buffer(uint8_t *buffer, bool packet_type, sensor_jetiex_t **sensor);
static bool add_sensor_text(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor);
static bool add_sensor_value(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor);
static void add_sensor(sensor_jetiex_t *new_sensor, sensor_jetiex_t **sensor);
static int64_t timeout_callback(alarm_id_t id, void *parameters);
static uint8_t crc8(uint8_t *crc, uint8_t crc_length);
static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
static uint16_t crc16(uint8_t *p, uint16_t len);
static uint16_t update_crc16(uint16_t crc, uint8_t data);
static void set_config(sensor_jetiex_t **sensor);

void jetiex_task(void *parameters)
{
    uint baudrate = 125000L;
    sensor_jetiex_t *sensor[32] = {NULL};
    led_cycle_duration = 6;
    led_cycles = 1;
    uart0_begin(baudrate, UART_RECEIVER_TX, UART_RECEIVER_RX, JETIEX_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    set_config(sensor);
    if (debug)
        printf("\nJeti Ex init");
    while (1)
    {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&baudrate, sensor);
    }
}

static void process(uint *baudrate, sensor_jetiex_t **sensor)
{
    static alarm_id_t timeout_alarm_id = 0;
    static bool mute = true;
    uint8_t packetId;
    uint8_t length = uart0_available();
    if (length)
    {
        uint8_t data[length];
        uart0_read_bytes(data, length);
        if (debug == 2)
        {
            printf("\nJeti Ex(%u) < ", uxTaskGetStackHighWaterMark(NULL));
            for (uint8_t i = 0; i < length; i++)
            {
                printf("%X ", data[i]);
            }
        }
        uint8_t packet[JETIEX_PACKET_LENGHT];
        if (data[0] == 0x3E && data[1] == 0x3 && length - data[2] == JETIEX_PACKET_LENGHT)
        {
            memcpy(packet, data + data[2], JETIEX_PACKET_LENGHT);
            if (debug)
            {
                printf("\nJeti Ex(%u) < ", uxTaskGetStackHighWaterMark(NULL));
                for (uint8_t i = 0; i < JETIEX_PACKET_LENGHT; i++)
                {
                    printf("%X ", packet[i]);
                }
            }
        }
        else if (length == JETIEX_PACKET_LENGHT)
        {
            memcpy(packet, data, JETIEX_PACKET_LENGHT);
        }
        else
        {
            return;
        }
        if (crc16(packet, JETIEX_PACKET_LENGHT) == 0)
        {
            if (packet[0] == 0x3D && packet[1] == 0x01 && packet[4] == 0x3A)
            {
                if (!mute)
                {
                    if (timeout_alarm_id)
                        cancel_alarm(timeout_alarm_id);
                    uint8_t packet_id = packet[3];
                    send_packet(packet_id, sensor);
                    timeout_alarm_id = add_alarm_in_ms(JETIEX_BAUDRATE_TIMEOUT_MS, timeout_callback, &baudrate, false);
                }
                mute = !mute;
            }
        }
    }
}

static void send_packet(uint8_t packet_id, sensor_jetiex_t **sensor)
{
    static uint8_t packet_count = 0;
    uint8_t ex_buffer[36] = {0};
    uint8_t length_telemetry_buffer = create_telemetry_buffer(ex_buffer + 6, packet_count % 16, sensor);
    ex_buffer[0] = 0x3B;
    ex_buffer[1] = 0x01;
    ex_buffer[2] = length_telemetry_buffer + 8;
    ex_buffer[3] = packet_id;
    ex_buffer[4] = 0x3A;
    ex_buffer[5] = length_telemetry_buffer;
    uint16_t crc = crc16(ex_buffer, length_telemetry_buffer + 6);
    ex_buffer[length_telemetry_buffer + 6] = crc;
    ex_buffer[length_telemetry_buffer + 7] = crc >> 8;
    uart0_write_bytes(ex_buffer, length_telemetry_buffer + 8);
    if (debug)
    {
        char type[7];
        if (packet_count % 16)
            strcpy(type, "Values");

        else
            strcpy(type, "Text");
        printf("\nJeti Ex %s (%u) > ", type, uxTaskGetStackHighWaterMark(NULL));
        for (uint8_t i = 0; i < length_telemetry_buffer + 8; i++)
        {
            printf("%X ", ex_buffer[i]);
        }
    }
    packet_count++;

    // blink led
    vTaskResume(led_task_handle);
}

static uint8_t create_telemetry_buffer(uint8_t *buffer, bool packet_type, sensor_jetiex_t **sensor)
{
    static uint8_t sensor_index_value = 0;
    static uint8_t sensor_index_text = 0;
    uint8_t buffer_index = 7;
    if (sensor[0] == NULL)
        return 0;
    if (packet_type)
    {
        while (add_sensor_value(buffer, &buffer_index, sensor_index_value + 1, sensor[sensor_index_value]) && sensor[sensor_index_value] != NULL)
        {
            sensor_index_value++;
        }
        if (sensor[sensor_index_value] == NULL)
            sensor_index_value = 0;
        buffer[1] = 0x40;
    }
    else
    {
        /*while*/ (add_sensor_text(buffer, &buffer_index, sensor_index_text + 1, sensor[sensor_index_text]) && sensor[sensor_index_text] != NULL);
        sensor_index_text++;
        if (sensor[sensor_index_text] == NULL)
            sensor_index_text = 0;
    }
    buffer[0] = 0x0F;
    buffer[1] |= buffer_index - 1;
    buffer[2] = JETIEX_MFG_ID_LOW;
    buffer[3] = JETIEX_MFG_ID_HIGH;
    buffer[4] = JETIEX_DEV_ID_LOW;
    buffer[5] = JETIEX_DEV_ID_HIGH;
    buffer[6] = 0x00;
    buffer[buffer_index] = crc8(buffer + 1, buffer_index - 1);
    return buffer_index + 1;
}

static bool add_sensor_value(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor)
{
    if (sensor)
    {
        uint8_t format = sensor->format << 5;
        if (sensor->type == JETIEX_TYPE_INT6)
        {
            if (*buffer_index > 25) // 29 bytes max:  25+2=pos27=byte28 +1crc=byte29
                return false;
            else
            {
                int8_t value = *sensor->value * pow(10, sensor->format);
                if (value > 0x1F)
                    value = 0x1F;
                else if (value < -0x1F)
                    value = -0x1F;
                value &= ~(3 << 5);
                value |= format;
                *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
                *(buffer + *buffer_index + 1) = value;
                *buffer_index += 2;
            }
        }
        else if (sensor->type == JETIEX_TYPE_INT14)
        {
            if (*buffer_index > 24)
                return false;
            else
            {
                int16_t value = *sensor->value * pow(10, sensor->format);
                if (value > 0x1FFF)
                    value = 0x1FFF;
                if (value < -0x1FFF)
                    value = -0x1FFF;
                value &= ~((uint16_t)3 << (5 + 8));
                value |= (uint16_t)format << 8;
                *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
                *(buffer + *buffer_index + 1) = value;
                *(buffer + *buffer_index + 2) = value >> 8;
                *buffer_index += 3;
            }
        }
        else if (sensor->type == JETIEX_TYPE_INT22)
        {
            if (*buffer_index > 23)
                return false;
            else
            {
                int32_t value = *sensor->value * pow(10, sensor->format);
                if (value > 0x1FFFFF)
                    value = 0x1FFFFF;
                else if (value < -0x1FFFFF)
                    value = -0x1FFFFF;
                value &= ~((uint32_t)3 << (5 + 16));
                value |= (uint32_t)format << 16;
                *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
                *(buffer + *buffer_index + 1) = value;
                *(buffer + *buffer_index + 2) = value >> 8;
                *(buffer + *buffer_index + 3) = value >> 16;
                *buffer_index += 4;
            }
        }
        else if (sensor->type == JETIEX_TYPE_INT30)
        {
            if (*buffer_index > 22)
                return false;
            else
            {
                int32_t value = *sensor->value * pow(10, sensor->format);
                if (value > 0x1FFFFFFF)
                    value = 0x1FFFFFFF;
                else if (value < -0x1FFFFFFF)
                    value = -0x1FFFFFFF;
                value &= ~((uint32_t)3 << (5 + 24));
                value |= (uint32_t)format << 24;
                *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
                *(buffer + *buffer_index + 1) = value;
                *(buffer + *buffer_index + 2) = value >> 8;
                *(buffer + *buffer_index + 3) = value >> 16;
                *(buffer + *buffer_index + 4) = value >> 24;
                *buffer_index += 5;
            }
        }
        else if (sensor->type == JETIEX_TYPE_TIMEDATE)
        {
            if (*buffer_index > 23)
                return false;
            else
            {
                // rawvalue: yymmdd/hhmmss
                // byte 1: day/second
                // byte 2: month/minute
                // byte 3(bits 1-5): year/hour
                // byte 3(bit 6): 0=time 1=date
                uint32_t value = *sensor->value;
                uint8_t hourYearFormat = format;
                hourYearFormat |= value / 10000;                             // hour, year
                uint8_t minuteMonth = (value / 100 - (value / 10000) * 100); // minute, month
                uint8_t secondDay = value - (value / 100) * 100;             // second, day
                *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
                *(buffer + *buffer_index + 1) = secondDay;
                *(buffer + *buffer_index + 2) = minuteMonth;
                *(buffer + *buffer_index + 3) = hourYearFormat;
                *buffer_index += 4;
            }
        }
        else if (sensor->type == JETIEX_TYPE_COORDINATES)
        {
            if (*buffer_index > 22)
                return false;
            else
            {
                // rawvalue: minutes
                // byte 1-2: degrees (decimals)
                // byte 3: degrees (integer)
                // byte 4(bit 6): 0=lat 1=lon
                // byte 4(bit 7): 0=+(N,E), 1=-(S,W)
                uint32_t value = *sensor->value;
                if (value < 0)
                    format |= 1 << 6;
                *(buffer + *buffer_index) = sensor_index << 4 | sensor->type;
                uint8_t degrees = value / 60;
                uint16_t degreesDecimals = (value / 60 - degrees) * 10000;
                *(buffer + *buffer_index + 1) = degreesDecimals;      // degrees (dec, l)
                *(buffer + *buffer_index + 2) = degreesDecimals >> 8; // degrees (dec, h)
                *(buffer + *buffer_index + 3) = degrees;              // degrees (int)
                *(buffer + *buffer_index + 4) = format;               // format
                *buffer_index += 5;
            }
        }
    }
    else
        return false;
    //printf("[%i %s]", sensor_index, sensor->text);
    return true;
}

static bool add_sensor_text(uint8_t *buffer, uint8_t *buffer_index, uint8_t sensor_index, sensor_jetiex_t *sensor)
{
    if (sensor)
    {
        uint8_t lenText = strlen(sensor->text);
        uint8_t lenUnit = strlen(sensor->unit);

        if (*buffer_index + lenText + lenUnit + 2 < 28)
        {
            *(buffer + *buffer_index) = sensor_index;
            *(buffer + *buffer_index + 1) = lenText << 3 | lenUnit;
            *buffer_index += 2;
            strcpy((char *)buffer + *buffer_index, sensor->text);
            *buffer_index += lenText;
            strcpy((char *)buffer + *buffer_index, sensor->unit);
            *buffer_index += lenUnit;
            //printf("[%i %s]", sensor_index, sensor->text);
            return true;
        }
    }
    return false;
}

static void add_sensor(sensor_jetiex_t *new_sensor, sensor_jetiex_t **sensors)
{
    static uint8_t sensor_count = 0;
    if (sensor_count < 32)
    {
        sensors[sensor_count] = new_sensor;
        new_sensor->data_id = sensor_count;
        sensor_count++;
    }
}

static int64_t timeout_callback(alarm_id_t id, void *parameters)
{
    float *baudrate = (float *)parameters;
    if (*baudrate == 125000L)
        *baudrate = 250000L;
    else
        *baudrate = 125000L;
    uart0_begin(*baudrate, UART_RECEIVER_TX, UART_RECEIVER_RX, JETIEX_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    if (debug)
        printf("\nJeti Ex timeout. Baudrate");
    return 0;
}

static uint8_t crc8(uint8_t *crc, uint8_t crc_length)
{
    uint8_t crc_up = 0;
    uint8_t c;
    for (c = 0; c < crc_length; c++)
    {
        crc_up = update_crc8(crc[c], crc_up);
    }
    return crc_up;
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u;
    uint8_t i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
    {
        crc_u = (crc_u & 0x80) ? 0x07 ^ (crc_u << 1) : (crc_u << 1);
    }
    return crc_u;
}

static uint16_t crc16(uint8_t *p, uint16_t len)
{
    uint16_t crc16_data = 0;
    while (len--)
    {
        crc16_data = update_crc16(crc16_data, p[0]);
        p++;
    }
    return (crc16_data);
}

static uint16_t update_crc16(uint16_t crc, uint8_t data)
{
    uint16_t ret_val;
    data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
    data ^= data << 4;
    ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
    return ret_val;
}

static void set_config(sensor_jetiex_t **sensor)
{
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_jetiex_t *new_sensor;
    if (config->esc_protocol == ESC_PWM)
    {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3)
    {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW4)
    {
        esc_hw4_parameters_t parameter = {config->rpm_multiplier, config->enable_pwm_out,
                                          config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature, config->esc_hw4_divisor, config->esc_hw4_ampgain, config->esc_hw4_current_thresold, config->esc_hw4_current_max,
                                          malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw4_task, "esc_hw4_task", STACK_ESC_HW4, (void *)&parameter, 2, &task_handle);
        //uart1_notify_task_handle = task_handle;
        uart_pio_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (config->enable_pwm_out)
        {
            xTaskCreate(pwm_out_task, "pwm_out", STACK_PWM_OUT, (void *)parameter.rpm, 2, &task_handle);
            pwm_out_task_handle = task_handle;
            xQueueSendToBack(tasks_queue_handle, task_handle, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp FET", "C", parameter.temperature_fet};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp BEC", "C", parameter.temperature_bec};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Cell Voltage", "V", parameter.cell_voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Consumption", "mAh", parameter.consumption};
        add_sensor(new_sensor, sensor);
    }
    if (config->esc_protocol == ESC_CASTLE)
    {
        esc_castle_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_hw4_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temperature", "C", parameter.temperature};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Ripple Voltage BEC", "V", parameter.ripple_voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "BEC Voltage", "V", parameter.voltage_bec};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "BEC Current", "A", parameter.current_bec};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Cell Voltage", "V", parameter.cell_voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Consumption", "mAh", parameter.consumption};
        add_sensor(new_sensor, sensor);
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

        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Current BEC", "A", parameter.current_bec};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage BEC", "V", parameter.voltage_bec};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp FET", "C", parameter.temperature_fet};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp BEC", "C", parameter.temperature_bec};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Cell Voltage", "V", parameter.cell_voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Consumption", "mAh", parameter.consumption};
        add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp", "C", parameter.temperature};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Cell Voltage", "V", parameter.cell_voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Consumption", "mAh", parameter.consumption};
        add_sensor(new_sensor, sensor);
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
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "RPM", "RPM", parameter.rpm};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temp", "C", parameter.temperature};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Cell Voltage", "V", parameter.cell_voltage};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Consumption", "mAh", parameter.consumption};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps)
    {
        nmea_parameters_t parameter = {config->gps_baudrate,
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        uart_pio_notify_task_handle = task_handle;
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT6, JETIEX_FORMAT_0_DECIMAL, "Sats", "", parameter.sat};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_COORDINATES, JETIEX_FORMAT_LAT, "Latitude", "", parameter.lat};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_COORDINATES, JETIEX_FORMAT_LON, "Longitude", "", parameter.lon};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_1_DECIMAL, "Altitude", "m", parameter.alt};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        if (config->jeti_gps_speed_units_kmh)
            *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Speed", "km/h", parameter.spd_kmh};
        else
            *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Speed", "kts", parameter.spd};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "COG", "", parameter.cog};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Dist", "m", parameter.dist};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_TIMEDATE, JETIEX_FORMAT_TIME, "Time", "", parameter.time};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_TIMEDATE, JETIEX_FORMAT_DATE, "Date", "", parameter.date};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "HDOP", "", parameter.hdop};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage)
    {
        voltage_parameters_t parameter = {0, config->alpha_voltage, config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_2_DECIMAL, "Voltage", "V", parameter.voltage};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current)
    {
        current_parameters_t parameter = {1, config->alpha_current, config->analog_current_multiplier, config->analog_current_offset, config->analog_current_autoffset, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Current", "A", parameter.current};
        add_sensor(new_sensor, sensor);
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Consumption", "mAh", parameter.consumption};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc)
    {
        ntc_parameters_t parameter = {2, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Temperature", "C", parameter.ntc};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed)
    {
        airspeed_parameters_t parameter = {3, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_1_DECIMAL, "Air speed", "km/h", parameter.airspeed};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280)
    {
        bmp280_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, config->bmp280_filter, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Air temperature", "C", parameter.temperature};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Altitude", "m", parameter.altitude};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611)
    {
        ms5611_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Air temperature", "C", parameter.temperature};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Altitude", "m", parameter.altitude};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180)
    {
        bmp180_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Air temperature", "C", parameter.temperature};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT14, JETIEX_FORMAT_0_DECIMAL, "Altitude", "m", parameter.altitude};
        add_sensor(new_sensor, sensor);
        new_sensor = malloc(sizeof(sensor_jetiex_t));
        *new_sensor = (sensor_jetiex_t){0, JETIEX_TYPE_INT22, JETIEX_FORMAT_0_DECIMAL, "Vspeed", "m/s", parameter.vspeed};
        add_sensor(new_sensor, sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
