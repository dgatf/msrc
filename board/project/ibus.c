#include "ibus.h"

static void process(sensor_ibus_t **sensor);
static void send_packet(uint8_t command, uint8_t address, sensor_ibus_t *sensor_ibus);
static void send_byte(uint8_t c, uint16_t *crc_p);
static int32_t format(uint8_t data_id, float value);
static bool check_crc(uint8_t *data);
static void add_sensor(sensor_ibus_t *new_sensor, sensor_ibus_t **sensor, uint16_t sensormask);
static void set_config(sensor_ibus_t **sensor, uint16_t sensormask);

void ibus_task(void *parameters)
{
    sensor_ibus_t *sensor[16] = {NULL};
    uint16_t sensor_mask = 0B1111111111111110;
    led_cycle_duration = 6;
    led_cycles = 1;
    uart0_begin(115200, UART_RECEIVER_TX, UART_RECEIVER_RX, IBUS_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    set_config(sensor, sensor_mask);
    if (debug)
        printf("\nIbus init");
    while (1)
    {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(sensor);
    }
}

static void process(sensor_ibus_t **sensor)
{
    if (uart0_available() == IBUS_PACKET_LENGHT)
    {
        uint8_t command = 0;
        uint8_t address = 0;
        uint8_t data[IBUS_PACKET_LENGHT];
        uart0_read_bytes(data, IBUS_PACKET_LENGHT);
        if (data[0] == IBUS_PACKET_LENGHT)
        {
            if (debug)
            {
                printf("\nIbus (%u) < ", uxTaskGetStackHighWaterMark(NULL));
                for (uint8_t i = 0; i < data[0]; i++)
                {
                    printf("%X ", data[i]);
                }
            }
            if (check_crc(data))
            {
                command = data[1] >> 4;
                address = data[1] & 0x0F;
                if (!sensor[address])
                    return;
                if (command == IBUS_COMMAND_DISCOVER || command == IBUS_COMMAND_TYPE || IBUS_COMMAND_MEASURE)
                    send_packet(command, address, sensor[address]);
            }
        }
    }
}

static void send_packet(uint8_t command, uint8_t address, sensor_ibus_t *sensor)
{
    uint8_t *u8_p = NULL;
    uint16_t crc = 0;
    uint16_t type;
    uint8_t lenght = 0;
    int32_t value_formatted = 0;

    switch (sensor->type)
    {
    case IBUS_TYPE_S16:
    case IBUS_TYPE_U16:
        lenght = 2;
        break;
    case IBUS_TYPE_S32:
        lenght = 4;
        break;
    case IBUS_TYPE_GPS:
        lenght = 14;
        break;
    }
    switch (command)
    {
    case IBUS_COMMAND_DISCOVER:
        lenght = 0;
        break;
    case IBUS_COMMAND_TYPE:
        type = lenght << 8 | sensor->data_id;
        u8_p = (uint8_t *)&type;
        lenght = 2;
        break;
    case IBUS_COMMAND_MEASURE:
        if (sensor->value)
        {
            value_formatted = format(sensor->data_id, *sensor->value);
        }
        u8_p = (uint8_t *)&value_formatted;
        break;
    }
    if (debug)
        printf("\nIbus (%u) > ", uxTaskGetStackHighWaterMark(NULL));
    // lenght
    send_byte(4 + lenght, &crc);

    // command & address
    send_byte(command << 4 | address, &crc);

    // value
    for (uint8_t i = 0; i < lenght; i++)
    {
        send_byte(u8_p[i], &crc);
    }

    // crc
    crc = 0xFFFF - crc;
    u8_p = (uint8_t *)&crc;
    send_byte(u8_p[0], NULL);
    send_byte(u8_p[1], NULL);

    // blink led
    vTaskResume(led_task_handle);
}

static void send_byte(uint8_t c, uint16_t *crc_p)
{
    if (crc_p != NULL)
    {
        uint16_t crc = *crc_p;
        crc += c;
        *crc_p = crc;
    }
    uart0_write(c);
    if (debug)
        printf("%X ", c);
}

static bool check_crc(uint8_t *data)
{
    uint16_t crc = 0xFFFF;
    uint8_t lenght = data[0];
    for (uint8_t i = 0; i < lenght - 2; i++)
        crc -= data[i];
    if (crc == (uint16_t)data[lenght - 1] << 8 || data[lenght])
        return true;
    return false;
}

static void add_sensor(sensor_ibus_t *new_sensor, sensor_ibus_t **sensor, uint16_t sensor_mask)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (sensor[i] == NULL && sensor_mask & 1 << i)
        {
            sensor[i] = new_sensor;
            return;
        }
    }
}

static int32_t format(uint8_t data_id, float value)
{

    if (data_id == AFHDS2A_ID_TEMPERATURE)
        return round((value + 40) * 10);

    if (data_id == AFHDS2A_ID_EXTV ||
        data_id == AFHDS2A_ID_CELL_VOLTAGE ||
        data_id == AFHDS2A_ID_BAT_CURR ||
        data_id == AFHDS2A_ID_CLIMB_RATE ||
        data_id == AFHDS2A_ID_COG ||
        data_id == AFHDS2A_ID_VERTICAL_SPEED ||
        data_id == AFHDS2A_ID_GROUND_SPEED ||
        data_id == AFHDS2A_ID_GPS_ALT ||
        data_id == AFHDS2A_ID_PRES ||
        data_id == AFHDS2A_ID_ALT)
        return round(value * 100);

    if (data_id == AFHDS2A_ID_GPS_LAT ||
        data_id == AFHDS2A_ID_GPS_LON)
        return round(value / 60 * 1E7);

    if (data_id == AFHDS2A_ID_SPE)
        return round(value * 100 * 1.852);

    if (data_id == AFHDS2A_ID_GPS_STATUS)
        return value * 256;

    if (data_id == AFHDS2A_ID_S84 ||
        data_id == AFHDS2A_ID_S85)
        return value / 60 * 1e5;

    return round(value);
}

static void set_config(sensor_ibus_t **sensor, uint16_t sensormask)
{
    /*
    - Sensor at address 0x00 is reserved
    - Sensor at address 0x01 is recerved in some receivers types. But the poll at address 0x01, if present, has to be answered (so there is a dummy sensor at address 0x01), otherwise the sensor poll scan is stopped from receiver
    - TODO: dinamically set the sensor address to allocate an additional sensor in receivers with only one sensor masked
   */
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_ibus_t *new_sensor;
    new_sensor = malloc(sizeof(sensor_ibus_t));
    *new_sensor = (sensor_ibus_t){AFHDS2A_ID_END, 0, NULL};
    add_sensor(new_sensor, sensor, sensormask);
    if (config->esc_protocol == ESC_PWM)
    {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);

        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3)
    {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
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

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_fet};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
    }
    if (config->esc_protocol == ESC_CASTLE)
    {
        esc_castle_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);

        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.ripple_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current_bec};
        add_sensor(new_sensor, sensor, sensormask);

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
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_fet};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature_bec};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
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
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
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
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_MOT, IBUS_TYPE_U16, parameter.rpm};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CELL_VOLTAGE, IBUS_TYPE_U16, parameter.cell_voltage};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps)
    {
        nmea_parameters_t parameter = {config->gps_baudrate,
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        uart_pio_notify_task_handle = task_handle;
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_GPS_STATUS, IBUS_TYPE_U16, parameter.sat};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_GPS_LAT, IBUS_TYPE_S32, parameter.lat};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_GPS_LON, IBUS_TYPE_S32, parameter.lon};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_GPS_ALT, IBUS_TYPE_S32, parameter.alt};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_SPE, IBUS_TYPE_U16, parameter.spd};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_COG, IBUS_TYPE_U16, parameter.cog};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_GPS_DIST, IBUS_TYPE_U16, parameter.dist};
        add_sensor(new_sensor, sensor, sensormask);
        if (config->ibus_alternative_coordinates)
        {
            new_sensor = malloc(sizeof(sensor_ibus_t));
            *new_sensor = (sensor_ibus_t){AFHDS2A_ID_S84, IBUS_TYPE_S32, parameter.lat};
            add_sensor(new_sensor, sensor, sensormask);
            new_sensor = malloc(sizeof(sensor_ibus_t));
            *new_sensor = (sensor_ibus_t){AFHDS2A_ID_S85, IBUS_TYPE_S32, parameter.lon};
            add_sensor(new_sensor, sensor, sensormask);
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage)
    {
        voltage_parameters_t parameter = {0, config->alpha_voltage, config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_EXTV, IBUS_TYPE_U16, parameter.voltage};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current)
    {
        current_parameters_t parameter = {1, config->alpha_current, config->analog_current_multiplier, config->analog_current_offset, config->analog_current_autoffset, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_BAT_CURR, IBUS_TYPE_U16, parameter.current};
        add_sensor(new_sensor, sensor, sensormask);
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_FUEL, IBUS_TYPE_U16, parameter.consumption};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc)
    {
        ntc_parameters_t parameter = {2, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.ntc};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed)
    {
        airspeed_parameters_t parameter = {3, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_SPE, IBUS_TYPE_U16, parameter.airspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280)
    {
        bmp280_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, config->bmp280_filter, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_ALT, IBUS_TYPE_S32, parameter.altitude};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611)
    {
        ms5611_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_ALT, IBUS_TYPE_S32, parameter.altitude};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180)
    {
        bmp180_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_TEMPERATURE, IBUS_TYPE_U16, parameter.temperature};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_ALT, IBUS_TYPE_S32, parameter.altitude};
        add_sensor(new_sensor, sensor, sensormask);
        new_sensor = malloc(sizeof(sensor_ibus_t));
        *new_sensor = (sensor_ibus_t){AFHDS2A_ID_CLIMB_RATE, IBUS_TYPE_S16, parameter.vspeed};
        add_sensor(new_sensor, sensor, sensormask);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
