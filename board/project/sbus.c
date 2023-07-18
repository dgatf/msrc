#include "sbus.h"

static sensor_sbus_t *sbus_sensor[32] = {NULL};
static uint packet_id;

static void process();
static int64_t send_slot_callback(alarm_id_t id, void *parameters);
static inline void send_slot(uint8_t slot);
static uint16_t format(uint8_t data_id, float value);
static void add_sensor(uint8_t slot, sensor_sbus_t *new_sensor);
static void set_config();
static uint8_t get_slot_id(uint8_t slot);

void sbus_task(void *parameters)
{
    led_cycle_duration = 6;
    led_cycles = 1;
    set_config();
    uart0_begin(100000, UART_RECEIVER_TX, UART_RECEIVER_RX, SBUS_TIMEOUT_US, 8, 2, UART_PARITY_EVEN, true);
    if (debug)
        printf("\nSbus init");
    while (1)
    {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

static void process()
{
    if (uart0_available() == SBUS_PACKET_LENGHT)
    {
        uint8_t data[SBUS_PACKET_LENGHT];
        uart0_read_bytes(data, SBUS_PACKET_LENGHT);
        if (debug)
        {
            printf("\nSbus (%u) < ", uxTaskGetStackHighWaterMark(NULL));
            for (uint8_t i = 0; i < SBUS_PACKET_LENGHT; i++)
            {
                printf("%X ", data[i]);
            }
        }
        if (data[0] == 0x0F)
        {
            if (data[24] == 0x04 || data[24] == 0x14 || data[24] == 0x24 || data[24] == 0x34)
            {
                packet_id = data[24] >> 4;
                add_alarm_in_us(SBUS_SLOT_0_DELAY /*- uart0_get_time_elapsed()*/, send_slot_callback, NULL, true);
                // vTaskResume(led_task_handle);
                //  printf("\nTE: %u", uart0_get_time_elapsed());
                if (debug)
                    printf("\nSbus (%u) > ", uxTaskGetStackHighWaterMark(NULL));
            }
        }
    }
}

static int64_t send_slot_callback(alarm_id_t id, void *parameters)
{
    static uint8_t index = 0;
    uint8_t slot = index + packet_id * 8;
    send_slot(slot);
    if (index < 7)
    {
        index++;
        return SBUS_INTER_SLOT_DELAY;
    }
    index = 0;
    return 0;
}

static inline void send_slot(uint8_t slot)
{
    if (debug == 2)
    {
        static uint32_t timestamp;
        if (slot == 0 || slot == 8 || slot == 16 || slot == 24)
            printf(" %u", uart0_get_time_elapsed());
        else
            printf(" %u", time_us_32() - timestamp);
        timestamp = time_us_32();
    }
    if (debug)
        printf(" (%u)", slot);
    uint16_t value = 0;
    if (sbus_sensor[slot])
    {
        if (sbus_sensor[slot]->value)
            value = format(sbus_sensor[slot]->data_id, *sbus_sensor[slot]->value);
        uint8_t data[3];
        data[0] = get_slot_id(slot);
        data[1] = value;
        data[2] = value >> 8;
        uart0_write_bytes(data, 3);
        if (debug)
            printf("%X:%X:%X ", data[0], data[1], data[2]);
    }
}

static uint16_t format(uint8_t data_id, float value)
{
    if (data_id == FASST_RPM)
    {
        return (uint16_t)round(value / 6);
    }
    if (data_id == FASST_TEMP)
    {
        return (uint16_t)round(value + 100) | 0X8000;
    }
    if (data_id == FASST_VOLT_V1)
    {
        return __builtin_bswap16((uint16_t)round(value * 10) | 0x8000);
    }
    if (data_id == FASST_VOLT_V2)
    {
        return __builtin_bswap16((uint16_t)round(value * 10));
    }
    if (data_id == FASST_VARIO_SPEED)
    {
        return __builtin_bswap16((int16_t)round(value * 100));
    }
    if (data_id == FASST_VARIO_ALT)
    {
        return __builtin_bswap16((int16_t)round(value) | 0x4000);
    }
    if (data_id == FASST_POWER_CURR)
    {
        return __builtin_bswap16((uint16_t)round(value * 100) | 0x4000);
    }
    if (data_id == FASST_POWER_VOLT)
    {
        return __builtin_bswap16((uint16_t)round((value)*100));
    }
    if (data_id == FASST_AIR_SPEED)
    {
        return __builtin_bswap16((uint16_t)round(value) | 0x4000);
    }
    if (data_id == FASST_GPS_SPEED)
    {
        return __builtin_bswap16((uint16_t)round(value * 1.852) | 0x4000);
    }
    if (data_id == FASST_GPS_VARIO_SPEED)
    {
        return __builtin_bswap16((int16_t)round(value * 10));
    }
    if (data_id == FASST_GPS_ALTITUDE)
    {
        return __builtin_bswap16((int16_t)round(value) | 0x4000);
    }
    if (data_id == FASST_GPS_LATITUDE1 || data_id == FASST_GPS_LONGITUDE1)
    {
        // FFFF = (deg,deg,S/W,min) -> min *10000 (prec 4)
        uint16_t lat;
        if (value < 0)
        {
            lat = 1 << FASST_SOUTH_WEST_BIT;
            value *= -1;
        }
        uint8_t degrees = value / 60;
        lat |= degrees << 8;
        uint32_t minutes = fmod(value, 60) * 10000; // minutes precision 4
        lat |= minutes >> 16;
        return __builtin_bswap16(lat);
    }
    if (data_id == FASST_GPS_LATITUDE2 || data_id == FASST_GPS_LONGITUDE2)
    {
        // FFFF = (min) -> min *10000 (prec 4)
        if (value < 0)
        {
            value *= -1;
        }
        uint32_t minutes = fmod(value, 60) * 10000; // minutes precision 4
        return __builtin_bswap16(minutes);
    }
    if (data_id == FASST_GPS_TIME)
    {
        if (value > 120000)
            value -= 120000;
        uint8_t hours = value / 10000;
        uint8_t minutes = (uint8_t)(value / 100) - hours * 100;
        uint8_t seconds = value - hours * 10000 - minutes * 100;
        return __builtin_bswap16(hours * 3600 + minutes * 60 + seconds);
    }
    return __builtin_bswap16(round(value));
}

static uint8_t get_slot_id(uint8_t slot)
{
    uint8_t slot_id[32] = {0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
                           0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                           0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
                           0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB};
    return slot_id[slot];
}

static void add_sensor(uint8_t slot, sensor_sbus_t *new_sensor)
{
    sbus_sensor[slot] = new_sensor;
}

static void set_config(sensor_sbus_t *sensor[])
{
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_sbus_t *new_sensor;
    if (config->esc_protocol == ESC_PWM)
    {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_RPM, parameter.rpm};
        add_sensor(SBUS_SLOT_RPM, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3)
    {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_RPM, parameter.rpm};
        add_sensor(SBUS_SLOT_RPM, new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_RPM, parameter.rpm};
        add_sensor(SBUS_SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, parameter.voltage};
        add_sensor(SBUS_SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current};
        add_sensor(SBUS_SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CONS, parameter.consumption};
        add_sensor(SBUS_SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.temperature_fet};
        add_sensor(SBUS_SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.temperature_bec};
        add_sensor(SBUS_SLOT_TEMP2, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
    }
    if (config->esc_protocol == ESC_CASTLE)
    {
        esc_castle_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_RPM, parameter.rpm};
        add_sensor(SBUS_SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, parameter.voltage};
        add_sensor(SBUS_SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current};
        add_sensor(SBUS_SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, parameter.voltage_bec};
        add_sensor(SBUS_SLOT_POWER_VOLT3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current_bec};
        add_sensor(SBUS_SLOT_POWER_CURR3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CONS, parameter.consumption};
        add_sensor(SBUS_SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.temperature};
        add_sensor(SBUS_SLOT_TEMP1, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_RPM, parameter.rpm};
        add_sensor(SBUS_SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, parameter.voltage};
        add_sensor(SBUS_SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current};
        add_sensor(SBUS_SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CONS, parameter.consumption};
        add_sensor(SBUS_SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, parameter.voltage_bec};
        add_sensor(SBUS_SLOT_POWER_VOLT3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current_bec};
        add_sensor(SBUS_SLOT_POWER_CURR3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CONS, parameter.consumption};
        add_sensor(SBUS_SLOT_POWER_CONS3, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.temperature_fet};
        add_sensor(SBUS_SLOT_TEMP1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.temperature_bec};
        add_sensor(SBUS_SLOT_TEMP2, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_RPM, parameter.rpm};
        add_sensor(SBUS_SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, parameter.voltage};
        add_sensor(SBUS_SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current};
        add_sensor(SBUS_SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CONS, parameter.consumption};
        add_sensor(SBUS_SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.temperature};
        add_sensor(SBUS_SLOT_TEMP1, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
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

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_RPM, parameter.rpm};
        add_sensor(SBUS_SLOT_RPM, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, parameter.voltage};
        add_sensor(SBUS_SLOT_POWER_VOLT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current};
        add_sensor(SBUS_SLOT_POWER_CURR1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CONS, parameter.consumption};
        add_sensor(SBUS_SLOT_POWER_CONS1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.temperature};
        add_sensor(SBUS_SLOT_TEMP1, new_sensor);
        // new_sensor = malloc(sizeof(sensor_sbus_t));
        //*new_sensor = (sensor_sbus_t){AFHDS2A_ID_CELL_VOLTAGE, parameter.cell_voltage};
        // add_sensor(new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps)
    {
        nmea_parameters_t parameter = {config->gps_baudrate,
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        uart_pio_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_LATITUDE1, parameter.lat};
        add_sensor(SBUS_SLOT_GPS_LAT1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_LATITUDE2, parameter.lat};
        add_sensor(SBUS_SLOT_GPS_LAT2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_LONGITUDE1, parameter.lon};
        add_sensor(SBUS_SLOT_GPS_LON1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_LONGITUDE2, parameter.lon};
        add_sensor(SBUS_SLOT_GPS_LON2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_ALTITUDE, parameter.alt};
        add_sensor(SBUS_SLOT_GPS_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_SPEED, parameter.spd};
        add_sensor(SBUS_SLOT_GPS_SPD, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_VARIO_SPEED, parameter.vspeed};
        add_sensor(SBUS_SLOT_GPS_VARIO, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_GPS_TIME, parameter.time};
        add_sensor(SBUS_SLOT_GPS_TIME, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage)
    {
        voltage_parameters_t parameter = {0, config->alpha_voltage, config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VOLT_V1, parameter.voltage};
        add_sensor(SBUS_SLOT_VOLT_V1, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VOLT_V2, NULL};
        add_sensor(SBUS_SLOT_VOLT_V2, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current)
    {
        current_parameters_t parameter = {1, config->alpha_current, config->analog_current_multiplier, config->analog_current_offset, config->analog_current_autoffset, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CURR, parameter.current};
        add_sensor(SBUS_SLOT_POWER_CURR2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_CONS, parameter.consumption};
        add_sensor(SBUS_SLOT_POWER_CONS2, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_POWER_VOLT, NULL};
        add_sensor(SBUS_SLOT_POWER_VOLT2, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc)
    {
        ntc_parameters_t parameter = {2, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_TEMP, parameter.ntc};
        add_sensor(SBUS_SLOT_TEMP1, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed)
    {
        airspeed_parameters_t parameter = {3, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_AIR_SPEED, parameter.airspeed};
        add_sensor(SBUS_SLOT_AIR_SPEED, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280)
    {
        bmp280_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, config->bmp280_filter, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VARIO_ALT, parameter.altitude};
        add_sensor(SBUS_SLOT_VARIO_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VARIO_SPEED, parameter.vspeed};
        add_sensor(SBUS_SLOT_VARIO_SPEED, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611)
    {
        ms5611_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VARIO_ALT, parameter.altitude};
        add_sensor(SBUS_SLOT_VARIO_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VARIO_SPEED, parameter.vspeed};
        add_sensor(SBUS_SLOT_VARIO_SPEED, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180)
    {
        bmp180_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VARIO_ALT, parameter.altitude};
        add_sensor(SBUS_SLOT_VARIO_ALT, new_sensor);
        new_sensor = malloc(sizeof(sensor_sbus_t));
        *new_sensor = (sensor_sbus_t){FASST_VARIO_SPEED, parameter.vspeed};
        add_sensor(SBUS_SLOT_VARIO_SPEED, new_sensor);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
