#include "hitec.h"

#define I2C_INTR_MASK_RD_REQ 0x00000020

static volatile uint32_t *const I2C1_INTR_MASK = (volatile uint32_t *const)(I2C1_BASE + 0x30);
static volatile uint32_t *const I2C1_CLR_RD_REQ = (volatile uint32_t *const)(I2C1_BASE + 0x50);

static sensor_hitec_t *sensor;

static void i2c_handler();
static void set_config();

void hitec_i2c_handler()
{
    i2c_handler();
}

void hitec_task(void *parameters)
{
    sensor = malloc(sizeof(sensor_hitec_t));
    *sensor = (sensor_hitec_t){{0}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}, {NULL}};

    led_cycle_duration = 6;
    led_cycles = 1;

    i2c_init(i2c1, 100 * 1000);
    i2c_set_slave_mode(i2c1, true, HITEC_I2C_ADDRESS);
    gpio_set_function(I2C1_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_GPIO);
    gpio_pull_up(I2C1_SCL_GPIO);
    *I2C1_INTR_MASK = I2C_INTR_MASK_RD_REQ;
    irq_set_exclusive_handler(I2C1_IRQ, i2c_handler);
    irq_set_enabled(I2C1_IRQ, true);
    set_config();
    if (debug)
        printf("\nHitec init");
    vTaskSuspend(NULL);
    free(sensor);
    vTaskDelete(NULL);
}

static void i2c_handler()
{
    *I2C1_CLR_RD_REQ;
    if (uxQueueMessagesWaiting(tasks_queue_handle) > 1)
    {
        static uint8_t frame = 0;
        int32_t valueS32;
        uint16_t valueU16;
        uint16_t valueS16;
        uint8_t valueU8;
        uint8_t buffer[7] = {0};
        do
        {
            frame++;
            frame %= 11;
        } while (!sensor->is_enabled_frame[frame]);
        buffer[0] = frame + 0x11;
        buffer[6] = frame + 0x11;
        switch (frame)
        {
        case HITEC_FRAME_0X11:
            buffer[1] = 0xAF;
            buffer[3] = 0x2D;
            if (sensor->frame_0x11[HITEC_FRAME_0X11_RX_BATT])
            {
                valueU16 = *sensor->frame_0x11[HITEC_FRAME_0X11_RX_BATT] * 28;
                buffer[4] = valueU16 >> 8;
                buffer[5] = valueU16;
            }
            break;
        case HITEC_FRAME_0X12:
            if (sensor->frame_0x12[HITEC_FRAME_0X12_GPS_LAT])
            {
                float degF = *sensor->frame_0x12[HITEC_FRAME_0X12_GPS_LAT] / 60;
                int8_t deg = degF;
                int8_t min = (degF - deg) * 60;
                float sec = ((degF - deg) * 60 - min) * 60;
                int16_t sec_x_100 = sec * 100;
                int16_t deg_min = deg * 100 + min;
                buffer[1] = sec_x_100 >> 8;
                buffer[2] = sec_x_100;
                buffer[3] = deg_min >> 8;
                buffer[4] = deg_min;
            }
            if (sensor->frame_0x12[HITEC_FRAME_0X12_TIME])
            {
                valueU8 = *sensor->frame_0x12[HITEC_FRAME_0X12_TIME];
            }
            break;
        case HITEC_FRAME_0X13:
            if (sensor->frame_0x13[HITEC_FRAME_0X13_GPS_LON])
            {
                float degF = *sensor->frame_0x13[HITEC_FRAME_0X13_GPS_LON] / 60;
                int8_t deg = degF;
                int8_t min = (degF - deg) * 60;
                float sec = ((degF - deg) * 60 - min) * 60;
                int16_t sec_x_100 = sec * 100;
                int16_t deg_min = deg * 100 + min;
                buffer[1] = sec_x_100 >> 8;
                buffer[2] = sec_x_100;
                buffer[3] = deg_min >> 8;
                buffer[4] = deg_min;
            }
            if (sensor->frame_0x13[HITEC_FRAME_0X13_TEMP2])
            {
                valueU8 = round(*sensor->frame_0x13[HITEC_FRAME_0X13_TEMP2] + 40);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X14:
            if (sensor->frame_0x14[HITEC_FRAME_0X14_GPS_SPD])
            {
                valueU16 = round(*sensor->frame_0x14[HITEC_FRAME_0X14_GPS_SPD] * 1.852);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (sensor->frame_0x14[HITEC_FRAME_0X14_GPS_ALT])
            {
                valueS16 = round(*sensor->frame_0x14[HITEC_FRAME_0X14_GPS_ALT]);
                buffer[3] = valueS16 >> 8;
                buffer[4] = valueS16;
            }
            if (sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1])
            {
                valueU8 = round(*sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] + 40);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X15:
            if (sensor->frame_0x15[HITEC_FRAME_0X15_RPM1])
            {
                valueU16 = round(*sensor->frame_0x15[HITEC_FRAME_0X15_RPM1]);
                buffer[2] = valueU16;
                buffer[3] = valueU16 >> 8;
            }
            if (sensor->frame_0x15[HITEC_FRAME_0X15_RPM2])
            {
                valueU16 = round(*sensor->frame_0x15[HITEC_FRAME_0X15_RPM2]);
                buffer[4] = valueU16;
                buffer[5] = valueU16 >> 8;
            }
            break;
        case HITEC_FRAME_0X16:
            if (sensor->frame_0x16[HITEC_FRAME_0X16_DATE])
            {
                valueS32 = *sensor->frame_0x16[HITEC_FRAME_0X16_DATE];
                buffer[3] = valueS32 / 10000;                                 // year
                buffer[2] = (valueS32 - buffer[3] * 10000UL) / 100;           // month
                buffer[1] = valueS32 - buffer[3] * 10000UL - buffer[2] * 100; // day
            }
            if (sensor->frame_0x16[HITEC_FRAME_0X16_TIME])
            {
                valueS32 = *sensor->frame_0x16[HITEC_FRAME_0X16_TIME];
                buffer[4] = valueS32 / 10000;                       // hour
                buffer[5] = (valueS32 - buffer[4] * 10000UL) / 100; // minute
            }
            break;
        case HITEC_FRAME_0X17:
            if (sensor->frame_0x17[HITEC_FRAME_0X17_COG])
            {
                valueU16 = round(*sensor->frame_0x17[HITEC_FRAME_0X17_COG]);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (sensor->frame_0x17[HITEC_FRAME_0X17_SATS])
            {
                valueU8 = *sensor->frame_0x17[HITEC_FRAME_0X17_SATS];
                buffer[3] = valueU8;
            }
            if (sensor->frame_0x17[HITEC_FRAME_0X17_TEMP3])
            {
                valueU8 = round(*sensor->frame_0x17[HITEC_FRAME_0X17_TEMP3] + 40);
                buffer[4] = valueU8;
            }
            if (sensor->frame_0x17[HITEC_FRAME_0X17_TEMP4])
            {
                valueU8 = round(*sensor->frame_0x17[HITEC_FRAME_0X17_TEMP4] + 40);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X18:
            if (sensor->frame_0x18[HITEC_FRAME_0X18_VOLT])
            {
                valueU16 = round((*sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] - 0.2) * 10);
                buffer[1] = valueU16;
                buffer[2] = valueU16 >> 8;
            }
            if (sensor->frame_0x18[HITEC_FRAME_0X18_AMP])
            {
                /* value for stock transmitter (tbc) */
                // valueU16 = (*sensor->frame_0x18[HITEC_FRAME_0X18_AMP] + 114.875) * 1.441;

                /* value for opentx transmitter  */
                valueU16 = round(*sensor->frame_0x18[HITEC_FRAME_0X18_AMP]);

                buffer[3] = valueU16;
                buffer[4] = valueU16 >> 8;
            }
            break;
        case HITEC_FRAME_0X19:
            if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP1])
            {
                valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP1] * 10);
                buffer[5] = valueU8;
            }
            if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP2])
            {
                valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP2] * 10);
                buffer[5] = valueU8;
            }
            if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP3])
            {
                valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP3] * 10);
                buffer[5] = valueU8;
            }
            if (sensor->frame_0x19[HITEC_FRAME_0X19_AMP4])
            {
                valueU8 = round(*sensor->frame_0x19[HITEC_FRAME_0X19_AMP4] * 10);
                buffer[5] = valueU8;
            }
            break;
        case HITEC_FRAME_0X1A:
            if (sensor->frame_0x1A[HITEC_FRAME_0X1A_ASPD])
            {
                valueU16 = round(*sensor->frame_0x1A[HITEC_FRAME_0X1A_ASPD]);
                buffer[3] = valueU16 >> 8;
                buffer[4] = valueU16;
            }
            break;
        case HITEC_FRAME_0X1B:
            if (sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU])
            {
                valueU16 = round(*sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU]);
                buffer[1] = valueU16 >> 8;
                buffer[2] = valueU16;
            }
            if (sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTF])
            {
                valueU16 = round(*sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTF]);
                buffer[3] = valueU16 >> 8;
                buffer[4] = valueU16;
            }
            break;
        }

        i2c_write_raw_blocking(i2c1, buffer, 7);

        // blink led
        vTaskResume(led_task_handle);

        if (debug)
        {
            printf("\nHitec (%u) > ", uxTaskGetStackHighWaterMark(receiver_task_handle));
            for (int i = 0; i < 7; i++)
            {
                printf("%X ", buffer[i]);
            }
        }
    }
}

static void set_config()
{
    config_t *config = config_read();
    TaskHandle_t task_handle;
    sensor_hitec_t *new_sensor;
    if (config->esc_protocol == ESC_PWM)
    {
        esc_pwm_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_pwm_task, "esc_pwm_task", STACK_ESC_PWM, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x15[HITEC_FRAME_0X15_RPM1] = parameter.rpm;
        sensor->is_enabled_frame[HITEC_FRAME_0X15] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->esc_protocol == ESC_HW3)
    {
        esc_hw3_parameters_t parameter = {config->rpm_multiplier, config->alpha_rpm, malloc(sizeof(float))};
        xTaskCreate(esc_hw3_task, "esc_hw3_task", STACK_ESC_HW3, (void *)&parameter, 2, &task_handle);
        uart1_notify_task_handle = task_handle;
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        sensor->frame_0x15[HITEC_FRAME_0X15_RPM1] = parameter.rpm;
        sensor->is_enabled_frame[HITEC_FRAME_0X15] = true;

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
        sensor->frame_0x15[HITEC_FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[HITEC_FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] = parameter.temperature_fet;
        sensor->frame_0x13[HITEC_FRAME_0X13_TEMP2] = parameter.temperature_bec;
        sensor->is_enabled_frame[HITEC_FRAME_0X15] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X14] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X13] = true;
    }
    if (config->esc_protocol == ESC_CASTLE)
    {
        esc_castle_parameters_t parameter = {config->rpm_multiplier,
                                             config->alpha_rpm, config->alpha_voltage, config->alpha_current, config->alpha_temperature,
                                             malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(uint8_t))};
        xTaskCreate(esc_castle_task, "esc_castle_task", STACK_ESC_CASTLE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);
        sensor->frame_0x15[HITEC_FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[HITEC_FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] = parameter.temperature;
        sensor->is_enabled_frame[HITEC_FRAME_0X15] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X14] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X13] = true;

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

        sensor->frame_0x15[HITEC_FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[HITEC_FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] = parameter.temperature_fet;
        sensor->frame_0x13[HITEC_FRAME_0X13_TEMP2] = parameter.temperature_bec;
        sensor->is_enabled_frame[HITEC_FRAME_0X15] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X14] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X13] = true;

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

        sensor->frame_0x15[HITEC_FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[HITEC_FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] = parameter.temperature;
        sensor->is_enabled_frame[HITEC_FRAME_0X15] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X14] = true;

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

        sensor->frame_0x15[HITEC_FRAME_0X15_RPM1] = parameter.rpm;
        sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] = parameter.voltage;
        sensor->frame_0x18[HITEC_FRAME_0X18_AMP] = parameter.current;
        sensor->frame_0x14[HITEC_FRAME_0X14_TEMP1] = parameter.temperature;
        sensor->is_enabled_frame[HITEC_FRAME_0X15] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X14] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_gps)
    {
        nmea_parameters_t parameter = {config->gps_baudrate,
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)),
                                       malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(nmea_task, "nmea_task", STACK_GPS, (void *)&parameter, 2, &task_handle);
        uart_pio_notify_task_handle = task_handle;

        sensor->frame_0x17[HITEC_FRAME_0X17_SATS] = parameter.sat;
        sensor->frame_0x12[HITEC_FRAME_0X12_GPS_LAT] = parameter.lat;
        sensor->frame_0x13[HITEC_FRAME_0X13_GPS_LON] = parameter.lon;
        sensor->frame_0x14[HITEC_FRAME_0X14_GPS_ALT] = parameter.alt;
        sensor->frame_0x14[HITEC_FRAME_0X14_GPS_SPD] = parameter.spd;
        sensor->frame_0x17[HITEC_FRAME_0X17_COG] = parameter.cog;
        sensor->frame_0x16[HITEC_FRAME_0X16_DATE] = parameter.date;
        sensor->frame_0x16[HITEC_FRAME_0X16_TIME] = parameter.time;
        sensor->is_enabled_frame[HITEC_FRAME_0X17] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X12] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X13] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X14] = true;
        sensor->is_enabled_frame[HITEC_FRAME_0X16] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_voltage)
    {
        voltage_parameters_t parameter = {0, config->alpha_voltage, config->analog_voltage_multiplier, malloc(sizeof(float))};
        xTaskCreate(voltage_task, "voltage_task", STACK_VOLTAGE, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x18[HITEC_FRAME_0X18_VOLT] = parameter.voltage;
        sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_current)
    {
        current_parameters_t parameter = {1, config->alpha_current, config->analog_current_multiplier, config->analog_current_offset, config->analog_current_autoffset, malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(current_task, "current_task", STACK_CURRENT, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x18[HITEC_FRAME_0X18_AMP] = parameter.current;
        sensor->is_enabled_frame[HITEC_FRAME_0X18] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_ntc)
    {
        ntc_parameters_t parameter = {2, config->alpha_temperature, malloc(sizeof(float))};
        xTaskCreate(ntc_task, "ntc_task", STACK_NTC, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x17[HITEC_FRAME_0X17_TEMP3] = parameter.ntc;
        sensor->is_enabled_frame[HITEC_FRAME_0X17] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->enable_analog_airspeed)
    {
        airspeed_parameters_t parameter = {3, config->alpha_airspeed, malloc(sizeof(float))};
        xTaskCreate(airspeed_task, "airspeed_task", STACK_AIRSPEED, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x1A[HITEC_FRAME_0X1A_ASPD] = parameter.airspeed;
        sensor->is_enabled_frame[HITEC_FRAME_0X1A] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP280)
    {
        bmp280_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, config->bmp280_filter, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp280_task, "bmp280_task", STACK_BMP280, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[HITEC_FRAME_0X1B] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_MS5611)
    {
        ms5611_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(ms5611_task, "ms5611_task", STACK_MS5611, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[HITEC_FRAME_0X1B] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (config->i2c_module == I2C_BMP180)
    {
        bmp180_parameters_t parameter = {config->alpha_vario, config->vario_auto_offset, config->i2c_address, malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float)), malloc(sizeof(float))};
        xTaskCreate(bmp180_task, "bmp180_task", STACK_BMP180, (void *)&parameter, 2, &task_handle);
        xQueueSendToBack(tasks_queue_handle, task_handle, 0);

        sensor->frame_0x1B[HITEC_FRAME_0X1B_ALTU] = parameter.altitude;
        sensor->is_enabled_frame[HITEC_FRAME_0X1B] = true;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
