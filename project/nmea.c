#include "nmea.h"

static void process(nmea_parameters_t *parameter);
static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, nmea_parameters_t *parameter);

void nmea_task(void *parameters)
{
    nmea_parameters_t parameter = *(nmea_parameters_t *)parameters;
    xTaskNotifyGive(receiver_task_handle);
    *parameter.lat = 0;
    *parameter.lon = 0;
    *parameter.alt = 0;
    *parameter.spd = 0;
    *parameter.cog = 0;
    *parameter.hdop = 0;
    *parameter.sat = 0;
    *parameter.time = 0;
    *parameter.date = 0;
    *parameter.vspeed = 0;
    *parameter.dist = 0;
    *parameter.spd_kmh = 0;
#ifdef SIM_SENSORS
    *parameter.lat = -692.761166667;  // 11º32'45.67" +N, -S
    *parameter.lon = -2500; //-1251.964833333; // 20º51'57.89" +E, -W
    *parameter.alt = 1283;            // m
    *parameter.spd = 158;             // kts
    *parameter.cog = 123.45;          // º
    *parameter.sat = 10;              //
    *parameter.date = 141012;         // yymmdd
    *parameter.time = 162302;         // hhmmss
    *parameter.hdop = 12.35;          //
    *parameter.dist = 1000;
    *parameter.spd_kmh = 123;
#endif
    TaskHandle_t task_handle;
    uint vspeed_interval = 500;
    vspeed_parameters_t parameters_vspeed = {vspeed_interval, parameter.alt, parameter.vspeed};
    xTaskCreate(vspeed_task, "vspeed_task", STACK_VSPEED, (void *)&parameters_vspeed, 2, &task_handle);
    xQueueSendToBack(tasks_queue_handle, task_handle, 0);
    distance_parameters_t parameters_distance = {parameter.dist, parameter.lat, parameter.lon, parameter.alt, parameter.sat};
    xTaskCreate(distance_task, "distance_task", STACK_DISTANCE, (void *)&parameters_distance, 2, &task_handle);
    xQueueSendToBack(tasks_queue_handle, task_handle, 0);
    // uart_begin(UART_ESC, parameter.baudrate, UART1_TX_GPIO, UART_ESC_RX, NMEA_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false);
    uart_pio_begin(parameter.baudrate, UART_RX_PIO_GPIO, NMEA_TIMEOUT_US, pio0, PIO0_IRQ_1);

    while (1)
    {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(nmea_parameters_t *parameter)
{
    static uint8_t nmea_cmd = 0;
    static uint8_t cmd_field = 0;
    static char buffer[20] = {0};
    while (uart_pio_available())
    {
        static uint8_t data_pos = 0;
        char data = uart_pio_read();
        switch (data)
        {
        case '$':
            cmd_field = 0;
            data_pos = 0;
            nmea_cmd = NMEA_UNK;
            break;
        case '\r':
        case ',':
            if (cmd_field == 0)
            {
                if (memcmp(buffer + 2, "GGA", 3) == 0)
                {
                    nmea_cmd = NMEA_GGA;
                }
                else if (memcmp(buffer + 2, "RMC", 3) == 0)
                {
                    nmea_cmd = NMEA_RMC;
                }
                if (debug)
                {
                    if (nmea_cmd != NMEA_UNK)
                    {
                        printf("\nGPS (%u) < %s(%i): ", uxTaskGetStackHighWaterMark(NULL), buffer + 2, nmea_cmd);
                    }
                }
            }
            else
            {
                if (nmea_cmd != NMEA_UNK)
                    parser(nmea_cmd, cmd_field, buffer, parameter);
            }

            cmd_field++;
            data_pos = 0;
            buffer[0] = 0;
            break;
        case '\n':
            break;
        default:
            if (data_pos < 19)
            {
                buffer[data_pos] = data;
                data_pos++;
                buffer[data_pos] = 0;
            }
        }
    }
}

static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, nmea_parameters_t *parameter)
{
    uint8_t nmea_field[2][17] =
        {{0, 0, 0, 0, 0, 0, 0, NMEA_SAT, NMEA_HDOP, NMEA_ALT, 0, 0, 0, 0, 0, 0, 0},                                                // GGA
         {0, NMEA_TIME, 0, NMEA_LAT, NMEA_LAT_SIGN, NMEA_LON, NMEA_LON_SIGN, NMEA_SPD, NMEA_COG, NMEA_DATE, 0, 0, 0, 0, 0, 0, 0}}; // RMC
    static int8_t lat_dir = 1, lon_dir = 1;
    static uint32_t timestamp_vspeed = 0, timestamp_dist = 0;
    if (strlen(buffer))
    {
        if (nmea_field[nmea_cmd][cmd_field] == NMEA_TIME)
        {
            *parameter->time = atof(buffer);
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LAT)
        {
            char degrees[3] = {0};
            float minutes;
            strncpy(degrees, buffer, 2);
            minutes = atof(buffer + 2);
            *parameter->lat = lat_dir * (atof(degrees) * 60 + minutes);
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LON)
        {
            char degrees[4] = {0};
            float minutes;
            strncpy(degrees, buffer, 3);
            minutes = atof(buffer + 3);
            *parameter->lon = lon_dir * (atof(degrees) * 60 + minutes);
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_ALT)
        {
            *parameter->alt = atof(buffer);
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_SPD)
        {
            *parameter->spd = atof(buffer);
            *parameter->spd_kmh = *parameter->spd * 1.852;
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_COG)
        {
            *parameter->cog = atof(buffer);
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_DATE)
        {
            *parameter->date = atof(buffer);
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_SAT)
        {
            *parameter->sat = atof(buffer);
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LAT_SIGN)
        {
            lat_dir = (buffer[0] == 'N') ? 1 : -1;
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LON_SIGN)
        {
            lon_dir = (buffer[0] == 'E') ? 1 : -1;
        }
        else if (nmea_field[nmea_cmd][cmd_field] == NMEA_HDOP)
        {
            *parameter->hdop = atof(buffer);
        }
        if (debug)
            printf("%s(%i),", buffer, nmea_field[nmea_cmd][cmd_field]);
    }
}
