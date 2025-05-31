#include "nmea.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "distance.h"
#include "pico/stdlib.h"
#include "stdlib.h"
#include "uart_pio.h"
#include "vspeed.h"

#define COMMAND_GGA 0
#define COMMAND_RMC 1
#define COMMAND_UNK 2

#define COMMAND_VTG 4
#define COMMAND_GLL 5

#define NMEA_LON 1
#define NMEA_ALT 2
#define NMEA_SPD 3
#define NMEA_COG 4
#define NMEA_FIX 5
#define NMEA_SAT 6
#define NMEA_DATE 7
#define NMEA_TIME 8
#define NMEA_LAT_SIGN 9
#define NMEA_LON_SIGN 10
#define NMEA_HDOP 11
#define NMEA_END 12
#define NMEA_LAT 13

#define TIMEOUT_US 5000
#define VSPEED_INTERVAL_MS 2000

static void process(nmea_parameters_t *parameter);
static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, nmea_parameters_t *parameter);
static void send_ublox_message(uint8_t *buf, uint len);

void nmea_task(void *parameters) {
    nmea_parameters_t parameter = *(nmea_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
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
    *parameter.lon = -2500;           //-1251.964833333; // 20º51'57.89" +E, -W
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

    distance_parameters_t parameters_distance = {parameter.dist, parameter.lat, parameter.lon, parameter.alt,
                                                 parameter.sat};
    xTaskCreate(distance_task, "distance_task", STACK_DISTANCE, (void *)&parameters_distance, 2, &task_handle);
    xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    uint baudrate[] = {115200, 57600, 38400, 19200, 14400, 9600};
    uint8_t buffer[30] = {0};

    uart_pio_begin(parameter.baudrate, UART_TX_PIO_GPIO, UART_RX_PIO_GPIO, TIMEOUT_US, pio0, PIO0_IRQ_1);
    uart_pio_write(0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_pio_remove();

    // Try to change GPS config. ublox compatible devices only...

    // Set baudrate
    for (uint i = 0; i < sizeof(baudrate) / sizeof(uint); i++) {
        sprintf(buffer, "$PUBX,41,1,3,3,%i,0\r\n", parameter.baudrate);
        uart_pio_begin(baudrate[i], UART_TX_PIO_GPIO, UART_RX_PIO_GPIO, TIMEOUT_US, pio0, PIO0_IRQ_1);
        uart_pio_write_bytes(buffer, strlen(buffer));
        vTaskDelay(200 / portTICK_PERIOD_MS);
        uart_pio_remove();
    }

    uart_pio_begin(parameter.baudrate, UART_TX_PIO_GPIO, UART_RX_PIO_GPIO, TIMEOUT_US, pio0, PIO0_IRQ_1);

    // Disable unneeded messages
    char *msg_str[] = {"GLL", "GSV", "GSA", "VTG", "ZDA"};
    for (uint i = 0; i < sizeof(msg_str) / sizeof(msg_str[0]); i++) {
        sprintf(buffer, "$PUBX,40,%s,0,0,0,0,0,0\r\n", msg_str[i]);
        uart_pio_write_bytes(buffer, strlen(buffer));
    }

    // Set rate
    switch (parameter.rate) {
        case 1: {
            uint8_t msg_rate[] = {0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};
            send_ublox_message(msg_rate, sizeof(msg_rate));
            break;
        }
        case 5: {
            uint8_t msg_rate[] = {0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
            send_ublox_message(msg_rate, sizeof(msg_rate));
            break;
        }
        case 10: {
            uint8_t msg_rate[] = {0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
            send_ublox_message(msg_rate, sizeof(msg_rate));
            break;
        }
        case 20: {
            uint8_t msg_rate[] = {0x06, 0x08, 0x06, 0x00, 0x00, 0x32, 0x01, 0x00, 0x01, 0x00};
            send_ublox_message(msg_rate, sizeof(msg_rate));
            break;
        }
    }

    // Save cghanges
    uint8_t msg_save[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
                          0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB};
    send_ublox_message(msg_save, sizeof(msg_save));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static inline void send_ublox_message(uint8_t *buf, uint len) {
    uint8_t a = 0, b = 0;
    uart_pio_write(0xB5);
    uart_pio_write(0x62);
    for (uint i = 0; i < len; i++) {
        a += buf[i];
        b += a;
        uart_pio_write(buf[i]);
    }
    uart_pio_write(a);
    uart_pio_write(b);
}

static void process(nmea_parameters_t *parameter) {
    static uint8_t nmea_cmd = 0;
    static uint8_t cmd_field = 0;
    static char buffer[20] = {0};
    while (uart_pio_available()) {
        static uint8_t data_pos = 0;
        char data = uart_pio_read();
        switch (data) {
            case '$':
                cmd_field = 0;
                data_pos = 0;
                nmea_cmd = COMMAND_UNK;
                break;
            case '\r':
            case ',':
                if (cmd_field == 0) {
                    if (memcmp(buffer + 2, "GGA", 3) == 0) {
                        nmea_cmd = COMMAND_GGA;
                    } else if (memcmp(buffer + 2, "RMC", 3) == 0) {
                        nmea_cmd = COMMAND_RMC;
                    }
                    if (nmea_cmd != COMMAND_UNK)
                        debug("\nGPS (%u) < %s(%i): ", uxTaskGetStackHighWaterMark(NULL), buffer + 2, nmea_cmd);
                } else {
                    if (nmea_cmd != COMMAND_UNK) parser(nmea_cmd, cmd_field, buffer, parameter);
                }

                cmd_field++;
                data_pos = 0;
                buffer[0] = 0;
                break;
            case '\n':
                break;
            default:
                if (data_pos < 19) {
                    buffer[data_pos] = data;
                    data_pos++;
                    buffer[data_pos] = 0;
                }
        }
    }
}

static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, nmea_parameters_t *parameter) {
    uint8_t nmea_field[2][17] = {{0, 0, 0, 0, 0, 0, 0, NMEA_SAT, NMEA_HDOP, NMEA_ALT, 0, 0, 0, 0, 0, 0, 0},  // GGA
                                 {0, NMEA_TIME, 0, NMEA_LAT, NMEA_LAT_SIGN, NMEA_LON, NMEA_LON_SIGN, NMEA_SPD, NMEA_COG,
                                  NMEA_DATE, 0, 0, 0, 0, 0, 0, 0}};  // RMC
    static int8_t lat_dir = 1, lon_dir = 1;
    static uint32_t timestamp_vspeed = 0, timestamp_dist = 0;
    if (strlen(buffer)) {
        if (nmea_field[nmea_cmd][cmd_field] == NMEA_TIME) {
            *parameter->time = atof(buffer);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LAT) {
            char degrees[3] = {0};
            float minutes;
            strncpy(degrees, buffer, 2);
            minutes = atof(buffer + 2);
            *parameter->lat = lat_dir * (atof(degrees) * 60 + minutes);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LON) {
            char degrees[4] = {0};
            float minutes;
            strncpy(degrees, buffer, 3);
            minutes = atof(buffer + 3);
            *parameter->lon = lon_dir * (atof(degrees) * 60 + minutes);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_ALT) {
            *parameter->alt = atof(buffer);
            get_vspeed_gps(parameter->vspeed, *parameter->alt, VSPEED_INTERVAL_MS);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_SPD) {
            *parameter->spd = atof(buffer);
            *parameter->spd_kmh = *parameter->spd * 1.852;
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_COG) {
            *parameter->cog = atof(buffer);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_DATE) {
            *parameter->date = atof(buffer);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_SAT) {
            *parameter->sat = atof(buffer);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LAT_SIGN) {
            lat_dir = (buffer[0] == 'N') ? 1 : -1;
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LON_SIGN) {
            lon_dir = (buffer[0] == 'E') ? 1 : -1;
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_HDOP) {
            *parameter->hdop = atof(buffer);
        }
        debug("%s(%i),", buffer, nmea_field[nmea_cmd][cmd_field]);
    }
}
