#include "gps.h"

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

typedef struct ublox_msg_info_t {
    uint8_t class;
    uint8_t id;
    uint16_t len;
} __attribute__((packed)) ublox_msg_info_t;

typedef struct ublox_navpvt_t {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;  // mm
    int32_t hMSL;
    uint32_t hAcc;   // mm
    uint32_t vAcc;   // mm
    int32_t velN;    // mm/s
    int32_t velE;    // mm/s
    int32_t velD;    // mm/s
    int32_t gSpeed;  // mm/s
    int32_t headMot;
    uint32_t sAcc;     // mm
    uint32_t headAcc;  // deg * 10000
    uint16_t pDOP;
    uint16_t flags3;
    uint32_t reserved1;
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
    uint8_t crc_a;
    uint8_t crc_b;
} __attribute__((packed)) ublox_navpvt_t;

typedef struct ublox_navdop_t {
    uint32_t iTOW;
    uint16_t gDOP;  // DOP * 100
    uint16_t pDOP;  // DOP * 100
    uint16_t tDOP;  // DOP * 100
    uint16_t vDOP;  // DOP * 100
    uint16_t hDOP;  // DOP * 100
    uint16_t nDOP;  // DOP * 100
    uint16_t eDOP;  // DOP * 100
    uint8_t crc_a;
    uint8_t crc_b;
} __attribute__((packed)) ublox_navdop_t;

typedef struct alarm_parameters_t {
    bool is_ublox;
    uint rate;
} alarm_parameters_t;

// static alarm_id_t alarm_id_ublox = 0, alarm_id_nmea = 0;
// static alarm_parameters_t alarm_parameters;

static void process(gps_parameters_t *parameter);
static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, gps_parameters_t *parameter);
static void send_ublox_message(uint8_t *buf, uint len);
static void set_baudrate(uint baudrate);
static void set_nmea_config(uint rate);
static void set_ublox_config(uint rate);
static void nmea_msg(char *cmd, bool enable);
static void ubx_cfg_msg(uint8_t class, uint8_t id, bool enable);
static void ubx_cfg_rate(uint16_t rate);
static void ubx_cfg_cfg(void);
// static int64_t alarm_nmea_timeout(alarm_id_t id, void *parameters);
// static int64_t alarm_ublox_timeout(alarm_id_t id, void *parameters);

void gps_task(void *parameters) {
    gps_parameters_t parameter = *(gps_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    *parameter.lat = 0;
    *parameter.lon = 0;
    *parameter.alt = 0;
    *parameter.spd = 0;
    *parameter.cog = 0;
    *parameter.hdop = 0;
    *parameter.vdop = 0;
    *parameter.sat = 0;
    *parameter.time = 0;
    *parameter.date = 0;
    *parameter.vspeed = 0;
    *parameter.dist = 0;
    *parameter.spd_kmh = 0;
    *parameter.fix = 0;

    *parameter.n_vel = 0;
    *parameter.e_vel = 0;
    *parameter.v_vel = 0;
    *parameter.speed_acc = 0;
    *parameter.track_acc = 0;
    *parameter.alt_elipsiod = 0;
    *parameter.h_acc = 0;
    *parameter.v_acc = 0;
#ifdef SIM_SENSORS
    *parameter.lat = 123.456789;   // deg * 1e7  11º32'45.67" +N, -S
    *parameter.lon = -123.456789;  //-deg + 10e7 1251.964833333; // 20º51'57.89" +E, -W
    *parameter.alt = 1283;         // m
    *parameter.spd = 158;          // kts
    *parameter.cog = 123.45;       // º
    *parameter.sat = 10;           //
    *parameter.date = 141012;      // yymmdd
    *parameter.time = 162302;      // hhmmss
    *parameter.hdop = 12.35;       //
    *parameter.dist = 1000;
    *parameter.spd_kmh = 123;
#endif
    TaskHandle_t task_handle;

    static distance_parameters_t parameters_distance;
    parameters_distance.distance  = parameter.dist;
    parameters_distance.altitude  = parameter.alt;
    parameters_distance.sat       = parameter.sat;
    parameters_distance.latitude  = parameter.lat;
    parameters_distance.longitude = parameter.lon;
    xTaskCreate(distance_task, "distance_task", STACK_DISTANCE, (void *)&parameters_distance, 2, &task_handle);
    // xQueueSendToBack(context.tasks_queue_handle, task_handle, 0);

    /* Change GPS config. For ublox compatible devices */

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_baudrate(parameter.baudrate);
    uart_pio_begin(parameter.baudrate, UART_TX_PIO_GPIO, UART_RX_PIO_GPIO, TIMEOUT_US, pio0, PIO0_IRQ_1, 8, 1,
                   UART_PARITY_NONE);
    if (parameter.protocol == UBLOX)
        set_ublox_config(parameter.rate);
    else
        set_nmea_config(parameter.rate);

    // alarm_parameters.is_ublox = true;
    // alarm_id_ublox = add_alarm_in_ms(20 * 1000L, alarm_ublox_timeout, &alarm_parameters, false);

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
    }
}

static void process(gps_parameters_t *parameter) {
    static char buffer[20] = {0};
    if (parameter->protocol == NMEA) {
        static uint8_t nmea_cmd = 0;
        static uint8_t cmd_field = 0;
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
                        if (nmea_cmd != COMMAND_UNK) {
                            // cancel_alarm(alarm_id_nmea);
                            // alarm_id_nmea = add_alarm_in_ms(2000, alarm_nmea_timeout, &alarm_parameters, false);
                            debug("\nGPS (%u) < %s(%i): ", uxTaskGetStackHighWaterMark(NULL), buffer + 2, nmea_cmd);
                        }
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
    } else {
        while (uart_pio_available()) {
            while (uart_pio_available() && uart_pio_read() != 0xB5)
                ;
            while (uart_pio_available() && uart_pio_read() != 0x62)
                ;
            if (uart_pio_available() < sizeof(ublox_msg_info_t)) return;
            ublox_msg_info_t msg_info;
            uart_pio_read_bytes((uint8_t *)&msg_info, sizeof(ublox_msg_info_t));
            debug("\nGPS UBLOX MSG. Class: %u Id: %u Len: %u Avail: %u", msg_info.class, msg_info.id, msg_info.len,
                  uart_pio_available());
            if (uart_pio_available() < msg_info.len + 2) return;
            if (msg_info.class == 0x01 && msg_info.id == 0x07 && msg_info.len == sizeof(ublox_navpvt_t) - 2) {
                // cancel_alarm(alarm_id_ublox);
                // alarm_id_ublox = add_alarm_in_ms(2000, alarm_ublox_timeout, &alarm_parameters, false);
                ublox_navpvt_t navpvt;
                uart_pio_read_bytes((uint8_t *)&navpvt, sizeof(ublox_navpvt_t));
                *parameter->alt = navpvt.hMSL / 1000.0F;
                *parameter->lat = navpvt.lat * 1.0e-7;
                *parameter->lon = navpvt.lon * 1.0e-7;
                *parameter->alt = navpvt.hMSL / 1000.0F;
                *parameter->cog = navpvt.headMot / 100000.0F;
                *parameter->sat = navpvt.numSV;
                *parameter->time = navpvt.hour * 10000L + navpvt.min * 100 + navpvt.sec;
                *parameter->date = navpvt.day * 10000L + navpvt.month * 100 + (navpvt.year - 2000);
                *parameter->vspeed = navpvt.velD / 1000.0F;
                *parameter->spd_kmh = navpvt.gSpeed * 3600L / 1000000.0F;
                *parameter->spd = 0.5144444F * navpvt.gSpeed * 3600L / 1000000.0F;
                *parameter->fix = navpvt.fixType;
                *parameter->n_vel = navpvt.velN / 1000.0F;
                *parameter->e_vel = navpvt.velE / 1000.0F;
                *parameter->v_vel = navpvt.velD / 1000.0F;
                *parameter->speed_acc = navpvt.sAcc / 1000.0F;
                *parameter->track_acc = navpvt.headAcc / 1000.0F;
                *parameter->alt_elipsiod = navpvt.height / 1000.0F;
                *parameter->h_acc = navpvt.hAcc / 1000.0F;
                *parameter->v_acc = navpvt.vAcc / 1000.0F;
                *parameter->pdop = navpvt.pDOP / 100.0F;
                debug(
                    "\nGPS (%u) < NAV-PTV: Date: %.0f Time: %.0f Fix: %.0f Sat: %.0f Lon: %.5f Lat: %.5f Alt: %.2f "
                    "Vspeed: %.2f Speed: mm/s %i knots %.2f kmh %.2f Pdop: %.2f",
                    uxTaskGetStackHighWaterMark(NULL), *parameter->date, *parameter->time, *parameter->fix,
                    *parameter->sat, *parameter->lon, *parameter->lat, *parameter->alt, *parameter->vspeed,
                    navpvt.gSpeed, *parameter->spd, *parameter->spd_kmh, *parameter->pdop);
            } else if (msg_info.class == 0x01 && msg_info.id == 0x04 && msg_info.len == sizeof(ublox_navdop_t) - 2) {
                // cancel_alarm(alarm_id_ublox);
                // alarm_id_ublox = add_alarm_in_ms(2000, alarm_ublox_timeout, &alarm_parameters, false);
                ublox_navdop_t navdop;
                uart_pio_read_bytes((uint8_t *)&navdop, sizeof(ublox_navpvt_t));
                *parameter->hdop = navdop.hDOP / 100.0F;
                *parameter->vdop = navdop.vDOP / 100.0F;
                debug("\nGPS (%u) < NAV-DOP: h: %.2f v: %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter->hdop,
                      *parameter->vdop);
            }
        }
    }
}

static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, gps_parameters_t *parameter) {
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
            double minutes;
            strncpy(degrees, buffer, 2);
            minutes = atof(buffer + 2);
            *parameter->lat = lat_dir * (atoi(degrees) + minutes / 60);
        } else if (nmea_field[nmea_cmd][cmd_field] == NMEA_LON) {
            char degrees[4] = {0};
            double minutes;
            strncpy(degrees, buffer, 3);
            minutes = atof(buffer + 3);
            *parameter->lon = lon_dir * (atoi(degrees) + minutes / 60);
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

static void set_ublox_config(uint rate) {
    nmea_msg("GLL", false);
    nmea_msg("GSV", false);
    nmea_msg("GSA", false);
    nmea_msg("VTG", false);
    nmea_msg("ZDA", false);
    nmea_msg("GGA", false);
    nmea_msg("RMC", false);
    ubx_cfg_msg(0x01, 0x07, true);  // Enable message UBX-NAV-PVT
    ubx_cfg_msg(0x01, 0x04, true);  // Enable message UBX-NAV-DOP
    ubx_cfg_rate(rate);             // Set messages rate (UBX-CFG-RATE (0x06 0x08))
    ubx_cfg_cfg();                  // Save changes
}

static void set_nmea_config(uint rate) {
    nmea_msg("GLL", false);
    nmea_msg("GSA", false);
    nmea_msg("GSV", false);
    nmea_msg("VTG", false);
    nmea_msg("ZDA", false);
    nmea_msg("GGA", true);
    nmea_msg("RMC", true);
    ubx_cfg_msg(0x01, 0x07, false);  // Disable message UBX-NAV-PVT
    ubx_cfg_msg(0x01, 0x04, false);  // Disable message UBX-NAV-DOP
    ubx_cfg_rate(rate);              // Set messages rate (UBX-CFG-RATE (0x06 0x08))
    ubx_cfg_cfg();                   // Save changes
}

static void set_baudrate(uint baudrate) {
    char msg[300];
    uint baudrates[] = {115200, 57600, 38400, 9600};
    sprintf(msg, "$PUBX,41,1,3,3,%u,0\r\n", baudrate);
    for (uint i = 0; i < sizeof(baudrates) / sizeof(uint); i++) {
        uart_pio_begin(baudrates[i], UART_TX_PIO_GPIO, UART_RX_PIO_GPIO, TIMEOUT_US, pio0, PIO0_IRQ_1, 8, 1,
                       UART_PARITY_NONE);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        uart_pio_write_bytes(msg, strlen(msg));
        vTaskDelay(200 / portTICK_PERIOD_MS);
        uart_pio_remove();
    }
}

static void nmea_msg(char *cmd, bool enable) {
    char msg[30];
    sprintf(msg, "$PUBX,40,%s,0,%u,0,0,0,0\r\n", cmd, enable);
    uart_pio_write_bytes(msg, strlen(msg));
}

static void ubx_cfg_msg(uint8_t class, uint8_t id, bool enable) {
    uint8_t msg[] = {0x06, 0x01, 0x03, 0x00, class, id, enable};
    send_ublox_message(msg, sizeof(msg));
}

static void ubx_cfg_rate(uint16_t rate) {
    uint16_t ms = 1000 / rate;
    uint8_t msg[] = {0x06, 0x08, 0x06, 0x00, ms, ms >> 8, 0x01, 0x00, 0x01, 0x00};
    send_ublox_message(msg, sizeof(msg));
}

static void ubx_cfg_cfg(void) {
    uint8_t msg[] = {0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
                     0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
    send_ublox_message(msg, sizeof(msg));
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

/*static int64_t alarm_nmea_timeout(alarm_id_t id, void *parameters) {
    alarm_parameters_t *parameter;
    parameter = (alarm_parameters_t *)parameters;
    parameter->is_ublox = true;
    set_ublox_config(parameter->rate);
    return 10000 * 1000L;
}

static int64_t alarm_ublox_timeout(alarm_id_t id, void *parameters) {
    alarm_parameters_t *parameter;
    parameter = (alarm_parameters_t *)parameters;
    parameter->is_ublox = false;
    //set_nmea_config(parameter->rate);
    set_ublox_config(parameter->rate);
    return 10000 * 1000L;
}*/