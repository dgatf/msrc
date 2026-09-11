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
#define COMMAND_GSA 2
#define COMMAND_UNK 3

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
#define NMEA_END 11
#define NMEA_LAT 12
#define NMEA_GSA_FIX 13  // 1,2,3
#define NMEA_GSA_PDOP 14
#define NMEA_GSA_HDOP 15
#define NMEA_GSA_VDOP 16

#define TIMEOUT_US 5000
#define VSPEED_INTERVAL_MS 2000

typedef struct ublox_nav_pvt_t {
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
} __attribute__((packed)) ublox_nav_pvt_t;

typedef struct ublox_nav_dop_t {
    uint32_t iTOW;
    uint16_t gDOP;  // DOP * 100
    uint16_t pDOP;  // DOP * 100
    uint16_t tDOP;  // DOP * 100
    uint16_t vDOP;  // DOP * 100
    uint16_t hDOP;  // DOP * 100
    uint16_t nDOP;  // DOP * 100
    uint16_t eDOP;  // DOP * 100
} __attribute__((packed)) ublox_nav_dop_t;

typedef struct ublox_cfg_nav5_t {
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDOP;
    uint16_t tDOP;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgpsTimeOut;
    uint8_t cnoThreshNumSVs;
    uint8_t cnoThresh;
    uint8_t reserved1[2];
    uint16_t staticHoldMaxDist;
    uint8_t utcStandard;
    uint8_t reserved2[5];
} __attribute__((packed)) ublox_cfg_nav5_t;

typedef struct ublox_cfg_msg_t {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t msgRate;
} __attribute__((packed)) ublox_cfg_msg_t;

typedef struct ublox_cfg_rate_t {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
} __attribute__((packed)) ublox_cfg_rate_t;

typedef struct ublox_cfg_cfg_t {
    uint32_t clearMask;
    uint32_t saveMask;
    uint32_t loadMask;
    uint8_t deviceMask;
} __attribute__((packed)) ublox_cfg_cfg_t;

typedef struct ublox_header_t {
    uint16_t header;
    uint8_t class;
    uint8_t id;
    uint16_t size;
} __attribute__((packed)) ublox_header_t;

typedef struct ublox_crc_t {
    uint8_t crc_a;
    uint8_t crc_b;
} __attribute__((packed)) ublox_crc_t;

typedef struct ublox_msg_t {
    ublox_header_t header;
    uint8_t *payload;
    ublox_crc_t crc;
} ublox_msg_t;

static void process(gps_parameters_t *parameter);
static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, gps_parameters_t *parameter);
static void send_ublox_message(ublox_msg_t *msg);
static void set_baudrate(uint baudrate);
static void set_nmea_config(uint rate, uint dynmodel);
static void set_ublox_config(uint rate, uint dynmodel);
static void nmea_msg(char *cmd, bool enable);
static void ubx_cfg_msg(uint8_t class, uint8_t id, bool enable);
static void ubx_cfg_rate(uint16_t rate);
static void ubx_cfg_dynmodel(uint8_t dynmodel);
static void ubx_cfg_cfg(void);
static bool set_home_altitude(uint fix_type);
static bool check_ublox_crc(ublox_msg_t *msg);
static void set_ublox_crc(ublox_msg_t *msg);

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
    *parameter.fix = 0;       // raw fix: nmea or ublox
    *parameter.fix_type = 0;  // internal fix MSRC. 0 no fix, 1 2D fix, 2 3D fix
    *parameter.home_set = false;
    *parameter.n_vel = 0;
    *parameter.e_vel = 0;
    *parameter.v_vel = 0;
    *parameter.speed_acc = 0;
    *parameter.track_acc = 0;
    *parameter.alt_elipsiod = 0;
    *parameter.h_acc = 0;
    *parameter.v_acc = 0;
    *parameter.alt_home = 0;
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
    parameters_distance.distance = parameter.dist;
    parameters_distance.altitude = parameter.alt;
    parameters_distance.sat = parameter.sat;
    parameters_distance.latitude = parameter.lat;
    parameters_distance.longitude = parameter.lon;
    parameters_distance.fix = parameter.fix_type;
    parameters_distance.hdop = parameter.hdop;
    parameters_distance.home_set = parameter.home_set;
    xTaskCreate(distance_task, "distance_task", STACK_DISTANCE, (void *)&parameters_distance, 2, &task_handle);

    /* Change GPS config. For ublox compatible devices */

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_baudrate(parameter.baudrate);
    uart_pio_begin(parameter.baudrate, UART_TX_PIO_GPIO, UART_RX_PIO_GPIO, TIMEOUT_US, pio0, PIO0_IRQ_1, 8, 1,
                   UART_PARITY_NONE);
    if (parameter.protocol == UBLOX)
        set_ublox_config(parameter.rate, parameter.dynmodel);
    else
        set_nmea_config(parameter.rate, parameter.dynmodel);

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
                    // If we were ignoring checksum, reset cleanly
                    if (cmd_field == 255) {
                        cmd_field = 0;
                        data_pos = 0;
                        nmea_cmd = COMMAND_UNK;
                        break;
                    }
                case ',':
                    if (cmd_field == 255) {  // ignoring checksum
                        data_pos = 0;
                        buffer[0] = 0;
                        break;
                    }
                    if (cmd_field == 0) {
                        if (memcmp(buffer + 2, "GGA", 3) == 0) {
                            nmea_cmd = COMMAND_GGA;
                        } else if (memcmp(buffer + 2, "RMC", 3) == 0) {
                            nmea_cmd = COMMAND_RMC;
                        } else if (memcmp(buffer + 2, "GSA", 3) == 0) {
                            nmea_cmd = COMMAND_GSA;
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
                case '*':
                    // End current field; ignore checksum bytes until '\r'
                    if (nmea_cmd != COMMAND_UNK && cmd_field != 0) {
                        parser(nmea_cmd, cmd_field, (uint8_t *)buffer, parameter);
                    }
                    cmd_field = 255;  // special: ignore until '\r'
                    data_pos = 0;
                    buffer[0] = 0;
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
        bool sync = false;
        while (uart_pio_available() > 1) {
            if (uart_pio_read() == 0xB5) {
                if (uart_pio_read() == 0x62) {
                    sync = true;
                    break;
                }
            }
        }
        if (!sync) return;
        if (uart_pio_available() < sizeof(ublox_header_t) - 2) return;
        ublox_msg_t msg;
        msg.header = (ublox_header_t){.header = 0x62B5};
        uart_pio_read_bytes((uint8_t *)&msg.header + 2, sizeof(ublox_header_t) - 2);
        debug("\nGPS UBLOX MSG. Class: %u Id: %u Len: %u Avail: %u", msg.header.class, msg.header.id, msg.header.size,
              uart_pio_available());
        if (uart_pio_available() < msg.header.size + 2) return;

        // NAV PVT
        if (msg.header.class == 0x01 && msg.header.id == 0x07 && msg.header.size == sizeof(ublox_nav_pvt_t)) {
            ublox_nav_pvt_t navpvt;
            uart_pio_read_bytes((uint8_t *)&navpvt, sizeof(ublox_nav_pvt_t));
            uart_pio_read_bytes((uint8_t *)&msg.crc, sizeof(ublox_crc_t));
            msg.payload = (uint8_t *)&navpvt;
            if (!check_ublox_crc(&msg)) {
                debug("\nGPS UBLOX MSG. CRC ERROR");
                return;
            }
            *parameter->alt = navpvt.hMSL / 1000.0F;
            *parameter->lat = navpvt.lat * 1.0e-7;
            *parameter->lon = navpvt.lon * 1.0e-7;
            *parameter->alt = navpvt.hMSL / 1000.0F;
            *parameter->cog = navpvt.headMot / 100000.0F;
            *parameter->sat = navpvt.numSV;
            *parameter->time = navpvt.hour * 10000L + navpvt.min * 100 + navpvt.sec;
            *parameter->date = navpvt.day * 10000L + navpvt.month * 100 + (navpvt.year - 2000);
            *parameter->vspeed = -navpvt.velD / 1000.0F;
            *parameter->spd_kmh = navpvt.gSpeed * 3600.0F / 1000000.0F;
            *parameter->spd = navpvt.gSpeed * 0.001943844F;  // 1 mm/s = 0.001943844 Knot
            uint8_t fix = navpvt.fixType;
            *parameter->fix = fix;
            if (fix == 2)
                *parameter->fix_type = 1;  // 2D
            else if (fix == 3 || fix == 4)
                *parameter->fix_type = 2;  // 3D
            else
                *parameter->fix_type = 0;  // no fix
            if (!(navpvt.flags & 0x01)) *parameter->fix_type = 0;
            *parameter->n_vel = navpvt.velN / 1000.0F;
            *parameter->e_vel = navpvt.velE / 1000.0F;
            *parameter->v_vel = -navpvt.velD / 1000.0F;
            *parameter->speed_acc = navpvt.sAcc / 1000.0F;
            *parameter->track_acc = navpvt.headAcc * 1e-5f;
            *parameter->alt_elipsiod = navpvt.height / 1000.0F;
            *parameter->h_acc = navpvt.hAcc / 1000.0F;
            *parameter->v_acc = navpvt.vAcc / 1000.0F;
            *parameter->pdop = navpvt.pDOP / 100.0F;
            if (set_home_altitude(*parameter->fix_type)) {
                *parameter->alt_home = *parameter->alt;
            }
            debug(
                "\nGPS (%u) < NAV-PTV: Date: %.0f Time: %.0f Fix: %.0f Sat: %.0f Lon: %.5f Lat: %.5f Alt: %.2f "
                "Vspeed: %.2f Speed: mm/s %i knots %.2f kmh %.2f Pdop: %.2f, Alt home: %.2f",
                uxTaskGetStackHighWaterMark(NULL), *parameter->date, *parameter->time, *parameter->fix, *parameter->sat,
                *parameter->lon, *parameter->lat, *parameter->alt, *parameter->vspeed, navpvt.gSpeed, *parameter->spd,
                *parameter->spd_kmh, *parameter->pdop, *parameter->alt_home);
        }

        // NAV DOP
        else if (msg.header.class == 0x01 && msg.header.id == 0x04 && msg.header.size == sizeof(ublox_nav_dop_t)) {
            ublox_nav_dop_t navdop;
            uart_pio_read_bytes((uint8_t *)&navdop, sizeof(ublox_nav_dop_t));
            uart_pio_read_bytes((uint8_t *)&msg.crc, sizeof(ublox_crc_t));
            msg.payload = (uint8_t *)&navdop;
            if (!check_ublox_crc(&msg)) {
                debug("\nGPS UBLOX MSG. CRC ERROR");
                return;
            }
            *parameter->hdop = navdop.hDOP / 100.0F;
            *parameter->vdop = navdop.vDOP / 100.0F;
            debug("\nGPS (%u) < NAV-DOP: h: %.2f v: %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter->hdop,
                  *parameter->vdop);
        }

        // Unknown
        else {
            for (uint i = 0; i < msg.header.size + sizeof(ublox_crc_t); i++) {
                uart_pio_read();
            }
        }
    }
}

static bool set_home_altitude(uint fix_type) {
    static bool altitude_offset_set = false;
    if (!altitude_offset_set) {
        static uint cont = 0;
        if (fix_type == 2) {
            cont++;
            if (cont > 5) {
                altitude_offset_set = true;
                return true;
            }
        } else
            cont = 0;
    }
    return false;
}

static void parser(uint8_t nmea_cmd, uint8_t cmd_field, uint8_t *buffer, gps_parameters_t *parameter) {
#define NMEA_MAX_FIELDS 18  // 0..17 (field 1..17 plus dummy 0)

    uint8_t nmea_field[3][NMEA_MAX_FIELDS] = {
        // GGA: time, lat, N/S, lon, E/W, fix, sats, hdop, alt
        {0, NMEA_TIME, NMEA_LAT, NMEA_LAT_SIGN, NMEA_LON, NMEA_LON_SIGN, NMEA_FIX, NMEA_SAT, 0, NMEA_ALT, 0, 0, 0, 0, 0,
         0, 0, 0},

        // RMC: time, status, lat, N/S, lon, E/W, spd, cog, date
        {0, NMEA_TIME, 0, NMEA_LAT, NMEA_LAT_SIGN, NMEA_LON, NMEA_LON_SIGN, NMEA_SPD, NMEA_COG, NMEA_DATE, 0, 0, 0, 0,
         0, 0, 0, 0},

        // GSA: mode1, fix, sat1..sat12, PDOP, HDOP, VDOP
        {0, 0, NMEA_GSA_FIX, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // sat1..sat12 ignored (fields 3..14)
         NMEA_GSA_PDOP, NMEA_GSA_HDOP, NMEA_GSA_VDOP}};
    static int8_t lat_dir = 1, lon_dir = 1;
    static uint32_t timestamp_vspeed = 0, timestamp_dist = 0;
    if (strlen(buffer)) {
        if (cmd_field >= NMEA_MAX_FIELDS) return;
        uint8_t field_type = nmea_field[nmea_cmd][cmd_field];
        if (field_type == NMEA_TIME) {
            *parameter->time = atof(buffer);
        } else if (field_type == NMEA_LAT) {
            char degrees[3] = {0};
            float minutes = 0;
            strncpy(degrees, buffer, 2);
            minutes = atof(buffer + 2);
            *parameter->lat = atoi(degrees) + minutes / 60;
        } else if (field_type == NMEA_LON) {
            char degrees[4] = {0};
            float minutes = 0;
            strncpy(degrees, buffer, 3);
            minutes = atof(buffer + 3);
            *parameter->lon = atoi(degrees) + minutes / 60;
        } else if (field_type == NMEA_ALT) {
            *parameter->alt = atof(buffer);
            get_vspeed_gps(parameter->vspeed, *parameter->alt, VSPEED_INTERVAL_MS);
            if (set_home_altitude(*parameter->fix_type)) {
                *parameter->alt_home = *parameter->alt;
            }
            debug("(alt home %.2f),", *parameter->alt_home);
        } else if (field_type == NMEA_SPD) {
            *parameter->spd = atof(buffer);
            *parameter->spd_kmh = *parameter->spd * 1.852;
        } else if (field_type == NMEA_COG) {
            *parameter->cog = atof(buffer);
        } else if (field_type == NMEA_DATE) {
            *parameter->date = atof(buffer);
        } else if (field_type == NMEA_SAT) {
            *parameter->sat = atof(buffer);
        } else if (field_type == NMEA_LAT_SIGN) {
            lat_dir = (buffer[0] == 'N') ? 1 : -1;
            *parameter->lat = fabsf(*parameter->lat) * lat_dir;
        } else if (field_type == NMEA_LON_SIGN) {
            lon_dir = (buffer[0] == 'E') ? 1 : -1;
            *parameter->lon = fabsf(*parameter->lon) * lon_dir;
        } else if (field_type == NMEA_GSA_HDOP) {
            *parameter->hdop = atof(buffer);
        } else if (field_type == NMEA_GSA_VDOP) {
            *parameter->vdop = atof(buffer);
        } else if (field_type == NMEA_GSA_PDOP) {
            *parameter->pdop = atof(buffer);
        } else if (field_type == NMEA_GSA_FIX) {
            uint fix = atoi((char *)buffer);
            if (fix >= 1 && fix <= 3) {
                *parameter->fix = fix;
                if (fix == 2)
                    *parameter->fix_type = 1;  // 2D
                else if (fix == 3)
                    *parameter->fix_type = 2;  // 3D
                else
                    *parameter->fix_type = 0;  // no fix
            }
        }
        debug("%s(%i),", buffer, field_type);
    }
}

static void set_ublox_config(uint rate, uint dynmodel) {
    nmea_msg("GLL", false);
    nmea_msg("GSV", false);
    nmea_msg("GSA", false);
    nmea_msg("VTG", false);
    nmea_msg("ZDA", false);
    nmea_msg("GGA", false);
    nmea_msg("RMC", false);
    ubx_cfg_msg(0x01, 0x07, true);  // Enable message UBX-NAV-PVT
    ubx_cfg_msg(0x01, 0x04, true);  // Enable message UBX-NAV-DOP
    ubx_cfg_dynmodel(dynmodel);     // Set dynmodel UBX-CFG-NAV5
    ubx_cfg_rate(rate);             // Set messages rate (UBX-CFG-RATE (0x06 0x08))
    ubx_cfg_cfg();                  // Save changes
}

static void set_nmea_config(uint rate, uint dynmodel) {
    nmea_msg("GLL", false);
    nmea_msg("GSA", true);
    nmea_msg("GSV", false);
    nmea_msg("VTG", false);
    nmea_msg("ZDA", false);
    nmea_msg("GGA", true);
    nmea_msg("RMC", true);
    ubx_cfg_msg(0x01, 0x07, false);  // Disable message UBX-NAV-PVT
    ubx_cfg_msg(0x01, 0x04, false);  // Disable message UBX-NAV-DOP
    ubx_cfg_dynmodel(dynmodel);      // Set dynmodel UBX-CFG-NAV5
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
    ublox_msg_t ublox_msg;
    ublox_msg.header = (ublox_header_t){.header = 0x62B5, .class = 0x06, .id = 0x01, .size = sizeof(ublox_cfg_msg_t)};
    ublox_cfg_msg_t payload = {.msgClass = class, .msgID = id, .msgRate = enable ? 1 : 0};
    ublox_msg.payload = (uint8_t *)&payload;
    send_ublox_message(&ublox_msg);
}

static void ubx_cfg_rate(uint16_t rate) {
    ublox_msg_t ublox_msg;
    ublox_msg.header = (ublox_header_t){.header = 0x62B5, .class = 0x06, .id = 0x08, .size = sizeof(ublox_cfg_rate_t)};
    ublox_cfg_rate_t payload = {.measRate = 1000 / rate, .navRate = 1, .timeRef = 0};
    ublox_msg.payload = (uint8_t *)&payload;
    send_ublox_message(&ublox_msg);
}

static void ubx_cfg_cfg(void) {
    ublox_msg_t ublox_msg;
    ublox_msg.header = (ublox_header_t){.header = 0x62B5, .class = 0x06, .id = 0x09, .size = sizeof(ublox_cfg_cfg_t)};
    ublox_cfg_cfg_t payload = {.clearMask = 0x00, .saveMask = 0xFFFF, .loadMask = 0x00, .deviceMask = 0x03};
    ublox_msg.payload = (uint8_t *)&payload;
    send_ublox_message(&ublox_msg);
}

static void ubx_cfg_dynmodel(uint8_t value) {
    ublox_msg_t ublox_msg;
    ublox_msg.header = (ublox_header_t){.header = 0x62B5, .class = 0x06, .id = 0x24, .size = sizeof(ublox_cfg_nav5_t)};
    ublox_cfg_nav5_t payload = {.mask = 0x0001, .dynModel = value};
    ublox_msg.payload = (uint8_t *)&payload;
    send_ublox_message(&ublox_msg);
}

static inline void send_ublox_message(ublox_msg_t *msg) {
    set_ublox_crc(msg);
    uart_pio_write_bytes((uint8_t *)&msg->header, sizeof(ublox_header_t));
    uart_pio_write_bytes((uint8_t *)msg->payload, msg->header.size);
    uart_pio_write_bytes((uint8_t *)&msg->crc, sizeof(ublox_crc_t));
}

static void set_ublox_crc(ublox_msg_t *msg) {
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (uint i = 2; i < sizeof(ublox_header_t); i++) {
        CK_A += ((uint8_t *)msg)[i];
        CK_B += CK_A;
    }
    for (uint i = 0; i < msg->header.size; i++) {
        CK_A += ((uint8_t *)msg->payload)[i];
        CK_B += CK_A;
    }
    msg->crc.crc_a = CK_A;
    msg->crc.crc_b = CK_B;
}

static bool check_ublox_crc(ublox_msg_t *msg) {
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (uint i = 2; i < sizeof(ublox_header_t); i++) {
        CK_A += ((uint8_t *)msg)[i];
        CK_B += CK_A;
    }
    for (uint i = 0; i < msg->header.size; i++) {
        CK_A += ((uint8_t *)msg->payload)[i];
        CK_B += CK_A;
    }
    return (msg->crc.crc_a == CK_A) && (msg->crc.crc_b == CK_B);
}
