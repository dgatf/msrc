#include "sim_rx.h"

#include <stdio.h>

#include "config.h"
#include "hitec.h"
#include "uart.h"
#include "xbus.h"

#define SIM_RX_INTERVAL_MS 1000  // ms
#define SIM_RX_TIMEOUT_MS 1      // ms
#define IBUS_COMMAND_DISCOVER 0x8
#define IBUS_COMMAND_TYPE 0x9
#define IBUS_COMMAND_MEASURE 0xA

static uint8_t sim_rx_status = 0;
static QueueHandle_t uart_queue_handle;

static void process(rx_protocol_t rx_protocol);
static void ibus_send_data(uint8_t command, uint8_t address);
static void ibus_send_byte(uint8_t c, uint16_t *crcP);
static uint16_t jetiex_crc16(uint8_t *p, uint16_t len);
static uint16_t jetiex_update_crc16(uint16_t crc, uint8_t data);
static uint8_t smartport_get_crc(uint8_t *data);

void sim_rx_task(void *parameters) {
    sim_rx_parameters_t *parameter = (sim_rx_parameters_t *)parameters;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (UART_RECEIVER == uart0)
        uart_queue_handle = context.uart0_queue_handle;
    else
        uart_queue_handle = context.uart1_queue_handle;
    debug("\nSim Rx init");
    while (1) {
        vTaskDelay(SIM_RX_INTERVAL_MS / portTICK_PERIOD_MS);
        process(parameter->rx_protocol);
    }
}

static void process(rx_protocol_t rx_protocol) {
#ifdef SIM_RX
    // printf("\nSim (%u) < ", uxTaskGetStackHighWaterMark(NULL));
    xQueueReset(uart_queue_handle);
    if (rx_protocol == RX_SMARTPORT) {
        vTaskResume(context.led_task_handle);
        uint8_t c[10] = {0};
        c[0] = 0x7E;
        c[1] = 0x71;  // sensor id 18 = 0x71 (10 = 0xE9)
        xQueueSendToBack(uart_queue_handle, &c[0], 0);
        xQueueSendToBack(uart_queue_handle, &c[1], 0);
#ifdef SIM_SMARTPORT_SEND_CONFIG_LUA
        if (sim_rx_status == 0)  // maintenance mode on
        {
            c[2] = 0x21;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        } else if (sim_rx_status == 1)  // request config
        {
            c[2] = 0x30;  // type_id
            c[3] = 0x00;  // data_id
            c[4] = 0x50;
            c[5] = 0x00;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        } else if (sim_rx_status == 6)  // maintenance mode off
        {
            c[2] = 0x20;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        sim_rx_status++;
#endif
#ifdef SIM_SMARTPORT_RECEIVE_CONFIG_LUA
        if (sim_rx_status == 0)  // maintenance mode on
        {
            c[2] = 0x21;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        if (sim_rx_status == 1)  // packet 1
        {
            c[2] = 0x31;  // type_id
            c[3] = 0x00;  // data_id
            c[4] = 0x50;
            c[5] = 0xF1;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        if (sim_rx_status == 2)  // packet 2
        {
            c[2] = 0x31;  // type_id
            c[3] = 0x00;  // data_id
            c[4] = 0x50;
            c[5] = 0xF2;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        if (sim_rx_status == 3)  // packet 3
        {
            c[2] = 0x31;  // type_id
            c[3] = 0x00;  // data_id
            c[4] = 0x50;
            c[5] = 0xF3;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        if (sim_rx_status == 5)  // maintenance mode off
        {
            c[2] = 0x20;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        sim_rx_status++;
#endif
#ifdef SIM_SMARTPORT_SEND_SENSOR_ID
        if (sim_rx_status == 0)  // maintenance mode on
        {
            c[2] = 0x21;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        } else if (sim_rx_status == 1)  // request sensor id
        {
            c[2] = 0x30;  // type_id
            c[3] = 0x00;  // data_id
            c[4] = 0x50;
            c[5] = 0x01;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        } else if (sim_rx_status == 3)  // maintenance mode off
        {
            c[2] = 0x20;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        sim_rx_status++;
#endif
#ifdef SIM_SMARTPORT_RECEIVE_SENSOR_ID
        if (sim_rx_status == 0)  // maintenance mode on
        {
            c[2] = 0x21;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        } else if (sim_rx_status == 1)  // change sensor id
        {
            c[2] = 0x31;  // type_id
            c[3] = 0x00;  // data_id
            c[4] = 0x50;
            c[5] = 0x01;    // value
            c[6] = 10 - 1;  // sensor id = 10 -> lua sensor id = 9
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        } else if (sim_rx_status == 3)  // maintenance mode off
        {
            c[2] = 0x20;  // type_id
            c[3] = 0xFF;  // data_id
            c[4] = 0xFF;
            c[5] = 0x80;  // value
            c[6] = 0x00;
            c[7] = 0x00;
            c[8] = 0x00;
            c[9] = smartport_get_crc(c);
            for (uint i = 2; i < 10; i++) xQueueSendToBack(uart_queue_handle, &c[i], 0);
        }
        sim_rx_status++;
#endif
    }

    else if (rx_protocol == RX_XBUS) {
        uint8_t sensor_id[] = {
            XBUS_AIRSPEED_ID, XBUS_ALTIMETER_ID, XBUS_GPS_LOC_ID,     XBUS_GPS_STAT_ID,  XBUS_ESC_ID,
            XBUS_BATTERY_ID,  XBUS_VARIO_ID,     XBUS_RPMVOLTTEMP_ID, XBUS_FUEL_FLOW_ID, XBUS_STRU_TELE_DIGITAL_AIR_ID};
        static uint8_t sensor_index = 0;
        xbus_i2c_handler(sensor_id[sensor_index]);
        sensor_index++;
        if (sensor_index > sizeof(sensor_id)) sensor_index = 0;
    }

    else if (rx_protocol == RX_SRXL) {
        static uint8_t data[] = {0xA5, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        for (uint8_t i = 0; i < sizeof(data); i++) {
            xQueueSendToBack(uart_queue_handle, &data[i], 0);
        }
    }

    else if (rx_protocol == RX_SRXL2) {
        static uint8_t status = 0;
        static uint8_t handshake_request[] = {0xA6, 0x21, 0xE,  0x10, 0x30, 0xA,  0x1,
                                              0x1,  0xFC, 0x96, 0x8C, 0x4B, 0x30, 0xD9};
        static uint8_t telemetry_request[] = {0xA6, 0xCD, 0x14, 0x0,  0x30, 0xEC, 0x0, 0x0,  0x91, 0x0,
                                              0x0,  0x0,  0x60, 0x80, 0x0,  0x80, 0x0, 0x80, 0xB2, 0xA0};
        if (status < 10) {
            for (uint8_t i = 0; i < sizeof(handshake_request); i++) {
                xQueueSendToBack(uart_queue_handle, &handshake_request[i], 0);
            }
            status++;
        } else if (status == 10) {
            for (uint8_t i = 0; i < sizeof(telemetry_request); i++) {
                xQueueSendToBack(uart_queue_handle, &telemetry_request[i], 0);
            }
        }
    }

    else if (rx_protocol == RX_FRSKY_D) {
    }

    else if (rx_protocol == RX_IBUS) {
        static uint8_t command = IBUS_COMMAND_DISCOVER;
        static uint8_t address = 0;
        command = IBUS_COMMAND_MEASURE;
        ibus_send_data(command, address);
        address++;
        if (address == 16) {
            address = 0;
            if (command < IBUS_COMMAND_MEASURE) command++;
        }
    }

    else if (rx_protocol == RX_SBUS) {
        static uint8_t data[] = {0x0F, 0x0F, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04};
        for (uint8_t i = 0; i < sizeof(data); i++) {
            xQueueSendToBack(uart_queue_handle, &data[i], 0);
        }
        data[24] += 0x10;
        if (data[24] == 0x44) data[24] = 0x04;
    }

    else if (rx_protocol == RX_MULTIPLEX) {
        static uint8_t cont = 0;
        xQueueSendToBack(uart_queue_handle, &cont, 0);
        cont++;
        cont = cont % 16;
    }

    else if (rx_protocol == RX_JETIEX) {
        uint8_t data[] = {0x3E, 0x3,  0x28, 0x2,  0x31, 0x20, 0x80, 0x3E, 0xDD, 0x2E, 0xEB, 0x2E,
                          0xEC, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E,
                          0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E,
                          0xE0, 0x2E, 0xC5, 0xFE, 0x3D, 0x1,  0x8,  0x2,  0x3A, 0x0,  0xF9, 0xE2};
        for (uint8_t i = 0; i < sizeof(data); i++) {
            xQueueSendToBack(uart_queue_handle, &data[i], 0);
        }
    }

    else if (rx_protocol == RX_HITEC) {
        // hitec_i2c_handler();
    }

    else if (rx_protocol == RX_CRSF) {
        // uint8_t data[] = {0xC8, 0x06, 0x2C, 0xEE, 0xEF, 0x01, 0x00, 0x76};
        // uint8_t data[] = {0xC8, 0x4, 0x7, 0x0, 0x5, 0x8};
        uint8_t data[] = {0xC8, 0x18, 0x16, 0xDB, 0xE3, 0x62, 0xF9, 0xB6, 0xF7, 0x8B, 0xF2, 0x95, 0xAF,
                          0x7C, 0xE5, 0x2B, 0x5F, 0xF9, 0xCA, 0x7,  0x0,  0x0,  0x4C, 0x7C, 0xE2, 0xA7};
        /*uint8_t data[] = {0xC8, 0xC,  0x14, 0x11, 0x0,  0x64, 0xB,  0x0,  0x9,  0x7,  0x0,  0x0,  0x0,  0xF8,
                          0xC8, 0x18, 0x16, 0xDB, 0xE3, 0x62, 0xF9, 0xB6, 0xF7, 0x8B, 0xF2, 0x95, 0xAF, 0x7C,
                          0xE5, 0x2B, 0x5F, 0xF9, 0xCA, 0x7,  0x0,  0x0,  0x4C, 0x7C, 0xE2, 0xA7};*/
        for (uint8_t i = 0; i < sizeof(data); i++) {
            xQueueSendToBack(uart_queue_handle, &data[i], 0);
        }
    }

    else if (rx_protocol == RX_SANWA) {
        static uint8_t type = 0;
        static const uint8_t data[5][10] = {{0x1, 0x4, 0x82, 0x2, 0xFF, 0x0, 0x3, 0x3, 0xFF, 0x8D},
                                            {0x1, 0x1, 0x30, 0x4, 0xA7, 0x3, 0xFF, 0x3, 0xFF, 0xE1},
                                            {0x1, 0x1, 0x31, 0x4, 0xA7, 0x0, 0x3, 0x3, 0xFF, 0xE3},
                                            {0x1, 0x1, 0x30, 0x4, 0xA7, 0x3, 0xFF, 0x3, 0xFF, 0xE1},
                                            {0x1, 0x1, 0x31, 0x4, 0xA8, 0x0, 0x3, 0x3, 0xFF, 0xE4}};
        for (uint8_t i = 0; i < sizeof(data[type % 5]); i++) xQueueSendToBack(uart_queue_handle, &data[type % 5][i], 0);
        type++;
    }

    else if (rx_protocol == RX_HOTT) {
        uint8_t address[5] = {0x89, 0x8A, 0x8C, 0x8D, 0x8E};
        static uint index = 0;
        uint8_t type = 0x80;
        xQueueSendToBack(uart_queue_handle, &type, 0);
        xQueueSendToBack(uart_queue_handle, &address[index % 5], 0);
        index++;
    }

    else if (rx_protocol == RX_JR_PROPO) {
        static uint address = 0;
        xQueueSendToBack(uart_queue_handle, &address, 0);
        address++;
        if (address > 10) address = 0;
    }

    uart0_set_timestamp();
    vTaskDelay(SIM_RX_TIMEOUT_MS / portTICK_PERIOD_MS);
    xTaskNotifyGiveIndexed(context.uart0_notify_task_handle, 1);
#endif
}

static void ibus_send_data(uint8_t command, uint8_t address) {
    uint16_t crc = 0;
    uint8_t *u8P;

    // lenght
    ibus_send_byte(4, &crc);

    // command & address
    ibus_send_byte(command << 4 | address, &crc);

    // crc
    crc = 0xFFFF - crc;
    u8P = (uint8_t *)&crc;
    ibus_send_byte(u8P[0], NULL);
    ibus_send_byte(u8P[1], NULL);
}

void ibus_send_byte(uint8_t c, uint16_t *crcP) {
    if (crcP != NULL) {
        uint16_t crc = *crcP;
        crc += c;
        *crcP = crc;
    }
    xQueueSendToBack(uart_queue_handle, &c, 0);
}

static uint16_t jetiex_crc16(uint8_t *p, uint16_t len) {
    uint16_t crc16_data = 0;
    while (len--) {
        crc16_data = jetiex_update_crc16(crc16_data, p[0]);
        p++;
    }
    return (crc16_data);
}

static uint16_t jetiex_update_crc16(uint16_t crc, uint8_t data) {
    uint16_t ret_val;
    data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
    data ^= data << 4;
    ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
    return ret_val;
}

static uint8_t smartport_get_crc(uint8_t *data) {
    uint16_t crc = 0;
    for (uint8_t i = 2; i < 9; i++) {
        crc += data[i];
        crc += crc >> 8;
        crc &= 0x00FF;
    }
    return 0xFF - (uint8_t)crc;
}