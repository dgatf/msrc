#include "smart_esc.h"

#include <stdio.h>

#include "auto_offset.h"
#include "capture_edge.h"
#include "cell_count.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "srxl.h"
#include "srxl2.h"
#include "string.h"
#include "uart.h"

#define SRXL2_RECEIVER_ID 0x21
#define SRXL2_ESC_ID 0x40
#define SRXL2_RECEIVER_PRIORITY 0xA
#define SRXL2_RECEIVER_BAUDRATE 1
#define SRXL2_RECEIVER_INFO 0x7
#define SRXL2_INTERVAL_MS 10
#define SRXL2_RECEIVER_UID 0x27A2C29C  // 0x12345678

#define XBUS_SMART_BAT 0x42
#define XBUS_SMART_BAT_REALTIME 0x00
#define XBUS_SMART_BAT_CELLS_1 0x10
#define XBUS_SMART_BAT_CELLS_2 0x20
#define XBUS_SMART_BAT_CELLS_3 0x30
#define XBUS_SMART_BAT_ID 0x80
#define XBUS_SMART_BAT_LIMITS 0x90

#define PWM_TIMEOUT_MS 100
#define CLOCK_DIV 1

typedef struct srxl2_smart_bat_realtime_t {
    uint8_t identifier;  // Source device 0x42
    uint8_t s_id;        // Secondary ID
    uint8_t type;        // 0x00
    int8_t temp;
    uint32_t current;      // A
    uint16_t consumption;  // mAh
    uint16_t min_cel;
    uint16_t max_cel;
} __attribute__((packed)) srxl2_smart_bat_realtime_t;

typedef struct srxl2_smart_bat_cells_1_t {
    uint8_t identifier;  // Source device 0x42
    uint8_t s_id;        // Secondary ID
    uint8_t type;        // 0x10
    int8_t temp;
    uint16_t cell_1;  // V * 1000
    uint16_t cell_2;
    uint16_t cell_3;
    uint16_t cell_4;
    uint16_t cell_5;
    uint16_t cell_6;
} __attribute__((packed)) srxl2_smart_bat_cells_1_t;

typedef struct srxl2_smart_bat_cells_2_t {
    uint8_t identifier;  // Source device 0x42
    uint8_t s_id;        // Secondary ID
    uint8_t type;        // 0x20
    int8_t temp;
    uint16_t cell_7;
    uint16_t cell_8;
    uint16_t cell_9;
    uint16_t cell_10;
    uint16_t cell_11;
    uint16_t cell_12;
} __attribute__((packed)) srxl2_smart_bat_cells_2_t;

typedef struct srxl2_smart_bat_cells_3_t {
    uint8_t identifier;  // Source device 0x42
    uint8_t s_id;        // Secondary ID
    uint8_t type;        // 0x30
    int8_t temp;
    uint16_t cell_13;
    uint16_t cell_14;
    uint16_t cell_15;
    uint16_t cell_16;
    uint16_t cell_17;
    uint16_t cell_18;
} __attribute__((packed)) srxl2_smart_bat_cells_3_t;

typedef struct srxl2_smart_bat_id_t {
    uint8_t identifier;  // Source device 0x42
    uint8_t s_id;        // Secondary ID
    uint8_t type;        // 0x80
    uint8_t chemistery;
    uint8_t cells;
    uint8_t mfg_code;
    uint16_t cycles;
    uint8_t uid;
} __attribute__((packed)) srxl2_smart_bat_id_t;

typedef struct srxl2_smart_bat_limits_t {
    uint8_t identifier;  // Source device 0x42
    uint8_t s_id;        // Secondary ID
    uint8_t type;        // 0x90
    uint8_t rfu;
    uint16_t capacity;
    uint16_t discharge_rate;
    uint16_t overdischarge;
    uint16_t zero_capacity;
    uint16_t fully_charged;
    uint8_t min_temp;
    uint8_t mex_temp;
} __attribute__((packed)) srxl2_smart_bat_limits_t;

typedef struct srxl2_channel_data_t {
    int8_t rssi;
    uint16_t frame_losses;
    uint32_t channel_mask;
    uint16_t channel_data_ch1;  // throttle
    uint16_t channel_data_ch7;  // reverse
} __attribute__((packed)) srxl2_channel_data_t;

typedef struct srxl2_control_packet_t {
    uint8_t header;
    uint8_t type;
    uint8_t len;
    uint8_t command;
    uint8_t reply_id;
    srxl2_channel_data_t channel_data;
    uint16_t crc;
} __attribute__((packed)) srxl2_control_packet_t;

#define SRXL2_CONTROL_LEN_CHANNEL (5 + sizeof(srxl2_channel_data_t) + 2)  // header + channel data + crc

static volatile uint8_t esc_id = 0, esc_priority = 10;
static volatile uint16_t throttle = 0, reverse = 0;
static volatile bool packet_pending = false;

static void process(smart_esc_parameters_t *parameter);
static void read_packet(uint8_t *buffer, smart_esc_parameters_t *parameter);
static void send_packet(void);
static void capture_pwm_throttle_handler(uint counter, edge_type_t edge);
static void capture_pwm_reverse_handler(uint counter, edge_type_t edge);
static int64_t timeout_throttle_callback(alarm_id_t id, void *user_data);
static int64_t timeout_reverse_callback(alarm_id_t id, void *user_data);
static int64_t alarm_packet(alarm_id_t id, void *user_data);

void smart_esc_task(void *parameters) {
    smart_esc_parameters_t parameter = *(smart_esc_parameters_t *)parameters;
    *parameter.rpm = 0;
    *parameter.voltage = 0;
    *parameter.current = 0;
    *parameter.temperature_fet = 0;
    *parameter.temperature_bec = 0;
    *parameter.voltage_bec = 0;
    *parameter.current_bec = 0;
    *parameter.current_bat = 0;
    *parameter.consumption = 0;
    xTaskNotifyGive(context.receiver_task_handle);
#ifdef SIM_SENSORS
    *parameter.rpm = 12345.67;
    *parameter.consumption = 123.4;
    *parameter.voltage = 12.34;
    *parameter.current = 5.678;
    *parameter.temperature_fet = 12.34;
    *parameter.temperature_bec = 23.45;
    *parameter.cell_voltage = 3.75;
#endif
    capture_edge_init(pio0, SMART_ESC_PWM_GPIO, 2, CLOCK_DIV, PIO0_IRQ_0);
    capture_edge_set_handler(0, capture_pwm_throttle_handler);
    capture_edge_set_handler(1, capture_pwm_reverse_handler);
    gpio_pull_up(SMART_ESC_PWM_GPIO);
    gpio_pull_up(SMART_ESC_PWM_GPIO + 1);
    uart1_begin(115200, UART1_TX_GPIO, UART_ESC_RX, SRXL2_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    add_alarm_in_us(0, alarm_packet, NULL, true);

    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process(&parameter);
        if (packet_pending) {
            packet_pending = false;
            send_packet();
        }
    }
}

static void process(smart_esc_parameters_t *parameter) {
    static uint32_t timestamp = 0;
    uint8_t length = uart1_available();
    if (length) {
        uint8_t data[length];
        uart1_read_bytes(data, length);
        debug("\nSmart ESC (%u) < ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(data, length, " 0x%X");
        uint16_t crc = srxl_get_crc(data, length - 2);
        if ((crc >> 8) != data[length - 2] || (crc & 0xFF) != data[length - 1]) {
            debug(" -> BAD CRC 0x%X", crc);
            return;
        }
        // Request for handshake to enable srxl2 mode
        if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_HANDSHAKE && data[4] == 0) {
            debug("\nSmart ESC. Handshake request from 0x%X", data[3]);
            if (data[3] == SRXL2_ESC_ID)
                srxl2_send_handshake(uart1, SRXL2_RECEIVER_ID, SRXL2_ESC_ID, SRXL2_RECEIVER_PRIORITY,
                                     SRXL2_RECEIVER_BAUDRATE, SRXL2_RECEIVER_INFO, SRXL2_RECEIVER_UID);
        }
        // Handshake confirmation
        else if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_HANDSHAKE && data[3] == SRXL2_ESC_ID) {
            esc_id = data[3];
            esc_priority = data[5];
            debug("\nSmart ESC. Handshake with 0x%X completed ", esc_id);
        }
        // Telemetry packet
        else if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_TELEMETRY && data[3] == SRXL2_RECEIVER_ID) {
            read_packet(data + 4, parameter);
        }
        // Re-handshake request
        else if (data[0] == SRXL2_HEADER && data[1] == SRXL2_PACKET_TYPE_TELEMETRY && data[3] == 0xFF) {
            srxl2_send_handshake(uart1, SRXL2_RECEIVER_ID, SRXL2_ESC_ID, SRXL2_RECEIVER_PRIORITY,
                                 SRXL2_RECEIVER_BAUDRATE, SRXL2_RECEIVER_INFO, SRXL2_RECEIVER_UID);
        }
    }
}

static void read_packet(uint8_t *buffer, smart_esc_parameters_t *parameter) {
    if (buffer[0] == XBUS_ESC_ID) {
        xbus_esc_t esc;
        memcpy(&esc, buffer, sizeof(xbus_esc_t));
        *parameter->rpm = swap_16(esc.rpm) * 10;
        *parameter->voltage = swap_16(esc.volts_input) / 100.0;
        *parameter->current = swap_16(esc.current_motor) / 100.0;
        *parameter->voltage_bec = esc.voltage_bec == 0xFF ? 0 : esc.voltage_bec / 20.0;
        *parameter->current_bec = esc.current_bec == 0xFF ? 0 : esc.current_bec / 100.0;
        *parameter->temperature_fet = esc.temp_fet == 0xFFFF ? 0 : (swap_16(esc.temp_fet) / 10.0);
        *parameter->temperature_bec = esc.temp_bec == 0xFFFF ? 0 : swap_16(esc.temp_bec) / 10.0;
        debug("\nSmart ESC (%u) < Rpm: %.0f Volt: %0.2f Curr: %.2f TempFet: %.0f TempBec: %.0f Vbec: %.1f Cbec: %.1f ",
              uxTaskGetStackHighWaterMark(NULL), *parameter->rpm, *parameter->voltage, *parameter->current,
              *parameter->temperature_fet, *parameter->temperature_bec, *parameter->voltage_bec,
              *parameter->current_bec);
    } else if (buffer[0] == XBUS_SMART_BAT) {
        switch (buffer[2]) {
            case XBUS_SMART_BAT_REALTIME: {
                srxl2_smart_bat_realtime_t bat_realtime;
                memcpy(&bat_realtime, buffer, sizeof(srxl2_smart_bat_realtime_t));
                *parameter->temperature_bat = bat_realtime.temp;
                *parameter->current_bat = bat_realtime.current / 1000;
                *parameter->consumption = bat_realtime.consumption / 10;
                debug("\nSmart ESC (Bat 0x%X) (%u) < Temp: %.0f Curr: %0.2f Cons: %.2f ", XBUS_SMART_BAT_REALTIME,
                      uxTaskGetStackHighWaterMark(NULL), *parameter->temperature_bat, *parameter->current_bat,
                      *parameter->consumption);
                break;
            }
            case XBUS_SMART_BAT_CELLS_1: {
                srxl2_smart_bat_cells_1_t bat_cells_1;
                memcpy(&bat_cells_1, buffer, sizeof(srxl2_smart_bat_cells_1_t));
                *parameter->temperature_bat = bat_cells_1.temp;
                *parameter->cell[1] = bat_cells_1.cell_1 / 1000.0;
                *parameter->cell[2] = bat_cells_1.cell_2 / 1000.0;
                *parameter->cell[3] = bat_cells_1.cell_3 / 1000.0;
                *parameter->cell[4] = bat_cells_1.cell_4 / 1000.0;
                *parameter->cell[5] = bat_cells_1.cell_5 / 1000.0;
                *parameter->cell[6] = bat_cells_1.cell_6 / 1000.0;
                debug("\nSmart ESC (Bat 0x%X) (%u) < Temp: %.0f C1: %.3f C2: %.3f C3: %.3f C4: %.3f C5: %.3f C6: %.3f ",
                      XBUS_SMART_BAT_CELLS_1, uxTaskGetStackHighWaterMark(NULL), *parameter->temperature_bat,
                      *parameter->cell[1], *parameter->cell[2], *parameter->cell[3], *parameter->cell[4],
                      *parameter->cell[5], *parameter->cell[6]);
                break;
            }
            case XBUS_SMART_BAT_CELLS_2: {
                srxl2_smart_bat_cells_2_t bat_cells_2;
                memcpy(&bat_cells_2, buffer, sizeof(srxl2_smart_bat_cells_2_t));
                *parameter->temperature_bat = bat_cells_2.temp;
                *parameter->cell[7] = bat_cells_2.cell_7 / 1000.0;
                *parameter->cell[8] = bat_cells_2.cell_8 / 1000.0;
                *parameter->cell[9] = bat_cells_2.cell_9 / 1000.0;
                *parameter->cell[10] = bat_cells_2.cell_10 / 1000.0;
                *parameter->cell[11] = bat_cells_2.cell_11 / 1000.0;
                *parameter->cell[12] = bat_cells_2.cell_12 / 1000.0;
                debug(
                    "\nSmart ESC (Bat 0x%X) (%u) < Temp: %.0f C7: %.3f C8: %.3f C9: %.3f C10: %.3f C11: %.3f C12: "
                    "%.3f ",
                    XBUS_SMART_BAT_CELLS_2, uxTaskGetStackHighWaterMark(NULL), *parameter->temperature_bat,
                    *parameter->cell[7], *parameter->cell[8], *parameter->cell[9], *parameter->cell[10],
                    *parameter->cell[11], *parameter->cell[12]);
                break;
            }
            case XBUS_SMART_BAT_CELLS_3: {
                srxl2_smart_bat_cells_3_t bat_cells_3;
                memcpy(&bat_cells_3, buffer, sizeof(srxl2_smart_bat_cells_3_t));
                *parameter->temperature_bat = bat_cells_3.temp;
                *parameter->cell[13] = bat_cells_3.cell_13 / 1000.0;
                *parameter->cell[14] = bat_cells_3.cell_14 / 1000.0;
                *parameter->cell[15] = bat_cells_3.cell_15 / 1000.0;
                *parameter->cell[16] = bat_cells_3.cell_16 / 1000.0;
                *parameter->cell[17] = bat_cells_3.cell_17 / 1000.0;
                *parameter->cell[18] = bat_cells_3.cell_18 / 1000.0;
                debug(
                    "\nSmart ESC (Bat 0x%X) (%u) < Temp: %.0f C13: %.3f C14: %.3f C15: %.3f C16: %.3f C17: %.3f C18: "
                    "%.3f ",
                    XBUS_SMART_BAT_CELLS_3, uxTaskGetStackHighWaterMark(NULL), *parameter->temperature_bat,
                    *parameter->cell[13], *parameter->cell[14], *parameter->cell[15], *parameter->cell[16],
                    *parameter->cell[17], *parameter->cell[18]);
                break;
            }
            case XBUS_SMART_BAT_ID: {
                srxl2_smart_bat_id_t bat_id;
                memcpy(&bat_id, buffer, sizeof(srxl2_smart_bat_id_t));
                *parameter->cells = bat_id.cells;
                *parameter->cycles = bat_id.cycles;
                debug("\nSmart ESC (Bat 0x%X) (%u) < Cells: %u Cycles: %u ", XBUS_SMART_BAT_ID,
                      uxTaskGetStackHighWaterMark(NULL), *parameter->cells, *parameter->cycles);
                break;
            }
        }
    }
}

static void send_packet(void) {
    static uint cont = 0;
    if (!esc_id) {
        srxl2_send_handshake(uart1, SRXL2_RECEIVER_ID, SRXL2_ESC_ID, SRXL2_RECEIVER_PRIORITY, SRXL2_RECEIVER_BAUDRATE,
                             SRXL2_RECEIVER_INFO, SRXL2_RECEIVER_UID);
    } else {
        srxl2_control_packet_t packet;
        packet.header = SRXL2_HEADER;
        packet.type = SRXL2_PACKET_TYPE_CONTROL;
        packet.len = SRXL2_CONTROL_LEN_CHANNEL;
        packet.command = SRXL2_CONTROL_CMD_CHANNEL;
        if (!(cont % 10)) packet.reply_id = esc_id;
        srxl2_channel_data_t channel_data;
        channel_data.rssi = 0x64;
        channel_data.frame_losses = 0;
        channel_data.channel_mask = 0B1000001;  // ch1 throttle, ch7 reverse
        channel_data.channel_data_ch1 = throttle;
        channel_data.channel_data_ch7 = reverse;
        packet.channel_data = channel_data;
        uint16_t crc = srxl_get_crc((uint8_t *)&packet, sizeof(packet) - 2);
        packet.crc = swap_16(crc);
        uart1_write_bytes((uint8_t *)&packet, sizeof(packet));
        cont++;
        debug("\nSmart ESC (%u) Thr: %u Rev: %u > ", uxTaskGetStackHighWaterMark(NULL), throttle, reverse);
        debug_buffer((uint8_t *)&packet, sizeof(packet), " 0x%X");
    }
}

static void capture_pwm_throttle_handler(uint counter, edge_type_t edge) {
    static uint counter_edge_rise = 0;
    static alarm_id_t timeout_throttle_alarm_id = 0;
    if (timeout_throttle_alarm_id) cancel_alarm(timeout_throttle_alarm_id);
    if (edge == EDGE_RISE) {
        counter_edge_rise = counter;
    } else if (edge == EDGE_FALL && counter_edge_rise) {
        uint count = counter - counter_edge_rise;
        float duration_pulse =
            (float)(counter - counter_edge_rise) / clock_get_hz(clk_sys) * COUNTER_CYCLES * 1000000;  // us
        float delta = duration_pulse - 1000;
        if (delta < 0)
            delta = 0;
        else if (delta > 1000)
            delta = 1000;
        throttle = delta / 1000 * 65532;  // 1000us->0% 2000us->100%
        // debug("\nSmart Esc. Throttle (%u%%) %u", throttle / 65532 * 100, throttle);
    }
    timeout_throttle_alarm_id = add_alarm_in_ms(PWM_TIMEOUT_MS, timeout_throttle_callback, NULL, true);
}

static void capture_pwm_reverse_handler(uint counter, edge_type_t edge) {
    static uint counter_edge_rise = 0;
    static alarm_id_t timeout_reverse_alarm_id = 0;
    if (timeout_reverse_alarm_id) cancel_alarm(timeout_reverse_alarm_id);
    if (edge == EDGE_RISE) {
        counter_edge_rise = counter;
    } else if (edge == EDGE_FALL && counter_edge_rise) {
        uint count = counter - counter_edge_rise;
        float duration_pulse =
            (float)(counter - counter_edge_rise) / clock_get_hz(clk_sys) * COUNTER_CYCLES * 1000000;  // us
        float delta = duration_pulse - 1000;
        if (delta < 0)
            delta = 0;
        else if (delta > 1000)
            delta = 1000;
        reverse = delta / 1000 * 65532;  // 1000us->0% 2000us->100%
        // debug("\nSmart Esc. Reverse (%u%%) %u", reverse / 65532 * 100, reverse);
    }
    timeout_reverse_alarm_id = add_alarm_in_ms(PWM_TIMEOUT_MS, timeout_reverse_callback, NULL, true);
}

static int64_t timeout_throttle_callback(alarm_id_t id, void *user_data) {
    throttle = 0;
    debug("\nSmart Esc. Signal timeout. Throttle 0");
    return 0;
}

static int64_t timeout_reverse_callback(alarm_id_t id, void *user_data) {
    reverse = 0;
    debug("\nSmart Esc. Signal timeout. Reverse 0");
    return 0;
}

static int64_t alarm_packet(alarm_id_t id, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    packet_pending = true;
    vTaskNotifyGiveIndexedFromISR(context.uart1_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return SRXL2_INTERVAL_MS * 1000;
}
