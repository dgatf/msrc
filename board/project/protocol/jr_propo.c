#include "jr_propo.h"

#include <math.h>
#include <stdio.h>

#include "airspeed.h"
#include "bmp180.h"
#include "bmp280.h"
#include "config.h"
#include "current.h"
#include "esc_apd_f.h"
#include "esc_apd_hv.h"
#include "esc_castle.h"
#include "esc_hw3.h"
#include "esc_hw4.h"
#include "esc_hw5.h"
#include "esc_kontronik.h"
#include "esc_pwm.h"
#include "ms5611.h"
#include "nmea.h"
#include "ntc.h"
#include "pico/stdlib.h"
#include "pwm_out.h"
#include "stdlib.h"
#include "uart.h"
#include "uart_pio.h"
#include "voltage.h"
//#include "uart_jr.h"

#define JR_PROPO_TEMPERATURE_SENSOR 1
#define JR_PROPO_RPM_SENSOR 2
#define JR_PROPO_VARIO_SENSOR 3
#define JR_PROPO_AIRSPEED_SENSOR 5
#define JR_PROPO_BATTERY_SENSOR 8

#define JR_PROPO_TEMPERATURE_TEMPERATURE_INDEX 0  // 1ºC
#define JR_PROPO_RPM_RPM_INDEX 0                  // 1 rpm
#define JR_PROPO_VARIO_ALTITUDE_INDEX 3           // 0.1m
#define JR_PROPO_VARIO_VSPEED_INDEX 2             // 0.1 m/s
#define JR_PROPO_VARIO_PRESSURE_INDEX 1           // 0.1 hPa
#define JR_PROPO_AIRSPEED_AIRSPEED_INDEX 0        // 1 km/h
#define JR_PROPO_BATTERY_VOLTAGE_INDEX 0          // 0.01 V
#define JR_PROPO_BATTERY_CURRENT_INDEX 0          // 0.01 A
#define JR_PROPO_BATTERY_CAPACITY_INDEX 0         // 1 mAh
#define JR_PROPO_BATTERY_POWER_INDEX 0            // 0.1 W

#define JR_PROPO_TIMEOUT_US 2000
#define JR_PROPO_PACKET_LENGHT 1

#define PIN_BASE 12

static void process(void);
static void send_packet(uint8_t address);

void jr_propo_task(void *parameters) {
    context.led_cycle_duration = 6;
    context.led_cycles = 1;
    uart0_begin(250000, UART_RECEIVER_TX, UART_RECEIVER_RX, JR_PROPO_TIMEOUT_US, 8, 1, UART_PARITY_NONE, false, true);
    //uart_jr_init(pio0, PIN_BASE);
    debug("\nJR Propo init");
    while (1) {
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        process();
    }
}

static void process(void) {
    uint8_t len = uart0_available();
    uint8_t buffer[len];
    uart0_read_bytes(buffer, len);
    if (len == JR_PROPO_PACKET_LENGHT) {
        debug("\nJR Propo (%u) < %X", uxTaskGetStackHighWaterMark(NULL), buffer[0]);
        send_packet(buffer[0]);
    }
}

static void send_packet(uint8_t address) {
    if (address == JR_PROPO_TEMPERATURE_SENSOR) {
        uint8_t buffer[] = {0xE1, 0x3, 0x0, 0x0, 0x1E, 0x1D};
        uart0_write_bytes(buffer, sizeof(buffer));
        vTaskResume(context.led_task_handle);
        debug("\nJR Propo (%u) > ", uxTaskGetStackHighWaterMark(NULL));
        debug_buffer(buffer, sizeof(buffer), "0x%X ");
    }
}
