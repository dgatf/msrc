#include "uart_pio.h"

#include <stdio.h>

#include "common.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

static volatile uint uart_pio_timeout, uart_pio_timestamp;
static volatile bool uart_pio_is_timedout = true;
static uint uart_pio_sm;

static int64_t uart_pio_timeout_callback(alarm_id_t id, void *user_data);
static void uart_pio_handler(uint8_t data);

void uart_pio_begin(uint baudrate, int gpio_tx, int gpio_rx, uint timeout, PIO pio, uint irq, uint8_t data_bits,
                    uint8_t stop_bits, uint8_t parity) {
    if (gpio_rx != UART_GPIO_NONE) {
        if (context.uart_alarm_pool == NULL) context.uart_alarm_pool = alarm_pool_create(2, 10);
        uart_pio_sm = uart_rx_init(pio, gpio_rx, baudrate, irq);
        uart_rx_set_handler(uart_pio_handler);
        uart_pio_timeout = timeout;
        context.uart_rx_pio_queue_handle = xQueueCreate(UART_PIO_BUFFER_SIZE, sizeof(uint8_t));
    }
    if (gpio_tx != UART_GPIO_NONE) {
        uart_pio_sm = uart_tx_init(pio, gpio_tx, baudrate, data_bits, stop_bits, parity);
        context.uart_rx_pio_queue_handle = xQueueCreate(UART_PIO_BUFFER_SIZE, sizeof(uint8_t));
    }
}

static int64_t uart_pio_timeout_callback(alarm_id_t id, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart_pio_is_timedout = true;
    // printf("\n");
    vTaskNotifyGiveIndexedFromISR(context.uart_pio_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

static void uart_pio_handler(uint8_t data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static alarm_id_t uart_pio_timeout_alarm_id = 0;
    if (uart_pio_timeout_alarm_id) alarm_pool_cancel_alarm(context.uart_alarm_pool, uart_pio_timeout_alarm_id);
    if (uart_pio_is_timedout) {
        xQueueReset(context.uart_rx_pio_queue_handle);
        uart_pio_is_timedout = false;
    }
    // debug("-%X-", data);
    xQueueSendToBackFromISR(context.uart_rx_pio_queue_handle, &data, &xHigherPriorityTaskWoken);
    if (uart_pio_timeout == 0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveIndexedFromISR(context.uart_pio_notify_task_handle, 1, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    if (uart_pio_timeout) {
        uart_pio_timeout_alarm_id = alarm_pool_add_alarm_in_us(context.uart_alarm_pool, uart_pio_timeout,
                                                               uart_pio_timeout_callback, NULL, true);
    }
    uart_pio_timestamp = time_us_32();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t uart_pio_read(void) {
    uint8_t value = 0;
    xQueueReceive(context.uart_rx_pio_queue_handle, &value, 0);
    return value;
}

void uart_pio_read_bytes(uint8_t *data, uint8_t lenght) {
    for (uint8_t i = 0; i < lenght; i++) {
        xQueueReceive(context.uart_rx_pio_queue_handle, data + i, 0);
    }
}

void uart_pio_write(uint32_t c) { uart_tx_write(c); }

void uart_pio_write_bytes(void *data, uint8_t lenght) { uart_tx_write_bytes(data, lenght); }

uint8_t uart_pio_available(void) { return uxQueueMessagesWaiting(context.uart_rx_pio_queue_handle); }

uint8_t uart_pio_tx_available(void) {
    return UART_PIO_BUFFER_SIZE - uxQueueMessagesWaiting(context.uart_rx_pio_queue_handle);
}

uint uart_pio_get_time_elapsed(void) { return time_us_32() - uart_pio_timestamp; }

void uart_pio_remove(void) {
    uart_rx_remove();
    uart_tx_remove();
}