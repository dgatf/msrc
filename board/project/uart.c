#include "uart.h"

#include <stdio.h>

#include "hardware/irq.h"
#include "hardware/uart.h"

#define UART0_BUFFER_SIZE 512
#define UART1_BUFFER_SIZE 512

#define enable_rx(UART) hw_set_bits(&uart_get_hw(UART)->cr, 0x00000200)
#define disable_rx(UART) hw_clear_bits(&uart_get_hw(UART)->cr, 0x00000200)

static volatile uint uart0_timeout, uart1_timeout, uart0_timestamp, uart1_timestamp;
static volatile bool uart0_is_timedout = true, uart1_is_timedout = true;
static bool half_duplex0, half_duplex1, inverted0, inverted1;
static int gpio_tx0, gpio_rx0, gpio_tx1, gpio_rx1;
static int64_t uart0_timeout_callback(alarm_id_t id, void *user_data);
static int64_t uart1_timeout_callback(alarm_id_t id, void *user_data);
static void uart0_rx_handler();
static void uart1_rx_handler();

void uart0_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits,
                 uart_parity_t parity, bool inverted, bool half_duplex) {
    half_duplex0 = half_duplex;
    gpio_tx0 = gpio_tx;
    gpio_rx0 = gpio_rx;
    inverted0 = inverted;
    if (context.uart_alarm_pool == NULL) context.uart_alarm_pool = alarm_pool_create(2, 10);
    uart_init(uart0, baudrate);
    uart_set_fifo_enabled(uart0, false);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    if (!half_duplex) gpio_set_function(gpio_tx, GPIO_FUNC_UART);
    if (!inverted) {
        gpio_pull_up(gpio_tx);
        gpio_pull_up(gpio_rx);
    } else {
        gpio_pull_down(gpio_tx);
        gpio_pull_down(gpio_rx);
        gpio_set_outover(gpio_tx, GPIO_OVERRIDE_INVERT);
        gpio_set_inover(gpio_rx, GPIO_OVERRIDE_INVERT);
    }
    uart_set_format(uart0, databits, stopbits, parity);
    irq_set_exclusive_handler(UART0_IRQ, uart0_rx_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart0_timeout = timeout;
    context.uart0_queue_handle = xQueueCreate(UART0_BUFFER_SIZE, sizeof(uint8_t));
    uart_set_irq_enables(uart0, true, false);
}

void uart1_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits,
                 uart_parity_t parity, bool inverted, bool half_duplex) {
    half_duplex1 = half_duplex;
    gpio_tx1 = gpio_tx;
    gpio_rx1 = gpio_rx;
    inverted1 = inverted;
    if (context.uart_alarm_pool == NULL) context.uart_alarm_pool = alarm_pool_create(2, 10);
    uart_init(uart1, baudrate);
    uart_set_fifo_enabled(uart1, false);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    if (!half_duplex) gpio_set_function(gpio_tx, GPIO_FUNC_UART);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    if (!inverted) {
        gpio_pull_up(gpio_tx);
        gpio_pull_up(gpio_rx);
    } else {
        gpio_pull_down(gpio_tx);
        gpio_pull_down(gpio_rx);
        gpio_set_outover(gpio_tx, GPIO_OVERRIDE_INVERT);
        gpio_set_inover(gpio_rx, GPIO_OVERRIDE_INVERT);
    }
    uart_set_format(uart1, databits, stopbits, parity);
    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    uart1_timeout = timeout;
    context.uart1_queue_handle = xQueueCreate(UART1_BUFFER_SIZE, sizeof(uint8_t));
    uart_set_irq_enables(uart1, true, false);
}

static int64_t uart0_timeout_callback(alarm_id_t id, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart0_is_timedout = true;
    vTaskNotifyGiveIndexedFromISR(context.uart0_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

static int64_t uart1_timeout_callback(alarm_id_t id, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart1_is_timedout = true;
    vTaskNotifyGiveIndexedFromISR(context.uart1_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

static void uart0_rx_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static alarm_id_t uart0_timeout_alarm_id = 0;
    if (uart0_timeout_alarm_id) alarm_pool_cancel_alarm(context.uart_alarm_pool, uart0_timeout_alarm_id);
    if (uart0_is_timedout) {
        xQueueReset(context.uart0_queue_handle);
        uart0_is_timedout = false;
    }
    while (uart_is_readable(uart0)) {
        uint8_t data = uart_getc(uart0);
        // debug("-%X-", data);
        xQueueSendToBackFromISR(context.uart0_queue_handle, &data, &xHigherPriorityTaskWoken);
        if (uart0_timeout == 0) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveIndexedFromISR(context.uart0_notify_task_handle, 1, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    if (uart0_timeout) {
        uart0_timeout_alarm_id =
            alarm_pool_add_alarm_in_us(context.uart_alarm_pool, uart0_timeout, uart0_timeout_callback, NULL, true);
    }
    uart0_timestamp = time_us_32();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void uart1_rx_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static alarm_id_t uart1_timeout_alarm_id = 0;
    if (uart1_timeout_alarm_id) {
        alarm_pool_cancel_alarm(context.uart_alarm_pool, uart1_timeout_alarm_id);
    }
    if (uart1_is_timedout) {
        xQueueReset(context.uart1_queue_handle);
        uart1_is_timedout = false;
    }
    while (uart_is_readable(uart1)) {
        uint8_t data = uart_getc(uart1);
        // debug("%X ", data);
        xQueueSendToBackFromISR(context.uart1_queue_handle, &data, &xHigherPriorityTaskWoken);
        if (uart1_timeout == 0) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveIndexedFromISR(context.uart1_notify_task_handle, 1, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    if (uart1_timeout) {
        uart1_timeout_alarm_id =
            alarm_pool_add_alarm_in_us(context.uart_alarm_pool, uart1_timeout, uart1_timeout_callback, NULL, true);
    }
    uart1_timestamp = time_us_32();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t uart0_read() {
    uint8_t value = 0;
    xQueueReceive(context.uart0_queue_handle, &value, 0);
    return value;
}

uint8_t uart1_read() {
    uint8_t value = 0;
    xQueueReceive(context.uart1_queue_handle, &value, 0);
    return value;
}

void uart0_read_bytes(uint8_t *data, uint8_t lenght) {
    for (uint8_t i = 0; i < lenght; i++) xQueueReceive(context.uart0_queue_handle, data + i, 0);
}

void uart1_read_bytes(uint8_t *data, uint8_t lenght) {
    for (uint8_t i = 0; i < lenght; i++) xQueueReceive(context.uart1_queue_handle, data + i, 0);
}

void uart0_write(uint8_t data) {
    if (half_duplex0) {
        disable_rx(uart0);
        gpio_set_function(gpio_tx0, GPIO_FUNC_UART);
        if (inverted0) gpio_set_outover(gpio_tx0, GPIO_OVERRIDE_INVERT);
    }
    uart_putc_raw(uart0, data);
    if (half_duplex0) {
        uart_tx_wait_blocking(uart0);
        enable_rx(uart0);
        gpio_set_function(gpio_tx0, GPIO_FUNC_NULL);
    }
}

void uart1_write(uint8_t data) {
    if (half_duplex1) {
        disable_rx(uart1);
        gpio_set_function(gpio_tx1, GPIO_FUNC_UART);
        if (inverted1) gpio_set_outover(gpio_tx1, GPIO_OVERRIDE_INVERT);
    }
    uart_putc_raw(uart1, data);
    if (half_duplex1) {
        uart_tx_wait_blocking(uart1);
        enable_rx(uart1);
        gpio_set_function(gpio_tx1, GPIO_FUNC_NULL);
    }
}

void uart0_write_bytes(uint8_t *data, uint8_t lenght) {
    if (half_duplex0) {
        disable_rx(uart0);
        gpio_set_function(gpio_tx0, GPIO_FUNC_UART);
        if (inverted0) gpio_set_outover(gpio_tx0, GPIO_OVERRIDE_INVERT);
    }
    uart_write_blocking(uart0, data, lenght);
    if (half_duplex0) {
        uart_tx_wait_blocking(uart0);
        enable_rx(uart0);
        gpio_set_function(gpio_tx0, GPIO_FUNC_NULL);
    }
}

void uart1_write_bytes(uint8_t *data, uint8_t lenght) {
    if (half_duplex1) {
        disable_rx(uart1);
        gpio_set_function(gpio_tx1, GPIO_FUNC_UART);
        if (inverted1) gpio_set_outover(gpio_tx1, GPIO_OVERRIDE_INVERT);
    }
    uart_write_blocking(uart1, data, lenght);
    if (half_duplex1) {
        uart_tx_wait_blocking(uart1);
        enable_rx(uart1);
        gpio_set_function(gpio_tx1, GPIO_FUNC_NULL);
    }
}

uint8_t uart0_available() { return uxQueueMessagesWaiting(context.uart0_queue_handle); }

uint8_t uart1_available() { return uxQueueMessagesWaiting(context.uart1_queue_handle); }

uint uart0_get_time_elapsed() { return time_us_32() - uart0_timestamp; }

uint uart1_get_time_elapsed() { return time_us_32() - uart1_timestamp; }

/* Use with sim rx */

void uart0_set_timestamp() { uart0_timestamp = time_us_32(); }

void uart1_set_timestamp() { uart1_timestamp = time_us_32(); }