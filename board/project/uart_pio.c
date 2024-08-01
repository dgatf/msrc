#include "uart_pio.h"

#include "hardware/irq.h"
#include "hardware/pio.h"

static volatile uint uart_pio_timeout, uart_pio_timestamp;
static volatile bool uart_pio_is_timedout = true;
static uint uart_pio_sm;

static int64_t uart_pio_timeout_callback(alarm_id_t id, void *user_data);
static void uart_pio_handler(uint8_t data);

void uart_pio_begin(uint baudrate, uint gpio_rx, uint timeout, PIO pio, uint irq) {
    if (context.uart_alarm_pool == NULL) context.uart_alarm_pool = alarm_pool_create(2, 10);
    uart_pio_sm = uart_rx_init(pio, gpio_rx, baudrate, irq);
    uart_rx_set_handler(uart_pio_handler);
    uart_pio_timeout = timeout;
    context.uart_pio_queue_handle = xQueueCreate(UART_PIO_BUFFER_SIZE, sizeof(uint8_t));
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
        xQueueReset(context.uart_pio_queue_handle);
        uart_pio_is_timedout = false;
    }
    // printf("%X ", data);
    xQueueSendToBackFromISR(context.uart_pio_queue_handle, &data, &xHigherPriorityTaskWoken);
    if (uart_pio_timeout) {
        uart_pio_timeout_alarm_id =
            alarm_pool_add_alarm_in_us(context.uart_alarm_pool, uart_pio_timeout, uart_pio_timeout_callback, NULL, true);
    }
    uart_pio_timestamp = time_us_32();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t uart_pio_read() {
    uint8_t value = 0;
    xQueueReceive(context.uart_pio_queue_handle, &value, 0);
    return value;
}

void uart_pio_read_bytes(uint8_t *data, uint8_t lenght) {
    for (uint8_t i = 0; i < lenght; i++) {
        xQueueReceive(context.uart_pio_queue_handle, data + i, 0);
    }
}

uint8_t uart_pio_available() { return uxQueueMessagesWaiting(context.uart_pio_queue_handle); }

uint uart_pio_get_time_elapsed() { return time_us_32() - uart_pio_timestamp; }
