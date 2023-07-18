#include "uart.h"

static volatile uint uart0_timeout, uart1_timeout, uart0_timestamp, uart1_timestamp;
static volatile bool uart0_is_timedout = true, uart1_is_timedout = true;

static int64_t uart0_timeout_callback(alarm_id_t id, void *user_data);
static int64_t uart1_timeout_callback(alarm_id_t id, void *user_data);
static void uart0_rx_handler();
static void uart1_rx_handler();

void uart0_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uint parity, bool inverted)
{
    if (uart_alarm_pool == NULL)
        uart_alarm_pool = alarm_pool_create(2, 10);
    uart_init(uart0, baudrate);
    uart_set_fifo_enabled(uart0, false);
    gpio_set_function(gpio_tx, GPIO_FUNC_UART);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    if (inverted)
    {
        gpio_set_outover(gpio_tx, GPIO_OVERRIDE_INVERT);
        gpio_set_inover(gpio_rx, GPIO_OVERRIDE_INVERT);
    }
    uart_set_format(uart0, databits, stopbits, parity);
    irq_set_exclusive_handler(UART0_IRQ, uart0_rx_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart0_timeout = timeout;
    uart0_queue_handle = xQueueCreate(UART0_BUFFER_SIZE, sizeof(uint8_t));
    uart_set_irq_enables(uart0, true, false);
}

void uart1_begin(uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uint parity, bool inverted)
{
    if (uart_alarm_pool == NULL)
        uart_alarm_pool = alarm_pool_create(2, 10);
    uart_init(uart1, baudrate);
    uart_set_fifo_enabled(uart1, false);
    gpio_set_function(gpio_tx, GPIO_FUNC_UART);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    if (inverted)
    {
        gpio_set_outover(gpio_tx, GPIO_OVERRIDE_INVERT);
        gpio_set_inover(gpio_rx, GPIO_OVERRIDE_INVERT);
    }
    uart_set_format(uart1, databits, stopbits, parity);
    irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    uart1_timeout = timeout;
    uart1_queue_handle = xQueueCreate(UART1_BUFFER_SIZE, sizeof(uint8_t));
    uart_set_irq_enables(uart1, true, false);
}

static int64_t uart0_timeout_callback(alarm_id_t id, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart0_is_timedout = true;
    vTaskNotifyGiveIndexedFromISR(uart0_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

static int64_t uart1_timeout_callback(alarm_id_t id, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart1_is_timedout = true;
    vTaskNotifyGiveIndexedFromISR(uart1_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

static void uart0_rx_handler()
{
    if (uart_get_hw(uart0)->fr & UART_UARTFR_BUSY_BITS)
    {
        while (uart_is_readable(uart0))
            uart_getc(uart0);
        return;
    }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static alarm_id_t uart0_timeout_alarm_id = 0;
    if (uart0_timeout_alarm_id)
        alarm_pool_cancel_alarm(uart_alarm_pool, uart0_timeout_alarm_id);
    if (uart0_is_timedout)
    {
        xQueueReset(uart0_queue_handle);
        uart0_is_timedout = false;
    }
    while (uart_is_readable(uart0))
    {
        uint8_t data = uart_getc(uart0);
        // printf("-%X-", data);
        // busy_wait_us(1);
        xQueueSendToBackFromISR(uart0_queue_handle, &data, &xHigherPriorityTaskWoken);
    }
    if (uart0_timeout)
    {
        uart0_timeout_alarm_id = alarm_pool_add_alarm_in_us(uart_alarm_pool, uart0_timeout, uart0_timeout_callback, NULL, true);
    }
    uart0_timestamp = time_us_32();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void uart1_rx_handler()
{
    if (uart_get_hw(uart1)->fr & UART_UARTFR_BUSY_BITS)
    {
        while (uart_is_readable(uart1))
            uart_getc(uart1);
        return;
    }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static alarm_id_t uart1_timeout_alarm_id = 0;
    if (uart1_timeout_alarm_id)
    {
        alarm_pool_cancel_alarm(uart_alarm_pool, uart1_timeout_alarm_id);
    }
    if (uart1_is_timedout)
    {
        xQueueReset(uart1_queue_handle);
        uart1_is_timedout = false;
    }
    while (uart_is_readable(uart1))
    {
        uint8_t data = uart_getc(uart1);
        // printf("%X ", data);
        xQueueSendToBackFromISR(uart1_queue_handle, &data, &xHigherPriorityTaskWoken);
        // printf("-%X/%i-", data, uxQueueMessagesWaiting(uart1_queue_handle));
    }
    if (uart1_timeout)
    {
        uart1_timeout_alarm_id = alarm_pool_add_alarm_in_us(uart_alarm_pool, uart1_timeout, uart1_timeout_callback, NULL, true);
    }
    uart1_timestamp = time_us_32();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t uart0_read()
{
    uint8_t value = 0;
    xQueueReceive(uart0_queue_handle, &value, 0);
    return value;
}

uint8_t uart1_read()
{
    uint8_t value = 0;
    xQueueReceive(uart1_queue_handle, &value, 0);
    return value;
}

void uart0_read_bytes(uint8_t *data, uint8_t lenght)
{
    for (uint8_t i = 0; i < lenght; i++)
        xQueueReceive(uart0_queue_handle, data + i, 0);
}

void uart1_read_bytes(uint8_t *data, uint8_t lenght)
{
    for (uint8_t i = 0; i < lenght; i++)
        xQueueReceive(uart1_queue_handle, data + i, 0);
}

void uart0_write(uint8_t data)
{
    uart_putc_raw(uart0, data);
}

void uart1_write(uint8_t data)
{
    uart_putc_raw(uart1, data);
}

void uart0_write_bytes(uint8_t *data, uint8_t lenght)
{
    uart_write_blocking(uart0, data, lenght);
}

void uart1_write_bytes(uint8_t *data, uint8_t lenght)
{
    uart_write_blocking(uart1, data, lenght);
}

uint8_t uart0_available()
{
    return uxQueueMessagesWaiting(uart0_queue_handle);
}

uint8_t uart1_available()
{
    return uxQueueMessagesWaiting(uart1_queue_handle);
}

uint uart0_get_time_elapsed()
{
    return time_us_32() - uart0_timestamp;
}

uint uart1_get_time_elapsed()
{
    return time_us_32() - uart1_timestamp;
}

/* Use with sim rx */

void uart0_set_timestamp()
{
    uart0_timestamp = time_us_32();
}

void uart1_set_timestamp()
{
    uart1_timestamp = time_us_32();
}
