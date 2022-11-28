#include "uart.h"

static volatile uint uart0_timeout, uart1_timeout, uart0_timestamp, uart1_timestamp;
static volatile bool uart0_is_timedout = true, uart1_is_timedout = true;

static int64_t uart0_timeout_callback(alarm_id_t id, void *user_data);
static int64_t uart1_timeout_callback(alarm_id_t id, void *user_data);
static void uart0_rx_handler();
static void uart1_rx_handler();

void uart_begin(uart_inst_t *uart, uint baudrate, uint gpio_tx, uint gpio_rx, uint timeout, uint databits, uint stopbits, uint parity, bool inverted)
{
    if (uart_alarm_pool == NULL)
        uart_alarm_pool = alarm_pool_create(2, 10);
    uart_init(uart, baudrate);
    gpio_set_function(gpio_tx, GPIO_FUNC_UART);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    if (inverted)
    {
        gpio_set_outover(gpio_tx, GPIO_OVERRIDE_INVERT);
        gpio_set_inover(gpio_rx, GPIO_OVERRIDE_INVERT);
    }
    uart_set_format(uart, databits, stopbits, parity);

    if (uart == uart0)
    {
        irq_set_exclusive_handler(UART0_IRQ, uart0_rx_handler);
        irq_set_enabled(UART0_IRQ, true);
        uart0_timeout = timeout;
        uart0_queue_handle = xQueueCreate(UART0_BUFFER_SIZE, sizeof(uint8_t));
    }
    else
    {
        irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
        irq_set_enabled(UART1_IRQ, true);
        uart1_timeout = timeout;
        uart1_queue_handle = xQueueCreate(UART1_BUFFER_SIZE, sizeof(uint8_t));
    }
    uart_set_irq_enables(uart, true, false);
    uart_get_hw(uart)->cr |= UART_UARTCR_SIREN_BITS;
}

static int64_t uart0_timeout_callback(alarm_id_t id, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart0_is_timedout = true;
    // printf("\nTimeout %i %i", time_us_32(), id);
    // printf("\nReceived uart0 size: %i", uxQueueMessagesWaiting(uart1_queue_handle));
    // vTaskNotifyGiveFromISR(uart0_notify_task_handle, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveIndexedFromISR(uart0_notify_task_handle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

static int64_t uart1_timeout_callback(alarm_id_t id, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uart1_is_timedout = true;
    // printf("\nTimeout %i %i", time_us_32(), id);
    // printf("\nReceived uart1 size: %i", uxQueueMessagesWaiting(uart1_queue_handle));
    // vTaskNotifyGiveFromISR(uart1_notify_task_handle, &xHigherPriorityTaskWoken);
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
    // printf("\nSet %i %i", time_us_32(), timeout_alarm_id);
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
        alarm_pool_cancel_alarm(uart_alarm_pool, uart1_timeout_alarm_id);
    if (uart1_is_timedout)
    {
        xQueueReset(uart1_queue_handle);
        uart1_is_timedout = false;
    }
    while (uart_is_readable(uart1))
    {
        uint8_t data = uart_getc(uart1);
        // printf("-%X-", data);
        xQueueSendToBackFromISR(uart1_queue_handle, &data, &xHigherPriorityTaskWoken);
        // printf("-%X/%i-", data, uxQueueMessagesWaiting(uart1_queue_handle));
    }
    if (uart1_timeout)
    {
        uart1_timeout_alarm_id = alarm_pool_add_alarm_in_us(uart_alarm_pool, uart1_timeout, uart1_timeout_callback, NULL, true);
    }
    uart1_timestamp = time_us_32();
    // printf("\nSet %i %i", time_us_32(), timeout_alarm_id);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t uart_read(uart_inst_t *uart)
{
    uint8_t value = 0;
    QueueHandle_t uart_queue_handle;
    if (uart == uart0)
        uart_queue_handle = uart0_queue_handle;
    else
        uart_queue_handle = uart1_queue_handle;
    xQueueReceive(uart_queue_handle, &value, 0);
    return value;
}

void uart_read_bytes(uart_inst_t *uart, uint8_t *data, uint8_t lenght)
{
    QueueHandle_t uart_queue_handle;
    if (uart == uart0)
        uart_queue_handle = uart0_queue_handle;
    else
        uart_queue_handle = uart1_queue_handle;
    for (uint8_t i = 0; i < lenght; i++)
    {
        xQueueReceive(uart_queue_handle, data + i, 0);
    }
}

void uart_write(uart_inst_t *uart, uint8_t data)
{
    uart_putc_raw(uart, data);
}

void uart_write_bytes(uart_inst_t *uart, uint8_t *data, uint8_t lenght)
{
    uart_write_blocking(uart, data, lenght);
}

uint8_t uart_available(uart_inst_t *uart)
{
    QueueHandle_t uart_queue_handle;
    if (uart == uart0)
        uart_queue_handle = uart0_queue_handle;
    else
        uart_queue_handle = uart1_queue_handle;
    return uxQueueMessagesWaiting(uart_queue_handle);
}

uint uart_get_time_elapsed(uart_inst_t *uart)
{
    uint time_elapsed;
    if (uart == uart0)
        time_elapsed = time_us_32() - uart0_timestamp;
    else
        time_elapsed = time_us_32() - uart1_timestamp;
    return time_elapsed;
}

/* Use with sim rx */
void uart_set_timestamp(uart_inst_t *uart)
{
    if (uart == uart0)
        uart0_timestamp = time_us_32();
    else
        uart1_timestamp = time_us_32();
}
