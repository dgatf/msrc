#include <stdio.h>

#include "config.h"
#include "frsky_d.h"
#include "hitec.h"
#include "ibus.h"
#include "jetiex.h"
#include "led.h"
#include "multiplex.h"
#include "sbus.h"
#include "serial_monitor.h"
#include "sim_rx.h"
#include "smartport.h"
#include "srxl.h"
#include "srxl2.h"
#include "usb.h"
#include "xbus.h"
#include "crsf.h"
#include "hott.h"
#include "sanwa.h"
#include "pixhawk.h"

context_t context;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    debug("stack overflow %x %s\r\n", xTask, (portCHAR *)pcTaskName);
    while (1)
        ;
}

int main() {
    stdio_init_all();

    if (CONFIG_FORZE_WRITE) config_forze_write();
    config_t *config = config_read();

    context.debug = config->debug;
    if (context.debug) sleep_ms(1000);
    debug("\n\nMSRC init");

    context.tasks_queue_handle = xQueueCreate(64, sizeof(QueueHandle_t));

    context.led_cycle_duration = 200;
    context.led_cycles = 3;
    xTaskCreate(led_task, "led_task", STACK_LED, NULL, 1, &context.led_task_handle);

    xTaskCreate(usb_task, "usb_task", STACK_USB, NULL, 1, &context.usb_task_handle);

    switch (config->rx_protocol) {
        case RX_XBUS:
            xTaskCreate(xbus_task, "xbus_task", STACK_RX_XBUS, NULL, 3, &context.receiver_task_handle);
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_IBUS:
            xTaskCreate(ibus_task, "ibus_task", STACK_RX_IBUS, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_FRSKY_D:
            xTaskCreate(frsky_d_task, "frsky_d_task", STACK_RX_FRSKY_D, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_MULTIPLEX:
            xTaskCreate(multiplex_task, "multiplex_task", STACK_RX_MULTIPLEX, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_SMARTPORT:
            xTaskCreate(smartport_task, "smartport_task", STACK_RX_SMARTPORT, NULL, 4, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_JETIEX:
            xTaskCreate(jetiex_task, "jetiex_task", STACK_RX_JETIEX, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_SBUS:
            xTaskCreate(sbus_task, "sbus_task", STACK_RX_SBUS, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_HITEC:
            xTaskCreate(hitec_task, "hitec_task", STACK_RX_HITEC, NULL, 3, &context.receiver_task_handle);
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_SRXL:
            xTaskCreate(srxl_task, "srxl_task", STACK_RX_SRXL, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_SRXL2:
            xTaskCreate(srxl2_task, "srxl2_task", STACK_RX_SRXL2, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case SERIAL_MONITOR:
            xTaskCreate(serial_monitor_task, "serial_monitor", STACK_SERIAL_MONITOR, NULL, 3,
                        &context.receiver_task_handle);
            context.uart1_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_CRSF:
            xTaskCreate(crsf_task, "crfs_task", STACK_RX_CRSF, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_HOTT:
            xTaskCreate(hott_task, "hott_task", STACK_RX_HOTT, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_SANWA:
            xTaskCreate(sanwa_task, "sanwa_task", STACK_RX_SANWA, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
        case RX_PIXHAWK:
            xTaskCreate(pixhawk_task, "pixhawk_task", STACK_RX_PIXHAWK, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            xQueueSendToBack(context.tasks_queue_handle, context.receiver_task_handle, 0);
            break;
    }

#ifdef SIM_RX
    sim_rx_parameters_t parameter = {config->rx_protocol};
    xTaskCreate(sim_rx_task, "sim_rx_task", STACK_SIM_RX, &parameter, 3, NULL);
#endif

    vTaskStartScheduler();

    vTaskResume(context.led_task_handle);
    while (1)
        ;
}
