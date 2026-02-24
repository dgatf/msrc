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
#include "jr_dmss.h"
#include "fport.h"
#include "fbus.h"
#include "ghst.h"
#include "jetiex_sensor.h"
#include "dbg_task.h"

context_t context;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("stack overflow %p %s\r\n", xTask, pcTaskName);
    while (1)
        ;
}

int main() {
    stdio_init_all();

    gpio_init(RESTORE_GPIO);
    gpio_pull_up(RESTORE_GPIO);
    if (CONFIG_FORZE_WRITE || !gpio_get(RESTORE_GPIO)) config_forze_write();
    config_t *config = config_read();

    context.debug = config->debug;
    if (context.debug) sleep_ms(1000);

    xTaskCreate(usb_task, "usb_task", STACK_USB, NULL, 1, &context.usb_task_handle);

    dbg_task_init();
    dbg_task_start(512, 1);

    debug("\n\nMSRC init");

    context.led_cycle_duration = 200;
    context.led_cycles = 3;
    xTaskCreate(led_task, "led_task", STACK_LED, NULL, 1, &context.led_task_handle);

    switch (config->rx_protocol) {
        case RX_XBUS:
            xTaskCreate(xbus_task, "xbus_task", STACK_RX_XBUS, NULL, 3, &context.receiver_task_handle);
            break;
        case RX_IBUS:
            xTaskCreate(ibus_task, "ibus_task", STACK_RX_IBUS, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_FRSKY_D:
            xTaskCreate(frsky_d_task, "frsky_d_task", STACK_RX_FRSKY_D, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_MULTIPLEX:
            xTaskCreate(multiplex_task, "multiplex_task", STACK_RX_MULTIPLEX, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_SMARTPORT:
            xTaskCreate(smartport_task, "smartport_task", STACK_RX_SMARTPORT, NULL, 4, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_JETIEX:
            xTaskCreate(jetiex_task, "jetiex_task", STACK_RX_JETIEX, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_SBUS:
            xTaskCreate(sbus_task, "sbus_task", STACK_RX_SBUS, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_HITEC:
            xTaskCreate(hitec_task, "hitec_task", STACK_RX_HITEC, NULL, 3, &context.receiver_task_handle);
            break;
        case RX_SRXL:
            xTaskCreate(srxl_task, "srxl_task", STACK_RX_SRXL, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_SRXL2:
            xTaskCreate(srxl2_task, "srxl2_task", STACK_RX_SRXL2, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case SERIAL_MONITOR:
            xTaskCreate(serial_monitor_task, "serial_monitor", STACK_SERIAL_MONITOR, NULL, 3,
                        &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            context.uart1_notify_task_handle = context.receiver_task_handle;
            context.uart_pio_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_CRSF:
            xTaskCreate(crsf_task, "crfs_task", STACK_RX_CRSF, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_HOTT:
            xTaskCreate(hott_task, "hott_task", STACK_RX_HOTT, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_SANWA:
            xTaskCreate(sanwa_task, "sanwa_task", STACK_RX_SANWA, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_JR_PROPO:
            xTaskCreate(jr_dmss_task, "jr_dmss_task", STACK_RX_JR_PROPO, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_FPORT:
            xTaskCreate(fport_task, "fport_task", STACK_RX_FPORT, NULL, 4, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_FBUS:
            xTaskCreate(fbus_task, "fbus_task", STACK_RX_FBUS, NULL, 4, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_GHST:
            xTaskCreate(ghst_task, "ghst_task", STACK_RX_GHST, NULL, 3, &context.receiver_task_handle);
            context.uart0_notify_task_handle = context.receiver_task_handle;
            break;
        case RX_JETIEX_SENSOR:
            xTaskCreate(jetiex_sensor_task, "jetiex_sensor_task", STACK_RX_JETIEX_SENSOR, NULL, 3, &context.receiver_task_handle);
            context.uart_pio_notify_task_handle = context.receiver_task_handle;
            break;
    }

#ifdef SIM_RX
    sim_rx_parameters_t parameter = {config->rx_protocol};
    xTaskCreate(sim_rx_task, "sim_rx_task", STACK_SIM_RX, &parameter, 3, NULL);
#endif

    vTaskStartScheduler();

    while (1)
        ;
}
