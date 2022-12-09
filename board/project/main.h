#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "pico/stdlib.h"

#include "config.h"
#include "led.h"
#include "ibus.h"
#include "frsky_d.h"
#include "multiplex.h"
#include "smartport.h"
#include "jetiex.h"
#include "sbus.h"
#include "hitec.h"
#include "xbus.h"
#include "srxl.h"
#include "sim_rx.h"
#include "usb.h"

extern uint16_t led_cycle_duration;
extern uint8_t led_cycles;

extern TaskHandle_t pwm_out_task_handle, uart0_notify_task_handle, uart1_notify_task_handle, uart_pio_notify_task_handle, receiver_task_handle, led_task_handle, usb_task_handle;
extern QueueHandle_t uart0_queue_handle, uart1_queue_handle, uart_pio_queue_handle, tasks_queue_handle, sensors_queue_handle;
extern alarm_pool_t *uart_alarm_pool;
extern uint8_t debug;

int main();