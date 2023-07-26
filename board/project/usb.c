#include "usb.h"

#define AIRCR_Register (*((volatile uint32_t *)(PPB_BASE + 0x0ED0C)))

static uint8_t buffer_rx[USB_BUFFER_LENGTH];

static int read_usb();
static void process_usb();

void usb_task()
{
    while (1)
    {
        int length = read_usb();
        if (length)
            process_usb(length);
        if (debug)
            printf("\nUSB (%u)", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(USB_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void process_usb(int lenght)
{

    if (debug)
        printf("\nUSB. Processing (%i) 0x%X 0x%X", lenght, buffer_rx[0], buffer_rx[1]);
    
    /*
    header - 0x30
    command - 0x30 - read config from usb
              0x31 - request to send config to usb
              0x32 - answer to send config
              0x33 - debug on
              0x34 - debug off
    */

    if (buffer_rx[0] == 0x30)
    {
        if (buffer_rx[1] == 0x30 && lenght == sizeof(config_t) + 2)
        { // read config from usb, write to flash and reboot

            config_t *config = (config_t *)(buffer_rx + 2);
            if (config->version == CONFIG_VERSION)
            {
            led_cycles = 3;
            led_cycle_duration = 1000;
            vTaskResume(led_task_handle);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            led_cycles = 1;
            led_cycle_duration = 6;
            vTaskResume(led_task_handle);
                config_write(config);
                if (debug)
                    printf("\nUSB. Updated config");
                //sleep_ms(1000);
                //AIRCR_Register = 0x5FA0004;
            }
            else if (debug)
                printf("\nUSB. Incompatible config version");
        }
        else if (buffer_rx[1] == 0x31 && lenght == 2)
        { // send config
            uint8_t debug_state = debug;
            debug = 0;
            led_cycles = 2;
            led_cycle_duration = 1000;
            vTaskResume(led_task_handle);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            led_cycles = 1;
            led_cycle_duration = 6;
            vTaskResume(led_task_handle);
            config_t *config = config_read();
            putchar_raw(0x30);
            putchar_raw(0x32);
            uint8_t data[sizeof(config_t)];
            memcpy(data, config, sizeof(config_t));
            for (int i = 0; i < sizeof(config_t); i++)
                putchar_raw(data[i]);
            if (debug_state)
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            debug = debug_state;
            if (debug)
                printf("\nUSB. Send config");
        }
        else if (buffer_rx[1] == 0x33 && lenght == 2)
        { // debug enable
            debug = 1;
            if (debug)
                printf("\nUSB. Debug enabled");
        }
        else if (buffer_rx[1] == 0x34 && lenght == 2)
        { // debug disable
            debug = 0;
            if (debug)
                printf("\nUSB. Debug disabled");
        }
    }
}

static int read_usb()
{
    int buffer_index = 0;
    while (1)
    {
        int c = getchar_timeout_us(1000);
        if (c != PICO_ERROR_TIMEOUT && buffer_index < USB_BUFFER_LENGTH)
        {
            buffer_rx[buffer_index++] = (c & 0xFF);
        }
        else
        {
            break;
        }
    }
    return buffer_index;
}
