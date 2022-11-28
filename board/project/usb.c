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

    if (debug == 3)
        printf("\nUSB. Processing %i %X %X", lenght, buffer_rx[0], buffer_rx[1]);
    debug = 0;
    
    /*
    header - 0x30
    command - 0x30 - read config from usb
              0x31 - request to send config to usb
              0x32 - answer to send config
    */

    if (buffer_rx[0] == 0x30)
    {
        if (buffer_rx[1] == 0x30 && lenght == sizeof(config_t) + 2)
        { // read config from usb, write to flash and reboot

            config_t *config = (config_t *)(buffer_rx + 2);
            if (config->version == CONFIG_VERSION)
            {
                led_cycles = 3;
                vTaskResume(led_task_handle);
                config_write(config);
                if (debug == 3)
                    printf("\nUSB. Updated config");
                //sleep_ms(1000);
                //AIRCR_Register = 0x5FA0004;
            }
            else if (debug == 3)
                printf("\nUSB. Incompatible config version");
        }
        else if (buffer_rx[1] == 0x31 && lenght == 2)
        { // send config
            led_cycles = 2;
            vTaskResume(led_task_handle);
            config_t *config = config_read();
            putchar_raw(0x30);
            putchar_raw(0x32);
            uint8_t data[sizeof(config_t)];
            memcpy(data, config, sizeof(config_t));
            for (int i = 0; i < sizeof(config_t); i++)
                putchar_raw(data[i]);
            if (debug == 3)
                printf("\nUSB. Send config");
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
