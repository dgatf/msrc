#include "pwm_out.h"

#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/pwm.h"

void pwm_out_task(void *parameters) {
    float *rpm = (float *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);

    const float clock_div = 100;
    gpio_set_function(PWM_OUT_GPIO, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_OUT_GPIO);
    uint wrap = 0;
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, clock_div);
    pwm_init(slice_num, &c, true);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (*rpm < 1300)
            pwm_set_enabled(slice_num, false);
        else {
            wrap = 60 / *rpm * clock_get_hz(clk_sys) / clock_div;
            pwm_set_enabled(slice_num, true);
            pwm_set_wrap(slice_num, wrap);
            pwm_set_gpio_level(PWM_OUT_GPIO, wrap / 2);
        }
        debug("\nPWM out (%u) > Rpm %.2f Wrap %i", uxTaskGetStackHighWaterMark(NULL), *rpm, wrap);
    }
}
