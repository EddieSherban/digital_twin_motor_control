#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board/Led.hpp"

extern "C" void app_main(void)
{
    LED my_led(GPIO_NUM_19);

    while(1)
    {
        ESP_LOGI("Main", "Hi, this is eddies %dst project.", 1);
        vTaskDelay(500/portTICK_PERIOD_MS);
        my_led.toggle();
    }
}
