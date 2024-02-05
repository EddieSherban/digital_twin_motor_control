// INCLUDES
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

extern "C" void app_main(void)
{
    while (1)
    {
        ESP_LOGI("Main", "Simon is cool.");
        ESP_LOGE("Main", "Eddie is cooler");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}
