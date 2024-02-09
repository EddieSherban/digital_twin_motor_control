// INCLUDES
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "L298N.hpp"

static const char* TAG = "Main";

extern "C" void app_main(void)
{
    L298N test;
    test.init();

    while (1)
    {
        ESP_LOGI(TAG, "Test 1");
        ESP_LOGE(TAG, "Test 2");
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}
