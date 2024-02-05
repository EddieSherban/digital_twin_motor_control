// INCLUDES
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_control.h"

extern "C" void app_main(void)
{
    while (1)
    {
        ESP_LOGI("Main", "Test 1");
        ESP_LOGE("Main", "Test 2");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}
