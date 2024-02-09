// INCLUDES
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

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
/*
void init()
{
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Initializing system.");
    ESP_LOGI(TAG, "Creating ESP timer.");
    esp_timer_handle_t esp_timer;
    esp_timer_create_args_t esp_timer =
    {
        .callback = 
    };

    ESP_LOGI(TAG, "----------------------------------------");

}

esp_timer_cb_t esp_timer_callback()
{
    
}
*/