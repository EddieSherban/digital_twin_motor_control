// INCLUDES
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "L298N.hpp"

static constexpr char* TAG = "Main";

static constexpr uint32_t SEC_TO_USEC = 1000000;

extern "C" void app_main(void)
{
    uint64_t start_time = esp_timer_get_time();
    uint64_t current_time;
    ESP_LOGI(TAG, "%llu", start_time);

    L298N l298n;
    l298n.init();
    //l298n.set_speed(50);
    //l298n.set_direction(l298n.CLOCKWISE);

    while (1)
    {
        current_time = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "Timestamp: %llus (%lluus)", current_time / SEC_TO_USEC, current_time);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}