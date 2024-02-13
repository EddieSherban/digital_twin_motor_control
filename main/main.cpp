// INCLUDES
#include <stdio.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "motor.hpp"

static constexpr char* TAG = "Main";

static constexpr uint32_t MSEC_TO_USEC = 1000;
static constexpr uint32_t SEC_TO_USEC = 1000000;

QueueHandle_t queue = xQueueCreate(10, sizeof(int));


extern "C" void app_main(void)
{
    uint64_t time_prev = 0;
    uint64_t time_curr = 0;
    uint64_t time_diff = 0;
    int pcnt_prev = 0;
    int pcnt_curr = 0;
    int pcnt_diff = 0;
    float encoder_freq = 0;
    float encoder_rpm = 0;
    int event_count = 0;

    motor motor;
    motor.init();
    motor.set_direction(motor.CLOCKWISE);

    time_prev = esp_timer_get_time();
    motor.set_speed(100);

    while (1)
    {
        time_curr = esp_timer_get_time();
        ESP_ERROR_CHECK(pcnt_unit_get_count(motor.unit, &pcnt_curr));

        time_diff = time_curr - time_prev;
        pcnt_diff = pcnt_curr - pcnt_prev;

        time_prev = time_curr;
        pcnt_prev = pcnt_curr;

        encoder_freq = fabs((float)pcnt_diff / (float)time_diff * (float)SEC_TO_USEC / 4);
        encoder_rpm = encoder_freq / 2200 * 60;

        if (pcnt_diff > 0)
            ESP_LOGI(TAG, "Timestamp (ms): %llu | Diff Time (ms): %llu | Speed (RPM): %.2f | Freq. (Hz): %.2f | Diff Count: %d | Total Count: %d | Dir.: CW ", time_curr / MSEC_TO_USEC, time_diff / MSEC_TO_USEC, encoder_rpm, encoder_freq, pcnt_diff, pcnt_curr);
        else
            ESP_LOGI(TAG, "Timestamp (ms): %llu | Diff Time (ms): %llu | Speed (RPM): %.2f | Freq. (Hz): %.2f | Diff Count: %d | Total Count: %d | Dir.: CCW", time_curr / MSEC_TO_USEC, time_diff / MSEC_TO_USEC, encoder_rpm, encoder_freq, pcnt_diff, pcnt_curr);
        
        vTaskDelay(10/portTICK_PERIOD_MS);
        
        /*
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) 
        {
            time_curr = esp_timer_get_time();
            time_diff = time_curr - time_prev;
            time_prev = time_curr;
            encoder_freq = fabs((float)20 / (float)time_diff * (float)SEC_TO_USEC / 4);
            encoder_rpm = encoder_freq / 2200 * 60;
            ESP_LOGI(TAG, "Time (ms): %llu (%llu) | Speed (RPM): %f | Frequency (Hz): %f | Count: %d", time_diff / MSEC_TO_USEC, time_curr / MSEC_TO_USEC, encoder_rpm, encoder_freq, event_count);
        }
        */
    }
}

bool get_current_speed(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // Send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}