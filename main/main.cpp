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

extern "C" void app_main(void)
{
    uint64_t prev_time = 0;
    uint64_t curr_time = 0;
    uint64_t diff_time = 0;
    int prev_pulse_count = 0;
    int curr_pulse_count = 0;
    int diff_pulse_count = 0;
    float encoder_freq = 0;
    float encoder_rpm = 0;

    motor motor;
    motor.init();    
    motor.set_direction(motor.COUNTER_CLOCKWISE);
    motor.set_speed(80);

    curr_time = esp_timer_get_time();
    ESP_ERROR_CHECK(pcnt_unit_get_count(motor.unit, &curr_pulse_count));
    prev_time = curr_time;
    prev_pulse_count = curr_pulse_count;

    while (1)
    {
        curr_time = esp_timer_get_time();
        ESP_ERROR_CHECK(pcnt_unit_get_count(motor.unit, &curr_pulse_count));

        diff_time = curr_time - prev_time;
        diff_pulse_count = curr_pulse_count - prev_pulse_count;

        encoder_freq = fabs((float)diff_pulse_count / (float)diff_time * (float)SEC_TO_USEC / 4);
        encoder_rpm = encoder_freq / 2200 * 60;

        if (diff_pulse_count > 0)
            ESP_LOGI(TAG, "Time (ms): %llu (%llu) | Speed (RPM): %f | Frequency (Hz): %f | Direction: CW | Count: %d (%d)", diff_time / MSEC_TO_USEC, curr_time / MSEC_TO_USEC, encoder_rpm, encoder_freq, diff_pulse_count, curr_pulse_count);
        else
            ESP_LOGI(TAG, "Time (ms): %llu (%llu) | Speed (RPM): %f | Frequency (Hz): %f | Direction: CCW | Count: %d (%d)", diff_time / MSEC_TO_USEC, curr_time / MSEC_TO_USEC, encoder_rpm, encoder_freq, diff_pulse_count, curr_pulse_count);
        
        prev_time = curr_time;
        prev_pulse_count = curr_pulse_count;
        
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}