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

void test();
void monitor();

motor motor;

extern "C" void app_main(void)
{
    motor.init();
    motor.set_speed(100);
    motor.set_direction(motor.CLOCKWISE);
    while (1)
    {
        monitor();
    }
}

void monitor()
{
    static uint64_t time_prev = 0;
    static uint64_t time_curr = 0;
    static uint64_t time_diff = 0;

    static int pcnt_prev = 0;
    static int pcnt_curr = 0;
    static int pcnt_diff = 0;

    static float encoder_freq = 0;
    static float encoder_rpm = 0;
    static float encoder_rotations = 0;
    static float encoder_deg = 0;

    ESP_LOGI(TAG, "| Timestamp (ms): %llu | Speed (RPM): %.2f | Δ Position (Deg): %.2f | Rotation(s): %.2f |", time_curr / MSEC_TO_USEC, encoder_rpm, encoder_deg, encoder_rotations);

    time_curr = esp_timer_get_time();
    pcnt_curr = motor.get_count();

    time_diff = time_curr - time_prev;
    pcnt_diff = pcnt_curr - pcnt_prev;

    time_prev = time_curr;
    pcnt_prev = pcnt_curr;

    encoder_freq = fabs((float)pcnt_diff / (float)time_diff * (float)SEC_TO_USEC / 4);
    encoder_rpm = encoder_freq / 2200 * 60;
    encoder_rotations = (float)pcnt_curr / 8800;
    encoder_deg = encoder_rotations * 360;
        
    vTaskDelay(50/portTICK_PERIOD_MS);
}

void test()
{
    uint64_t time_prev = 0;
    uint64_t time_curr = 0;
    uint64_t time_diff = 0;

    int pcnt_prev = 0;
    int pcnt_curr = 0;
    int pcnt_diff = 0;

    float encoder_freq = 0;
    float encoder_rpm = 0;
    float encoder_rotations = 0;
    float encoder_deg = 0;

    motor.init();
    motor.set_speed(100);

    ESP_LOGI(TAG, "| Timestamp (ms): %llu | Speed (RPM): %.2f | Δ Position (Deg): %.2f | Rotations: %.2f |", time_curr / MSEC_TO_USEC, encoder_rpm, encoder_deg, encoder_rotations);
    motor.set_direction(motor.CLOCKWISE);

    while (1)
    {
        time_curr = esp_timer_get_time();
        //ESP_ERROR_CHECK(pcnt_unit_get_count(motor.unit, &pcnt_curr));

        time_diff = time_curr - time_prev;
        pcnt_diff = pcnt_curr - pcnt_prev;

        time_prev = time_curr;
        pcnt_prev = pcnt_curr;

        encoder_freq = fabs((float)pcnt_diff / (float)time_diff / 4 * (float)SEC_TO_USEC);
        encoder_rpm = encoder_freq / 2200 * 60;
        encoder_rotations = (float)pcnt_curr / 8800;
        encoder_deg = encoder_rotations * 360;
        
        if (encoder_rpm > 0)
            ESP_LOGI(TAG, "| Timestamp (ms): %llu | Speed (RPM): %.2f | Δ Position (Deg): %.2f | Rotations: %.2f |", time_curr / MSEC_TO_USEC, encoder_rpm, encoder_deg, encoder_rotations);

        if (encoder_deg >= 360)
            motor.stop();

        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}