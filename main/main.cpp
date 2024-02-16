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

motor motor;

extern "C" void app_main(void)
{
    motor.init();
    motor.set_speed(100);
    motor.enable_monitor();
    motor.set_direction(motor.CLOCKWISE);
    while (1)
    {        
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
}