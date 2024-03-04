/*
TODO:
  - Add Rx UART communication with commands and framing
  - Move PID to task
*/

// Includes
#include <stdio.h>

#include "configuration.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "motor_controller.hpp"

static constexpr char *TAG = "Main";

static constexpr double RPM_TO_RAD = 2 * M_PI / 60;

static MotorController motor;

extern "C" void app_main(void)
{
  motor.init();
  motor.set_direction(CLOCKWISE);
  motor.enable_display();
  motor.set_duty_cycle(0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // motor.set_duty_cycle(0.5);

  while (1)
  {
    motor.pid_velocity(17.3 * RPM_TO_RAD);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}