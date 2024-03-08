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

static MotorController motor;

extern "C" void app_main(void)
{
  motor.init();
  motor.stop_motor();
  motor.set_duty_cycle(0);

  // motor.enable_display();
  motor.set_direction(CLOCKWISE);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // motor.set_duty_cycle(0.80);

  while (1)
  {
    motor.pid_velocity(15);
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}