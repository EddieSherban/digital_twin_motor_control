/*
TODO:
  - Add Rx UART communication with commands and framing
  - Write driver for current sensor
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
  motor.set_mode(MANUAL);
  motor.enable_display();

  motor.set_direction(CLOCKWISE);

  // motor.set_duty_cycle(0.20);
  // vTaskDelay(5000 / portTICK_PERIOD_MS);
  // motor.set_duty_cycle(0.80);

  motor.set_mode(AUTO);
  motor.set_velocity(3);
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  while (1)
  {
    motor.set_velocity(15);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}