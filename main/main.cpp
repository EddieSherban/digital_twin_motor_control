/*
TODO:
  - Add Rx UART communication with commands and framing
  - Figure out way to zero current sensor more efficiently
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
  motor.set_duty_cycle(0.00);

  motor.enable_display();
  motor.set_direction(COUNTERCLOCKWISE);

  // motor.set_duty_cycle(0.50);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);
  // motor.set_duty_cycle(0.60);
  // vTaskDelay(7000 / portTICK_PERIOD_MS);

  motor.set_mode(AUTO);
  motor.set_velocity(0);
  // vTaskDelay(5000 / portTICK_PERIOD_MS);

  while (1)
  {
    motor.set_velocity(0);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    motor.set_velocity(6);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    motor.set_velocity(12);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    motor.set_velocity(18);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    motor.set_velocity(24);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    // motor.set_duty_cycle(0.00);
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    // motor.set_duty_cycle(0.50);
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    // motor.set_duty_cycle(1.00);
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}