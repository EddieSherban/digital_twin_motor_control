#include <stdio.h>

#include "azure_iot_freertos_esp32_main.h"
#include "motor_controller.hpp"

#include "freertos/FreeRTOS.h"

static constexpr char *TAG = "Main";

void get_data(uint64_t *timestamp, int8_t *direction, double *duty_cycle, double *velocity, double *position, double *current);

static MotorController motor;

extern "C" void app_main(void)
{
  motor.init();
  azure_init();

  motor.enable_display();
  motor.set_direction(CLOCKWISE);
  // motor.set_duty_cycle(0.05);
  // vTaskDelay(2000 / portTICK_PERIOD_MS);

  while (1)
  {
    motor.set_direction(CLOCKWISE);
    motor.set_duty_cycle(1.00);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // motor.stop_motor();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // motor.set_duty_cycle(0.20);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // motor.set_duty_cycle(0.40);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // motor.set_duty_cycle(0.60);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // motor.set_duty_cycle(0.80);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void get_data(uint64_t *timestamp, int8_t *direction, double *duty_cycle, double *velocity, double *position, double *current)
{
  *timestamp = motor.get_timestamp();
  *direction = motor.get_direction();
  *duty_cycle = motor.get_duty_cycle();
  *velocity = motor.get_velocity();
  *position = motor.get_position();
  *current = motor.get_current();
}