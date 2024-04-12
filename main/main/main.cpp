#include <stdio.h>
#include <string>
#include <string.h>

#include "azure_iot_freertos.h"
#include "motor_controller.hpp"

#include "freertos/FreeRTOS.h"

static constexpr char *TAG = "Main";

static MotorController motor;

extern "C" void app_main(void)
{
  float temp_duty_cycle = 0;

  motor.init();
  // azure_init();
  motor.set_mode(MANUAL);
  motor.set_direction(CLOCKWISE);
  motor.set_duty_cycle(0.1);
  motor.enable_display();

  // vTaskDelay(20000 / portTICK_PERIOD_MS);

  // while (1)
  // {
  //   motor.set_mode(MANUAL);
  //   motor.set_duty_cycle(temp_duty_cycle);
  //   temp_duty_cycle += 0.2;

  //   motor.set_direction(CLOCKWISE);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  //   motor.set_direction(COUNTERCLOCKWISE);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);

  //   if (temp_duty_cycle > 1.0)
  //     temp_duty_cycle = 0.0;
  // }
}

void get_data(uint64_t *timestamp, int32_t *direction, float *duty_cycle, float *velocity, float *position, float *current)
{
  *timestamp = motor.get_timestamp();
  *direction = motor.get_direction();
  *duty_cycle = motor.get_duty_cycle();
  *velocity = motor.get_velocity();
  *position = motor.get_position();
  *current = motor.get_current();
}

void set_desired_mode(int32_t mode)
{
  motor.set_mode(mode);
}

void set_desired_direction(int32_t direction)
{
  motor.set_direction(direction);
}

void set_desired_duty_cycle(float duty_cycle)
{
  motor.set_duty_cycle(duty_cycle);
}

void set_desired_velocity(float velocity)
{
  motor.set_velocity(velocity);
}

bool get_sample_string(char *char_array)
{
  if (motor.get_telemetry_ready())
  {
    strcpy(char_array, motor.get_sample_string().c_str());
    return true;
  }
  else
  {
    return false;
  }
}