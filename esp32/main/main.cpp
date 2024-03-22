#include <stdio.h>

#include "azure_iot_freertos.h"
#include "motor_controller.hpp"

#include "freertos/FreeRTOS.h"

static constexpr char *TAG = "Main";

void get_data(uint64_t *timestamp, int8_t *direction, double *duty_cycle, double *velocity, double *position, double *current);
void set_velocity(double *velocity);

static MotorController motor;

extern "C" void app_main(void)
{
  motor.init();
  azure_init();
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

void set_desired_mode(int32_t mode)
{
  motor.set_mode(mode);
}

void set_desired_direction(int32_t direction)
{
  motor.set_direction(direction);
}

void set_desired_duty_cycle(double duty_cycle)
{
  motor.set_duty_cycle(duty_cycle);
}

void set_desired_velocity(double velocity)
{
  motor.set_velocity(velocity);
}
