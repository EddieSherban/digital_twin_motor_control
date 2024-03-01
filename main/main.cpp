// Includes
#include <stdio.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "motor_controller.hpp"
#include "communication.hpp"

static constexpr char *TAG = "main";

MotorController motor;
Communication comm;
TaskHandle_t tx_data_task_hdl = NULL;

void bump_test(float start, float end);
static void tx_data(void *arg);

extern "C" void app_main(void)
{
  comm.init();
  motor.init();
  motor.set_direction(CLOCKWISE);
  motor.enable_display();
  // motor.set_duty_cycle(0.5);
  xTaskCreatePinnedToCore(tx_data, "tx data", 1024 * 4, nullptr, configMAX_PRIORITIES - 2, &tx_data_task_hdl, 0);

  while (1)
  {
    motor.pid_velocity(2.513274); // 2.513274 1.256637
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

static void tx_data(void *arg)
{
  static double timestamp = 0;
  static double duty_cycle = 0;
  static int8_t direction = 0;
  static double velocity = 0;
  static double position = 0;
  char data[64] = "";

  while (1)
  {
    timestamp = motor.get_timestamp();
    duty_cycle = motor.get_duty_cycle();
    direction = motor.get_direction();
    velocity = motor.get_velocity();
    position = motor.get_position();

    sprintf(data, "%.3f,%.3f,%d,%.3f,%.3f\n", timestamp, duty_cycle, direction, velocity, position);
    comm.send_data(data);

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void bump_test(float start, float end)
{
  // // Initial conditions
  // motor.set_dir(CLOCKWISE);
  // motor.set_speed(0);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // motor.set_speed(start);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);

  // // Wait for steady state then bump and monitor
  // motor.enable_monitor();
  // motor.set_speed(end);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);

  // // Wait for steady state and then disable monitoring and revert speed
  // motor.disable_monitor();
}
