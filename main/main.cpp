/* 
TODO:
  - Add semaphores
  - Fix velocity exponential moving average
  - Add Rx UART communication with commands and framing
*/
// Includes
#include <stdio.h>

#include "configuration.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "motor_controller.hpp"
#include "communication.hpp"

static constexpr char *TAG = "Main";

static MotorController motor;
static Communication comm;
TaskHandle_t tx_data_task_hdl = NULL;

static void tx_data(void *arg);

extern "C" void app_main(void)
{
  comm.init();
  xTaskCreatePinnedToCore(tx_data, "TX Data Task", tx_config.stack_size, nullptr, tx_config.priority, &tx_data_task_hdl, tx_config.core);

  motor.init();
  motor.set_direction(CLOCKWISE);
  motor.enable_display();
  // motor.set_duty_cycle(0);
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  // motor.set_duty_cycle(0.8);

  while (1)
  {
    motor.pid_velocity(0.314159265); // 2.513274 1.256637
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

static void tx_data(void *arg)
{
  static double timestamp = 0;
  static double duty_cycle = 0;
  static int8_t direction = 0;
  static double velocity = 0;
  static double velocity_ema = 0;
  static double position = 0;
  static char data[64] = "";

  while (1)
  {
    timestamp = motor.get_timestamp();
    duty_cycle = motor.get_duty_cycle();
    direction = motor.get_direction();
    velocity = motor.get_velocity();
    velocity_ema = motor.get_velocity_ema();
    position = motor.get_position();

    sprintf(data, "%d,%.3f,%.3f,%d,%.3f,%.3f,%.3f\n", 0x1, timestamp, duty_cycle, direction, velocity, position, velocity_ema);
    comm.send_data(data);

    vTaskDelay(tx_config.delay / portTICK_PERIOD_MS);
  }
}