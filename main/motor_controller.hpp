#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

// Includes
#include <stdio.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

// Global enums
enum MotorDir
{
  CLOCKWISE = 1,
  COUNTERCLOCKWISE = -1,
};

class MotorController
{
private:
  // Handles
  mcpwm_timer_handle_t timer_hdl;
  mcpwm_oper_handle_t oper_hdl;
  mcpwm_cmpr_handle_t cmpr_hdl;
  mcpwm_gen_handle_t gen_hdl;
  pcnt_unit_handle_t unit_hdl;
  pcnt_channel_handle_t channel_a_hdl;
  pcnt_channel_handle_t channel_b_hdl;
  TaskHandle_t monitor_task_hdl;

  // Monitor task
  static void monitor_trampoline(void *arg);
  void monitor_motor();

  // Class Variables
  int8_t dir;
  float timestamp;
  float speed;
  float pos;
  bool monitor;

public:
  MotorController();

  void init();
  void stop_motor();

  // Accessors and mutators
  void set_speed(float duty_cycle);
  void set_pos(float pos);
  void set_dir(MotorDir dir);
  float get_speed();
  float get_pos();
  int8_t get_dir();

  void enable_monitor();
  void disable_monitor();
};

#endif // MOTOR_CONTROLLER_H_
