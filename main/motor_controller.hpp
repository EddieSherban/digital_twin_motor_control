#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

// Includes
#include <stdio.h>
#include <cmath>
#include <algorithm>

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
  STOPPED = 0,
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

  // Update task
  TaskHandle_t update_task_hdl;
  static void update_trampoline(void *arg);
  void update();

  // Display task
  TaskHandle_t display_task_hdl;
  static void display_trampoline(void *arg);
  void display();

  // Class Variables
  double timestamp;
  int8_t direction;
  double duty_cycle;
  double velocity;
  double position;

public:
  MotorController();

  void init();
  void stop_motor();

  // Accessors and mutators
  void set_direction(MotorDir dir);
  void set_duty_cycle(double dc);
  void set_position(double pos);

  double get_timestamp();
  double get_duty_cycle();
  int8_t get_direction();
  double get_velocity();
  double get_position();

  void pid_velocity(double sp);

  void enable_display();
  void disable_display();
};

#endif // MOTOR_CONTROLLER_H_
