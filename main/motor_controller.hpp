#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

// Headers
#include <stdio.h>
#include <cmath>
#include <string.h>

#include "configuration.hpp"
#include "communication.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

// Global enumerations
enum MotorDirection
{
  CLOCKWISE = 1,
  COUNTERCLOCKWISE = -1,
  STOPPED = 0,
};

class MotorController
{
private:
  // ESP handles
  mcpwm_timer_handle_t timer_hdl;
  mcpwm_oper_handle_t oper_hdl;
  mcpwm_cmpr_handle_t cmpr_hdl;
  mcpwm_gen_handle_t gen_hdl;
  pcnt_unit_handle_t unit_hdl;
  pcnt_channel_handle_t channel_a_hdl;
  pcnt_channel_handle_t channel_b_hdl;

  // Semaphores
  SemaphoreHandle_t data_semaphore;

  // Update task
  TaskHandle_t update_task_hdl;
  static void update_trampoline(void *arg);
  void update_task();

  // Display task
  TaskHandle_t display_task_hdl;
  static void display_task(void *arg);

  // TX Data task
  TaskHandle_t tx_data_task_hdl;
  static void tx_data_task(void *arg);

  // System properties
  static constexpr double REDUCTION_RATIO = 200.0;
  static constexpr double MIN_DUTY_CYCLE = 0.5;
  static constexpr double ALPHA = 0.1;
  static constexpr double VELOCITY_SLOPE = 1.0404475763;
  static constexpr double POSITION_SLOPE = 1.0404475763;

  // PID controller properties
  static constexpr double PID_MAX_OUTPUT = 1.0;
  static constexpr double PID_MIN_OUTPUT = 0.0;
  static constexpr double PID_HYSTERESIS = 0.01;

  // static constexpr double kc = 0.02704;
  // static constexpr double ti = 0.06142;
  // static constexpr double td = 0.01536;

  static constexpr double kp = 0.1;
  static constexpr double ki = 3.3;
  static constexpr double kd = 0.01;

  // MCPWM properties
  static constexpr uint32_t TIMER_RES = 80000000; // 80 MHz
  static constexpr uint32_t TIMER_FREQ = 20000;   // 20 kHz
  static constexpr uint32_t TIMER_PERIOD = TIMER_RES / TIMER_FREQ;

  // PCNT properties
  static constexpr int16_t ENCODER_HIGH_LIMIT = REDUCTION_RATIO * 11.0 * 4.0;
  static constexpr int16_t ENCODER_LOW_LIMIT = -ENCODER_HIGH_LIMIT;
  static constexpr int16_t ENCODER_GLITCH_NS = 1000; // Glitch filter width in ns

  // Conversion constants
  static constexpr double US_TO_MS = 1000.0;
  static constexpr double US_TO_S = 1000000.0;
  static constexpr double PPUS_TO_RAD_S = (2 * M_PI) / (REDUCTION_RATIO * 11.0 * 4.0) * US_TO_S;
  static constexpr double PULSE_TO_RAD = (2 * M_PI) / (REDUCTION_RATIO * 11.0 * 4.0);

  // Class variables
  double timestamp;
  int8_t direction;
  double duty_cycle;
  double velocity;
  double velocity_ema;
  double position;

public:
  MotorController();

  void init();
  void stop_motor();

  // Accessor and mutator functions
  void set_duty_cycle(double dc);
  void set_direction(MotorDirection dir);
  void set_velocity(double sp);
  void set_position(double pos);

  double get_timestamp();
  double get_duty_cycle();
  int8_t get_direction();
  double get_velocity();
  double get_velocity_ema();
  double get_position();

  void pid_velocity(double sp);

  void enable_display();
  void disable_display();
};

#endif // MOTOR_CONTROLLER_H_
