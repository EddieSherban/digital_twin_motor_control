// Includes
#include "motor_controller.hpp"

static constexpr char *TAG = "motor";

// Pin configurations
static constexpr gpio_num_t GPIO_ENA = GPIO_NUM_1;  // MCPWM output (Connected to ENA)
static constexpr gpio_num_t GPIO_IN1 = GPIO_NUM_2;  // GPIO output (Connected to IN1)
static constexpr gpio_num_t GPIO_IN2 = GPIO_NUM_42; // GPIO output (Connected to IN2)

static constexpr gpio_num_t GPIO_C1 = GPIO_NUM_41; // GPIO output (Connected to Encoder A) GREEN
static constexpr gpio_num_t GPIO_C2 = GPIO_NUM_40; // GPIO output (Connected to Encoder B) YELLOW

// MCPWM properties
static constexpr uint32_t TIMER_RES = 80000000; // 80 MHz
static constexpr uint16_t TIMER_FREQ = 20000;   // 20 kHz
static constexpr uint32_t TIMER_PERIOD = TIMER_RES / TIMER_FREQ;

// PCNT properties
static constexpr int32_t ENCODER_HIGH_LIMIT = 2200;
static constexpr int32_t ENCODER_LOW_LIMIT = -ENCODER_HIGH_LIMIT;
static constexpr int32_t ENCODER_GLITCH_NS = 1000; // Glitch filter width in ns

// Monitor task properties
static constexpr uint8_t SAMPLE_RATE = 10; // Monitoring sample rate in ms
static constexpr int32_t STACK_SIZE = 4096;
static constexpr UBaseType_t TASK_PRIO = tskIDLE_PRIORITY; // Priority level Idle
static constexpr int8_t TASK_CORE = 1;                     // Run task on Core 1

// Conversion constants
static constexpr double USEC_PER_MSEC = 1000.0;
static constexpr double USEC_PER_SEC = 1000000.0;
static constexpr double RPM_PER_PULSE_US = USEC_PER_SEC * 60.0 / 8800.0;
static constexpr double PULSE_PER_DEG = 8800.0 / 360.0;
static constexpr double MIN_DUTY_CYCLE = 0.5;

// PID controller constants
static constexpr double PID_MAX = 1.0;
static constexpr double PID_MIN = 0.0;
static constexpr double PID_HYSTERESIS = 0.5;

static constexpr double kc = 0.02704;
static constexpr double ti = 0.06142;
static constexpr double td = 0.01536;

static MotorController *motor_obj;

MotorController::MotorController()
{
  motor_obj = this;

  timer_hdl = nullptr;
  oper_hdl = nullptr;
  cmpr_hdl = nullptr;
  gen_hdl = nullptr;
  unit_hdl = nullptr;
  channel_a_hdl = nullptr;
  channel_b_hdl = nullptr;

  monitor_task_hdl = NULL;
  monitor = false;

  timestamp = 0;
  speed = 0;
  pos = 0;
  dir = 0;
}

void MotorController::init()
{
  ESP_LOGD(TAG, "Setting up output to ENA.");
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = TIMER_RES,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = TIMER_PERIOD,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_hdl));

  mcpwm_operator_config_t oper_config = {
      .group_id = 0,
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper_hdl));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_hdl, timer_hdl));

  mcpwm_comparator_config_t cmpr_config = {
      .flags = {
          .update_cmp_on_tez = true,
      },
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper_hdl, &cmpr_config, &cmpr_hdl));

  mcpwm_generator_config_t gen_config = {
      .gen_gpio_num = GPIO_ENA,
      .flags = {
          .pull_down = 1,
      },
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper_hdl, &gen_config, &gen_hdl));

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_hdl, TIMER_PERIOD * 0.5));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_hdl, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_hdl, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_hdl, MCPWM_GEN_ACTION_LOW)));

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer_hdl));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_hdl, MCPWM_TIMER_START_NO_STOP));

  ESP_LOGD(TAG, "Setting up outputs to IN1 and IN2.");
  gpio_config_t output_config = {
      .pin_bit_mask = ((1ULL << GPIO_IN1) | (1ULL << GPIO_IN2)),
      .mode = GPIO_MODE_OUTPUT,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&output_config);

  ESP_LOGD(TAG, "Setting up inputs for encoder A and B.");
  pcnt_unit_config_t unit_config = {
      .low_limit = ENCODER_LOW_LIMIT,
      .high_limit = ENCODER_HIGH_LIMIT,
      .flags = {
          .accum_count = 1,
      },
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit_hdl));

  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = ENCODER_GLITCH_NS,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit_hdl, &filter_config));

  pcnt_chan_config_t channel_a_config = {
      .edge_gpio_num = GPIO_C1,
      .level_gpio_num = GPIO_C2,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(unit_hdl, &channel_a_config, &channel_a_hdl));
  pcnt_chan_config_t channel_b_config = {
      .edge_gpio_num = GPIO_C2,
      .level_gpio_num = GPIO_C1,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(unit_hdl, &channel_b_config, &channel_b_hdl));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_a_hdl, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_a_hdl, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_b_hdl, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_b_hdl, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_hdl, ENCODER_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_hdl, ENCODER_HIGH_LIMIT));

  ESP_ERROR_CHECK(pcnt_unit_enable(unit_hdl));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit_hdl));
  ESP_ERROR_CHECK(pcnt_unit_start(unit_hdl));

  ESP_LOGD(TAG, "Setting up monitoring task.");
  xTaskCreatePinnedToCore(monitor_trampoline, "Monitor Task", STACK_SIZE, nullptr, TASK_PRIO, &monitor_task_hdl, TASK_CORE);
}

void MotorController::monitor_trampoline(void *arg)
{
  while (1)
  {
    motor_obj->monitor_motor();
    vTaskDelay(SAMPLE_RATE / portTICK_PERIOD_MS);
  }
}

void MotorController::monitor_motor()
{
  static uint64_t time_prev = 0;
  static uint64_t time_curr = 0;
  static uint64_t time_diff = 0;
  static uint64_t time_temp = 0;

  static int pcnt_prev = 0;
  static int pcnt_curr = 0;
  static int pcnt_diff = 0;
  static int pcnt_temp = 0;

  time_temp = esp_timer_get_time();
  ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt_temp));
  while (abs(pcnt_temp) >= abs(pcnt_curr) || (time_temp - esp_timer_get_time()) / USEC_PER_MSEC < (SAMPLE_RATE / 4.0))
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt_curr));
  time_curr = esp_timer_get_time();

  time_diff = time_curr - time_prev;
  pcnt_diff = pcnt_curr - pcnt_prev;

  // if (pcnt_diff < 0)
  //   dir = COUNTERCLOCKWISE;
  // else if (pcnt_diff > 0)
  //   dir = CLOCKWISE;
  // else
  //   dir = 0;

  timestamp = (double)time_curr / USEC_PER_MSEC;
  speed = ((double)abs(pcnt_diff) / (double)time_diff) * RPM_PER_PULSE_US;
  pos = (double)pcnt_curr / PULSE_PER_DEG;

  if (monitor)
    ESP_LOGI(TAG, "Timestamp (ms): %.3f | Speed (RPM): %.3f | Position (Deg): %.3f", timestamp, speed, pos);

  // ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt_prev));
  time_temp = esp_timer_get_time();
  ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt_temp));
  while (abs(pcnt_temp) >= abs(pcnt_prev) || (time_temp - esp_timer_get_time()) / USEC_PER_MSEC < (SAMPLE_RATE / 4.0))
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt_prev));
  time_prev = esp_timer_get_time();
}

void MotorController::enable_monitor()
{
  ESP_LOGI(TAG, "Enabling monitoring output.");
  monitor = true;
}

void MotorController::disable_monitor()
{
  ESP_LOGI(TAG, "Disabling monitoring output.");
  monitor = false;
}

void MotorController::stop_motor()
{
  ESP_LOGI(TAG, "Stopping motor.");
  gpio_set_level(GPIO_IN1, 0);
  gpio_set_level(GPIO_IN2, 0);
}

void MotorController::set_speed(double duty_cycle)
{
  // ESP_LOGI(TAG, "Setting motor duty cycle to %.2f.", duty_cycle);
  duty_cycle = duty_cycle * MIN_DUTY_CYCLE + MIN_DUTY_CYCLE; // Changes scale
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_hdl, TIMER_PERIOD * duty_cycle));
}

void MotorController::set_dir(MotorDir dir)
{
  if (dir == CLOCKWISE)
  {
    ESP_LOGI(TAG, "Setting motor direction to clockwise.");
    gpio_set_level(GPIO_IN2, 0);
    gpio_set_level(GPIO_IN1, 1);
  }
  else if (dir == COUNTERCLOCKWISE)
  {
    ESP_LOGI(TAG, "Setting motor direction to counter-clockwise.");
    gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 1);
  }
}

double MotorController::get_speed()
{
  return speed;
}

int8_t MotorController::get_dir()
{
  return dir;
}

void MotorController::pid_speed(double sp)
{

  static uint64_t time_prev = 0;
  static uint64_t timer_curr = 0;
  static double dt = 0;

  static double error_prev = 0;
  static double error = 0;
  static double integral = 0;
  static double derivative = 0;
  static double output = 0;

  timer_curr = esp_timer_get_time();
  dt = (timer_curr - time_prev) / USEC_PER_SEC;

  error = sp - get_speed();
  integral += error * dt;
  derivative = (error - error_prev) / dt;

  // Resets integral when process variable is within hysteresis threshold
  if (fabs(error) < PID_HYSTERESIS)
    integral = 0;

  output = kc * (error + (1 / ti) * integral + td * derivative);
  // output = kp * error + ki * integral + kd * derivative;

  // Keep output within range
  if (output > PID_MAX)
    output = PID_MAX;
  else if (output < PID_MIN)
    output = PID_MIN;

  // Only sets new output when error is large enough
  if (fabs(error) > PID_HYSTERESIS)
    set_speed(output);

  error_prev = sp - get_speed();
  time_prev = esp_timer_get_time();
}