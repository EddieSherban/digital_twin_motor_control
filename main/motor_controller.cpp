// Includes
#include "motor_controller.hpp"

static constexpr char *TAG = "Motor";

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
static constexpr int16_t ENCODER_HIGH_LIMIT = 2200;
static constexpr int16_t ENCODER_LOW_LIMIT = -ENCODER_HIGH_LIMIT;
static constexpr int16_t ENCODER_GLITCH_NS = 1000; // Glitch filter width in ns

// Update task properties
static constexpr uint8_t UPDATE_RATE = 5; // Monitoring sample rate in ms
static constexpr int16_t UPDATE_STACK_SIZE = 1024 * 4;
static constexpr UBaseType_t UPDATE_TASK_PRIO = configMAX_PRIORITIES - 1; // High priority
static constexpr int8_t UPDATE_TASK_CORE = 1;                             // Run task on Core 1

// Display task properties
static constexpr uint8_t DISPLAY_RATE = 10; // Display rate in ms
static constexpr int16_t DISPLAY_STACK_SIZE = 1024 * 4;
static constexpr UBaseType_t DISPLAY_TASK_PRIO = configMAX_PRIORITIES - 3; // Priority level Idle
static constexpr int8_t DISPLAY_TASK_CORE = 0;                             // Run task on Core 0

// Conversion constants
static constexpr double REDUCTION_RATIO = 200.0;
static constexpr double US_TO_MS = 1000.0;
static constexpr double US_TO_S = 1000000.0;
static constexpr double PPUS_TO_RAD_S = (2 * M_PI) / (REDUCTION_RATIO * 11.0  * 4.0) * US_TO_S;
static constexpr double PULSE_TO_RAD = (2 * M_PI) / (REDUCTION_RATIO * 11.0  * 4.0);
static constexpr double MIN_DUTY_CYCLE = 0.5;

// PID controller constants
static constexpr double PID_MAX = 1.0;
static constexpr double PID_MIN = 0.0;
static constexpr double PID_HYSTERESIS = 0.125 * 1.2; // 0.125;

static constexpr double kc = 0.02704;
static constexpr double ti = 0.06142;
static constexpr double td = 0.01536;

static constexpr double kp = 0.1;
static constexpr double ki = 3.3;
static constexpr double kd = 0;

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

  update_task_hdl = NULL;
  display_task_hdl = NULL;

  timestamp = 0;
  direction = STOPPED;
  velocity = 0;
  position = 0;
}

void MotorController::init()
{
  ESP_LOGI(TAG, "Setting up output to ENA.");
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

  ESP_LOGI(TAG, "Setting up outputs to IN1 and IN2.");
  gpio_config_t output_config = {
      .pin_bit_mask = ((1ULL << GPIO_IN1) | (1ULL << GPIO_IN2)),
      .mode = GPIO_MODE_OUTPUT,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&output_config);

  ESP_LOGI(TAG, "Setting up inputs for encoder A and B.");
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

  ESP_LOGI(TAG, "Setting up updating task.");
  xTaskCreatePinnedToCore(update_trampoline, "Update", UPDATE_STACK_SIZE, nullptr, UPDATE_TASK_PRIO, &update_task_hdl, UPDATE_TASK_CORE);

  ESP_LOGI(TAG, "Setting up display task.");
  xTaskCreatePinnedToCore(display, "Display", DISPLAY_STACK_SIZE, nullptr, DISPLAY_TASK_PRIO, &display_task_hdl, DISPLAY_TASK_CORE);
  vTaskSuspend(display_task_hdl);
}

void MotorController::update_trampoline(void *arg)
{
  while (1)
  {
    motor_obj->update();
    vTaskDelay(UPDATE_RATE / portTICK_PERIOD_MS);
  }
}

void MotorController::update()
{
  static uint64_t time_prev = 0;
  static uint64_t time_curr = 0;
  static uint64_t time_diff = 0;

  static int pcnt_prev = 0;
  static int pcnt_curr = 0;
  static int pcnt_diff = 0;

  time_curr = esp_timer_get_time();
  ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt_curr));

  time_diff = time_curr - time_prev;
  pcnt_diff = pcnt_curr - pcnt_prev;

  if (pcnt_diff < 0)
    direction = COUNTERCLOCKWISE;
  else if (pcnt_diff > 0)
    direction = CLOCKWISE;
  else
    direction = STOPPED;

  timestamp = (double)time_curr / US_TO_MS;
  velocity = ((double)abs(pcnt_diff) / (double)time_diff) * PPUS_TO_RAD_S;
  position = (double)pcnt_curr * PULSE_TO_RAD;

  ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt_prev));
  time_prev = esp_timer_get_time();
}

void MotorController::display(void *arg)
{
  while (1)
  {
    ESP_LOGI(TAG, "Timestamp (ms): %.3f, Direction: %d, Velocity (rad/s): %.3f , Position (rad): %.3f", motor_obj->timestamp, motor_obj->direction, motor_obj->velocity, motor_obj->position);
    vTaskDelay(DISPLAY_RATE / portTICK_PERIOD_MS);
  }
}

void MotorController::enable_display()
{
  ESP_LOGI(TAG, "Enabling display.");
  vTaskResume(display_task_hdl);
}

void MotorController::disable_display()
{
  ESP_LOGI(TAG, "Disabling display.");
  vTaskSuspend(display_task_hdl);
}

void MotorController::stop_motor()
{
  ESP_LOGI(TAG, "Stopping motor.");
  gpio_set_level(GPIO_IN1, 0);
  gpio_set_level(GPIO_IN2, 0);
}

void MotorController::set_direction(MotorDir dir)
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

void MotorController::set_duty_cycle(double dc)
{
  // ESP_LOGI(TAG, "Setting motor duty cycle to %.3f.", duty_cycle);
  duty_cycle = dc;
  dc = (dc * MIN_DUTY_CYCLE) + MIN_DUTY_CYCLE; // Changes scale
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_hdl, TIMER_PERIOD * dc));
}

double MotorController::get_timestamp()
{
  return timestamp;
}

double MotorController::get_duty_cycle()
{
  return duty_cycle;
}

int8_t MotorController::get_direction()
{
  return direction;
}

double MotorController::get_velocity()
{
  return velocity;
}

double MotorController::get_position()
{
  return position;
}

void MotorController::pid_velocity(double set_point)
{
  static uint64_t time_prev = 0;
  static uint64_t timer_curr = 0;
  static double dt = 0;

  static double error_prev = 0;
  static double error = 0;
  static double integral = 0;
  static double derivative = 0;
  static double prev_output = 0;
  static double output = 0;

  timer_curr = esp_timer_get_time();
  dt = (timer_curr - time_prev) / US_TO_S;

  error = set_point - get_velocity();
  integral += error * dt;
  derivative = (error - error_prev) / dt;

  // output = kc * (error + (1 / ti) * integral + td * derivative);
  output = kp * error + ki * integral + kd * derivative;

  // Keep output within range
  if (output > PID_MAX)
    output = PID_MAX;
  else if (output < PID_MIN)
    output = PID_MIN;

  // Only sets new output when error is large enough
  if (fabs(error) <= PID_HYSTERESIS)
    output = prev_output;

  set_duty_cycle(output);

  prev_output = output;
  error_prev = set_point - get_velocity();
  time_prev = esp_timer_get_time();
}