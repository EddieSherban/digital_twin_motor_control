// Includes
#include "motor_controller.hpp"

static constexpr char *TAG = "Motor";

static MotorController *motor_obj;
static Communication comm;
static CurrentSensor curr_sen;

MotorController::MotorController()
{
  motor_obj = this;

  // Allocate memory
  timestamp_string[0].reserve(timestamp_size);
  direction_string[0].reserve(direction_size);
  duty_cycle_string[0].reserve(duty_cycle_size);
  velocity_string[0].reserve(velocity_size);
  position_string[0].reserve(position_size);
  current_string[0].reserve(current_size);

  timestamp_string[1].reserve(timestamp_size);
  direction_string[1].reserve(direction_size);
  duty_cycle_string[1].reserve(duty_cycle_size);
  velocity_string[1].reserve(velocity_size);
  position_string[1].reserve(position_size);
  current_string[1].reserve(current_size);

  sample_string.reserve(sample_size);

  sample_time = 0;
  actual_direction = 0;
  duty_cycle_mag = 0;
  velocity_mag = 0;
  absolute_position = 0;

  mode = STOP;
  velocity_sp = 0;

  timestamp = 0;
  direction = 0;
  duty_cycle = 0;
  velocity = 0;
  position = 0;
  current = 0;

  curr_buffer = 0;
  buffer_ready = false;
  tx_ready = 0;

  cmpr_hdl = nullptr;
  unit_hdl = nullptr;

  // parameter_semaphore = xSemaphoreCreateMutex();

  update_task_hdl = NULL;
  format_task_hdl = NULL;
  pid_task_hdl = NULL;
  tx_data_task_hdl = NULL;
  display_task_hdl = NULL;
}

void MotorController::init()
{
  ESP_LOGI(TAG, "Setting up output to ENA.");

  mcpwm_timer_handle_t timer_hdl = nullptr;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = TIMER_RES,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = TIMER_PERIOD,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_hdl));

  mcpwm_oper_handle_t oper_hdl = nullptr;
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

  mcpwm_gen_handle_t gen_hdl = nullptr;
  mcpwm_generator_config_t gen_config = {
      .gen_gpio_num = GPIO_ENA,
      .flags = {
          .pull_down = 1,
      },
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper_hdl, &gen_config, &gen_hdl));

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_hdl, TIMER_PERIOD * MIN_DUTY_CYCLE));
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
  gpio_set_level(GPIO_IN1, 0);
  gpio_set_level(GPIO_IN2, 0);

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

  pcnt_channel_handle_t channel_a_hdl = nullptr;
  pcnt_chan_config_t channel_a_config = {
      .edge_gpio_num = GPIO_C1,
      .level_gpio_num = GPIO_C2,
  };
  ESP_ERROR_CHECK(pcnt_new_channel(unit_hdl, &channel_a_config, &channel_a_hdl));
  pcnt_channel_handle_t channel_b_hdl = nullptr;
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

  pcnt_event_callbacks_t pcnt_cbs = {
      .on_reach = pcnt_callback,
  };
  QueueHandle_t queue = xQueueCreate(10, sizeof(int));
  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(unit_hdl, &pcnt_cbs, queue));

  ESP_ERROR_CHECK(pcnt_unit_enable(unit_hdl));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit_hdl));
  ESP_ERROR_CHECK(pcnt_unit_start(unit_hdl));

  ESP_LOGI(TAG, "Initiate and zero current sensor.");
  curr_sen.init();
  stop_motor();
  curr_sen.zero();

  ESP_LOGI(TAG, "Setting up update task.");
  xTaskCreatePinnedToCore(update_trampoline, "Update Task", update_config.stack_size, nullptr, update_config.priority, &update_task_hdl, update_config.core);

  ESP_LOGI(TAG, "Setting up formatting task.");
  xTaskCreatePinnedToCore(format_trampoline, "Format Task", format_config.stack_size, nullptr, format_config.priority, &format_task_hdl, format_config.core);

  ESP_LOGI(TAG, "Setting up PID controller task.");
  xTaskCreatePinnedToCore(pid_trampoline, "PID Controller Task", pid_config.stack_size, nullptr, pid_config.priority, &pid_task_hdl, pid_config.core);
  vTaskSuspend(pid_task_hdl);

  ESP_LOGI(TAG, "Setting up display task.");
  xTaskCreatePinnedToCore(display_task, "Display Task", display_config.stack_size, nullptr, display_config.priority, &display_task_hdl, display_config.core);
  vTaskSuspend(display_task_hdl);

  ESP_LOGI(TAG, "Initiate and set up communication task.");
  comm.init();
  xTaskCreatePinnedToCore(tx_data_task, "TX Data Task", tx_config.stack_size, nullptr, tx_config.priority, &tx_data_task_hdl, tx_config.core);
}

bool MotorController::pcnt_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
  static uint64_t prev_time = 0;

  BaseType_t high_task_wakeup;
  QueueHandle_t queue = (QueueHandle_t)user_ctx;
  xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);

  motor_obj->sample_time = esp_timer_get_time();
  motor_obj->actual_direction = (edata->watch_point_value) / abs(edata->watch_point_value);
  motor_obj->velocity_mag = CALI_FACTOR * (VELOCITY_SAMPLE_SIZE / (esp_timer_get_time() - prev_time)) * PPUS_TO_RPM;

  prev_time = motor_obj->sample_time;
  return (high_task_wakeup == pdTRUE);
}

void MotorController::update_trampoline(void *arg)
{
  while (1)
  {
    motor_obj->update_task();

    vTaskDelay(update_config.delay / portTICK_PERIOD_MS);
  }
}

void MotorController::update_task()
{
  static int pcnt = 0;
  static MovingAverage velocity_average(VELOCITY_WINDOW_SIZE);

  static char temp_string[20];
  static uint16_t sample_count = 0;

  static uint64_t prev_time = 0;
  static uint64_t curr_time = 0;

  curr_time = esp_timer_get_time();
  if (curr_time - prev_time > (FREQUENCY * US_TO_MS))
  {
    if (direction == CLOCKWISE)
      set_direction(COUNTERCLOCKWISE);
    else if (direction == COUNTERCLOCKWISE)
      set_direction(CLOCKWISE);
    prev_time = curr_time;
  }

  ESP_ERROR_CHECK(pcnt_unit_get_count(unit_hdl, &pcnt));

  // Zero velocity if no counts for timeout interval
  if (esp_timer_get_time() - sample_time > (TIMEOUT * US_TO_MS))
    velocity = 0;

  auto now = chrono::system_clock::now();
  auto unix_time = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
  timestamp = (uint64_t)unix_time;
  duty_cycle = direction * duty_cycle_mag;
  velocity = velocity_average.next(actual_direction * velocity_mag);
  absolute_position = CALI_FACTOR * (float)pcnt * PULSE_TO_DEG;
  position = fmod(absolute_position, 360.0); // Use calibration factor to adjust position to true value
  current = curr_sen.read_current();

  sprintf(temp_string, "%llu,", timestamp);
  timestamp_string[curr_buffer] += temp_string;

  sprintf(temp_string, "%ld,", direction);
  direction_string[curr_buffer] += temp_string;

  sprintf(temp_string, "%.3f,", duty_cycle);
  duty_cycle_string[curr_buffer] += temp_string;

  sprintf(temp_string, "%.3f,", velocity);
  velocity_string[curr_buffer] += temp_string;

  sprintf(temp_string, "%.3f,", position);
  position_string[curr_buffer] += temp_string;

  sprintf(temp_string, "%.3f,", current);
  current_string[curr_buffer] += temp_string;

  sample_count++;

  if (sample_count >= VECTOR_SIZE)
  {
    curr_buffer = (curr_buffer + 1) % 2;
    buffer_ready = true;
    sample_count = 0;
  }
}

void MotorController::format_trampoline(void *arg)
{
  while (1)
  {
    motor_obj->format_task();

    vTaskDelay(format_config.delay / portTICK_PERIOD_MS);
  }
}

void MotorController::format_task()
{
  if (buffer_ready)
  {
    format_samples();
    tx_ready = true;
    telemetry_ready = true;
    buffer_ready = false;
  }
}

void MotorController::pid_trampoline(void *arg)
{
  while (1)
  {
    motor_obj->pid_task();

    vTaskDelay(pid_config.delay / portTICK_PERIOD_MS);
  }
}

void MotorController::pid_task()
{
  static uint64_t prev_time = esp_timer_get_time();
  static uint64_t curr_time = 0;
  static float diff_time = 0;

  static float error_prev = 0;
  static float error = 0;
  static float integral = 0;
  static float derivative = 0;

  static float output_prev = 0;
  static float output = 0;

  curr_time = esp_timer_get_time();
  diff_time = (curr_time - prev_time) / US_TO_S;

  error = velocity_sp - velocity;
  integral += error * diff_time;
  derivative = (error - error_prev) / diff_time;

  // Restrict integral to prevent integral windup
  if (integral > PID_WINDUP)
    integral = PID_WINDUP;
  if (integral < -PID_WINDUP)
    integral = -PID_WINDUP;

  output = kp * (error + 1 / ti * integral + td * derivative);

  // Restrict output to duty cycle range
  if (output > PID_MAX_OUTPUT)
    output = PID_MAX_OUTPUT;
  else if (output < PID_MIN_OUTPUT)
    output = PID_MIN_OUTPUT;

  // Controller only outputs a new value when error is outside oscillation threshold
  if (fabs(error) <= (velocity_sp * PID_OSCILLATION))
    output = output_prev;

  set_duty_cycle(output);

  prev_time = curr_time;
  error_prev = error;
  output_prev = output;
}

void MotorController::display_task(void *arg)
{
  while (1)
  {
    ESP_LOGI(TAG, "Timestamp: %llu, Direction: %ld, Duty Cycle: %.3f, Velocity (RPM): %.3f, Position (Deg): %.3f, Current (mA): %.3f",
             motor_obj->timestamp,
             motor_obj->direction,
             motor_obj->duty_cycle,
             motor_obj->velocity,
             motor_obj->position,
             motor_obj->current);

    vTaskDelay(display_config.delay / portTICK_PERIOD_MS);
  }
}

void MotorController::tx_data_task(void *arg)
{
  while (1)
  {
    if (motor_obj->tx_ready)
    {
      comm.send_data(motor_obj->sample_string.c_str(), motor_obj->sample_string.length());
      motor_obj->tx_ready = false;
    }

    vTaskDelay(tx_config.delay / portTICK_PERIOD_MS);
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
  mode = STOP;

  ESP_LOGI(TAG, "Stopping motor.");
  gpio_set_level(GPIO_IN1, 0);
  gpio_set_level(GPIO_IN2, 0);
}

void MotorController::set_mode(int32_t mode)
{
  this->mode = mode;

  switch (mode)
  {
  case MANUAL:
    ESP_LOGI(TAG, "Setting controller mode to manual.");
    vTaskSuspend(pid_task_hdl);
    break;
  case STOP:
    ESP_LOGI(TAG, "Stopping motor.");
    gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 0);
    vTaskSuspend(pid_task_hdl);
    break;
  case AUTO:
    ESP_LOGI(TAG, "Setting controller mode to auto.");
    vTaskResume(pid_task_hdl);
    break;
  }
}

void MotorController::set_direction(int32_t direction)
{
  this->direction = direction;

  switch (direction)
  {
  case CLOCKWISE:
    ESP_LOGI(TAG, "Setting motor direction to clockwise.");
    gpio_set_level(GPIO_IN1, 1);
    gpio_set_level(GPIO_IN2, 0);
    break;
  case COUNTERCLOCKWISE:
    ESP_LOGI(TAG, "Setting motor direction to counter-clockwise.");
    gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 1);
    break;
  }
}

void MotorController::set_duty_cycle(float duty_cycle)
{
  if (duty_cycle > 1.0)
    duty_cycle = 1.0;
  this->duty_cycle_mag = duty_cycle;

  if (mode == MANUAL)
    ESP_LOGI(TAG, "Setting motor duty cycle to %.3f.", duty_cycle);
  duty_cycle = (duty_cycle * (1 - MIN_DUTY_CYCLE)) + MIN_DUTY_CYCLE; // Changes scale
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_hdl, TIMER_PERIOD * duty_cycle));
}

void MotorController::set_velocity(float velocity_sp)
{
  this->velocity_sp = velocity_sp;

  ESP_LOGI(TAG, "Setting PID set point to %.3f.", velocity_sp);
}

uint64_t MotorController::get_timestamp()
{
  return timestamp;
}

float MotorController::get_duty_cycle()
{
  return duty_cycle;
}

int32_t MotorController::get_direction()
{
  return direction;
}

float MotorController::get_velocity()
{
  return velocity;
}

float MotorController::get_position()
{
  return position;
}

float MotorController::get_current()
{
  return current;
}

void MotorController::format_samples()
{
  static uint8_t prev_buffer = 0;
  prev_buffer = (curr_buffer + 1) % 2;

  timestamp_string[prev_buffer].pop_back();
  direction_string[prev_buffer].pop_back();
  duty_cycle_string[prev_buffer].pop_back();
  velocity_string[prev_buffer].pop_back();
  position_string[prev_buffer].pop_back();
  current_string[prev_buffer].pop_back();

  sample_string = "{\"timestamp\":[" + timestamp_string[prev_buffer] + "],\n" +
                  "\"direction\":[" + direction_string[prev_buffer] + "],\n" +
                  "\"duty_cycle\":[" + duty_cycle_string[prev_buffer] + "],\n" +
                  "\"velocity\":[" + velocity_string[prev_buffer] + "],\n" +
                  "\"position\":[" + position_string[prev_buffer] + "],\n" +
                  "\"current\":[" + current_string[prev_buffer] + "]}";

  timestamp_string[prev_buffer].clear();
  direction_string[prev_buffer].clear();
  duty_cycle_string[prev_buffer].clear();
  velocity_string[prev_buffer].clear();
  position_string[prev_buffer].clear();
  current_string[prev_buffer].clear();
}

string MotorController::get_sample_string()
{
  telemetry_ready = false;
  return sample_string;
}

bool MotorController::get_telemetry_ready()
{
  return telemetry_ready;
}