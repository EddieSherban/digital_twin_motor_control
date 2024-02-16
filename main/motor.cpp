#pragma once

#include "motor.hpp"

static constexpr char* TAG = "motor";

// PIN CONFIGURATIONS
static constexpr gpio_num_t GPIO_ENA = GPIO_NUM_1;  // MCPWM output (Connected to ENA)
static constexpr gpio_num_t GPIO_IN1 = GPIO_NUM_2;  // GPIO output (Connected to IN1)
static constexpr gpio_num_t GPIO_IN2 = GPIO_NUM_42; // GPIO output (Connected to IN2)

static constexpr gpio_num_t GPIO_ENCODER_A = GPIO_NUM_41;   // GPIO output (Connected to Encoder A)
static constexpr gpio_num_t GPIO_ENCODER_B = GPIO_NUM_40;   // GPIO output (Connected to Encoder B)

// MCPWM PROPORTIES
static constexpr uint32_t TIMER_RES = 10000000; // 10 MHz
static constexpr uint16_t TIMER_FREQ = 25000;   // 25 kHz
static constexpr uint32_t TIMER_PERIOD = TIMER_RES / TIMER_FREQ;

// PCNT PROPORTIES
static constexpr int32_t ENCODER_HIGH_LIMIT = 8800;
static constexpr int32_t ENCODER_LOW_LIMIT = -ENCODER_HIGH_LIMIT;
static constexpr int32_t ENCODER_GLITCH_NS = 1000;                  // Glitch filter width in ns

// MONITOR PROPORTIES
static constexpr float SAMPLE_RATE = 10;                // Monitoring sample rate in ms
static constexpr int32_t STACK_SIZE = 4096;
static constexpr int8_t TASK_PRIO = tskIDLE_PRIORITY;   // Priority level Idle
static constexpr int8_t TASK_CORE = 1;                  // Run task on Core 1

// CONVERSION CONSTANTS
static constexpr uint32_t MSEC_TO_USEC = 1000;
static constexpr uint32_t SEC_TO_USEC = 1000000;

static motor *motor_object;

motor::motor()
{
    motor_object = this;
}

void motor::init()
{
    ESP_LOGI(TAG, "Setting up output to ENA...");
    mcpwm_timer_handle_t timer = nullptr;
    mcpwm_timer_config_t timer_config = 
    {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // 160 MHz
        .resolution_hz = TIMER_RES,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = TIMER_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = nullptr;
    mcpwm_operator_config_t oper_config =
    {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    cmpr = nullptr;
    mcpwm_comparator_config_t cmpr_config =
    {
        .flags =
        {
            .update_cmp_on_tez = true,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &cmpr));

    mcpwm_gen_handle_t gen = nullptr;
    mcpwm_generator_config_t gen_config =
    {
        .gen_gpio_num = GPIO_ENA,
        .flags = 
        {
            .pull_down = 1,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, 0));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Setting up outputs to IN1 and IN2...");
    gpio_config_t output_config = 
    {
        .pin_bit_mask = ((1ULL<<GPIO_IN1) | (1ULL<<GPIO_IN2)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&output_config);

    ESP_LOGI(TAG, "Setting up inputs for encoder A and B...");
    unit = nullptr;
    pcnt_unit_config_t unit_config = 
    {
        .low_limit = ENCODER_LOW_LIMIT,
        .high_limit = ENCODER_HIGH_LIMIT,
        .flags = 
        {
            .accum_count = 1,
        },
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

    pcnt_glitch_filter_config_t filter_config = 
    {
        .max_glitch_ns = ENCODER_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

    pcnt_channel_handle_t channel_a = nullptr;
    pcnt_chan_config_t channel_a_config = 
    {
        .edge_gpio_num = GPIO_ENCODER_A,
        .level_gpio_num = GPIO_ENCODER_B,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel_a_config, &channel_a));
    pcnt_channel_handle_t channel_b = nullptr;
    pcnt_chan_config_t channel_b_config = 
    {
        .edge_gpio_num = GPIO_ENCODER_B,
        .level_gpio_num = GPIO_ENCODER_A,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel_b_config, &channel_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, ENCODER_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, ENCODER_HIGH_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));

    ESP_LOGI(TAG, "Setting up monitoring task...");
    monitor_task_hdl = NULL;
    xTaskCreatePinnedToCore(monitor_task, "Monitor Task", STACK_SIZE, nullptr, TASK_PRIO, &monitor_task_hdl, TASK_CORE);
    vTaskSuspend(monitor_task_hdl);
}

void motor::monitor_task(void *arg)
{
    while (1)
    {
        motor_object->monitor();
        vTaskDelay(SAMPLE_RATE/portTICK_PERIOD_MS);
    }
}

void motor::monitor()
{
    static uint64_t time_prev = 0;
    static uint64_t time_curr = 0;

    static int pcnt_prev = 0;
    static int pcnt_curr = 0;

    static float encoder_rpm = 0;
    static float encoder_rotations = 0;
    static float encoder_deg = 0;

    time_curr = esp_timer_get_time();
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &pcnt_curr));

    encoder_rpm = fabs((float)(pcnt_curr - pcnt_prev) / (float)(time_curr - time_prev) * (float)SEC_TO_USEC) / 8800 * 60;
    encoder_rotations = (float)pcnt_curr / 8800;
    encoder_deg = (float)pcnt_curr / 8800 * 360;

    ESP_LOGI(TAG, "| Timestamp (ms): %llu | Speed (RPM): %.2f | Δ Position (Deg): %.2f | Rotation(s): %.2f |", time_curr / MSEC_TO_USEC, encoder_rpm, encoder_deg, encoder_rotations);
    
    time_prev = esp_timer_get_time();
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &pcnt_prev));
}

void motor::enable_monitor()
{
    ESP_LOGI(TAG, "Enabling monitoring task...");
    vTaskResume(monitor_task_hdl);
}

void motor::disable_monitor()
{
    ESP_LOGI(TAG, "Disabling monitoring task...");
    vTaskSuspend(monitor_task_hdl);
}

void motor::stop()
{
    ESP_LOGI(TAG, "Stopping motor.");
    gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 0);
}

void motor::set_speed(float duty_cycle)
{
    ESP_LOGI(TAG, "Setting motor duty cycle to %.2f.", duty_cycle);
    duty_cycle = duty_cycle * 0.5 + 50;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, TIMER_PERIOD * (duty_cycle / 100)));
}

void motor::set_direction(int8_t direction)
{
    if (direction == CLOCKWISE)
    {
        ESP_LOGI(TAG, "Setting motor direction to clockwise.");
        gpio_set_level(GPIO_IN1, 1);
        gpio_set_level(GPIO_IN2, 0);
    }
    else
    {
        ESP_LOGI(TAG, "Setting motor direction to counter-clockwise.");
        gpio_set_level(GPIO_IN1, 0);
        gpio_set_level(GPIO_IN2, 1);
    }
}