#pragma once

#include "motor.hpp"

static constexpr char* TAG = "motor";

// PIN CONFIGURATIONS
static constexpr gpio_num_t GPIO_ENA = GPIO_NUM_1;  // MCPWM output on GPIO1 (Connected to ENA)
static constexpr gpio_num_t GPIO_IN1 = GPIO_NUM_2;  // GPIO output on GPIO2 (Connected to IN1)
static constexpr gpio_num_t GPIO_IN2 = GPIO_NUM_42; // GPIO output on GPIO42 (Connected to IN2)

static constexpr gpio_num_t GPIO_ENCODER_A = GPIO_NUM_41;   // GPIO output on GPIO41 (Connected to Encoder A)
static constexpr gpio_num_t GPIO_ENCODER_B = GPIO_NUM_40;   // GPIO output on GPIO40 (Connected to Encoder B)
// MCPWM PROPORTIES
static constexpr uint32_t TIMER_RES = 10000000; // 10 MHz Timer Resolution
static constexpr uint16_t TIMER_FREQ = 25000;   // 25 kHz Timer Frequency
static constexpr uint32_t TIMER_PERIOD = TIMER_RES / TIMER_FREQ;

// PCNT PROPORTIES
static constexpr int32_t ENCODER_HIGH_LIMIT = 8800;                 // 11 PPR * 200 Reduction Ratio * 4 (for Quadrature)
static constexpr int32_t ENCODER_LOW_LIMIT = -ENCODER_HIGH_LIMIT;
static constexpr int32_t ENCODER_GLITCH_NS = 1000;                  // 1 ns internal glitch filters 


void motor::init()
{
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Setting up L298N motor driver pins.");

    ESP_LOGI(TAG, "Setting up MCPWM output.");
    ESP_LOGI(TAG, "Creating timer.");
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

    ESP_LOGI(TAG, "Creating operator.");
    mcpwm_oper_handle_t oper = nullptr;
    mcpwm_operator_config_t oper_config =
    {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));

    ESP_LOGI(TAG, "Connecting timer and operator.");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Creating comparator.");
    cmpr = nullptr;
    mcpwm_comparator_config_t cmpr_config =
    {
        .flags =
        {
            .update_cmp_on_tez = true,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &cmpr));

    ESP_LOGI(TAG, "Creating generator.");
    mcpwm_gen_handle_t gen = nullptr;
    mcpwm_generator_config_t gen_config =
    {
        .gen_gpio_num = GPIO_ENA,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen));

    ESP_LOGI(TAG, "Setting initial comparator value to 0.");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, 0));

    ESP_LOGI(TAG, "Setting generator's actions on timer and compare events.");
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enabling and starting timer.");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Completed setting up MCPWM output.");

    ESP_LOGI(TAG, "Setting up GPIO outputs.");
    gpio_config_t output_config = 
    {
        .pin_bit_mask = ((1ULL<<GPIO_IN1) | (1ULL<<GPIO_IN2)),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&output_config);

    ESP_LOGI(TAG, "Completed setting up GPIO outputs.");

    ESP_LOGI(TAG, "Completed setting up L298N motor driver pins.");
    ESP_LOGI(TAG, "----------------------------------------");

    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Setting up encoder pins.");

    ESP_LOGI(TAG, "Setting up PCNT inputs.");

    ESP_LOGI(TAG, "Creating unit.");
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

    ESP_LOGI(TAG, "Creating glitch filter.");
    pcnt_glitch_filter_config_t filter_config = 
    {
        .max_glitch_ns = ENCODER_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

    ESP_LOGI(TAG, "Setting channels.");
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

    ESP_LOGI(TAG, "Setting edge and level actions for each channel.");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "Setting watch points.");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, ENCODER_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, ENCODER_HIGH_LIMIT));

    ESP_LOGI(TAG, "Enabling and starting PCNT.");
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));

    ESP_LOGI(TAG, "Completed setting up encoder pins.");
    ESP_LOGI(TAG, "----------------------------------------");

}

void motor::stop()
{
    ESP_LOGI(TAG, "Stopping motor.");
    gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 0);
}

void motor::set_speed(float duty_cycle)
{
    ESP_LOGI(TAG, "Setting motor speed to %f.", duty_cycle);
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