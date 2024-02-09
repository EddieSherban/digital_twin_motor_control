#pragma once

#include "L298N.hpp"

static constexpr char* TAG = "L298N";

static constexpr uint32_t TIMER_RES = 10000000;     // 10 MHz Timer Resolution
static constexpr uint16_t TIMER_FREQ = 25000;       // 25 kHz Timer Frequency
static constexpr gpio_num_t GEN_GPIO = GPIO_NUM_1;  // Generates PWM signal on GPIO1 (Connected to ENA)

static constexpr uint32_t TIMER_PERIOD = TIMER_RES / TIMER_FREQ;

void L298N::init()
{
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "Initializing PWM output.");
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
    mcpwm_cmpr_handle_t cmpr = nullptr;
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
        .gen_gpio_num = GEN_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen));

    ESP_LOGI(TAG, "Setting initial comparator value.");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, 0));

    ESP_LOGI(TAG, "Setting generator's actions on timer and compare events.");
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enabling and starting timer.");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, TIMER_PERIOD * 0.30));

    ESP_LOGI(TAG, "Completed MCPWM initialization.");
    ESP_LOGI(TAG, "----------------------------------------");

}