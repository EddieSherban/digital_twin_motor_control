#pragma once

#include "L298N.hpp"

#define TIMER_RES       9000000     // 9 MHz Timer Resolution
#define TIMER_FREQ      30000       // 30 kHz Timer Frequency
#define GEN_GPIO        1           // Generates PWM signal on GPIO1 (Connected to ENA)

#define TIMER_PERIOD    TIMER_RES / TIMER_FREQ

static const char* TAG = "L298N";

void L298N::init()
{
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
    mcpwm_new_timer(&timer_config, &timer);

    ESP_LOGI(TAG, "Creating operator.");
    mcpwm_oper_handle_t oper = nullptr;
    mcpwm_operator_config_t oper_config =
    {
        .group_id = 0,
    };
    mcpwm_new_operator(&oper_config, &oper);

    ESP_LOGI(TAG, "Connecting timer and operator.");
    mcpwm_operator_connect_timer(oper, timer);

    ESP_LOGI(TAG, "Creating comparator.");
    mcpwm_cmpr_handle_t cmpr = nullptr;
    mcpwm_comparator_config_t cmpr_config;
    cmpr_config.flags.update_cmp_on_tez = true;
    cmpr_config.intr_priority = 0;
    mcpwm_new_comparator(oper, &cmpr_config, &cmpr);

    ESP_LOGI(TAG, "Creating generator.");
    mcpwm_gen_handle_t gen = nullptr;
    mcpwm_generator_config_t gen_config =
    {
        .gen_gpio_num = GEN_GPIO,
    };
    mcpwm_new_generator(oper, &gen_config, &gen);

    ESP_LOGI(TAG, "Setting initial comparator value.");
    mcpwm_comparator_set_compare_value(cmpr, 0);

    ESP_LOGI(TAG, "Setting generator's actions on timer and compare events.");
    mcpwm_generator_set_action_on_timer_event(gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW));

    ESP_LOGI(TAG, "Enabling and starting timer.");
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    mcpwm_comparator_set_compare_value(cmpr, TIMER_PERIOD * 0.30);

    ESP_LOGI(TAG, "Completed MCPWM initialization.");
}