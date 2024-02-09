#pragma once

#include "L298N.hpp"

#define TIMER_RES   10000000    // 10 MHz 
#define TIMER_FREQ  30000       // 30 kHz
#define CMPR_TICKS  20000       

void L298N::init()
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = 
    {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMER_RES,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = TIMER_RES / TIMER_FREQ,
    };

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_config;
    oper_config.group_id = 0;

    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));

    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t cmpr_config;
    cmpr_config.flags.update_cmp_on_tez = true;

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &cmpr));

    mcpwm_comparator_set_compare_value(cmpr, CMPR_TICKS);
}