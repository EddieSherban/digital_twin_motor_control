#pragma once

#include "motor_control.h"

#define PWM_RES     1000000     // 1 MHz, 1 us
#define PWM_FREQ    20000       // 20 kHz

void motor_control::init()
{

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_config = 
    oper_config.group_id = 0;

    ESP_ERROR_CHECK(mcpwm_new_operator(oper_config, oper));

    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_comparator_config_t cmpr_config;
    cmpr_config.flags.update_cmp_on_tez = true;

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &cmpr));

    mcpwm_comparator_set_compare_value(cmpr,)
}