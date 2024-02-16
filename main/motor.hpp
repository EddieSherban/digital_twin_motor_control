// INCLUDES
#include <stdio.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

class motor
{
private:
    mcpwm_cmpr_handle_t cmpr;
    pcnt_unit_handle_t unit;
    TaskHandle_t monitor_task_hdl;

public:
    static constexpr int8_t CLOCKWISE = 1;
    static constexpr int8_t COUNTER_CLOCKWISE = -1;

    motor();

    void init();
    void stop();

    void set_speed(float duty_cycle);
    void set_pos(float position);
    void set_direction(int8_t direction);
    float get_speed();
    float get_position();
    int8_t get_direction();

    static void monitor_task(void *arg);
    void monitor();
    void enable_monitor();
    void disable_monitor();

};