// INCLUDES
#include <stdio.h>
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

class motor
{
private:
    mcpwm_cmpr_handle_t cmpr;
    
public:
    static constexpr int8_t CLOCKWISE = 1;
    static constexpr int8_t COUNTER_CLOCKWISE = -1;

    void init();
    void stop();
    void set_speed(float duty_cycle);
    void set_direction(int8_t direction);
    void get_speed();
    void get_direction();

};