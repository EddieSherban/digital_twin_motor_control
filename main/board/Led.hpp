#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

class LED 
{
    public:

    LED(gpio_num_t pin); 
    void toggle(); 
    gpio_num_t pin; 
};