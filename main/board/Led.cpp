#include "Led.hpp"

LED::LED(gpio_num_t pin):
pin(pin)
{
    gpio_config_t pin_config; //pin configuration struct

   //set pin mask to configure selected pins, we could set this to configure multiple pins at once if desired.
   //for ex. (1U << GPIO_NUM_1) | (1U << GPIO_NUM2) would configure pins 1 & 2.
   pin_config.pin_bit_mask = (1U << pin);

   //output because are driving an LED
   pin_config.mode = GPIO_MODE_OUTPUT;

   //disable internal pullups and pulldowns
   pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
   pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;

   //disable external interrupts
   pin_config.intr_type = GPIO_INTR_DISABLE;
    
   //initialize the gpio pin by passing the gpio_config function a pointer to our configuration struct
   gpio_config(&pin_config);

   //ensure LED is off
   gpio_set_level(pin, 0);

}

void LED::toggle()
{
    static bool level = false;
    
    level = !level; //toggle
    gpio_set_level(pin, level);
}