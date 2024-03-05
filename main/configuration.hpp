#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

// Includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

typedef struct
{
  uint16_t delay;                              // Task delay in ms
  const uint32_t stack_size = 1024 * 4;        // Stack size in bytes
  UBaseType_t priority = tskIDLE_PRIORITY + 1; // Task priority (0 - 25)
  const BaseType_t core = 0;                   // Core which task runs on (0 or 1)
} task_config;

// Pin configurations
// Motor driver pins
static constexpr gpio_num_t GPIO_ENA = GPIO_NUM_1;  // MCPWM output (Connected to ENA)
static constexpr gpio_num_t GPIO_IN1 = GPIO_NUM_2;  // GPIO output (Connected to IN1)
static constexpr gpio_num_t GPIO_IN2 = GPIO_NUM_42; // GPIO output (Connected to IN2)
static constexpr gpio_num_t GPIO_C1 = GPIO_NUM_41;  // GPIO output (Connected to Encoder A) GREEN
static constexpr gpio_num_t GPIO_C2 = GPIO_NUM_40;  // GPIO output (Connected to Encoder B) YELLOW

// UART pins
static constexpr gpio_num_t GPIO_TX = GPIO_NUM_43;
static constexpr gpio_num_t GPIO_RX = GPIO_NUM_44;

// Communication protocols and commands
static constexpr uint16_t FRAME_START = 0x1;
static constexpr uint16_t FRAME_END = 0x3;

static constexpr uint16_t COMMAND_DIRECTION = 0x11;
static constexpr uint16_t COMMAND_DUTY_CYCLE = 0x12;
static constexpr uint16_t COMMAND_POSITION = 0x13;

static constexpr uint16_t COMMAND_VELOCITY = 0x21;

// FreeRTOS task configurations
constexpr task_config update_config = {
    .delay = 1,
    .stack_size = 1024 * 4,
    .priority = configMAX_PRIORITIES - 1,
    .core = 1,
};

constexpr task_config display_config = {
    .delay = 5,
    .stack_size = 1024 * 4,
    .priority = configMAX_PRIORITIES - 2,
    .core = 0,
};

constexpr task_config tx_config = {
    .delay = 2,
    .stack_size = 1024 * 4,
    .priority = configMAX_PRIORITIES - 1,
    .core = 0,
};

constexpr task_config rx_config = {
    .delay = 10,
    .stack_size = 1024 * 4,
    .priority = configMAX_PRIORITIES - 3,
    .core = 0,
};

#endif // COMMUNICATION_H_
