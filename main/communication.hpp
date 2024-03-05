#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

// Includes
#include <string>
#include <cstring>

#include "configuration.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"

class Communication
{
private:
  // UART properties
  static constexpr uint32_t UART_BAUD_RATE = 921600;
  static constexpr uint32_t BUFFER_SIZE = 1024;

  // RX task
  TaskHandle_t rx_task_hdl;
  static void rx_trampoline(void *arg);
  void rx();

  uint64_t rx_num;

public:
  Communication();

  void init();
  void send_data(char *tx_data);

  uint64_t get_rx_num(); // TEMP
};

#endif // COMMUNICATION_H_
