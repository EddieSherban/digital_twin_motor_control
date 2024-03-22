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
  
public:
  Communication();

  void init();
  void send_data(char *tx_data);

};

#endif // COMMUNICATION_H_
