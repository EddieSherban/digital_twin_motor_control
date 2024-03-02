#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

// Includes
#include <string.h>

#include "configuration.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"

class Communication
{
private:
  // TX task
  TaskHandle_t tx_task_hdl;
  static void tx_trampoline(void *arg);
  void tx();

  // RX task
  TaskHandle_t rx_task_hdl;
  static void rx_trampoline(void *arg);
  void rx();

public:
  Communication();

  void init();
  void send_data(char *data);
};

#endif // COMMUNICATION_H_
