#ifndef CURRENT_SENSOR_H_
#define CURRENT_SENSOR_H_

// Includes
#include <stdio.h>

#include "configuration.hpp"
#include "moving_average.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"

class CurrentSensor
{
private:
  // Class variables
  int zero_voltage;
  int voltage;
  double current;

  // ESP handles
  adc_oneshot_unit_handle_t oneshot_hdl;
  adc_continuous_handle_t continuous_hdl;
  adc_cali_handle_t cali_hdl;

  // Filtering properties
  static constexpr uint64_t VOLTAGE_WINDOW_SIZE = 100; // Size of window for moving average
  static constexpr uint64_t CURRENT_WINDOW_SIZE = 10;
  static constexpr uint64_t ZEROING_SAMPLE_SIZE = 1000;

  // ADC continuous properties
  static constexpr uint32_t SAMPLE_FREQ = 80000;
  static constexpr uint32_t BUFFER_SIZE = 12 * 100;
  static constexpr uint32_t FRAME_SIZE = 12 * 3;

  // Conversion constants
  static constexpr double MV_TO_MA = 800.0 / 1000.0;

  // ADC task
  TaskHandle_t adc_task_hdl;
  static void adc_task(void *arg);

public:
  CurrentSensor();

  void init();
  void zero();

  int read_voltage();
  double read_current();
};

#endif // CURRENT_SENSOR_H_
