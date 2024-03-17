// Includes
#include "current_sensor.hpp"

static constexpr char *TAG = "Current Sensor";

static CurrentSensor *curr_sen_obj;

CurrentSensor::CurrentSensor()
{
  curr_sen_obj = this;

  zero_voltage = 0;
  voltage = 0;
  current = 0;

  oneshot_hdl = nullptr;
  continuous_hdl = nullptr;
  cali_hdl = nullptr;

  adc_task_hdl = NULL;
}

void CurrentSensor::init()
{
  ESP_LOGI(TAG, "Setting up pull-down resistor.");
  gpio_config_t adc_gpio_config = {
      .pin_bit_mask = (1ULL << GPIO_ADC),
      .mode = GPIO_MODE_INPUT,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  gpio_config(&adc_gpio_config);

  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .chan = ADC_CHANNEL_3,
      .atten = ADC_ATTEN_DB_6,
      .bitwidth = ADC_BITWIDTH_12,
  };
  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_hdl));

  adc_continuous_handle_cfg_t continuous_config = {
      .max_store_buf_size = BUFFER_SIZE,
      .conv_frame_size = FRAME_SIZE,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&continuous_config, &continuous_hdl));

  adc_digi_pattern_config_t pattern_config = {
      .atten = ADC_ATTEN_DB_6,
      .channel = ADC_CHANNEL_3,
      .unit = ADC_UNIT_1,
      .bit_width = ADC_BITWIDTH_12,
  };

  adc_continuous_config_t digi_cfg = {
      .pattern_num = 1,
      .adc_pattern = &pattern_config,
      .sample_freq_hz = SAMPLE_FREQ,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  ESP_ERROR_CHECK(adc_continuous_config(continuous_hdl, &digi_cfg));
  ESP_ERROR_CHECK(adc_continuous_start(continuous_hdl));

  xTaskCreatePinnedToCore(adc_task, "ADC Task", adc_config.stack_size, nullptr, adc_config.priority, &adc_task_hdl, adc_config.core);
}

void CurrentSensor::adc_task(void *arg)
{
  static MovingAverage voltage_average(VOLTAGE_WINDOW_SIZE);
  static MovingAverage current_average(CURRENT_WINDOW_SIZE);

  while (1)
  {
    curr_sen_obj->voltage = voltage_average.next(curr_sen_obj->read_voltage());
    curr_sen_obj->current = current_average.next((double)(curr_sen_obj->voltage - curr_sen_obj->zero_voltage) / MV_TO_MA);
    // ESP_LOGI(TAG, "Measured: %d, Zero: %d", curr_sen_obj->voltage, curr_sen_obj->zero_voltage);
    vTaskDelay(adc_config.delay / portTICK_PERIOD_MS);
  }
}

void CurrentSensor::zero()
{
  static int temp_zero_voltage = 0;
  static MovingAverage zero_average(ZEROING_SAMPLE_SIZE);

  vTaskSuspend(adc_task_hdl);
  for (int i = 0; i <= ZEROING_SAMPLE_SIZE; i++)
  {
    temp_zero_voltage = zero_average.next(read_voltage());
    vTaskDelay(adc_config.delay / portTICK_PERIOD_MS);
  }
  zero_voltage = temp_zero_voltage;
  vTaskResume(adc_task_hdl);
}

int CurrentSensor::read_voltage()
{
  static uint8_t result[FRAME_SIZE] = {0};
  static uint32_t length = 0;
  static adc_digi_output_data_t *digi_output;
  static int adc_raw = 0;
  static int voltage = 0;

  adc_continuous_read(continuous_hdl, result, FRAME_SIZE, &length, 0);
  digi_output = (adc_digi_output_data_t *)&result[0];
  adc_raw = digi_output->type2.data;
  ESP_ERROR_CHECK(adc_cali_raw_to_voltage(curr_sen_obj->cali_hdl, adc_raw, &voltage));
  return voltage;
}

double CurrentSensor::get_current()
{
  return current;
}