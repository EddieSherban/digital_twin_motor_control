// Includes
#include "communication.hpp"

static constexpr char *TAG = "communication";

static Communication *comm_obj;

Communication::Communication()
{
  comm_obj = this;

  rx_task_hdl = NULL;
}

void Communication::init()
{
  ESP_LOGI(TAG, "Setting up UART.");
  uart_config_t uart_config = {
      .baud_rate = UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUFFER_SIZE, BUFFER_SIZE, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_TX, GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_LOGI(TAG, "Setting up RX Data task.");
  xTaskCreatePinnedToCore(rx_trampoline, "RX Data Task", rx_config.stack_size, nullptr, rx_config.priority, &rx_task_hdl, rx_config.core);
}

void Communication::send_data(char *data)
{
  // sprintf(data, "%d,%s,%d", FRAME_START, data, FRAME_END); // Encapsulates data frame
  uart_write_bytes(UART_NUM_1, data, strlen(data));
}

void Communication::rx_trampoline(void *arg)
{
  while (1)
  {
    comm_obj->rx();
    vTaskDelay(rx_config.delay / portTICK_PERIOD_MS);
  }
}

void Communication::rx()
{
}