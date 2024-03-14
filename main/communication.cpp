// Includes
#include "communication.hpp"

static constexpr char *TAG = "Communication";

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

void Communication::send_data(char *tx_data)
{
  // sprintf(data, "%d,%s,%d", FRAME_START, data, FRAME_END); // Encapsulates data frame
  uart_write_bytes(UART_NUM_1, tx_data, strlen(tx_data));
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
  static uint8_t rx_data[64];
  static uint8_t rx_length;
  static std::string rx_str;

  while (1)
  {
    rx_length = uart_read_bytes(UART_NUM_1, rx_data, BUFFER_SIZE, (rx_config.delay / 2) / portTICK_PERIOD_MS);
    if (rx_length > 0)
    {
      // Convert to int
      for (uint8_t i = 0; i < rx_length; i++)
        rx_str += (char)rx_data[i];
      rx_num = std::stoi(rx_str);

      ESP_LOGI(TAG, "Read %d bytes: '%s' %lld", rx_length, rx_data, rx_num);

      // Clear temp data and buffer
      rx_str = "";
      for (uint8_t i = 0; i < rx_length; i++)
        rx_data[i] = 0;
    }
  }
}

// TEMP
uint64_t Communication::get_rx_num()
{
  return rx_num;
}