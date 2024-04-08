// Includes
#include "communication.hpp"

static constexpr char *TAG = "Communication";

static Communication *comm_obj;

Communication::Communication()
{
  comm_obj = this;
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
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUFFER_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_TX, GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void Communication::send_data(const char *tx_data)
{
  uart_write_bytes(UART_NUM_1, tx_data, strlen(tx_data));
}