// Includes
#include "communication.hpp"

static constexpr char *TAG = "communication";

// Pin configurations
static constexpr gpio_num_t GPIO_TX = GPIO_NUM_43;
static constexpr gpio_num_t GPIO_RX = GPIO_NUM_44;

// UART properties
static constexpr uint32_t UART_BAUD_RATE = 115200;
static constexpr uint32_t RX_BUFFER_SIZE = 1024;
static constexpr uint32_t TX_BUFFER_SIZE = 1024;

// TX task properties
static constexpr uint8_t TX_RATE = 1000; // Monitoring sample rate in ms
static constexpr int32_t TX_STACK_SIZE = 1024 * 1;
static constexpr UBaseType_t TX_TASK_PRIO = configMAX_PRIORITIES - 2; // High priority
static constexpr int8_t TX_TASK_CORE = 0;                             // Run task on Core 1

// RX task properties
static constexpr uint8_t RX_RATE = 1000; // Monitoring sample rate in ms
static constexpr int32_t RX_STACK_SIZE = 1024 * 1;
static constexpr UBaseType_t RX_TASK_PRIO = configMAX_PRIORITIES - 2; // High priority
static constexpr int8_t RX_TASK_CORE = 0;                             // Run task on Core 1

static Communication *comm_obj = this;

Communication::Communication()
{
  tx_task_hdl = NULL;
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
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUFFER_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_TX, GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_LOGI(TAG, "Setting up transmitter task.");
  // xTaskCreatePinnedToCore(tx_trampoline, "Transmitter", TX_STACK_SIZE, nullptr, TX_TASK_PRIO, &tx_task_hdl, TX_TASK_CORE);

  ESP_LOGI(TAG, "Setting up receiver task.");
  // xTaskCreatePinnedToCore(rx_trampoline, "Receiver", RX_STACK_SIZE, nullptr, RX_TASK_PRIO, &rx_task_hdl, RX_TASK_CORE);
}

void Communication::send_data(const char *data)
{
  uint8_t length = strlen(data);
  uart_write_bytes(UART_NUM_1, data, length);
}

void Communication::tx_trampoline(void *arg)
{
  while (1)
  {
    comm_obj->tx();
    vTaskDelay(TX_RATE / portTICK_PERIOD_MS);
  }
}

void Communication::tx()
{
  // send_data("test\n");
}

void Communication::rx_trampoline(void *arg)
{
  while (1)
  {
    comm_obj->rx();
    vTaskDelay(TX_RATE / portTICK_PERIOD_MS);
  }
}

void Communication::rx()
{
}