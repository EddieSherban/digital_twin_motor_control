#ifndef AZURE_IOT_FREERTOS_ESP32_MAIN_H
#define AZURE_IOT_FREERTOS_ESP32_MAIN_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void azure_init(void);
    extern void get_data(uint64_t *timestamp, int8_t *direction, double *duty_cycle, double *velocity, double *position, double *current);
    esp_err_t example_connect();

#ifdef __cplusplus
}
#endif

#endif /* AZURE_IOT_FREERTOS_ESP32_MAIN_H */
