#ifndef AZURE_IOT_FREERTOS_H
#define AZURE_IOT_FREERTOS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void azure_init(void);
    extern void get_data(uint64_t *timestamp, int8_t *direction, double *duty_cycle, double *velocity, double *position, double *current);
    extern void set_desired_mode(int32_t mode);
    extern void set_desired_direction(int32_t direction);
    extern void set_desired_duty_cycle(double duty_cycle);
    extern void set_desired_velocity(double velocity);

    esp_err_t example_connect();
    uint64_t ullGetUnixTime(void);

#ifdef __cplusplus
}
#endif

#endif /* AZURE_IOT_FREERTOS_H */
