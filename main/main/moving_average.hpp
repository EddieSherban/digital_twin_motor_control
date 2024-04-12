#ifndef MOVING_AVERAGE_H_
#define MOVING_AVERAGE_H_

// Includes
#include <stdio.h>
#include <vector>
#include "esp_log.h"

class MovingAverage
{
private:
  std::vector<float> window;
  uint64_t window_size;
  float sum;
  uint64_t index;

public:
  MovingAverage(uint64_t window_size);
  float next(float value);
};

#endif // MOVING_AVERAGE_H_
