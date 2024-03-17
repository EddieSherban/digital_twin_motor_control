#ifndef MOVING_AVERAGE_H_
#define MOVING_AVERAGE_H_

// Includes
#include <stdio.h>
#include <vector>
#include "esp_log.h"

class MovingAverage
{
private:
  std::vector<double> window;
  uint64_t window_size;
  double sum;
  uint64_t index;

public:
  MovingAverage(uint64_t window_size);
  double next(double value);
};

#endif // MOVING_AVERAGE_H_
