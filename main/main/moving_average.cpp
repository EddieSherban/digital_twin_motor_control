// Includes
#include "moving_average.hpp"

MovingAverage::MovingAverage(uint64_t window_size)
{
  this->window_size = window_size;
  sum = 0;
  index = 0;
}

float MovingAverage::next(float value)
{
  if (window.size() < window_size)
  {
    window.push_back(value);
    sum += value;
    return sum / (float)window.size();
  }
  else
  {
    sum -= window[index];
    sum += value;
    window[index] = value;
    index = (index + 1) % window_size;
    return sum / (float)window.size();
  }
}