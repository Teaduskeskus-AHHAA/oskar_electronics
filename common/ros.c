#include "include/ros.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t get_value(uint16_t channel_data)
{
  return map(channel_data, 272, 1712, 0, 100);
}

double get_ROSvalue(uint16_t channel_data)
{
  return mapd(channel_data, 272, 1712, -0.5, 0.5);
}
