#ifndef ROS_H
#define ROS_H

#include <stdint.h>

long map(long x, long in_min, long in_max, long out_min, long out_max);
double mapd(double x, double in_min, double in_max, double out_min, double out_max);
uint16_t get_value(uint16_t channel_data);

double get_ROSvalue(uint16_t channel_data);


#endif