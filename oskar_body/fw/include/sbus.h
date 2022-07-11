#ifndef SBUS_H
#define SBUS_H

#include <avr/io.h>

#define SBUS_PACKAGE_SIZE 25

uint16_t sbus_safety;
uint8_t OPER_MODE;

uint8_t rx_buffer[SBUS_PACKAGE_SIZE];
uint8_t rx_pointer;
uint16_t channels[16];

uint16_t get_value(uint16_t channel_data);
double get_ROSvalue(uint16_t channel_data);

void SBUS_init();
void SBUS_read();
void SBUS_decode(uint8_t *data);

#endif