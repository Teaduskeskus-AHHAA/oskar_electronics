#ifndef SPI_H
#define SPI_H
#include <avr/io.h>

void SPI_init();
void SPI_write(uint8_t data);
void SPI_unset_cs();
void SPI_set_cs();

#endif