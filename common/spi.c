#include "include/spi.h"

void SPI_init()
{
	DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2);
	SPCR = (1 << SPE) | (1 << MSTR);
	SPSR = (1 << SPI2X);
	PORTB |= (1 << PB0);
}

void SPI_write(uint8_t data)
{
	SPDR = data;
	while (!(SPSR & (1 << SPIF)))
	{
	};
}

void SPI_unset_cs()
{
	PORTB &= ~(1 << PB0);
}

void SPI_set_cs()
{
	PORTB |= (1 << PB0);
}