#ifndef USART_H
#define USART_H
#include <avr/io.h>
#define F_CPU 16000000UL

#define USART_BAUD 19200
#define BAUD_PRESCALE (((F_CPU / (USART_BAUD * 16UL))) - 1)

void USART_init();
void USART_putc(char c);
void USART_puts(const char *str);

#endif