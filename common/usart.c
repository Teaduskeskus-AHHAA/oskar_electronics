#include "include/usart.h"
#include <avr/io.h>

void USART_init()
{
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);
  UBRR0L = BAUD_PRESCALE;
  UBRR0H = (BAUD_PRESCALE >> 8);
}

void USART_putc(char c)
{
  while (!(UCSR0A & (1 << UDRE0)))
  {
  }
  UDR0 = c;
}

void USART_puts(const char *str)
{
  while (*str)
  {
    USART_putc(*str++);
  }
}
