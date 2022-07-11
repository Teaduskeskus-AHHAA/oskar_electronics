#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include "include/ws2812_config.h"

#include "include/light_ws2812.h"
#include <stdlib.h>
#define NS_PER_SEC (1000000000L) // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives
#define CYCLES_PER_SEC (F_CPU)
#define NS_PER_CYCLE (NS_PER_SEC / CYCLES_PER_SEC)
#define NS_TO_CYCLES(light_n) ((n) / NS_PER_CYCLE)
#define SBI(port, bit) asm volatile("sbi %0,%1" ::"I"(_SFR_IO_ADDR(port)), "I"(bit))
#define cbi(port, bit) port = ~(1 << bit)
uint16_t count = 0;

struct cRGB led[2];

uint16_t read_adc(uint8_t ch)
{
  ch &= 0b00000111;            // Constrain 0-7
  ADMUX = (ADMUX & 0xF8) | ch; // Tyhistame MUX-is praeguse valiku, ja OR-ime enda asja sisse.
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
  {
  }

  return (ADC);
}

void doColor(uint32_t col)
{
  cli();
  uint8_t i;
  for (i = 24; i;)
  {
    if (col & 0x800000)
    {
      PORTE |= (1 << PE0);
      _delay_us(0.7);
      i--;
      col <<= 1;
      PORTE &= ~(1 << PE0);
      _delay_us(0.6);
    }
    else
    {
      PORTE |= (1 << PE0);
      _delay_us(0.35);
      i--;
      cbi(PORTE, 0);
      col <<= 1;
      _delay_us(0.8);
    }
  }
  sei();
}

void blink()
{
  PORTE &= ~(1 << PE1);
  PORTB &= ~((1 << PB6) | (1 << PB7));
  _delay_ms(100);
  PORTE |= (1 << PE1);
  PORTB |= (1 << PB6) | (1 << PB7);
}

int main()
{

  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADPS1) | (1 << ADPS2) | (1 << ADEN); // Prescaler 64

  DDRE |= (1 << DDE0) | (1 << DDE1) | (1 << DDE2) | (1 << DDE3);

  DDRB |= (1 << DDB6) | (1 << DDB7);

  PORTB |= (1 << PB6) | (1 << PB7);

  PORTE |= (1 << PE1);

  led[0].r = 255;
  led[0].g = 255;
  led[0].b = 255; // Write red to array

  led[1].r = 255;
  led[1].g = 255;
  led[1].b = 255; // Write red to array

  blink();
  _delay_ms(1000);
  blink();

  while (1)
  {
    /* doColor(0xFF00FF);
     _delay_ms(1);
     ws2812_setleds(led, 2);*/

    uint16_t randbase = read_adc(13);
    if (randbase % 3)
    {
      count++;
    }
    if (count >= 50000)
    {
      blink();
      count = 0;
    }
  }
  return 0;
}
