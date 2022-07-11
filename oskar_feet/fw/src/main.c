#include <avr/io.h>
#include <avr/interrupt.h>

#include "../../../common/include/twislave.h"

#define TWI_ADDRESS 0xA2

volatile uint8_t twi_readbuf[8];
volatile uint8_t twi_readindex = 0;
volatile uint8_t twi_ready = 0;
volatile uint8_t twi_proc = 0;
volatile uint8_t twi_pckt[8];

ISR(TWI_vect)
{

  switch (TW_STATUS)
  {
  case TW_SR_SLA_ACK:

    TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
    break;
  case TW_SR_DATA_ACK:
    twi_readbuf[twi_readindex] = TWDR;
    twi_readindex++;
    if (twi_readindex >= 8)
    {
      twi_readindex = 0;
      if (!twi_proc)
      {
        memcpy(twi_pckt, twi_readbuf, sizeof(twi_readbuf));
      }
      twi_ready = 1;
    }
    TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN) | (1 << TWINT);

    break;
  case TW_SR_STOP:
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
    break;
  default:
    break;
  }
}

int main(void)
{

  DDRD = (1 << DDD7);
  DDRJ = (1 << DDJ7);

  PORTJ &= ~(1 << PJ7);

  uint8_t TWIS_ResponseType;
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t byte[8];

  TWAR = TWI_ADDRESS;
  TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
  sei();

  while (1)
  {
    if (twi_ready)
    {
      twi_proc = 1;
      if (twi_pckt[0] == 0x11 && twi_pckt[7] == 0x11 && twi_pckt[1] == 0x00 && twi_pckt[2] == 0x00 && twi_pckt[3] == 0x00 && twi_pckt[4] == 0x00 && twi_pckt[5] == 0x00 && twi_pckt[6] == 0x00)
      {
        PORTD ^= (1 << PD7);
        PORTJ ^= (1 << PJ7);

        twi_ready = 0;
        twi_proc = 0;
      }
    }
  }

  return 0;
}