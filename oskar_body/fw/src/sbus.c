#include "include/sbus.h"
#include <avr/interrupt.h>

ISR(USART2_RX_vect)
{

  uint8_t data = UDR2;
  rx_buffer[rx_pointer++] = data;
  if (rx_pointer == SBUS_PACKAGE_SIZE)
  {
    rx_pointer = 0;
    SBUS_read();
  }
}

void SBUS_init()
{
  rx_pointer = 0;
  UBRR2 = 9;
  UBRR2 = 9;
  UCSR2B |= (1 << RXEN2) | (1 << RXCIE2);
  UCSR2C |= (1 << USBS0) | (1 << UPM21);
}

void SBUS_read()
{
  if (rx_buffer[0] == 0x0F && rx_buffer[24] == 0x00)
  {
    SBUS_decode(rx_buffer);

    sbus_safety = 0;
    OPER_MODE = 0;
  }
}

void SBUS_decode(uint8_t *data)
{
  channels[0] = ((data[1] | data[2] << 8) & 0x07FF);
  channels[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF);
  channels[2] = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
  channels[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF);
  channels[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF);
  channels[5] = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
  channels[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
  channels[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF); // & the other 8 + 2 channels if you need them
  channels[8] = ((data[12] | data[13] << 8) & 0x07FF);
  channels[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF);
  channels[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF);
  channels[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF);
  channels[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF);
  channels[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF);
  channels[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF);
  channels[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF);
}
