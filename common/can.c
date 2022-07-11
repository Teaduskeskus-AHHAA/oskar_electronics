#include "include/can.h"
#include "include/spi.h"

void CAN_set_mode(uint8_t mode)
{
  SPI_unset_cs();
  SPI_write(BIT_MODIFY);
  SPI_write(CANCTRL);
  SPI_write(REQOP_MASK);
  SPI_write((mode << REQOP_OFFSET));
  SPI_set_cs();
}

uint8_t CAN_init(uint8_t freq_meg, long baudrate)
{
  CAN_set_mode(REQOP_CONFIG);
  while (!(CAN_read_register(CANSTAT) & 0x80))
  {
  };

  /* https://www.kvaser.com/support/calculators/bit-timing-calculator/ (16MHz, 1Mbps, MCP2510)*/
  uint8_t cnf1 = 0x00;
  uint8_t cnf2 = 0x91;
  uint8_t cnf3 = 0x01;

  CAN_set_register(CNF1, cnf1);
  CAN_set_register(CNF2, cnf2);
  CAN_set_register(CNF3, cnf3);

  CAN_set_register(RXB0CTRL, 0x64); // FIlters off, rollover enable
  CAN_set_register(RXB1CTRL, 0x60); // FIlters off

  CAN_set_register(BFPCTRL, 0b00001111);

  CAN_set_register(TXB0CTRL, 0);
  CAN_set_register(TXB1CTRL, 0);
  CAN_set_register(TXB2CTRL, 0);
}

uint8_t CAN_read_register(uint8_t reg)
{
  SPI_unset_cs();
  SPI_write(READ);
  SPI_write(reg);
  SPI_write(0x00);
  SPI_set_cs();
  return SPDR;
}

void CAN_set_register(uint8_t reg, uint8_t value)
{
  SPI_unset_cs();
  SPI_write(WRITE);
  SPI_write(reg);
  SPI_write(value);
  SPI_set_cs();
}

uint8_t CAN_verify_register(uint8_t reg, uint8_t expected)
{
  uint8_t reading = CAN_read_register(reg);
  return (reading == expected);
}

uint8_t CAN_init_regcheck()
{
  /* https://www.kvaser.com/support/calculators/bit-timing-calculator/ (16MHz, 1Mbps, MCP2510)*/
  uint8_t c1 = 0x00;
  uint8_t c2 = 0x91;
  uint8_t c3 = 0x01;
  return CAN_verify_register(CNF1, c1) && CAN_verify_register(CNF2, c2) && CAN_verify_register(CNF3, c3);
}

void CAN_load_message(uint8_t buffer, CAN_frame_t *frame)
{
  SPI_unset_cs();
  SPI_write(LOAD_TX_BUFFER | buffer);
  SPI_write((frame->SID >> 3));
  SPI_write((frame->SID << 5));
  SPI_write(0);
  SPI_write(0);

  uint8_t length = frame->header.len & 0x0F;
  if (frame->header.rtr)
  {
    SPI_write((1 << RTR) | length);
  }
  else
  {
    SPI_write(length);
    uint8_t i;
    for (i = 0; i < length; i++)
    {
      SPI_write(frame->data[i]);
    }
  }
  SPI_set_cs();
}
void CAN_request_to_send(uint8_t buffer)
{
  uint8_t address = (buffer == 0) ? 1 : buffer;
  SPI_unset_cs();
  SPI_write(RTS | address);
  SPI_set_cs();
}

void CAN_read(uint8_t buffer, CAN_frame_t *frame)
{
  uint8_t metadata[5];
  if (buffer == 0)
  {
    CAN_read_registers(RXB0SIDH, metadata, 5);
    CAN_read_registers(RXB0D0, frame->data, 8);
    SPI_unset_cs();
    SPI_write(BIT_MODIFY);
    SPI_write(CANINTF);
    SPI_write(RX0IF);
    SPI_write(0);
    SPI_set_cs();
  }
  else if (buffer == 1)
  {
    CAN_read_registers(RXB1SIDH, metadata, 5);
    CAN_read_registers(RXB1D0, frame->data, 8);
    SPI_unset_cs();
    SPI_write(BIT_MODIFY);
    SPI_write(CANINTF);
    SPI_write(RX1IF);
    SPI_write(0);
    SPI_set_cs();
  }

  frame->SID = (metadata[0] << 3) | (metadata[1] >> 5);
  frame->header.len = metadata[4];
}

void CAN_read_registers(uint8_t start, uint8_t target[], uint8_t len)
{
  SPI_unset_cs();
  SPI_write(READ);
  SPI_write(start);
  for (int i = 0; i < len; i++)
  {
    SPI_write(0x00);
    target[i] = SPDR;
  }
  SPI_set_cs();
}

CAN_error CAN_send(CAN_frame_t *frame)
{
  uint8_t abort, lost, txerr;
  uint8_t txbuffer_full[3];
  txbuffer_full[0] = CAN_check_bit(TXB0CTRL, 0x08);
  txbuffer_full[1] = CAN_check_bit(TXB1CTRL, 0x08);
  txbuffer_full[2] = CAN_check_bit(TXB2CTRL, 0x08);

  int selected_buffer = -1;

  for (int i = 0; i < 3; i++)
  {
    if (!txbuffer_full[i])
    {
      selected_buffer = i;
      break;
    }
  }
  if (selected_buffer == -1)
  {

    PORTD |= (1 << PD2) | (1 << PD7);

    return CAN_ERROR_ALLTXBUSY;
  }

  switch (selected_buffer)
  {
  case 0:
    CAN_load_message(TXB0ADDR, frame);
    CAN_request_to_send(TXB0ADDR);
    abort = CAN_check_bit(TXB0ADDR, ABTF);
    lost = CAN_check_bit(TXB0ADDR, MLOA);
    txerr = CAN_check_bit(TXB0ADDR, TXERR);
    break;
  case 1:
    CAN_load_message(TXB1ADDR, frame);
    CAN_request_to_send(TXB1ADDR);
    abort = CAN_check_bit(TXB1ADDR, ABTF);
    lost = CAN_check_bit(TXB1ADDR, MLOA);
    txerr = CAN_check_bit(TXB1ADDR, TXERR);
    break;
  case 2:
    CAN_load_message(TXB2ADDR, frame);
    CAN_request_to_send(TXB2ADDR);
    abort = CAN_check_bit(TXB2ADDR, ABTF);
    lost = CAN_check_bit(TXB2ADDR, MLOA);
    txerr = CAN_check_bit(TXB2ADDR, TXERR);
    break;
  default:
    PORTD |= (1 << PD2);
    return CAN_ERROR_UNKNOWN_FLOW;
  }

  if (abort)
  {
    PORTD |= (1 << PD5);
    return CAN_ERROR_ABORT;
  }
  if (lost)
  {
    PORTD |= (1 << PD6);
    return CAN_ERROR_LOSTARBITATION;
  }
  if (txerr)
  {
    PORTD |= (1 << PD7);
    return CAN_ERROR_TXERR;
  }

  return CAN_ERROR_NONE;
}

uint8_t CAN_check_bit(uint8_t reg, uint8_t bitmask)
{
  return CAN_read_register(reg) & bitmask;
}

uint8_t CAN_get_TEC()
{
  return CAN_read_register(0x1C);
}