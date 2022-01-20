#ifndef CAN_H
#define CAN_H

#include <stdint.h>

#include "can_defs.h"

/* Structures */

typedef struct
{
  // SID of the target;
  uint32_t SID;
  // Frame header
  struct
  {
    unsigned int rtr : 1;
    unsigned int len : 4;
  } header;
  // Frame data
  uint8_t data[8];
} CAN_frame_t;

typedef enum
{
  CAN_ERROR_NONE = 0,
  CAN_ERROR_ALLTXBUSY = 1,
  CAN_ERROR_UNKNOWN_FLOW = 2,
  CAN_ERROR_ABORT = 3,
  CAN_ERROR_LOSTARBITATION = 4,
  CAN_ERROR_TXERR = 5
} CAN_error;

uint8_t CAN_init(uint8_t freq_meg, long baudrate);
void CAN_set_mode(uint8_t mode);
void CAN_set_register(uint8_t reg, uint8_t value);
void CAN_enable_clkout();
void CAN_disable_clkout();
void CAN_read_registers(uint8_t start, uint8_t target[], uint8_t len);
uint8_t CAN_read_register(uint8_t reg);
void CAN_load_message(uint8_t buffer, CAN_frame_t* frame);
void CAN_request_to_send(uint8_t buffer);
void CAN_abort_send(uint8_t buffer);
CAN_error CAN_send(CAN_frame_t* frame);
void CAN_read(uint8_t buffer, CAN_frame_t* frame);

#endif  // CAN_H