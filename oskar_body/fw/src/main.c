#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "include/sbus.h"

#include "../../../common/include/spi.h"
#include "../../../common/include/twimaster.h"
#include "../../../common/include/motor.h"

#include "../../../common/include/can.h"
#define F_CPU 16000000U
#include <util/delay.h>
#include "../../../common/include/usart.h"
#include "../../../common/include/ros.h"

unsigned char address = 0x20, read = 1, write = 0;

gyems_motor motor_l_wrist;
gyems_motor motor_r_wrist;

CAN_frame_t recieved_frame;

ISR(PCINT0_vect)
{
  cli();
  if (!(PINB & (1 << PINB4)))
  {
    CAN_read(0, &recieved_frame);
    motors_impl_parse_can_msg();
  }
  if (!(PINB & (1 << PINB5)))
  {
    CAN_read(1, &recieved_frame);
    motors_impl_parse_can_msg();
  }
  sei();
}

void motors_impl_parse_can_msg()
{
  gyems_motor_parse_can(&motor_l_wrist, &recieved_frame);
  gyems_motor_parse_can(&motor_r_wrist, &recieved_frame);
}

void motors_impl_update()
{
  gyems_motor_request_status(&motor_l_wrist);
  // gyems_motor_request_status(&motor_r_wrist);
}

void motors_impl_safeties()
{
  gyems_motor_safety(&motor_l_wrist);
  // gyems_motor_safety(&motor_r_wrist);
}

void motors_impl_config()
{
  motor_l_wrist.id = MOTOR_BASEID + 2; // Base ID of gyems motors + this motor ID
  motor_l_wrist.endpoint_speed = -100;
  motor_l_wrist.multiturn_angle_range = 180;
  motor_l_wrist.operating_mode = 1;
  motor_l_wrist.has_zeropoint = 0;
  motor_l_wrist.endpoints_found = 0;
  motor_l_wrist.gear_ratio_multiplier = 1;
  motor_l_wrist.max_power_endpoints = 1;
  motor_l_wrist.max_power_operating = 3;

  motor_r_wrist.id = MOTOR_BASEID + 4;
  motor_r_wrist.endpoint_speed = -100;
  motor_r_wrist.multiturn_angle_range = 180;
  motor_r_wrist.gear_ratio_multiplier = 1;
  motor_r_wrist.has_zeropoint = 0;
  motor_r_wrist.operating_mode = 1;
  motor_r_wrist.endpoints_found = 0;
  motor_r_wrist.max_power_endpoints = 1;
  motor_r_wrist.max_power_operating = 3;
}

void blink_can_fail()
{
  while (1)
  {
    PORTD |= (1 << PD3);
    _delay_ms(1000);
    PORTD &= ~(1 << PD3);
    _delay_ms(1000);
  }
}

int main()
{

  OPER_MODE = 1;
  DDRH |= (1 << DDH6);
  DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);

  SPI_init();

  PORTH |= (1 << PH6);
  _delay_ms(1000);
  PORTH &= ~(1 << PH6);
  _delay_ms(1000);
  PORTH |= (1 << PH6);
  _delay_ms(1000);

  CAN_init(16, 1000000);

  if (CAN_init_regcheck())
  {
    CAN_set_mode(REQOP_NORMAL);
    while (!(CAN_read_register(CANSTAT) == 0x00))
    {
    };
  }
  else
  {
    blink_can_fail();
  }

  sbus_safety = 0;
  SBUS_init();

  USART_init();

  /*  i2c_init();

    i2c_start_wait(0xA2 + I2C_WRITE); // set device address and write mode

    i2c_write(0x11);
    i2c_write(0x00);
    i2c_write(0x00);
    i2c_write(0x00);
    i2c_write(0x00);
    i2c_write(0x00);
    i2c_write(0x00);
    motors_impl_safeties();

    i2c_write(0x11);

    i2c_stop();*/

  _delay_ms(1000);

  PCMSK0 |= (1 << PCINT4) | (1 << PCINT5);
  PCICR |= (1 << PCIE0);

  sei();

  motors_impl_config();

  gyems_motor_reset(&motor_l_wrist);
  _delay_ms(100);

  gyems_motor_reset(&motor_r_wrist);

  _delay_ms(100);

  gyems_motor_find_endpoints(&motor_l_wrist);
  // gyems_motor_find_endpoints(&motor_r_wrist);
  while (1)
  {

    /*
        if (OPER_MODE == 0)
        {
          sbus_safety+1
          if (sbus_safety > 1000)
          {
            OPER_MODE = 1;
          }
        }

        if (OPER_MODE == 0)
        {
          int8_t diff = 992-get_value(channels[11]);
          char buf[8];
          itoa(diff, buf,10);
          USART_puts(buf);
          USART_PUTS("\r\n");
          sbus_safety = 0;
        }*/

    motors_impl_update();
    motors_impl_safeties();

    char buf[8];
    itoa(CAN_get_TEC(), buf, 10);
    USART_puts("TEC: ");
    USART_puts(buf);
    USART_puts("\r\n");

    //  gyems_motor_set_multiturn_angle(&motor_l_wrist, 10, 90);
    //   gyems_motor_set_multiturn_angle(&motor_r_wrist, 10, 90);

    sbus_safety = 0;
  }
  return 0;
}