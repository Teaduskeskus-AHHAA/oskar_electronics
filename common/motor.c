#include <avr/io.h>
#include "include/motor.h"
#include "include/ros.h"
#define myabs(n) ((n) < 0 ? -(n) : (n))
#include <stdlib.h>
#include <util/delay.h>

void gyems_motor_request_status(gyems_motor *motor)
{
  // Needs check for recieve canbfrfer empty conditions
  CAN_frame_t frm;
  // Motor status 1
  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;

  frm.data[0] = READ_MOTOR_STATUS_1_ERROR_FLAGS;
  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  }
  frm.data[0] = READ_MOTOR_STATUS_2;
  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  }
}

void gyems_motor_get_multiturn_angle(gyems_motor *motor)
{
  CAN_frame_t frm;
  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;
  frm.data[0] = READ_MULTITURN_ANGLE;
  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  };
}

void gyems_motor_parse_can(gyems_motor *motor, CAN_frame_t *frame)
{
  if (frame->SID == motor->id)
  {
    switch (frame->data[0])
    {
    case READ_MOTOR_STATUS_1_ERROR_FLAGS:
      motor->temperature = frame->data[1];
      motor->voltage = frame->data[3] | (frame->data[4] << 8);
      motor->error_state = frame->data[7];
      break;
    case READ_MOTOR_STATUS_2:
      motor->temperature = frame->data[1];
      motor->torque_current = frame->data[2] | (frame->data[3] << 8);
      motor->speed = frame->data[4] | (frame->data[5] << 8);
      motor->encoder.current = frame->data[6] | (frame->data[7] << 8);
      break;
    case READ_MOTOR_STATUS_3:
      motor->temperature = frame->data[1];
      motor->torque_current = frame->data[2] | (frame->data[3] << 8);

      break;
    case READ_MULTITURN_ANGLE:
      if (motor->has_zeropoint == 0)
      {
        motor->multiturn_angle_0 =
            (int64_t)frame->data[1] | ((int64_t)frame->data[2] << 8) |
            ((int64_t)frame->data[3] << 16) | ((int64_t)frame->data[4] << 24) |
            ((int64_t)frame->data[5] << 32) | ((int64_t)frame->data[6] << 40) |
            ((int64_t)frame->data[7] << 48);
        motor->has_zeropoint = 1;
        break;
      }
      else
      {
        motor->multiturn_angle =
            (int64_t)frame->data[1] | ((int64_t)frame->data[2] << 8) |
            ((int64_t)frame->data[3] << 16) | ((int64_t)frame->data[4] << 24) |
            ((int64_t)frame->data[5] << 32) | ((int64_t)frame->data[6] << 40) |
            ((int64_t)frame->data[7] << 48);
        break;
      }
      break;
    default:
      break;
    }
  }
}

void gyems_motor_reset(gyems_motor *motor)
{
  CAN_frame_t frm;

  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;
  frm.data[0] = 0x80;

  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  };
  _delay_ms(500);
  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;
  frm.data[0] = 0x76;

  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  };
  _delay_ms(500);

  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;
  frm.data[0] = 0x88;

  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  };
  _delay_ms(500);
}

void gyems_motor_set_torque(gyems_motor *motor, int16_t torque)
{
  int16_t a = 62;
  CAN_frame_t frm;
  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;
  frm.data[0] = 0xA1;
  frm.data[1] = 0x00;
  frm.data[2] = 0x00;
  frm.data[3] = 0x00;
  frm.data[4] = *(uint8_t *)(&a);
  frm.data[5] = *((uint8_t *)(&a) + 1);
  frm.data[6] = 0x00;
  frm.data[7] = 0x00;

  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  };
}

void gyems_motor_set_speed(gyems_motor *motor, int32_t speed)
{
  motor->speed = speed;
  speed = speed * 100;
  CAN_frame_t frm;
  // Motor status 1
  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;
  frm.data[0] = SET_SPEED;
  frm.data[4] = *(uint8_t *)(&speed);
  frm.data[5] = *((uint8_t *)(&speed) + 1);
  frm.data[6] = *((uint8_t *)(&speed) + 2);
  frm.data[7] = *((uint8_t *)(&speed) + 3);

  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  }
}

void gyems_motor_find_endpoints(gyems_motor *motor)
{
  motor->endpoints_valid = 0;
  gyems_motor_set_speed(motor, motor->endpoint_speed);
}

void gyems_motor_parse_switches(gyems_motor *motor, uint8_t switchport)
{
  if (!(switchport & (1 << motor->endpoint_1_pin)))
  {
    if (motor->endpoints_valid == 0)
    {
      if (motor->endpoints_found == 0)
      {
        motor->endpoints_found = 1;
        gyems_motor_find_endpoints(motor);
      }
    }
  }
}

void gyems_motor_stop(gyems_motor *motor)
{
  CAN_frame_t frm;
  frm.SID = motor->id;
  frm.header.len = 8;
  frm.header.rtr = 0;
  frm.data[0] = 0x81;
  frm.data[1] = 0x00;
  frm.data[2] = 0x00;
  frm.data[3] = 0x00;
  frm.data[4] = 0x00;
  frm.data[5] = 0x00;
  frm.data[6] = 0x00;
  frm.data[7] = 0x00;
  while (CAN_send(&frm) != CAN_ERROR_NONE)
  {
  }
}

void gyems_motor_set_multiturn_angle(gyems_motor *motor, uint16_t speed,
                                     int32_t angle)
{
  if (motor->multiturn_angle_0 == 0 || motor->has_zeropoint == 0)
  {
    return;
  };

  if (angle < 0)
  {
    angle = 0;
  }
  else if (angle > myabs(motor->multiturn_angle_range))
  {
    angle = myabs(motor->multiturn_angle_range);
  }

  speed = speed;
  motor->angle = angle;

  if (motor->multiturn_angle_range > 0)
  {
    // Uus angle peab olema _0+angle
    int32_t workAngle = motor->multiturn_angle_0 + (angle * 100 * motor->gear_ratio_multiplier);
    if (workAngle <= (motor->multiturn_angle_0 + (motor->multiturn_angle_range * 100 * motor->gear_ratio_multiplier)))
    {
      CAN_frame_t frm;
      frm.SID = motor->id;
      frm.header.len = 8;
      frm.header.rtr = 0;
      frm.data[0] = SET_MULTITURN_ANGLE;
      frm.data[2] = *(uint8_t *)(&speed);
      frm.data[3] = *((uint8_t *)(&speed) + 1);
      frm.data[4] = *((uint8_t *)(&workAngle));
      frm.data[5] = *((uint8_t *)(&workAngle) + 1);
      frm.data[6] = *((uint8_t *)(&workAngle) + 2);
      frm.data[7] = *((uint8_t *)(&workAngle) + 3);
      while (CAN_send(&frm) != CAN_ERROR_NONE)
      {
      }
    }
  }
  else if (motor->multiturn_angle_range < 0)
  {
    int32_t workAngle = motor->multiturn_angle_0 - (angle * 1);
    if (workAngle <= (motor->multiturn_angle_0 - (motor->multiturn_angle_range * 1)))
    {
      CAN_frame_t frm;
      frm.SID = motor->id;
      frm.header.len = 8;
      frm.header.rtr = 0;
      frm.data[0] = SET_MULTITURN_ANGLE;
      frm.data[2] = *(uint8_t *)(&speed);
      frm.data[3] = *((uint8_t *)(&speed) + 1);
      frm.data[4] = *((uint8_t *)(&workAngle));
      frm.data[5] = *((uint8_t *)(&workAngle) + 1);
      frm.data[6] = *((uint8_t *)(&workAngle) + 2);
      frm.data[7] = *((uint8_t *)(&workAngle) + 3);
      while (CAN_send(&frm) != CAN_ERROR_NONE)
      {
      };
    }
  }
}

void gyems_motor_safety(gyems_motor *motor)
{
  if (!motor->has_zeropoint && (map(motor->torque_current, -2048, 2048, -33, 33) > motor->max_power_endpoints || map(motor->torque_current, -2048, 2048, -33, 33) < -(motor->max_power_endpoints)))
  {
    gyems_motor_stop(motor);
    gyems_motor_set_speed(motor, 0);
    gyems_motor_get_multiturn_angle(motor);
  }
  else if (motor->has_zeropoint && (map(motor->torque_current, -2048, 2048, -33, 33) > motor->max_power_operating || map(motor->torque_current, -2048, 2048, -33, 33) < -(motor->max_power_operating)))
  {
    //    gyems_motor_stop(motor);
    gyems_motor_stop(motor);

    gyems_motor_set_speed(motor, 0);
  }
}