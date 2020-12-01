/**
 * @file       x8_can.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2020-08-26
 * @author     Thuan Le
 * @brief      System module support control RMD X8 PRO motor via CAN BUS
 *             Arduino and Module Arduino CAN BUS Shield MCP2515
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "x8_can.h"

/* Private defines ---------------------------------------------------- */
#define RMD_X8_CAN_MSG_ID                (0x141)
#define RMD_X8_WRITE_ENCODER_OFFSET_CMD  (0x91)
#define RMD_X8_POSITION_CTRL_4_CMD       (0xA6)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static uint8_t can_tx_data[8];
static x8_can_msg_encode_offset_cmd_type_t msg_encode_offset_cmd;
static x8_can_msg_position_control_cmd_t   msg_position_control_cmd;

/* Private function prototypes ---------------------------------------- */
static void m_x8_can_pack_msg_encode_offset_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_control_cmd(uint8_t *can_data);
static void m_x8_can_send_msg(x8_can_t *me, x8_can_msg_handler_send_type_t msg_handler);

/* Function definitions ----------------------------------------------- */
void x8_can_send_position_control_cmd(x8_can_t *me ,uint16_t pos_ctrl, uint16_t speed_limited, x8_motor_dir_type_t dir)
{
  // Motor direction
  msg_position_control_cmd.spin_dir = dir;

  // Motor speed limited
  msg_position_control_cmd.speed_limited_low  = speed_limited;
  msg_position_control_cmd.speed_limited_high = speed_limited >> 8;

  // Motor positon control
  msg_position_control_cmd.pos_ctrl_low  = pos_ctrl;
  msg_position_control_cmd.pos_ctrl_high = pos_ctrl >> 8;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_POSITION_CONTROL_CMD);
}

void x8_can_send_encoder_offset_cmd(x8_can_t *me , uint16_t encoder_offset)
{
  // Encoder offset
  msg_encode_offset_cmd.encode_offset_low  = encoder_offset;
  msg_encode_offset_cmd.encode_offset_high = encoder_offset >> 8;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_ENCODER_OFFSET_CMD);
}

/* Private function definitions --------------------------------------- */
/**
 * @brief       Can pack message encode offset command
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_encode_offset_cmd(uint8_t *can_data)
{
  can_data[0] = msg_encode_offset_cmd.cmd_byte = RMD_X8_WRITE_ENCODER_OFFSET_CMD;
  can_data[1] = msg_encode_offset_cmd.data1;
  can_data[2] = msg_encode_offset_cmd.data2;
  can_data[3] = msg_encode_offset_cmd.data3;
  can_data[4] = msg_encode_offset_cmd.data4;
  can_data[5] = msg_encode_offset_cmd.data5;
  can_data[6] = msg_encode_offset_cmd.encode_offset_low;
  can_data[7] = msg_encode_offset_cmd.encode_offset_high;
}

/**
 * @brief       Can pack message position control command
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_position_control_cmd(uint8_t *can_data)
{
  can_data[0] = msg_position_control_cmd.cmd_byte = RMD_X8_POSITION_CTRL_4_CMD;
  can_data[1] = msg_position_control_cmd.spin_dir;
  can_data[2] = msg_position_control_cmd.speed_limited_low;
  can_data[3] = msg_position_control_cmd.speed_limited_high;
  can_data[4] = msg_position_control_cmd.pos_ctrl_low;
  can_data[5] = msg_position_control_cmd.pos_ctrl_high;
  can_data[6] = msg_position_control_cmd.data6;
  can_data[7] = msg_position_control_cmd.data7;
}

/**
 * @brief       Can send msg
 *
 * @param[in]   msg_handler   Message handler
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_send_msg(x8_can_t *me, x8_can_msg_handler_send_type_t msg_handler)
{
  switch (msg_handler)
  {
  case X8_MSG_ENCODER_OFFSET_CMD:
  {
    m_x8_can_pack_msg_encode_offset_cmd(can_tx_data);
    break;
  }

  case X8_MSG_POSITION_CONTROL_CMD:
  {
    m_x8_can_pack_msg_position_control_cmd(can_tx_data);
    break;
  }

  default:
    break;
  }

  me->cansend(RMD_X8_CAN_MSG_ID, can_tx_data);
}

/* End of file -------------------------------------------------------- */
