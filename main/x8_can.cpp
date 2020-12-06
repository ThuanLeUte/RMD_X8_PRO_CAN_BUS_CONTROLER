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
#define RMD_X8_CAN_MSG_ID                       (0x141)

#define RMD_X8_READ_PID_DATA_CMD                (0x30)
#define RMD_X8_WRITE_PID_TO_RAM_CMD             (0x31)
#define RMD_X8_WRITE_PID_TO_ROM_CMD             (0x32)
#define RMD_X8_READ_ACCELERATION_CMD            (0x33)
#define RMD_X8_WRITE_ACCELERATION_CMD           (0x34)
#define RMD_X8_READ_ENCODE_DATA_CMD             (0x90)
#define RMD_X8_WRITE_ENCODER_OFFSET_CMD         (0x91)
#define RMD_X8_WRITE_CURRENT_POSITION_CMD       (0x19)
#define RMD_X8_READ_MULTI_TURNS_ANGLE_CMD       (0x92)
#define RMD_X8_READ_SINGLE_CIRCLE_ANGLE_CMD     (0x94)
#define RMD_X8_READ_MOTOR_STATUS_CMD            (0x9A)
#define RMD_X8_CLEAR_MOTOR_ERROR_FLAG_CMD       (0x9B)
#define RMD_X8_READ_MOTOR_STATUS_2_CMD          (0x9C)
#define RMD_X8_READ_MOTOR_STATUS_3_CMD          (0x9D)
#define RMD_X8_MOTOR_OFF_CMD                    (0x80)
#define RMD_X8_MOTOR_STOP_CMD                   (0x81)
#define RMD_X8_MOTOR_RUNNING_CMD                (0x88)
#define RMD_X8_TORQUE_CLOSED_LOOP_CMD           (0xA1)
#define RMD_X8_SPEED_CLOSED_LOOP_CMD            (0xA2)
#define RMD_X8_POSITION_CTRL_1_CMD              (0xA3)
#define RMD_X8_POSITION_CTRL_2_CMD              (0xA4)
#define RMD_X8_POSITION_CTRL_3_CMD              (0xA5)
#define RMD_X8_POSITION_CTRL_4_CMD              (0xA6)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static uint8_t can_tx_data[8];
static x8_can_msg_write_encode_offset_cmd_t msg_write_encode_offset_cmd;
static x8_can_msg_position_ctrl_1_cmd_t     msg_position_ctrl_1_cmd;
static x8_can_msg_position_ctrl_2_cmd_t     msg_position_ctrl_2_cmd;
static x8_can_msg_position_ctrl_3_cmd_t     msg_position_ctrl_3_cmd;
static x8_can_msg_position_ctrl_4_cmd_t     msg_position_ctrl_4_cmd;


/* Private function prototypes ---------------------------------------- */
static void m_x8_can_pack_msg_encode_offset_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_1_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_2_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_3_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_4_cmd(uint8_t *can_data);
static void m_x8_can_send_msg(x8_can_t *me, x8_can_msg_handler_send_type_t msg_handler);

/* Function definitions ----------------------------------------------- */
void x8_can_send_position_ctrl_1_cmd(x8_can_t *me , uint32_t pos_ctrl)
{
  // Motor positon control
  msg_position_ctrl_1_cmd.pos_ctrl_lowest   = pos_ctrl;
  msg_position_ctrl_1_cmd.pos_ctrl_low      = pos_ctrl >> 8;
  msg_position_ctrl_1_cmd.pos_ctrl_high     = pos_ctrl >> 16;
  msg_position_ctrl_1_cmd.pos_ctrl_highest  = pos_ctrl >> 24;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_POSITION_CTRL_1_CMD);
}

void x8_can_send_position_ctrl_2_cmd(x8_can_t *me , uint16_t speed_limited, uint32_t pos_ctrl)
{
  // Motor speed limited
  msg_position_ctrl_2_cmd.speed_limited_low  = speed_limited;
  msg_position_ctrl_2_cmd.speed_limited_high = speed_limited >> 8;
  
  // Motor positon control
  msg_position_ctrl_2_cmd.pos_ctrl_lowest   = pos_ctrl;
  msg_position_ctrl_2_cmd.pos_ctrl_low      = pos_ctrl >> 8;
  msg_position_ctrl_2_cmd.pos_ctrl_high     = pos_ctrl >> 16;
  msg_position_ctrl_2_cmd.pos_ctrl_highest  = pos_ctrl >> 24;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_POSITION_CTRL_2_CMD);
}

void x8_can_send_position_ctrl_3_cmd(x8_can_t *me , uint16_t pos_ctrl,  x8_motor_dir_type_t dir)
{
  // Motor direction
  msg_position_ctrl_3_cmd.spin_dir = dir;
  
  // Motor positon control
  msg_position_ctrl_3_cmd.pos_ctrl_low      = pos_ctrl;
  msg_position_ctrl_3_cmd.pos_ctrl_high     = pos_ctrl >> 8;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_POSITION_CTRL_3_CMD);
}

void x8_can_send_position_ctrl_4_cmd(x8_can_t *me , uint16_t pos_ctrl, uint16_t speed_limited, x8_motor_dir_type_t dir)
{
  // Motor direction
  msg_position_ctrl_4_cmd.spin_dir = dir;

  // Motor speed limited
  msg_position_ctrl_4_cmd.speed_limited_low  = speed_limited;
  msg_position_ctrl_4_cmd.speed_limited_high = speed_limited >> 8;

  // Motor positon control
  msg_position_ctrl_4_cmd.pos_ctrl_low  = pos_ctrl;
  msg_position_ctrl_4_cmd.pos_ctrl_high = pos_ctrl >> 8;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_POSITION_CTRL_4_CMD);
}

void x8_can_send_encoder_offset_cmd(x8_can_t *me , uint16_t encoder_offset)
{
  // Encoder offset
  msg_write_encode_offset_cmd.encode_offset_low  = encoder_offset;
  msg_write_encode_offset_cmd.encode_offset_high = encoder_offset >> 8;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_WRITE_ENCODER_OFFSET_CMD);
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
  can_data[0] = msg_write_encode_offset_cmd.cmd_byte = RMD_X8_WRITE_ENCODER_OFFSET_CMD;
  can_data[1] = msg_write_encode_offset_cmd.data1;
  can_data[2] = msg_write_encode_offset_cmd.data2;
  can_data[3] = msg_write_encode_offset_cmd.data3;
  can_data[4] = msg_write_encode_offset_cmd.data4;
  can_data[5] = msg_write_encode_offset_cmd.data5;
  can_data[6] = msg_write_encode_offset_cmd.encode_offset_low;
  can_data[7] = msg_write_encode_offset_cmd.encode_offset_high;
}

/**
 * @brief       Can pack message position control command 1
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_position_ctrl_1_cmd(uint8_t *can_data)
{
  can_data[0] = msg_position_ctrl_1_cmd.cmd_byte = RMD_X8_POSITION_CTRL_1_CMD;
  can_data[1] = msg_position_ctrl_1_cmd.data1;
  can_data[2] = msg_position_ctrl_1_cmd.data2;
  can_data[3] = msg_position_ctrl_1_cmd.data3;
  can_data[4] = msg_position_ctrl_1_cmd.pos_ctrl_lowest;
  can_data[5] = msg_position_ctrl_1_cmd.pos_ctrl_low;
  can_data[6] = msg_position_ctrl_1_cmd.pos_ctrl_high;
  can_data[7] = msg_position_ctrl_1_cmd.pos_ctrl_highest;
}

/**
 * @brief       Can pack message position control command 2
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_position_ctrl_2_cmd(uint8_t *can_data)
{
  can_data[0] = msg_position_ctrl_2_cmd.cmd_byte = RMD_X8_POSITION_CTRL_2_CMD;
  can_data[1] = msg_position_ctrl_2_cmd.data1;
  can_data[2] = msg_position_ctrl_2_cmd.speed_limited_low;
  can_data[3] = msg_position_ctrl_2_cmd.speed_limited_high;
  can_data[4] = msg_position_ctrl_2_cmd.pos_ctrl_lowest;
  can_data[5] = msg_position_ctrl_2_cmd.pos_ctrl_low;
  can_data[6] = msg_position_ctrl_2_cmd.pos_ctrl_high;
  can_data[7] = msg_position_ctrl_2_cmd.pos_ctrl_highest;
}

/**
 * @brief       Can pack message position control command 3
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_position_ctrl_3_cmd(uint8_t *can_data)
{
  can_data[0] = msg_position_ctrl_3_cmd.cmd_byte = RMD_X8_POSITION_CTRL_3_CMD;
  can_data[1] = msg_position_ctrl_3_cmd.spin_dir;
  can_data[2] = msg_position_ctrl_3_cmd.data2;
  can_data[3] = msg_position_ctrl_3_cmd.data3;
  can_data[4] = msg_position_ctrl_3_cmd.pos_ctrl_low;
  can_data[5] = msg_position_ctrl_3_cmd.pos_ctrl_high;
  can_data[6] = msg_position_ctrl_3_cmd.data6;
  can_data[7] = msg_position_ctrl_3_cmd.data7;
}

/**
 * @brief       Can pack message position control command 4
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_position_ctrl_4_cmd(uint8_t *can_data)
{
  can_data[0] = msg_position_ctrl_4_cmd.cmd_byte = RMD_X8_POSITION_CTRL_4_CMD;
  can_data[1] = msg_position_ctrl_4_cmd.spin_dir;
  can_data[2] = msg_position_ctrl_4_cmd.speed_limited_low;
  can_data[3] = msg_position_ctrl_4_cmd.speed_limited_high;
  can_data[4] = msg_position_ctrl_4_cmd.pos_ctrl_low;
  can_data[5] = msg_position_ctrl_4_cmd.pos_ctrl_high;
  can_data[6] = msg_position_ctrl_4_cmd.data6;
  can_data[7] = msg_position_ctrl_4_cmd.data7;
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
  case X8_MSG_WRITE_ENCODER_OFFSET_CMD:
  {
    m_x8_can_pack_msg_encode_offset_cmd(can_tx_data);
    break;
  }

  case X8_MSG_POSITION_CTRL_1_CMD:
  {
    m_x8_can_pack_msg_position_ctrl_1_cmd(can_tx_data);
    break;
  }

  case X8_MSG_POSITION_CTRL_2_CMD:
  {
    m_x8_can_pack_msg_position_ctrl_2_cmd(can_tx_data);
    break;
  }

  case X8_MSG_POSITION_CTRL_3_CMD:
  {
    m_x8_can_pack_msg_position_ctrl_3_cmd(can_tx_data);
    break;
  }

  case X8_MSG_POSITION_CTRL_4_CMD:
  {
    m_x8_can_pack_msg_position_ctrl_4_cmd(can_tx_data);
    break;
  }


  default:
    break;
  }

  me->cansend(RMD_X8_CAN_MSG_ID, can_tx_data);
}

/* End of file -------------------------------------------------------- */
