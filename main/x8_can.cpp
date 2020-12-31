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
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static uint8_t can_tx_data[8];
static uint8_t can_rx_data[8];

// Can send message structure
static x8_can_msg_write_encode_offset_cmd_t msg_write_encode_offset_cmd;
static x8_can_msg_speed_close_loop_cmd_t    msg_speed_close_loop_cmd;
static x8_can_msg_torque_close_loop_cmd_t   msg_torque_close_loop_cmd;
static x8_can_msg_position_ctrl_1_cmd_t     msg_position_ctrl_1_cmd;
static x8_can_msg_position_ctrl_2_cmd_t     msg_position_ctrl_2_cmd;
static x8_can_msg_position_ctrl_3_cmd_t     msg_position_ctrl_3_cmd;
static x8_can_msg_position_ctrl_4_cmd_t     msg_position_ctrl_4_cmd;

// Can receive message structure
static x8_can_receive_msg_motor_status_t      msg_receive_motor_status;
static x8_can_receive_msg_multi_turn_angle_t  msg_receive_multi_turn_angle;

/* Private function prototypes ---------------------------------------- */
// Can pack transmit message
static void m_x8_can_pack_msg_encode_offset_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_speed_close_loop_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_torque_close_loop_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_1_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_2_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_3_cmd(uint8_t *can_data);
static void m_x8_can_pack_msg_position_ctrl_4_cmd(uint8_t *can_data);

// Can unpack receive message
static void m_x8_can_unpack_msg_receive_motor_status(uint8_t *can_data);
static void m_x8_can_unpack_msg_receive_multi_turn_angle(uint8_t *can_data);

static void m_x8_can_send_msg(x8_can_t *me, x8_can_msg_handler_send_type_t msg_handler);

/* Function definitions ----------------------------------------------- */
void x8_can_send_encoder_offset_cmd(x8_can_t *me , uint16_t encoder_offset)
{
  // Encoder offset
  msg_write_encode_offset_cmd.encode_offset_low  = encoder_offset;
  msg_write_encode_offset_cmd.encode_offset_high = encoder_offset >> 8;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_WRITE_ENCODER_OFFSET_CMD);
}

void x8_can_send_torque_close_loop_cmd(x8_can_t *me , int16_t torque)
{
  // Torque close loop
  msg_torque_close_loop_cmd.torque_current_low   = torque;
  msg_torque_close_loop_cmd.torque_current_high  = torque >> 8;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_TORQUE_CLOSED_LOOP_CMD);
}

void x8_can_send_speed_close_loop_cmd(x8_can_t *me , int32_t speed)
{
  // Cover rpm to dps
  speed = speed * 360;
  speed = speed / 60;

  // Cover 1dsp/LSB to 0.01dsp/LSB
  speed = speed * 100;

  // Speed close loop
  msg_speed_close_loop_cmd.speed_ctrl_lowest   = speed;
  msg_speed_close_loop_cmd.speed_ctrl_low      = speed >> 8;
  msg_speed_close_loop_cmd.speed_ctrl_high     = speed >> 16;
  msg_speed_close_loop_cmd.speed_ctrl_highest  = speed >> 24;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_SPEED_CLOSED_LOOP_CMD);
}

void x8_can_send_position_ctrl_1_cmd(x8_can_t *me , int32_t pos_ctrl)
{
  // Convert 1degree/LSB to 0.01degree/LSB
  pos_ctrl = pos_ctrl * 100;

  // Motor positon control
  msg_position_ctrl_1_cmd.pos_ctrl_lowest   = pos_ctrl;
  msg_position_ctrl_1_cmd.pos_ctrl_low      = pos_ctrl >> 8;
  msg_position_ctrl_1_cmd.pos_ctrl_high     = pos_ctrl >> 16;
  msg_position_ctrl_1_cmd.pos_ctrl_highest  = pos_ctrl >> 24;

  // Can send message
  m_x8_can_send_msg(me, X8_MSG_POSITION_CTRL_1_CMD);
}

void x8_can_send_position_ctrl_2_cmd(x8_can_t *me , uint16_t speed_limited, int32_t pos_ctrl)
{
  // Cover rpm to dps
  speed_limited = speed_limited * 360;
  speed_limited = speed_limited / 60;

  // Convert 1degree/LSB to 0.01degree/LSB
  pos_ctrl = pos_ctrl * 100;

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

void x8_can_send_position_ctrl_3_cmd(x8_can_t *me , uint16_t pos_ctrl, x8_motor_dir_type_t dir)
{
  // Convert 1degree/LSB to 0.01degree/LSB
  pos_ctrl = pos_ctrl * 100;

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
  // Cover rpm to dps
  speed_limited = speed_limited * 360;
  speed_limited = speed_limited  / 60;

  // Convert 1degree/LSB to 0.01degree/LSB
  pos_ctrl = pos_ctrl * 100;

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

void x8_can_send_get_motor_status(x8_can_t *me)
{
  // Can send message
  m_x8_can_send_msg(me, X8_MSG_READ_MOTOR_STATUS_2_CMD);
}

void x8_can_send_get_motor_multi_turn_angle(x8_can_t *me)
{
  // Can send message
  m_x8_can_send_msg(me, X8_MSG_READ_MULTI_TURNS_ANGLE_CMD);
}

void x8_can_get_motor_status(uint8_t *can_rx_data, x8_motor_status_t *motor_status)
{
  // Unpack CAN data
  m_x8_can_unpack_msg_receive_motor_status(can_rx_data);

  // Get motor temperature
  motor_status->temperature =  (int8_t)msg_receive_motor_status.temperature;

  // Get motor torque current
  motor_status->torque_current = (int16_t(msg_receive_motor_status.iq_high) << 8) |
                                           msg_receive_motor_status.iq_low;

  // Get motor speed
  motor_status->speed = (int16_t(msg_receive_motor_status.speed_high) << 8) |
                                  msg_receive_motor_status.speed_low;

  // Get motor encoder
  motor_status->encoder = (uint16_t(msg_receive_motor_status.encoder_high) << 8) |
                                    msg_receive_motor_status.encoder_low;

  // Cover dps to rpm
  motor_status->speed =  motor_status->speed * 60;
  motor_status->speed =  motor_status->speed / 360;
}

void x8_can_get_motor_multi_turn_angle(uint8_t *can_rx_data, int64_t *multi_turn_angle)
{
  // Unpack CAN data
  m_x8_can_unpack_msg_receive_multi_turn_angle(can_rx_data);

  // Get multi angle turn
  *multi_turn_angle =  (int64_t(msg_receive_multi_turn_angle.motor_angle_7) << 56) |
                       (int64_t(msg_receive_multi_turn_angle.motor_angle_7) << 48) |
                       (int64_t(msg_receive_multi_turn_angle.motor_angle_6) << 40) |
                       (int64_t(msg_receive_multi_turn_angle.motor_angle_5) << 32) |
                       (int64_t(msg_receive_multi_turn_angle.motor_angle_4) << 24) |
                       (int64_t(msg_receive_multi_turn_angle.motor_angle_3) << 16) |
                       (int64_t(msg_receive_multi_turn_angle.motor_angle_2) << 8 ) |
                                 msg_receive_multi_turn_angle.motor_angle_low_1;

  // Convert 0.01degree/LSB to 1degree/LSB
  *multi_turn_angle = *multi_turn_angle / 100;
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
 * @brief       Can pack message torque close loop command
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_torque_close_loop_cmd(uint8_t *can_data)
{
  can_data[0] = msg_torque_close_loop_cmd.cmd_byte = RMD_X8_TORQUE_CLOSED_LOOP_CMD;
  can_data[1] = msg_torque_close_loop_cmd.data1;
  can_data[2] = msg_torque_close_loop_cmd.data2;
  can_data[3] = msg_torque_close_loop_cmd.data3;
  can_data[4] = msg_torque_close_loop_cmd.torque_current_low;
  can_data[5] = msg_torque_close_loop_cmd.torque_current_high;
  can_data[6] = msg_torque_close_loop_cmd.data6;
  can_data[7] = msg_torque_close_loop_cmd.data7;
}

/**
 * @brief       Can pack message speed close loop command
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_pack_msg_speed_close_loop_cmd(uint8_t *can_data)
{
  can_data[0] = msg_speed_close_loop_cmd.cmd_byte = RMD_X8_SPEED_CLOSED_LOOP_CMD;
  can_data[1] = msg_speed_close_loop_cmd.data1;
  can_data[2] = msg_speed_close_loop_cmd.data2;
  can_data[3] = msg_speed_close_loop_cmd.data3;
  can_data[4] = msg_speed_close_loop_cmd.speed_ctrl_lowest;
  can_data[5] = msg_speed_close_loop_cmd.speed_ctrl_low;
  can_data[6] = msg_speed_close_loop_cmd.speed_ctrl_high;
  can_data[7] = msg_speed_close_loop_cmd.speed_ctrl_highest;
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
 * @brief       Can unpack message position control command 4
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_unpack_msg_receive_motor_status(uint8_t *can_data)
{
  msg_receive_motor_status.cmd_byte     = can_data[0];
  msg_receive_motor_status.temperature  = can_data[1];
  msg_receive_motor_status.iq_low       = can_data[2];
  msg_receive_motor_status.iq_high      = can_data[3];
  msg_receive_motor_status.speed_low    = can_data[4];
  msg_receive_motor_status.speed_high   = can_data[5];
  msg_receive_motor_status.encoder_low  = can_data[6];
  msg_receive_motor_status.encoder_high = can_data[7];
}

/**
 * @brief       Can unpack message multi turn angle
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
static void m_x8_can_unpack_msg_receive_multi_turn_angle(uint8_t *can_data)
{
  msg_receive_multi_turn_angle.cmd_byte          = can_data[0];
  msg_receive_multi_turn_angle.motor_angle_low_1 = can_data[1];
  msg_receive_multi_turn_angle.motor_angle_2     = can_data[2];
  msg_receive_multi_turn_angle.motor_angle_3     = can_data[3];
  msg_receive_multi_turn_angle.motor_angle_4     = can_data[4];
  msg_receive_multi_turn_angle.motor_angle_5     = can_data[5];
  msg_receive_multi_turn_angle.motor_angle_6     = can_data[6];
  msg_receive_multi_turn_angle.motor_angle_7     = can_data[7];
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

  case X8_MSG_SPEED_CLOSED_LOOP_CMD:
  {
    m_x8_can_pack_msg_speed_close_loop_cmd(can_tx_data);
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

  case X8_MSG_READ_MOTOR_STATUS_2_CMD:
  {
    can_tx_data[0] = RMD_X8_READ_MOTOR_STATUS_2_CMD;
    break;
  }

  case X8_MSG_READ_MULTI_TURNS_ANGLE_CMD:
  {
    can_tx_data[0] = RMD_X8_READ_MULTI_TURNS_ANGLE_CMD;
    break;
  }

  default:
    break;
  }

  me->cansend(RMD_X8_CAN_MSG_ID, can_tx_data);
}

/* End of file -------------------------------------------------------- */
