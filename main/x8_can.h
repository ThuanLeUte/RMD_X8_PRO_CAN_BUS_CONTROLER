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

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __X8_CAN_H
#define __X8_CAN_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <mcp_can.h>
#include <SPI.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Can message handler enum
 */
typedef enum
{
  X8_MSG_ENCODER_OFFSET_CMD,
  X8_MSG_POSITION_CONTROL_CMD,

  /*----------------------------------------------------------------- */
  X8_NUM_OF_SEND_MSF /* Do not insert elements behind this line !     */
  /*----------------------------------------------------------------- */
}
x8_can_msg_handler_send_type_t;

/**
 * @brief Can message encode offset command
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
  uint8_t data4;
  uint8_t data5;
  uint8_t encode_offset_low;
  uint8_t encode_offset_high;
}
x8_can_msg_encode_offset_cmd_type_t;

/**
 * @brief Can message position control command 4
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t spin_dir;
  uint8_t speed_limited_low;
  uint8_t speed_limited_high;
  uint8_t pos_ctrl_low;
  uint8_t pos_ctrl_high;
  uint8_t data6;
  uint8_t data7;
}
x8_can_msg_position_control_cmd_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern uint8_t can_tx_data[8];
extern x8_can_msg_encode_offset_cmd_type_t msg_encode_offset_cmd;
extern x8_can_msg_position_control_cmd_t   msg_position_control_cmd;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  Can pack message encode offset command
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_pack_msg_encode_offset_cmd(uint8_t *can_data);

/**
 * @brief  Can pack message position control command
 *
 * @param[in]   can_tx_data   Pointer to can tx data
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_pack_msg_position_control_cmd(uint8_t *can_data);

/**
 * @brief  Can send msg
 *
 * @param[in]   msg_handler   Message handler
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_msg(x8_can_msg_handler_send_type_t msg_handler);

#endif // __X8_CAN_H

/* End of file -------------------------------------------------------- */