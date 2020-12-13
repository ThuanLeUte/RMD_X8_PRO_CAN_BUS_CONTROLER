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

/* Public defines ----------------------------------------------------- */
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

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Can message handler enum
 */
typedef struct x8_can
{
  void (*cansend) (uint16_t msg_id, uint8_t * buffer);
}
x8_can_t;

/**
 * @brief Can message handler enum
 */
typedef enum
{
  X8_MSG_READ_PID_DATA_CMD,
  X8_MSG_WRITE_PID_TO_RAM_CMD,
  X8_MSG_WRITE_PID_TO_ROM_CMD,
  X8_MSG_READ_ACCELERATION_CMD,
  X8_MSG_WRITE_ACCELERATION_CMD,
  X8_MSG_READ_ENCODE_DATA_CMD,
  X8_MSG_WRITE_ENCODER_OFFSET_CMD,
  X8_MSG_WRITE_CURRENT_POSITION_CMD,
  X8_MSG_READ_MULTI_TURNS_ANGLE_CMD,
  X8_MSG_READ_SINGLE_CIRCLE_ANGLE_CMD,
  X8_MSG_READ_MOTOR_STATUS_CMD,
  X8_MSG_CLEAR_MOTOR_ERROR_FLAG_CMD,
  X8_MSG_READ_MOTOR_STATUS_2_CMD,
  X8_MSG_READ_MOTOR_STATUS_3_CMD,
  X8_MSG_MOTOR_OFF_CMD,
  X8_MSG_MOTOR_STOP_CMD,
  X8_MSG_MOTOR_RUNNING_CMD,
  X8_MSG_TORQUE_CLOSED_LOOP_CMD,
  X8_MSG_SPEED_CLOSED_LOOP_CMD,
  X8_MSG_POSITION_CTRL_1_CMD,
  X8_MSG_POSITION_CTRL_2_CMD,
  X8_MSG_POSITION_CTRL_3_CMD,
  X8_MSG_POSITION_CTRL_4_CMD,

  /*----------------------------------------------------------------- */
  X8_NUM_OF_SEND_MSG /* Do not insert elements behind this line !     */
  /*----------------------------------------------------------------- */
}
x8_can_msg_handler_send_type_t;

/**
 * @brief Motor direction enum
 */
typedef enum
{
  X8_CLOCKWISE,
  X8_COUNTER_CLOCKWISE
}
x8_motor_dir_type_t;

/**
 * @brief Motor direction enum
 */
typedef struct
{
  uint8_t   temperature;
  uint16_t  torque_current;
  uint16_t  speed;
  uint16_t  encoder;
}
x8_motor_status_t;

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
x8_can_msg_write_encode_offset_cmd_t;

/**
 * @brief Can message torque close loop command
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
  uint8_t torque_current_low;
  uint8_t torque_current_high;
  uint8_t data6;
  uint8_t data7;
}
x8_can_msg_torque_close_loop_cmd_t;

/**
 * @brief Can message speed close loop command
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
  uint8_t speed_ctrl_lowest;
  uint8_t speed_ctrl_low;
  uint8_t speed_ctrl_high;
  uint8_t speed_ctrl_highest;
}
x8_can_msg_speed_close_loop_cmd_t;

/**
 * @brief Can message position control command 1
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
  uint8_t pos_ctrl_lowest;
  uint8_t pos_ctrl_low;
  uint8_t pos_ctrl_high;
  uint8_t pos_ctrl_highest;
}
x8_can_msg_position_ctrl_1_cmd_t;
/**
 * @brief Can message position control command 2
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t data1;
  uint8_t speed_limited_low;
  uint8_t speed_limited_high;
  uint8_t pos_ctrl_lowest;
  uint8_t pos_ctrl_low;
  uint8_t pos_ctrl_high;
  uint8_t pos_ctrl_highest;
}
x8_can_msg_position_ctrl_2_cmd_t;

/**
 * @brief Can message position control command 3
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t spin_dir;
  uint8_t data2;
  uint8_t data3;
  uint8_t pos_ctrl_low;
  uint8_t pos_ctrl_high;
  uint8_t data6;
  uint8_t data7;
}
x8_can_msg_position_ctrl_3_cmd_t;

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
x8_can_msg_position_ctrl_4_cmd_t;

/**
 * @brief Can receive message motor status
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t temperature;
  uint8_t iq_low;
  uint8_t iq_high;
  uint8_t speed_low;
  uint8_t speed_high;
  uint8_t encoder_low;
  uint8_t encoder_high;
}
x8_can_receive_msg_motor_status_t;

/**
 * @brief Can  receive message multi turn angle command
 */
typedef struct
{
  uint8_t cmd_byte;
  uint8_t motor_angle_low_1;
  uint8_t motor_angle_2;
  uint8_t motor_angle_3;
  uint8_t motor_angle_4;
  uint8_t motor_angle_5;
  uint8_t motor_angle_6;
  uint8_t motor_angle_7;
}
x8_can_receive_msg_multi_turn_angle_t

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief       Can send encoder offset cmd
 *
 * @param[in]   me              Pointer to can handler
 *              encoder_offset  Encoder offset
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_encoder_offset_cmd(x8_can_t *me , uint16_t encoder_offset);

/**
 * @brief       Can send torque close loop cmd
 *
 * @param[in]   me              Pointer to can handler
 *              torque          Torque
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_torque_close_loop_cmd(x8_can_t *me , uint16_t torque);

/**
 * @brief       Can send speed close loop cmd
 *
 * @param[in]   me              Pointer to can handler
 *              speed           Speed (1 => 1 rpm)
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_speed_close_loop_cmd(x8_can_t *me , uint32_t speed);

/**
 * @brief       Can send position control cmd 1
 *
 * @param[in]   me              Pointer to can handler
 *              pos_ctrl        Position control
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_position_ctrl_1_cmd(x8_can_t *me , uint32_t pos_ctrl);

/**
 * @brief       Can send position control cmd 2
 *
 * @param[in]   me              Pointer to can handler
 *              pos_ctrl        Position control (1=> 1 degree; 360 => 360 degree (1 circle))
 *              speed_limited   Speed limited
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_position_ctrl_2_cmd(x8_can_t *me , uint16_t speed_limited, uint32_t pos_ctrl);

/**
 * @brief       Can send position control cmd 3
 *
 * @param[in]   me              Pointer to can handler
 *              pos_ctrl        Position control (1=> 1 degree; 360 => 360 degree (1 circle))
 *              dir             Direction
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_position_ctrl_3_cmd(x8_can_t *me , uint16_t pos_ctrl,  x8_motor_dir_type_t dir);

/**
 * @brief       Can send position control cmd 4
 *
 * @param[in]   me              Pointer to can handler
 *              pos_ctrl        Position control
 *              speed_limited   Speed limited
 *              dir             Direction
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_send_position_ctrl_4_cmd(x8_can_t *me ,uint16_t pos_ctrl, uint16_t speed_limited, x8_motor_dir_type_t dir);

/**
 * @brief       Get motor status
 *
 * @param[in]   can_rx_data       Pointer to can rx data
 *              motor_status      Pointer to motor status structure
 *
 * @attention   None
 *
 * @return      None
 */
void x8_can_get_motor_status(uint8_t *can_rx_data, x8_motor_status_t *motor_status);

#endif // __X8_CAN_H

/* End of file -------------------------------------------------------- */
