/**
 * @file       main.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2020-11-27
 * @author     Thuan Le
 * @brief      Main file support control RMD X8 PRO motor via CAN BUS
 *             Arduino and Module Arduino CAN BUS Shield MCP2515
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "x8_can.h"
#include <mcp_can.h>
#include <SPI.h>

/* Private defines ---------------------------------------------------- */
#ifdef ARDUNIO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif

// Define joy pins
#define UP                      (A1)
#define DOWN                    (A3)
#define LEFT                    (A2)
#define RIGHT                   (A5)
#define CLICK                   (A4)

// Define LED pins
#define LED2                    (8)
#define LED3                    (7)
#define STEP_VALUE              (50)
#define SPI_CS_PIN              (10)

#define RMD_X8_SPEED_LIMITED    (514)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private constan ---------------------------------------------------- */
static const String SET_SPEED_CMD               = "SP";
static const String CLOCKWISE_CMD               = "TL";
static const String COUNTER_CLOCKWISE_CMD       = "TR";
static const String READ_MULTI_TURN_ANGLE_CMD   = "MT";

static const String READ_SPEED_CMD              = "RP";
static const String READ_ENCODER_CMD            = "RE";
static const String READ_TEMP_CMD               = "RT";

static const String READ_ANGLE_KP_CMD           = "AP";
static const String READ_ANGLE_KI_CMD           = "AI";
static const String READ_SPEED_KP_CMD           = "VP";
static const String READ_SPEED_KI_CMD           = "VI";
static const String READ_TORQUE_KP_CMD          = "TP";
static const String READ_TORQUE_KI_CMD          = "TI";

/* Private variables -------------------------------------------------- */
MCP_CAN CAN(SPI_CS_PIN);
static x8_can_t m_x8_can;
static long     m_rmd_x8_postion        = 0;
static String   m_uart_data_receive     = "";
static String   m_uart_cmd              = "";
static String   m_uart_data             = "";
static boolean  m_uart_string_complete  = false;
static float    m_float_data_value      = 0;
static float    m_motor_speed           = 10;

static bool     m_get_angle_kp          = false;
static bool     m_get_angle_ki          = false;
static bool     m_get_speed_kp          = false;
static bool     m_get_speed_ki          = false;
static bool     m_get_torque_kp         = false;
static bool     m_get_torque_ki         = false;

static bool     m_get_speed             = false;
static bool     m_get_encoder           = false;
static bool     m_get_temp              = false;

/* Private function prototypes ---------------------------------------- */
static void uart_receive_and_execute(void);
static void bsp_x8_can_send(uint16_t msg_id, uint8_t *buffer);
static void x8_can_init(void);
static void btn_check(void);
static void m_can_receive(void);

/* Function definitions ----------------------------------------------- */
void setup()
{
  SERIAL.begin(115200);
  delay(1000);

  // Init CAN BUS
  x8_can_init();

  // Pin settings
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // Pulling analog pins HIGH and led pins low
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
}

void loop()
{
  uart_receive_and_execute();
  m_can_receive();

  btn_check();
}

/* Private function definitions --------------------------------------- */
/**
 * @brief       Uart receive data and execute
 *
 * @param[in]   None
 *
 * @attention   None
 *
 * @return      None
 */
static void uart_receive_and_execute(void)
{
  while (SERIAL.available()) // Receive data from computer
  {
    char data = (char)SERIAL.read();
    m_uart_data_receive += data;

    if (data == '\n')
    {
      m_uart_string_complete = true;
    }

    if (m_uart_string_complete)
    {
      m_uart_string_complete = false;
      SERIAL.println(m_uart_data_receive);

      m_uart_cmd = m_uart_data_receive.substring(0, 2);
      m_uart_data = m_uart_data_receive.substring(3);
      m_float_data_value = m_uart_data.toFloat();
      SERIAL.println(m_uart_cmd);
      SERIAL.println(m_uart_data);
      SERIAL.println(m_float_data_value);

      if (CLOCKWISE_CMD == m_uart_cmd)
      {
        SERIAL.println("Set motor run clockwise");
        x8_can_send_position_ctrl_2_cmd(&m_x8_can, (uint16_t)m_motor_speed, m_float_data_value);
      }
      else if (COUNTER_CLOCKWISE_CMD == m_uart_cmd)
      {
        SERIAL.println("Set motor run counter clockwise");
        x8_can_send_position_ctrl_2_cmd(&m_x8_can, (uint16_t)m_motor_speed, -(int32_t)m_float_data_value);
      }
      else if (SET_SPEED_CMD == m_uart_cmd)
      {
        SERIAL.println("Set speed for motor run");
        m_motor_speed = m_float_data_value;
        x8_can_send_speed_close_loop_cmd(&m_x8_can, (int32_t)m_motor_speed);
      } 
      else if (READ_MULTI_TURN_ANGLE_CMD == m_uart_cmd)
      {
        SERIAL.println("Get motor multi turns angle");
        x8_can_send_get_motor_multi_turn_angle(&m_x8_can);
      }
      else if (READ_ANGLE_KP_CMD == m_uart_cmd)
      {
        m_get_angle_kp = true;
        SERIAL.println("Get angle kp");
        x8_can_send_get_pid_data(&m_x8_can);
      }
      else if (READ_ANGLE_KI_CMD == m_uart_cmd)
      {
        m_get_angle_ki = true;
        SERIAL.println("Get angle ki");
        x8_can_send_get_pid_data(&m_x8_can);
      }
      else if (READ_SPEED_KP_CMD == m_uart_cmd)
      {
        m_get_speed_kp = true;
        SERIAL.println("Get speed kp");
        x8_can_send_get_pid_data(&m_x8_can);
      }
      else if (READ_SPEED_KI_CMD == m_uart_cmd)
      {
        m_get_speed_ki = true;
        SERIAL.println("Get speed ki");
        x8_can_send_get_pid_data(&m_x8_can);
      }
      else if (READ_TORQUE_KP_CMD == m_uart_cmd)
      {
        m_get_torque_kp = true;
        SERIAL.println("Get torque kp");
        x8_can_send_get_pid_data(&m_x8_can);
      }
      else if (READ_TORQUE_KI_CMD == m_uart_cmd)
      {
        m_get_torque_ki = true;
        SERIAL.println("Get torque ki");
        x8_can_send_get_pid_data(&m_x8_can);
      }
      else if (READ_SPEED_CMD == m_uart_cmd)
      {
        m_get_speed = true;
        SERIAL.println("Get speed");
        x8_can_send_get_motor_status(&m_x8_can);
      }
      else if (READ_ENCODER_CMD == m_uart_cmd)
      {
        m_get_encoder = true;
        SERIAL.println("Get encoder");
        x8_can_send_get_motor_status(&m_x8_can);
      }
      else if (READ_TEMP_CMD == m_uart_cmd)
      {
        m_get_temp = true;
        SERIAL.println("Get temperature");
        x8_can_send_get_motor_status(&m_x8_can);
      }

      m_uart_data_receive = "";
    }
  }
}

/**
 * @brief       CAN receive data
 *
 * @param[in]   None
 *
 * @attention   None
 *
 * @return      None
 */
static void m_can_receive(void)
{
  uint8_t can_rx_len = 0;
  uint8_t can_rx_data[8];
  int64_t motor_multi_angle = 0;
  x8_motor_status_t motor_status;
  x8_motor_pid_data_t motor_pid;

  // Check CAN data comming
  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    SERIAL.println("Can msg receive");

    // Read data and length
    CAN.readMsgBuf(&can_rx_len, can_rx_data);

    switch (can_rx_data[0])
    {
    case RMD_X8_READ_MOTOR_STATUS_2_CMD:
    //  case RMD_X8_TORQUE_CLOSED_LOOP_CMD:
    //  case RMD_X8_SPEED_CLOSED_LOOP_CMD:
    // case RMD_X8_POSITION_CTRL_1_CMD:
    // case RMD_X8_POSITION_CTRL_2_CMD:
    // case RMD_X8_POSITION_CTRL_3_CMD:
    // case RMD_X8_POSITION_CTRL_4_CMD:
    {
      // Get motor status
      x8_can_get_motor_status(can_rx_data, &motor_status);

      if (m_get_temp)
      {
       SERIAL.print("Motor temperature: ");
       SERIAL.println(motor_status.temperature);
      }

      if (m_get_speed)
      {
        SERIAL.print("Motor speed rpm: ");
        SERIAL.println(motor_status.speed);
      }

      if (m_get_encoder)
      {
        SERIAL.print("Motor encoder: ");
        SERIAL.println(motor_status.encoder);
      }
      m_get_temp    = false;
      m_get_speed   = false;
      m_get_encoder = false;

      break;
    }

    case RMD_X8_READ_MULTI_TURNS_ANGLE_CMD:
    {
      x8_can_get_motor_multi_turn_angle(can_rx_data, &motor_multi_angle);
      SERIAL.print("Motor multi turn angle:");
      SERIAL.println((int)motor_multi_angle);
      break;
    }

    case RMD_X8_READ_PID_DATA_CMD:
    {
      x8_can_get_pid_data(can_rx_data, &motor_pid);

      if (m_get_angle_kp)
      {
        SERIAL.print("Angle kp  :");
        SERIAL.println(motor_pid.angle_kp);
      }

      if (m_get_angle_ki)
      {
        SERIAL.print("Angle ki  :");
        SERIAL.println(motor_pid.angle_ki);
      }

      if (m_get_speed_kp)
      {
        SERIAL.print("Speed kp  :");
        SERIAL.println(motor_pid.speed_kp);
      }

      if (m_get_speed_ki)
      {
        SERIAL.print("Speed ki  :");
        SERIAL.println(motor_pid.speed_ki);
      }

      if (m_get_torque_kp)
      {
        SERIAL.print("Torque kp :");
        SERIAL.println(motor_pid.torque_kp);
      }

      if (m_get_torque_ki)
      {
        SERIAL.print("Torque ki :");
        SERIAL.println(motor_pid.torque_ki);
      }

      m_get_angle_kp  = false;
      m_get_angle_ki  = false;
      m_get_speed_kp  = false;
      m_get_speed_ki  = false;
      m_get_torque_kp = false;
      m_get_torque_ki = false;

      break;
    }

    default:
      break;
    }
  }
}

/**
 * @brief       Button check
 *
 * @param[in]   None
 *
 * @attention   None
 *
 * @return      None
 */
static void btn_check(void)
{
  if (digitalRead(UP) == LOW)
  {
    m_rmd_x8_postion += STEP_VALUE;
    if (m_rmd_x8_postion > 35999)
    {
      m_rmd_x8_postion = max(m_rmd_x8_postion - 35999, 0);
    }
    if (m_rmd_x8_postion < 0)
    {
      m_rmd_x8_postion = 0;
    }
    x8_can_send_position_ctrl_4_cmd(&m_x8_can, m_rmd_x8_postion, RMD_X8_SPEED_LIMITED, X8_CLOCKWISE);
  }

  if (digitalRead(DOWN) == LOW)
  {
    m_rmd_x8_postion -= STEP_VALUE;
    if (m_rmd_x8_postion >= 36000)
    {
      m_rmd_x8_postion = 35999;
    }
    if (m_rmd_x8_postion < 0)
    {
      m_rmd_x8_postion = max(m_rmd_x8_postion + 35999, 35999);
    }
    x8_can_send_position_ctrl_4_cmd(&m_x8_can, m_rmd_x8_postion, RMD_X8_SPEED_LIMITED, X8_COUNTER_CLOCKWISE);
  }
}


/**
 * @brief       Can message send
 *
 * @param[in]   msg_id   Message id
 *              buffer   Pointer to buffer
 *
 * @attention   None
 *
 * @return      None
 */
static void bsp_x8_can_send(uint16_t msg_id, uint8_t *buffer)
{
  CAN.sendMsgBuf(msg_id, 0, 8, buffer);
}

/**
 * @brief       Can message init
 *
 * @param[in]   None
 *
 * @attention   None
 *
 * @return      None
 */
static void x8_can_init(void)
{
  m_x8_can.cansend = bsp_x8_can_send;

  if (CAN_OK != CAN.begin(CAN_1000KBPS))
  {
    SERIAL.println("Init CAN BUS failed");
  }
  else
  {
    SERIAL.println("Init CAN BUS successfull");
  }
}

/* End of file -------------------------------------------------------- */
