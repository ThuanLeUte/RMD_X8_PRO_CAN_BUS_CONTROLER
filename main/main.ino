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
/* Private variables -------------------------------------------------- */
MCP_CAN CAN(SPI_CS_PIN);
static x8_can_t m_x8_can;

static long m_rmd_x8_postion = 0;

/* Private function prototypes ---------------------------------------- */
static void bsp_x8_can_send(uint16_t msg_id, uint8_t *buffer);
static void x8_can_init(void);

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
    x8_can_send_position_control_cmd(&m_x8_can, m_rmd_x8_postion, RMD_X8_SPEED_LIMITED, X8_CLOCKWISE);
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
    x8_can_send_position_control_cmd(&m_x8_can, m_rmd_x8_postion, RMD_X8_SPEED_LIMITED, X8_COUNTER_CLOCKWISE);
  }
}

static void bsp_x8_can_send(uint16_t msg_id, uint8_t *buffer)
{
  CAN.sendMsgBuf(msg_id, 0, 8, buffer);
}

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
