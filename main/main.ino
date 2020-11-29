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

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

static long GenPos = 0;
static long pass = 0;
static long a = 0;
static long vs;
static boolean state = false;
static boolean state2 = false;
static boolean state3 = true;

static unsigned char buffer[8];
unsigned char buffer2[8];

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

void setup()
{
  SERIAL.begin(115200);
  delay(1000);
  while (CAN_OK != CAN.begin(CAN_1000KBPS))
  {
    delay(100);
  }

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
  
  if (state3 == true)
  {
    state3 = false;
    
    buffer2[0] = 0x91;
    buffer2[1] = 0x00;
    buffer2[2] = 0x00;
    buffer2[3] = 0x00;
    buffer2[4] = 0x00;
    buffer2[5] = 0x00;
    buffer2[6] = 0;
    buffer2[7] = 1;
    CAN.sendMsgBuf(0x141, 0, 8, buffer2);
  }
  while (Serial.available() > 0)
  {
    float b = Serial.parseFloat();
    char junk = Serial.read();
    while (junk != '\n')
    {
      junk = Serial.read();
    }
    Serial.println(b);
    if (b != 0)
    {
      a = b * 600.00;
    }
    vs = a;
    Serial.println(a);
    state = false;
    state2 = false;
    pass = 0;
  }

  if (a < 0 && pass > a)
  {
    state = true;
    pass -= STEP_VALUE;
    GenPos -= STEP_VALUE;
    if (GenPos >= 36000)
    {
      GenPos = 35999;
    }
    if (GenPos < 0)
    {
      GenPos = max(GenPos + 35999, 35999);
    }
    buffer[0] = 0xA6;
    buffer[1] = 0x01;
    buffer[2] = 0x02;
    buffer[3] = 0x02;
    buffer[4] = GenPos;
    buffer[5] = GenPos >> 8;
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    CAN.sendMsgBuf(0x141, 0, 8, buffer);

    Serial.print(vs / 600);
    Serial.println(" Position");
    vs = vs + STEP_VALUE;
  }

  if (pass <= a && state == true)
  {
    state = false;
    pass = 0;
    a = 0;
    vs = 0;
    Serial.println("I STOPPED");
  }

  if (a > 0 && pass < a)
  {
    state2 = true;
    pass += STEP_VALUE;
    GenPos += STEP_VALUE;
    if (GenPos > 35999)
    {
      GenPos = max(GenPos - 35999, 0);
    }
    if (GenPos < 0)
    {
      GenPos = 0;
    }
    buffer[0] = 0xA6;
    buffer[1] = 0x00;
    buffer[2] = 0x02;
    buffer[3] = 0x02;
    buffer[4] = GenPos;
    buffer[5] = GenPos >> 8;
    buffer[6] = 0x00;
    buffer[7] = 0x00;

    CAN.sendMsgBuf(0x141, 0, 8, buffer);

    Serial.print(vs / 600);
    Serial.println(" Position");
    vs = vs - STEP_VALUE;
  }
  if (pass >= a && state2 == true)
  {
    state2 = false;
    pass = 0;
    a = 0;
    vs = 0;
    //delay(15);
    Serial.println("I STOPPED");
  }
  if (digitalRead(UP) == LOW)
  {
    GenPos += STEP_VALUE;
    if (GenPos > 35999)
    {
      GenPos = max(GenPos - 35999, 0);
    }
    if (GenPos < 0)
    {
      GenPos = 0;
    }
    //SERIAL.println(GenPos);
    buffer[0] = 0xA6;
    buffer[1] = 0x00;
    buffer[2] = 0x02;
    buffer[3] = 0x02;
    buffer[4] = GenPos;
    buffer[5] = GenPos >> 8;
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    CAN.sendMsgBuf(0x141, 0, 8, buffer);
  }
  if (digitalRead(DOWN) == LOW)
  {
    GenPos -= STEP_VALUE;
    if (GenPos >= 36000)
    {
      GenPos = 35999;
    }
    if (GenPos < 0)
    {
      GenPos = max(GenPos + 35999, 35999);
    }
    //SERIAL.println(GenPos);
    buffer[0] = 0xA6;
    buffer[1] = 0x01;
    buffer[2] = 0x02;
    buffer[3] = 0x02;
    buffer[4] = GenPos;
    buffer[5] = GenPos >> 8;
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    CAN.sendMsgBuf(0x141, 0, 8, buffer);
  }
}
/* End of file -------------------------------------------------------- */
