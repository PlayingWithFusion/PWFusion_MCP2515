
/***************************************************************************
* File Name: MCP2515_LoopbackTest.ino
*
* Copyright © 2014 Playing With Fusion, Inc.
* SOFTWARE LICENSE AGREEMENT: This code is released under the MIT License.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,M
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* **************************************************************************
* REVISION HISTORY:
* Author          Date        Comments
* J. Leonard      2014Dec10   Original version
* J. Leonard      2022Jan13   Updated to support refactored MCP2515 library
*
* Playing With Fusion, Inc. invests time and resources developing open-source
* code. Please support Playing With Fusion and continued open-source
* development by buying products from Playing With Fusion!
*
* **************************************************************************
* ADDITIONAL NOTES:
* This file contains functions to initialize and run an Arduino Uno or Mega in
* order to communicate with a MCP2515 (or MCP2510) CAN shield board
*
*  Circuit:
*    Arduino Uno   Arduino Mega  -->  SEN-30006
*    DIO pin 10      DIO pin 10  -->  CS_CAN
*    DIO pin  5      DIO pin  5  -->  LED1 - CAN board
*    DIO pin  4      DIO pin  4  -->  LED2 - CAN board
*    DIO pin  2      DIO pin  2  -->  CAN Controller Interrupt pin
*    MOSI: ICSP                  -->  SDI (must not be changed for hardware SPI)
*    MISO: ICSP                  -->  SDO (must not be changed for hardware SPI)
*    SCK:  ICSP                  -->  SCLK (must not be changed for hardware SPI)
*    5V power                    -->  CAN controller
***************************************************************************/

#include <string.h>
#include "PWFusion_MCP2515.h"

#define CS_CAN  10
#define INT_CAN 2

#define LED1    5
#define LED2    4

MCP2515 CAN;

void setup()
{
  // Initialize the serial port and display PWF header
  Serial.begin(115200);
  Serial.println(F("Playing With Fusion: MCP2515 CAN Shield"));
  Serial.println(F("Loopback Test"));
  
  // Initialize the Playing With Fusion MCP 2515 CAN shield
  // Pin 9 is SPI chip select, pin 2 is used for the MCP2515 interrupt line
  CAN.begin(CS_CAN, INT_CAN, true, 500);  //500kbps, loopback enabled
  

  // Turn on both shield LEDs
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, HIGH);
}


void loop()
{
  can_message_t txMsg = {0x18EF0102L,1,0,8,{1,2,3,4,5,6,7,0}};  // example message that will be transmitted
  
  for (int i=0; i<7; i++)
  {
    txMsg.data[7]=i; 
    // Break out of this for loop if unable to queue message (MCP2515 only has 3 transmit buffers)
    if (!CAN.send(&txMsg))
    {
      break;
    }
  }

  //Take a little nap
  delay(800);
   
  char myStr[36];      // Buffer to build string to send to serial port
  can_message_t rxMsg; // Allocate memory to store received CAN messages
  uint8_t numRxMsgs = 0;
  
  // Empty rx message queue by calling CAN.receive() until it returns false (queue empty)
  while (CAN.receive(&rxMsg))
  {
    numRxMsgs++;
    sprintf(myStr, "%3s %08lX %01d %d - %02X %02X %02X %02X %02X %02X %02X %02X",
            rxMsg.extended?"Ext":"Std",
            rxMsg.sid, 
            rxMsg.rtr, 
            rxMsg.dlc, 
            rxMsg.data[0],
            rxMsg.data[1],
            rxMsg.data[2],
            rxMsg.data[3],
            rxMsg.data[4],
            rxMsg.data[5],
            rxMsg.data[6],
            rxMsg.data[7] );

    Serial.println(myStr);
  }

  // Should have received loopback messages, if not re-do config
  if(0 == numRxMsgs)
  {
    Serial.println(F("No data looped back, Resetting Config..."));
    CAN.begin(CS_CAN, INT_CAN, true, 500);  // 500kbps, loopback enabled
  }
}
