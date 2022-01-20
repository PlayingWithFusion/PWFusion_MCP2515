
/***************************************************************************
* File Name: MCP2515_ThermocoupleDataReceive.ino
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
* J. Steinlage    2017Apr29   Original version
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

#define CS_CAN 10
#define INT_CAN 2

#define LED1    5
#define LED2    4

MCP2515 CAN;

// Calculate the absolute value of an 16-bit int
static int16_t Abs(int16_t value)
{
  if (value < 0)
  {
    value = -value;
  }

  return value;
}


void setup()
{
  // Initialize the serial port and display PWF header
  Serial.begin(115200);
  Serial.println(F("Playing With Fusion: MCP2515 CAN Shield"));
  
  // Initialize the Playing With Fusion MCP 2515 CAN shield
  // Pin 9 is SPI chip select, pin 2 is used for the MCP2515 interrupt line
  CAN.begin(CS_CAN, INT_CAN, false, 500);  //500kbps, normal mode (not loopback)
  
  // Turn on both shield LEDs
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, HIGH);
}


void loop()
{
  // set max and min TC SID range (same message format, 2 TC chans per SID)
  const uint32_t minCANMsgID = 0x6B1;
  const uint32_t maxCANMsgID = 0x6B8;

  // Uncomment below code and enable loopback to verify receive setup is working correctly
  //                                  TCXLB,TCXHB,TCYLB,TCYHB,TCSTAT, CJLB, CJHB
  //                                   1683.875C   -168.375C   noflt   25.0625C
  // delay(750);
  // can_message_t txMsg = {0x6B3,0,0,7,{0x3E, 0x69, 0x86,  0xF5,  0x00, 0x91, 0x01, 0}};  // example message that will be transmitted
  // CAN.send(&txMsg);
  
  can_message_t rxMsg; // Allocate memory to store received CAN messages
  char myStr[80];      // Buffer to build string to send to serial port

  //empty rx message queue by calling CAN.receive() untill it returns false (queue empty)
  while( CAN.receive(&rxMsg) )
  {
    if((minCANMsgID <= rxMsg.sid) && (maxCANMsgID >= rxMsg.sid)) // Identify thermocouple channels
    {
      uint8_t TCstartCh = (2 * (rxMsg.sid - minCANMsgID)) + 1;

      // TC1 Temperature - Integer value
      int16_t temperatureX16 = ((int16_t)rxMsg.data[1] << 8) | rxMsg.data[0];
      int16_t TC_IntVal1 = temperatureX16 >> 4;  // Divide by 16
      int16_t TC_DecVal1 = (Abs(temperatureX16) & 0x0F) * 063; // Each bit is 1/16th (0.0625) of a degree

      // TC2 Temperature - Integer value
      temperatureX16 = ((int16_t)rxMsg.data[3] << 8) | rxMsg.data[2];
      int16_t TC_IntVal2 = temperatureX16 >> 4;  // Divide by 16
      int16_t TC_DecVal2 = (Abs(temperatureX16) & 0x0F) * 063; // Each bit is 1/16th (0.0625) of a degree

      // Reference Junction Temperature
      int16_t temperatureX64 = ((int16_t)rxMsg.data[6] << 8) | rxMsg.data[5];
      int16_t TC_RJIntVal = temperatureX64 >> 6;  // Divide by 64
      int16_t TC_RJDecVal = (Abs(temperatureX64) & 0x3F) * 016; // Each bit is 1/64th (0.015625) of a degree
      
      sprintf( myStr, "%08lX TC%01u = %4d.%03d     TC%01d = %4d.%03d     RefJcn = %4d.%03d",
               rxMsg.sid,
               TCstartCh, 
               TC_IntVal1, 
               TC_DecVal1,
               (TCstartCh+1), 
               TC_IntVal2, 
               TC_DecVal2,
               TC_RJIntVal, 
               TC_RJDecVal );

       Serial.println(myStr); // print info to UART
    }
    else
    {
      // Message wasn't in list of expected inputs, ignore it (uncomment to print generic info for each msg)
      /*
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
      Serial.println(myStr); // print info to UART
      */
    }
  }
}
