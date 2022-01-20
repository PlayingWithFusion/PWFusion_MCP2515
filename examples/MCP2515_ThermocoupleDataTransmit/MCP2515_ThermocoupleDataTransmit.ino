
/***************************************************************************
* File Name: MCP2515_ThermocoupleDataTransmit.ino
*
* Copyright © 2017 Playing With Fusion, Inc.
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
* order to communicate with a MAX31856 quad channel thermocouple shield board
* and broadcast the data as a CAN node. Funcionality is as described below:
*  - Configure Arduino to broadcast results via CAN (MCP2510 controller)
*       - call PWF library to configure and read MAX31856 IC (SEN-30007, any type)
* - Broadcast results to CAN port in two separate CAN mesages
*
*  Circuit:
*    Arduino Uno   Arduino Mega  -->  SEN-30006
*    DIO pin 10      DIO pin 10  -->  CS_CAN
*    DIO pin  9      DIO pin  9  -->  CS_TC1
*    DIO pin  8      DIO pin  8  -->  CS_TC2
*    DIO pin         DIO pin  7  -->  CS_TC3
*    DIO pin  6      DIO pin  6  -->  CS_TC4
*    DIO pin  5      DIO pin  5  -->  LED1 - CAN board
*    DIO pin  4      DIO pin  4  -->  LED2 - CAN board
*    DIO pin  3      DIO pin  3  -->  NC
*    DIO pin  2      DIO pin  2  -->  CAN Controller Interrupt pin
*    MOSI: ICSP                  -->  SDI (must not be changed for hardware SPI)
*    MISO: ICSP                  -->  SDO (must not be changed for hardware SPI)
*    SCK:  ICSP                  -->  SCLK (must not be changed for hardware SPI)
*    3.3V power                  -->  MAX31856 chips
*    5V power                    -->  CAN controller, xlator ICs for MAX board
* The 0-ohm resistors on the MAX31856 PCB must be removed to enable CS on pin 10
* to be moved to pin6 on the MAX board. Pin 10 is used by the CAN controller.
* 'Data Ready' funcionality isn't typically used.
***************************************************************************/
#include <string.h>
#include "PWFusion_MCP2515.h"
#include "PWFusion_MAX31856.h"  // Part of the PWFusion_MAX31856 Arduino library

// set max and min TC SID range (same message format, 2 TC chans per SID)
const uint32_t minCANMsgID = 0x6B1;
const uint32_t maxCANMsgID = 0x6B8;

#define CS_CAN  10
#define INT_CAN 2

#define LED1    5
#define LED2    4

#define NUM_THERMOCOUPLES   (sizeof(tcChipSelects) / sizeof(uint8_t))

uint8_t tcChipSelects[] = {9, 8, 7, 6};  // define chip select pins for each thermocouple
MAX31856  thermocouples[NUM_THERMOCOUPLES];

MCP2515 CAN;


void setup()
{
  // Initialize the serial port and display PWF header
  Serial.begin(230400);  // fast to keep out of the way of CAN
  Serial.println(F("Playing With Fusion: Quad Thermocouple CAN Node"));
  Serial.println(F("IFB-10003 CAN Shield and SEN-30007 MAX31856 Quad TC"));
  
  // Initialize the Playing With Fusion MCP 2515 CAN shield
  // Pin 10 is SPI chip select, pin 2 is used for the MCP2515 interrupt line
  CAN.begin(CS_CAN, INT_CAN, false, 500);  //500kbps, loopback disabled

  // Initialize each MAX31856... options can be seen in the PWFusion_MAX31856.h file
  for (int i=0; i<NUM_THERMOCOUPLES; i++)
  {
    thermocouples[i].begin(tcChipSelects[i]);
    thermocouples[i].config(K_TYPE, CUTOFF_60HZ, AVG_SEL_4SAMP, CMODE_AUTO);
  }

  // Turn on both shield LEDs
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, HIGH);
}


void loop()
{
  for (int i=0; i<NUM_THERMOCOUPLES; i++)
  {
    // Get latest measurement from MAX31856 channels
    thermocouples[i].sample();

    // Print information to serial port
    print31856Results(i, thermocouples[i]);
  }

  // Pack, broadcast results on CAN 
  pack2ChThermSID(thermocouples[0], thermocouples[1], minCANMsgID);
  pack2ChThermSID(thermocouples[2], thermocouples[3], minCANMsgID+1);

  delay(1000); // delay then do it all again
}


void pack2ChThermSID(MAX31856 &tc0, MAX31856 &tc1, uint32_t sid)
{
  // Thermocouple temperature values will be formatted as int16 with 2^-4 resolution (0.0625 degC/bit)
  int16_t temp0x16 = tc0.getTemperature() * 16.0;
  int16_t temp1x16 = tc1.getTemperature() * 16.0;

  // Reference Junction packing (original datatype int16, 2^-6 degC/bit res)
  int16_t coldJunctTempX16 = tc0.getColdJunctionTemperature() * 64.0;
  
  // Status packing. 0x(TCA)|(TCB)
  //    bit0: TC Open
  //    bit1: TC Range Issue
  //    bit2: RefJcnIssue
  //    bit3: Unused/reserved
  uint8_t statusByte = 0;
  
  // Thermocouple A
  uint8_t status = tc0.getStatus();
  if(status & TC_FAULT_OPEN)
  {
    statusByte |= 0x10;
  }
  if(status & (TC_FAULT_TC_OOR | TC_FAULT_TC_TEMP_HIGH | TC_FAULT_TC_TEMP_LOW | TC_FAULT_VOLTAGE_OOR))
  {
    statusByte |= 0x20;
  }
  if(status & (TC_FAULT_CJ_OOR | TC_FAULT_CJ_TEMP_HIGH | TC_FAULT_CJ_TEMP_LOW))
  {
    statusByte |= 0x40;
  }

  // Thermocouple B
  status = tc1.getStatus();
  if(status & TC_FAULT_OPEN)
  {
    statusByte |= 0x01;
  }
  if(status & (TC_FAULT_TC_OOR | TC_FAULT_TC_TEMP_HIGH | TC_FAULT_TC_TEMP_LOW | TC_FAULT_VOLTAGE_OOR))
  {
    statusByte |= 0x02;
  }
  if(status & (TC_FAULT_CJ_OOR | TC_FAULT_CJ_TEMP_HIGH | TC_FAULT_CJ_TEMP_LOW))
  {
    statusByte |= 0x04;
  }


  // Now pack and send CAN message
  can_message_t txMsg;
  txMsg.sid = sid;
  txMsg.extended = 0;
  txMsg.rtr = 0;
  txMsg.dlc = 7;

  txMsg.data[0] = temp0x16 & 0xFF;
  txMsg.data[1] = temp0x16 >> 8;
  txMsg.data[2] = temp1x16 & 0xFF;
  txMsg.data[3] = temp1x16 >> 8;
  txMsg.data[4] = statusByte;
  txMsg.data[5] = coldJunctTempX16 & 0xFF;
  txMsg.data[6] = coldJunctTempX16 >> 8;

  if (!CAN.send(&txMsg))
  {
    Serial.println(F("Buffers full, failed CAN message"));
  }
}


void print31856Results(uint8_t channel, MAX31856 &tc)
{
  uint8_t status = tc.getStatus();

  Serial.print("Thermocouple ");
  Serial.print(channel);

  if(status)
  {
    // lots of faults possible at once, technically... handle all 8 of them
    // Faults detected can be masked, please refer to library file to enable faults you want represented
    Serial.print(F(": FAULTED - "));
    if(TC_FAULT_OPEN & status)        { Serial.print(F("OPEN, ")); }
    if(TC_FAULT_VOLTAGE_OOR & status) { Serial.print(F("Overvolt/Undervolt, ")); }
    if(TC_FAULT_TC_TEMP_LOW & status) { Serial.print(F("TC Low, ")); }
    if(TC_FAULT_TC_TEMP_HIGH & status){ Serial.print(F("TC High, ")); }
    if(TC_FAULT_CJ_TEMP_LOW & status) { Serial.print(F("CJ Low, ")); }
    if(TC_FAULT_CJ_TEMP_HIGH & status){ Serial.print(F("CJ High, ")); }
    if(TC_FAULT_TC_OOR & status)      { Serial.print(F("TC Range, ")); }
    if(TC_FAULT_CJ_OOR & status)      { Serial.print(F("CJ Range, ")); }
    Serial.println();
  }
  else  // no fault, print temperature data
  {
    Serial.println(F(": Good"));
    
    // MAX31856 External (thermocouple) Temp
    Serial.print(F("TC Temp = "));                   // print TC temp heading
    Serial.println(tc.getTemperature());
  }

  // MAX31856 Internal Temp
  Serial.print(F("Tint = "));
  float cjTemp = tc.getColdJunctionTemperature();
  if ((cjTemp > -100) && (cjTemp < 150))
  {
    Serial.println(cjTemp);
  }
  else
  {
    Serial.println(F("Unknown fault with cold junction measurement"));
  }
  Serial.println();
}