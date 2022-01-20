/***************************************************************************
* File Name: PWFusion_MCP2515.cpp
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
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* **************************************************************************
* REVISION HISTORY:
* Author          Date        Comments
* J. Leonard      2014Nov10   Original version
* J. Leonard      2022Jan13   Updated to Arduino library folder structure.  Removed
*                             MCP2515Class instantiation from library.  Renamed 
*                             attach() to begin().  Added support for alternative
*                             SPI ports
*
* Playing With Fusion, Inc. invests time and resources developing open-source
* code. Please support Playing With Fusion and continued open-source
* development by buying products from Playing With Fusion!
*
* **************************************************************************/
#include <stdint.h>
#include <string.h>
#include "PWFusion_MCP2515.h"
#include "PWFusion_MCP2515_Registers.h"


// SPI bus clock speed
#define SPI_CLOCK    8000000

// CAN bus bit rate calculation
// Fosc = 16 MHz
// TQ = 2 * BRP / FOSC
#define MCP2515_BRP_1000        1
#define MCP2515_BRP_500         1
#define MCP2515_BRP_250         2
#define MCP2515_BRP_125         4
#define MCP2515_BRP_100         5

// 100-500kbit, 75% Sample point
// Tbit = TQ * (SYNC_TIME + PROPSEG_TIME + PHASE1SEQ_TIME + PHASE2SEQ_TIME)
#define MCP2515_SYNC_TIME       1
#define MCP2515_PROPSEG_TIME    3
#define MCP2515_PHASE1SEQ_TIME  8
#define MCP2515_PHASE2SEQ_TIME  4

// 1Mbit, 75% sample point
#define MCP2515_1M_PROPSEG_TIME    2
#define MCP2515_1M_PHASE1SEQ_TIME  3
#define MCP2515_1M_PHASE2SEQ_TIME  2

#define NEXT_RX_INDEX(x)      ((x+1>=CAN_RX_QUEUE_LENGTH)?0:x+1)


// Define interrupt handlers
void MCP2515_RxInt0()
{
   if (MCP2515::instances[0])
   {
      MCP2515::instances[0]->handleInterrupt();
   }
}

void MCP2515_RxInt1()
{
   if (MCP2515::instances[1])
   {
      MCP2515::instances[1]->handleInterrupt();
   }
}

void MCP2515_RxInt2()
{
   if (MCP2515::instances[2])
   {
      MCP2515::instances[2]->handleInterrupt();
   }
}

void MCP2515_RxInt3()
{
   if (MCP2515::instances[3])
   {
      MCP2515::instances[3]->handleInterrupt();
   }
}

MCP2515 *MCP2515::instances[MAX_INSTANCES] = {NULL, NULL, NULL, NULL};
uint8_t MCP2515::interrupts[MAX_INSTANCES] = {0xFF, 0xFF, 0xFF, 0xFF};
uint8_t MCP2515::getInstanceIdx(uint8_t interrupt)
{
  int i;

   for (i=0; i<MAX_INSTANCES; i++)
   {
      // If the slot in the interrupts array is empty (0xFF) or if it is 
      // already mapped to the given interrupt number
      if ((interrupts[i] == 0xFF) || (interrupts[i] == interrupt))
      {
         interrupts[i] = interrupt;
         instances[i] = this;
         break;
      }
   }

   return i;
}
      
      
MCP2515::MCP2515() :
   _spiSettings(10000000, MSBFIRST, SPI_MODE0)
{
}


void MCP2515::begin(uint8_t chipSelectPin, uint8_t interruptPin, uint8_t loopback, uint16_t bitRate, SPIClass &spiPort)
{
   chipSelect = chipSelectPin;
   _spiPort = &spiPort;

   uint8_t interrupt = digitalPinToInterrupt(interruptPin);
   
   //txBufStatus is a bitmap of avaliable MCP2515 transmit buffers.  It reuses the bits from CININTF for simplicity
   txBufStatus = CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF;
   
   //Determine BRP based on the desired CAN bitrate
   uint8_t brp;
   switch (bitRate)
   {
     case 1000:
       brp = MCP2515_BRP_1000;
       break;

     case 500:
       brp = MCP2515_BRP_500;
       break;
       
     case 250:
       brp = MCP2515_BRP_250;
       break;
       
     case 125:
       brp = MCP2515_BRP_125;
       break;
       
     case 100:
     default:
       brp = MCP2515_BRP_100;
       break;
   }

   // Initalize the chip select pin
   pinMode(chipSelect, OUTPUT);
   pinMode(interruptPin, INPUT);

   // Tell SPI driver it will be accesed during an ISR
   _spiPort->usingInterrupt(interrupt);

   // Initialize MCP2515
   _spiPort->begin();          
   _spiPort->beginTransaction(_spiSettings);

   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_RESET);
   digitalWrite(chipSelect, HIGH);

   // Set CNF1, CNF2, and CNF3 registers
   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_WRITE);
   _spiPort->transfer(CNF3);
   if (bitRate == 1000)
   {
     _spiPort->transfer((MCP2515_1M_PHASE2SEQ_TIME-1) & CNF3_PHSEG2_MASK); //CNF3 value
     _spiPort->transfer(CNF2_BLTMODE | CNF2_SAM | 
                        (((MCP2515_1M_PHASE1SEQ_TIME-1)<<CNF2_PHSEG1_SHIFT)&CNF2_PHSEG1_MASK) | 
                        ((MCP2515_1M_PROPSEG_TIME-1) & CNF2_PRSEG_MASK)); //CNF2 value
   }
   else
   {
     _spiPort->transfer((MCP2515_PHASE2SEQ_TIME-1) & CNF3_PHSEG2_MASK); //CNF3 value
     _spiPort->transfer(CNF2_BLTMODE | CNF2_SAM | 
                        (((MCP2515_PHASE1SEQ_TIME-1)<<CNF2_PHSEG1_SHIFT)&CNF2_PHSEG1_MASK) | 
                        ((MCP2515_PROPSEG_TIME-1) & CNF2_PRSEG_MASK)); //CNF2 value
   }
   _spiPort->transfer( CNF1_SJW_1 | ((brp-1)>>1) ); //CNF1 value
   digitalWrite(chipSelect, HIGH);

   // Disable Receive Masks
   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_WRITE);
   _spiPort->transfer(RXB0CTRL);
   _spiPort->transfer(RXB0CTRL_RXM_EXT | RXB0CTRL_RXM_STD | RXB0CTRL_BUKT);   
   _spiPort->transfer(RXB1CTRL_RXM_EXT | RXB1CTRL_RXM_STD);   
   digitalWrite(chipSelect, HIGH); 

   // Enable MCP2515 interrupts
   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_WRITE);
   _spiPort->transfer(CANINTE);
   _spiPort->transfer(CANINTE_RX1IE | CANINTE_RX0IE | CANINTE_TX0IE | CANINTE_TX1IE | CANINTE_TX2IE);   
   digitalWrite(chipSelect, HIGH);   

   // Enter loopback/normal mode
   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_WRITE);
   _spiPort->transfer(CANCTRL);
   _spiPort->transfer(loopback ? CANCTRL_REQOP_LPBACK : CANCTRL_REQOP_NORMAL);
   digitalWrite(chipSelect, HIGH);

   _spiPort->endTransaction(); 
   
   // Interrupts may only call static functions.   Work a little magic to map this
   // class instance to a single static interrupt handler function
   uint8_t instanceIdx = getInstanceIdx(interrupt);
   if (instanceIdx == 0)
   {
      attachInterrupt(interrupt,MCP2515_RxInt0,LOW);
   }
   else if (instanceIdx == 1)
   {
      attachInterrupt(interrupt,MCP2515_RxInt1,LOW);
   }
   else if (instanceIdx == 2)
   {
      attachInterrupt(interrupt,MCP2515_RxInt2,LOW);
   }
   else if (instanceIdx == 3)
   {
      attachInterrupt(interrupt,MCP2515_RxInt3,LOW);
   }
}


void MCP2515::handleInterrupt(void)
{
   uint8_t canIntF;

   _spiPort->beginTransaction(_spiSettings);
   
   //Read pending interrupt flags
   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_READ);
   _spiPort->transfer(CANINTF);
   canIntF = _spiPort->transfer(0);
   digitalWrite(chipSelect, HIGH);

   if (canIntF & CANINTF_RX0IF)
   {
      this->queueRxMessage(0);
   }

   if (canIntF & CANINTF_RX1IF)
   {
      this->queueRxMessage(1);
   }
   
   // Update free tx buffers
   txBufStatus |= canIntF & (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF);
   

   // Clear handled interrupts by using bit-set instruction
   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_BIT_MODIFY);
   _spiPort->transfer(CANINTF);
   _spiPort->transfer(canIntF);
   _spiPort->transfer(0);
   digitalWrite(chipSelect, HIGH);

   _spiPort->endTransaction();
}


void MCP2515::queueRxMessage(uint8_t channel)
{
   uint8_t sidh, sidl, eid8, eid0, dlc;
   uint8_t nextHead;
   int i;

   digitalWrite(chipSelect, LOW);
   _spiPort->transfer(MCP_READ);
   _spiPort->transfer(channel ? RXB1SIDH : RXB0SIDH);
   sidh = _spiPort->transfer(0);
   sidl = _spiPort->transfer(0);
   eid8 = _spiPort->transfer(0);
   eid0 = _spiPort->transfer(0);
   dlc  = _spiPort->transfer(0);
   
   if (sidl & RXBnSIDL_IDE)
   {
     // Extended Identifier
     this->rxBuffer[this->rxHead].sid = sidh;
     this->rxBuffer[this->rxHead].sid = (uint32_t)(rxBuffer[this->rxHead].sid << 3) | ((sidl >> 5) & 0x07);
     this->rxBuffer[this->rxHead].sid = (uint32_t)(rxBuffer[this->rxHead].sid << 2) | (sidl & 0x03);     
     this->rxBuffer[this->rxHead].sid = (uint32_t)(rxBuffer[this->rxHead].sid << 8) | (uint32_t)eid8;  
     this->rxBuffer[this->rxHead].sid = (uint32_t)(rxBuffer[this->rxHead].sid << 8) | (uint32_t)eid0;
     
     this->rxBuffer[this->rxHead].extended = 1;
     this->rxBuffer[this->rxHead].rtr = (dlc & RXBnDLC_RTR) ? 1 : 0;
   }
   else
   {
     // Standard Identifier
     this->rxBuffer[this->rxHead].sid = sidh;
     this->rxBuffer[this->rxHead].sid = (rxBuffer[this->rxHead].sid << 3) | (sidl >> 5);     

     this->rxBuffer[this->rxHead].extended = 0;     
     this->rxBuffer[this->rxHead].rtr = (sidl & RXBnSIDL_SRR) ? 1 : 0;
   }
   
   this->rxBuffer[this->rxHead].dlc = dlc & RXBnDLC_DLC_MASK;
   
   for (i=0; (i<8) && (i<rxBuffer[this->rxHead].dlc); i++)
   {
     this->rxBuffer[this->rxHead].data[i] = _spiPort->transfer(0);
   }   
   
   digitalWrite(chipSelect, HIGH);
   
   nextHead = NEXT_RX_INDEX(this->rxHead);
   if (nextHead != this->rxTail)
   {
      this->rxHead = nextHead;
   }
}


uint8_t MCP2515::receive(can_message_t *pMsg)
{
   uint8_t receivedMessage = 0;

   if (pMsg && (this->rxTail != this->rxHead))
   {
      receivedMessage = 1;

      (void)memcpy(pMsg, &(this->rxBuffer[this->rxTail]), sizeof(can_message_t));

            // Advance tail to next message in the queue
      this->rxTail = NEXT_RX_INDEX(this->rxTail);
   }

   return receivedMessage;
}


uint8_t MCP2515::send(can_message_t *pMsg)
{
   int i;
   uint8_t txBuffer;
   uint8_t success = 0;
  noInterrupts();
   if (txBufStatus)
   {
      success = 1;
      
      //Find an empty buffer
      if (txBufStatus & CANINTF_TX0IF)
      {
        txBufStatus &= ~CANINTF_TX0IF;
        txBuffer = TXB0SIDH;
      }
      else if (txBufStatus & CANINTF_TX1IF)
      {
        txBufStatus &= ~CANINTF_TX1IF;
        txBuffer = TXB1SIDH;
      }
      else //if (txBufStatus & CANINTF_TX2IF)
      {
        txBufStatus &= ~CANINTF_TX2IF;
        txBuffer = TXB2SIDH;
      }
      
     _spiPort->beginTransaction(_spiSettings);  
     digitalWrite(chipSelect, LOW);
     _spiPort->transfer(MCP_WRITE);
  
     _spiPort->transfer(txBuffer);
  
     if (pMsg->extended)
     {
        _spiPort->transfer( (pMsg->sid >> 21) & 0xFF );
        _spiPort->transfer( ((pMsg->sid >> 13) & 0xE0) | TXBnSIDL_EXIDE | ((pMsg->sid >> 16) & 0x03) );
        _spiPort->transfer( (pMsg->sid >> 8) & 0xFF );
        _spiPort->transfer( pMsg->sid & 0xFF );
     }
     else
     {
       _spiPort->transfer( (pMsg->sid >> 3) & 0xFF );
       _spiPort->transfer( (pMsg->sid & 0x07) << 5 );
       _spiPort->transfer( 0 );
       _spiPort->transfer( 0 );
     }
  
     _spiPort->transfer((pMsg->rtr ? TXBnDLC_RTR : 0) | (pMsg->dlc & TXBnDLC_DLC_MASK));
  
     if(!pMsg->rtr)
     {
       for(i=0; i<pMsg->dlc & i<8; i++)
       {
         _spiPort->transfer(pMsg->data[i]);
       }
     }
     digitalWrite(chipSelect, HIGH);
  
     digitalWrite(chipSelect, LOW);
     if (txBuffer == TXB0SIDH)
     {
        _spiPort->transfer(MCP_RTS_0);
     }
     else if (txBuffer == TXB1SIDH)
     {
        _spiPort->transfer(MCP_RTS_1);
     }
     else
     {
        _spiPort->transfer(MCP_RTS_2);
     }
     digitalWrite(chipSelect, HIGH);
     _spiPort->endTransaction();
   }
   
   interrupts();
   
   return success;
}
