/***************************************************************************
* File Name: PWFusion_MCP2515_Registers.h
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
* J. Leonard      2014Sep09   Original version
* J. Leonard      2022Jan13   Updated to Arduino library folder structure
*
* Playing With Fusion, Inc. invests time and resources developing open-source
* code. Please support Playing With Fusion and continued open-source
* development by buying products from Playing With Fusion!
*
* **************************************************************************/

#ifndef __PWFUSION_MCP2515_REGISTERS_H
#define __PWFUSION_MCP2515_REGISTERS_H


// TXBnCTRL **********************
#define TXB0CTRL              0x30
#define TXB1CTRL              0x50
#define TXB2CTRL              0x40

#define TXBnCTRL_ABTF         0x40
#define TXBnCTRL_MLOA         0x20
#define TXBnCTRL_TXERR        0x10
#define TXBnCTRL_TXREQ        0x08

#define TXBnCTRL_TXP_MASK     0x03


// TXRTSCTRL *********************
#define TXRTSCTRL             0x0D

#define TXRTSCTRL_B2RTS       0x20
#define TXRTSCTRL_B1RTS       0x10
#define TXRTSCTRL_B0RTS       0x08
#define TXRTSCTRL_B2RTSM      0x04
#define TXRTSCTRL_B1RTSM      0x02
#define TXRTSCTRL_B0RTSM      0x01


// TXBnSIDH **********************
#define TXB0SIDH              0x31
#define TXB1SIDH              0x41
#define TXB2SIDH              0x51


// TXBnSIDL **********************
#define TXB0SIDL              0x32
#define TXB1SIDL              0x42
#define TXB2SIDL              0x52

#define TXBnSIDL_EXIDE        0x08


// TXBnEID8 **********************
#define TXB0EID8              0x33
#define TXB1EID8              0x43
#define TXB2EID8              0x53


// TXBnEID0 **********************
#define TXB0EID0              0x34
#define TXB1EID0              0x44
#define TXB2EID0              0x54


// TXBnDLC ***********************
#define TXB0DLC               0x35
#define TXB1DLC               0x45
#define TXB2DLC               0x55

#define TXBnDLC_RTR           0x40
#define TXBnDLC_DLC_MASK      0x0F


// TXBnDx ************************
#define TXB0D0                0x36
#define TXB1D0                0x46
#define TXB2D0                0x56


// RXB0CTRL **********************
#define RXB0CTRL              0x60

#define RXB0CTRL_RXM_EXT      0x40
#define RXB0CTRL_RXM_STD      0x20
#define RXB0CTRL_RXRTR        0x08
#define RXB0CTRL_BUKT         0x04
#define RXB0CTRL_BUKT1        0x02
#define RXB0CTRL_FILHIT0      0x01


// RXB1CTRL **********************
#define RXB1CTRL              0x70

#define RXB1CTRL_RXM_EXT      0x40
#define RXB1CTRL_RXM_STD      0x20
#define RXB1CTRL_RXRTR        0x08
#define RXB1CTRL_FILHIT_MASK  0x07


// BFPCTRL ***********************
#define BFPCTRL               0x0C

#define BFPCTRL_B1BFS         0x20
#define BFPCTRL_B0BFS         0x10
#define BFPCTRL_B1BFE         0x08
#define BFPCTRL_B0BFE         0x04
#define BFPCTRL_B1BFM         0x02
#define BFPCTRL_B0BFM         0x01


// RXBnSIDH **********************
#define RXB0SIDH              0x61
#define RXB1SIDH              0x71


// RXBnSIDL **********************
#define RXB0SIDL              0x62
#define RXB1SIDL              0x72

#define RXBnSIDL_SRR          0x10
#define RXBnSIDL_IDE          0x08


// RXBnEID8 **********************
#define RXB0EID8              0x63
#define RXB1EID8              0x73


// RXBnEID0 **********************
#define RXB0EID0              0x64
#define RXB1EID0              0x74


// RXBnDLC ***********************
#define RXB0DLC               0x65
#define RXB1DLC               0x75

#define RXBnDLC_RTR           0x40
#define RXBnDLC_DLC_MASK      0x0F


// RXBnDM ************************
#define RXB0DM                0x66
#define RXB1DM                0x76


// CNF1 **************************
#define CNF1                  0x2A

#define CNF1_SJW_1            0x00
#define CNF1_SJW_2            0x40
#define CNF1_SJW_3            0x80
#define CNF1_SJW_4            0xC0
#define CNF1_BRP_MASK         0x3F


// CNF2 **************************
#define CNF2                  0x29

#define CNF2_BLTMODE          0x80
#define CNF2_SAM              0x40
#define CNF2_PHSEG1_MASK      0x38
#define CNF2_PHSEG1_SHIFT     0x03
#define CNF2_PRSEG_MASK       0x07


// CNF3 **************************
#define CNF3                  0x28

#define CNF3_SOF              0x80
#define CNF3_WAKFIL           0x40
#define CNF3_PHSEG2_MASK      0x07


// TEC ***************************
#define TEC                   0x1C


// CNF3 **************************
#define REC                   0x1D


// EFLG **************************
#define EFLG                  0x2D

#define EFLG_RX1VR            0x80
#define EFLG_RXOVR            0x40
#define EFLG_TXBO             0x20
#define EFLG_TXEP             0x10
#define EFLG_RXEP             0x08
#define EFLG_TXWAR            0x04
#define EFLG_RXWAR            0x02
#define EFLG_EWARN            0x01


// CANINTE ***********************
#define CANINTE               0x2B

#define CANINTE_MERRE         0x80
#define CANINTE_WAKIE         0x40
#define CANINTE_ERRIE         0x20
#define CANINTE_TX2IE         0x10
#define CANINTE_TX1IE         0x08
#define CANINTE_TX0IE         0x04
#define CANINTE_RX1IE         0x02
#define CANINTE_RX0IE         0x01


// CANINTF ***********************
#define CANINTF               0x2C

#define CANINTF_MERRF         0x80
#define CANINTF_WAKIF         0x40
#define CANINTF_ERRIF         0x20
#define CANINTF_TX2IF         0x10
#define CANINTF_TX1IF         0x08
#define CANINTF_TX0IF         0x04
#define CANINTF_RX1IF         0x02
#define CANINTF_RX0IF         0x01


// CANCTRL ***********************
#define CANCTRL               0x0F

#define CANCTRL_REQOP_NORMAL  0x00
#define CANCTRL_REQOP_SLEEP   0x20
#define CANCTRL_REQOP_LPBACK  0x40
#define CANCTRL_REQOP_LISTEN  0x60
#define CANCTRL_REQOP_CONFIG  0x80
#define CANCTRL_ABAT          0x10
#define CANCTRL_OSM           0x08
#define CANCTRL_CLKEN         0x04
#define CANCTRL_CLKPRE_MASK   0x03


// CANSTAT ***********************
#define CANSTAT               0x0E

#define CANSTAT_OPMODE        0xE0
#define CANSTAT_ICOD          0x0E



// SPI Instructions *********
#define MCP_RESET        0xC0
#define MCP_READ         0x03
#define MCP_WRITE        0x02
#define MCP_RTS_0        0x81
#define MCP_RTS_1        0x82
#define MCP_RTS_2        0x84
#define MCP_READ_STATUS  0xA0
#define MCP_RX_STATUS    0xB0
#define MCP_BIT_MODIFY   0x05

#endif // __PWFUSION_MCP2515_REGISTERS_H
