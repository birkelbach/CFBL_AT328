/*  CANFix - An Open Source CANBus based Flight Information Protocol
 *  Copyright (c) 2012 Phil Birkelbach
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * This header contains definitions and macros for using
 * the MCP 2515 CAN controller IC. */

#ifndef MCP2515_H
#define MCP2515_H 1

//Port Pin
#define CAN_INT_PORT PORTD
#define CAN_INT_DDR  DDRD
#define CAN_INT_PIN  PD2


//Registers
#define CAN_RXF0SIDH 0x00
#define CAN_RXF0SIDL 0x01
#define CAN_RXF0EID8 0x02
#define CAN_RXF0EID0 0x03
#define CAN_RXF1SIDH 0x04
#define CAN_RXF1SIDL 0x05
#define CAN_RXF1EID8 0x06
#define CAN_RXF1EID0 0x07
#define CAN_RXF2SIDH 0x08
#define CAN_RXF2SIDL 0x09
#define CAN_RXF2EID8 0x0A
#define CAN_RXF2EID0 0x0B
#define CAN_BFPCTRL 0x0C
#define CAN_TXRTSCTRL 0x0D
#define CAN_CANSTAT 0x0E
#define CAN_CANCTRL 0x0F
#define CAN_RXF3SIDH 0x10
#define CAN_RXF3SIDL 0x11
#define CAN_RXF3EID8 0x12
#define CAN_RXF3EID0 0x13
#define CAN_RXF4SIDH 0x14
#define CAN_RXF4SIDL 0x15
#define CAN_RXF4EID8 0x16
#define CAN_RXF4EID0 0x17
#define CAN_RXF5SIDH 0x18
#define CAN_RXF5SIDL 0x19
#define CAN_RXF5EID8 0x1A
#define CAN_RXF5EID0 0x1B
#define CAN_TEC 0x1C
#define CAN_REC 0x1D
#define CAN_RXM0SIDH 0x20
#define CAN_RXM0SIDL 0x21
#define CAN_RXM0EID8 0x22
#define CAN_RXM0EID0 0x23
#define CAN_RXM1SIDH 0x24
#define CAN_RXM1SIDL 0x25
#define CAN_RXM1EID8 0x26
#define CAN_RXM1EID0 0x27
#define CAN_CNF3 0x28
#define CAN_CNF2 0x29
#define CAN_CNF1 0x2A
#define CAN_CANINTE 0x2B
	#define CAN_MERRE 7
	#define CAN_WAKIE 6
	#define CAN_ERRIE 5
	#define CAN_TX2IE 4
	#define CAN_TX1IE 3
	#define CAN_TX0IE 2
	#define CAN_RX1IE 1
	#define CAN_RX0IE 0
#define CAN_CANINTF 0x2C
	#define CAN_MERRF 7
	#define CAN_WAKIF 6
	#define CAN_ERRIF 5
	#define CAN_TX2IF 4
	#define CAN_TX1IF 3
	#define CAN_TX0IF 2
	#define CAN_RX1IF 1
	#define CAN_RX0IF 0
#define CAN_EFLG 0x2D
#define CAN_TXB0CTRL 0x30
	#define CAN_TXREQ 3
#define CAN_TXB0SIDH 0x31
#define CAN_TXB0SIDL 0x32
	#define CAN_EXIDE 3
#define CAN_TXB0EID8 0x33
#define CAN_TXB0EID0 0x34
#define CAN_TXB0DLC 0x35
#define CAN_TXB0D0 0x36 

#define CAN_RXB0CTRL 0x60
	#define CAN_RXM1 6
	#define CAN_RXM0 5
	#define CAN_RXRTR 3
	// Bits 2:0 FILHIT2:0
#define CAN_RXB0SIDH 0x61
#define CAN_RXB0SIDL 0x62
#define CAN_RXB0EID8 0x63
#define CAN_RXB0EID0 0x64
#define CAN_RXB0DLC  0x65
#define CAN_RXB0D0   0x66 
#define CAN_RXB1SIDH 0x71
#define CAN_RXB1SIDL 0x72
#define CAN_RXB1EID8 0x73
#define CAN_RXB1EID0 0x74
#define CAN_RXB1DLC  0x75
#define CAN_RXB1D0   0x76 

//Command Bytes
#define CAN_RESET 0xC0
#define CAN_READ 0x03
#define CAN_READ_RX_BUFFER_0 0x90
#define CAN_READ_RX_BUFFER_1 0x94
#define CAN_WRITE 0x02
#define CAN_LOAD_TX_BUFFER 0x40
#define CAN_RTS0 0x81
#define CAN_RTS1 0x82
#define CAN_RTS2 0x84
#define CAN_READ_STATUS 0xA0
#define CAN_RX_STATUS 0xB0
#define CAN_BIT_MODIFY 0x05




#endif
