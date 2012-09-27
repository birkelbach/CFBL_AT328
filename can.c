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
 *  This file contains the source code for the CANBus interface functions
 */

#include <string.h>
#include "can.h"
#include "util.h"
#include "mcp2515.h"

/* Sets up the MCP2515 chip.  
   Sets the CNFx registers according to the arguments.
   Turns on all Rx and Tx interrupts.
   Receive Standard frames only.
   Put the chip in 'Normal' mode.

   This function is exported with the jump table.  See
   boot_util.h and util.S.
*/
void
can_init(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3, uint8_t iflags)
{
    uint8_t wb[8];
    uint8_t rb[8];

    /* Reset the MCP2515 */
    wb[0]=CAN_RESET;
    spi_write(wb,rb,1);
    
    /* Sets up the Bit timing and interrupts in the MCP2515 */
    wb[0]=CAN_WRITE;
    wb[1]=CAN_CNF3;
    wb[2]=cnf3;
    wb[3]=cnf2;
    wb[4]=cnf1;
    wb[5]=iflags;
	spi_write(wb,rb,6);

    wb[1]=CAN_RXB0CTRL;
	wb[2]=0x20; /* Set to Rx Standard Frames only - RXB0CTRL */
	wb[3]=0x20; /* Set to Rx Standard Frames only - RXB1CTRL */
	spi_write(wb,rb,4);

    /* Put the chip in Normal Mode */
    can_mode(CAN_MODE_NORMAL, 0);
}


/* This reads the interrupt flags from the MCP2515 */
uint8_t
can_poll_int(void)
{
    uint8_t wb[3];
    uint8_t rb[3];
    
	wb[0]=CAN_READ;
	wb[1]=CAN_CANINTF;
    spi_write(wb,rb,3);

	return rb[2];
}

/* Read the data out of the given buffer and reset the interrupt flag 
   associated with that buffer. rxbuff is the buffer that we want to
   read.  It can be 0 or 1. */
void
can_read(uint8_t rxbuff, struct CanFrame *frame) 
{
    uint8_t wb[15];
    uint8_t rb[15];
    uint8_t mask;  /* This is for the bit modify to restet the interrupt flag */
    
    wb[0]=CAN_READ;
    if(rxbuff==0) {
        wb[1]=CAN_RXB0SIDH;
        mask = 1<<CAN_RX0IF;
    } else {
        wb[1]=CAN_RXB1SIDH;
        mask = 1<<CAN_RX1IF;
    }
    spi_write(wb,rb,15);
    frame->id =  rb[2]<<3;
    frame->id |= rb[3]>>5;
    frame->length = rb[6];
    memcpy(frame->data, &rb[7], frame->length);

    /* Reset the interrupt flag with Bit Modify Command */
    wb[0]=CAN_BIT_MODIFY;
    wb[1]=CAN_CANINTF;
    wb[2]=mask;
    wb[3]=0x00;
    spi_write(wb,rb,4);
}

/* Send a CAN frame using the transmit buffer given by txbuff 
   txbuff can be 0, 1 or 2.  Any other values and bad things
   will happen. Returns 0 on success and 1 if the TXREQ flag
   is still set from a previous transmission. */
uint8_t
can_send(uint8_t txbuff, uint8_t priority, struct CanFrame frame)
{
    uint8_t wb[16];
    uint8_t rb[16];

    /* First we read the TXREQ flag from the given TX buffer */
    wb[0] = CAN_READ;
    wb[1] = (txbuff + 3) << 4; /* Calculates the TXBxCTRL register address */
    spi_write(wb,rb,3);
    if(rb[2] & (1 << CAN_TXREQ)) return 1;

    wb[0] = CAN_WRITE;
    wb[1] = (txbuff + 3) << 4; /* Calculates the TXBxCTRL register address */
    wb[2] = priority & 0x03;   /* Lower two bits of the priority */
    wb[3] = frame.id >> 3;  /* TXBxSIDH */
    wb[4] = frame.id << 5;  /* TXBxSIDL */
    wb[5] = 0x00;           /* TXBxEID8 */
    wb[6] = 0x00;           /* TXBxEID0 */
    wb[7] = frame.length;
    memcpy(&wb[8], frame.data, frame.length);
    spi_write(wb, rb, 8 + frame.length);

    /* Set RTS */
    wb[0] = CAN_RTS | (1<<txbuff);
    spi_write(wb, rb, 1);

    return 0;
}

/* Put the MCP2515 into the given mode. If mode = CAN_MODE_QUERY 
   the current mode of the chip will be read and returned. If
   wait is set then the function will continuously poll the 
   chip until it's in the proper mode and the mode will be returned. */
uint8_t
can_mode(uint8_t mode, uint8_t wait)
{
    uint8_t wb[4];
    uint8_t rb[4];
    if(mode != CAN_MODE_QUERY) {
        wb[0]=CAN_BIT_MODIFY;
        wb[1]=CAN_CANCTRL;
        wb[2]=CAN_MODE_MASK;
        wb[3]=mode;
        spi_write(wb,rb,4);
    } else { /* Query the CAN Mode */
	    wb[0]=CAN_READ;
		wb[1]=CAN_CANSTAT;
		spi_write(wb,rb,3);
		return rb[2] & CAN_MODE_MASK;
    }
	/* TODO: Add waiting for mode logic */
    return 0;
}

/* Set the acceptance mask for a given Rx Buffer. */
void
can_mask(uint8_t rxbuff, uint16_t idmask)
{
    uint8_t wb[6];
    uint8_t rb[6];
    
    wb[0]=CAN_WRITE;
    if(rxbuff==0) {
        wb[1]=CAN_RXM0SIDH;
    } else {
        wb[1]=CAN_RXM1SIDH;
    }
	wb[2] = idmask >> 3;  /* RXMxSIDH */
    wb[3] = idmask << 5;  /* RXMxSIDL */
    spi_write(wb,rb,4);
}

/* Set the acceptance filter.  There are six filters and they can be
   selected here with regid.  The first two (0 and 1) are for RX0 and
   2-5 are for RX1. See the MCP2515 datasheet for details. regid is 
   assumed to be the address in the  MCP2515 for the filter. 
   see mcp2515.h */
void
can_filter(uint8_t regid, uint16_t idfilter)
{
    uint8_t wb[4];
    uint8_t rb[4];
	
    wb[0] = CAN_WRITE;
    wb[1] = regid;
    wb[2] = idfilter >> 3;       /* RXFxSIDH */
    wb[3] = idfilter << 5;       /* RXFxSIDL */
    spi_write(wb,rb,4);
}
