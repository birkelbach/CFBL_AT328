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

/* Send a CAN frame using the transmit buffer given by txbuff */
void
can_send(uint8_t txbuff, struct CanFrame frame)
{

}

/* Put the MCP2515 into the given mode. If mode = CAN_MODE_QUERY 
   the current mode of the chip will be read and returned. If
   wait is set then the function will continuously poll the 
   chip until it's in the proper mode and the mode will be returned. */
uint8_t
can_mode(uint8_t mode, uint8_t wait)
{
    uint8_t wb[6];
    uint8_t rb[6];
    if(mode != CAN_MODE_QUERY) {
        wb[0]=CAN_BIT_MODIFY;
        wb[1]=CAN_CANCTRL;
        wb[2]=CAN_MODE_MASK;
        wb[3]=mode;
        spi_write(wb,rb,4);
    }
    return 0;
}

/* Set the acceptance mask for a given Rx Buffer.  Since we are only
   using standard frames we use the extended frame mask as a data mask.
   See the MCP2515 Datasheet for details. */
uint8_t
can_mask(uint8_t rxbuff, uint16_t idmask, uint16_t datamask)
{
    uint8_t wb[15];
    uint8_t rb[15];
    // TODO: Need to check the mode and return error if not in config mode
    wb[0]=CAN_WRITE;
    if(rxbuff==0) {
        wb[1]=CAN_RXM0SIDH;
    } else {
        wb[1]=CAN_RXM1SIDH;
    }
    wb[3] =0; 
    wb[4] =0; 
    wb[5] =0;
    spi_write(wb,rb,6);
    return 0;
}

/* Set the acceptance filter.  There are six filters and they can be
   selected here with regid.  The first two (0 and 1) are for RX0 and
   2-5 are for RX1. Since we are only interested in standard frames
   we use the extended frame filters as a data filter.  See the MCP2515
   datasheet for details. */
uint8_t
can_filter(uint8_t regid, uint16_t idfilter, uint16_t datafilter)
{
    uint8_t wb[6];
    uint8_t rb[6];
    // TODO: Need to chck the mode and return error if not in config mode
    wb[0]=CAN_WRITE;
    if(regid==0) {
        wb[1]=CAN_RXM0SIDH;
    } else {
        wb[1]=CAN_RXM1SIDH;
    }
    wb[2] =0; 
    wb[3] =0; 
    wb[4] =0;
    wb[5] =0;
    spi_write(wb,rb,6);
    return 0;
}
