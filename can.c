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

#include "can.h"
#include "util.h"
#include "mcp2515.h"

static inline uint8_t
can_poll_int(void)
{
    uint8_t wb[2];
    uint8_t rb[2]={0,0};
    
	wb[0]=CAN_READ;
	wb[1]=CAN_CANINTF;
    spi_write(wb,rb,3);

	return rb[2];
}

void
can_read(uint8_t rxbuff, uint16_t *id, struct CanFrame *frame) 
{
    uint8_t wb[15];
    uint8_t rb[15];
    
    wb[0]=CAN_READ;
    if(rxbuff==0) {
        wb[1]=CAN_RXB0SIDH;
    } else {
        wb[1]=CAN_RXB1SIDH;
    }
    spi_write(wb,rb,15);
    
    //This is NOT the correct way to reset the flags but....
    wb[0]=CAN_WRITE;
    wb[1]=CAN_CANINTF;
    wb[2]=0x00;
    spi_write(wb,rb,3);
    

}
