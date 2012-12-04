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
 *  This file contains definitions and function declarations for the CAN protocol
 */

#ifndef __CAN_H
#define __CAN_H

#include <avr/io.h>

//Bitrate definitions for the can_init() function
#define BITRATE_125  0
#define BITRATE_250  1
#define BITRATE_500  2
#define BITRATE_1000 3

struct CanFrame {
    uint16_t id;
	uint8_t length;
	uint8_t data[8];
};

void can_init(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3, uint8_t iflags);
uint8_t can_poll_int(void);
void can_read(uint8_t rxbuff, struct CanFrame *frame);
uint8_t can_send(uint8_t txbuff, uint8_t priority, struct CanFrame frame);
uint8_t can_mode(uint8_t mode, uint8_t wait);
void can_mask(uint8_t rxbuff, uint16_t idmask);
void can_filter(uint8_t regid, uint16_t idfilter);

#endif
