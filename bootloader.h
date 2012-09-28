/*  CANFix Bootloader - An Open Source CAN Fix Bootloader for ATMega328P 
 *  Copyright (c) 2011 Phil Birkelbach
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
 */
#ifndef _BOOTLOADER_H
#define _BOOTLOADER_H


// EEPROM Data Locations 
#define EE_CAN_SPEED  (const uint8_t *)0x00
#define EE_NODE_ID    (const uint8_t *)0x01
#define EE_BAUD       (const uint8_t *)0x02

// Verification Code for Firmware Update
#define BL_VERIFY_LSB    0x01
#define BL_VERIFY_MSB    0xf7

// Comment this out to make all the UART debugging stuff go away.
#define UART_DEBUG 0x01

#define PGM_PAGE_SIZE 128 /* Page size in Bytes */
#define PGM_LAST_PAGE_START (0x3FC0U * 2) /* Starting address of the last page of flash */
#define PGM_LENGTH (const uint16_t *)0x7FFC
#define PGM_CRC    (const uint16_t *)0x7FFE


#endif
