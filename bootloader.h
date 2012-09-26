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
#define EE_CAN_SPEED (const uint8_t *)0x00
#define EE_NODE_ID   (const uint8_t *)0x01
#define EE_BAUD      (const uint8_t *)0x02

// Program Memory Locations
#define PGM_LENGTH 0x37FE *2
#define PGM_CRC    0x37FF *2



#endif
