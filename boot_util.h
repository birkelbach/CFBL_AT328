/*  CANFix Bootloader - An Open Source CAN Fix Bootloader for ATMega328P 
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
 *
 * This program is designed to be loaded into the bootloader section
 * of the AVR.  This is done by reassigning the .text section to
 * the starting address of the boot section with the linker.
 */

/* This header file is used to point projects to the utility functions
 * that are contained in the bootloader. It's not used for the bootloader
 * itself but rather for programs that might want to use some of the 
 * functionality that is contained within the bootloader code.
 */

#define BOOT_START 0x3800

/* Bitrate definitions */
#define BITRATE_125  0
#define BITRATE_250  1
#define BITRATE_500  2
#define BITRATE_1000 3

struct CanFrame {
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
};

void (*init_spi)(void)                                                      = BOOT_START + 1;
void (*spi_write)(uint8_t *write_buff, uint8_t *read_buff, uint8_t size)    = BOOT_START + 2;
void (*can_init)(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3, uint8_t iflags)  = BOOT_START + 3;
void (*can_read)(uint8_t rxbuff, struct CanFrame *frame)                    = BOOT_START + 4;
void (*can_send)(uint8_t txbuff, uint8_t priority, struct CanFrame frame)   = BOOT_START + 5;
uint8_t (*can_mode)(uint8_t mode, uint8_t wait)                             = BOOT_START + 6;
uint8_t (*can_mask)(uint8_t rxbuff, uint16_t idmask, uint16_t datamask)     = BOOT_START + 7;
uint8_t (*can_filter)(uint8_t regid, uint16_t idfilter, uint16_t datafilter)= BOOT_START + 8;
