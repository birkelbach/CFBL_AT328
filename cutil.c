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
 *  This file contains the bootloader jump table and other useful assembler code 
 *  functions needed for the bootloader.
 */

#include <avr/io.h>
#include "util.h"

/* Busy loop SPI write.  Takes the contents of *write_buff
   and sends each bit out the SPI port in turn.  Receives each
   bit into *read_buff at the end of each write.  Size indicates
   how many bytes to send.  No interrupts are used it busy waits
   between writes. The CS bit is fixed in this function.  For
   systems that only talk to a single SPI slave this is acceptable,
*/
void
spi_write(uint8_t *write_buff, uint8_t *read_buff, uint8_t size)
{
    uint8_t ptr = 0;  
    /* The timer 0 delay is there to make sure that we have enough time
       with the CS/SS bit high that the MCP2515 knows that we have a new
       command.  This delay is roughly 23uSec.  To shorten the delay a
       larger number than 0x00 could be writen below at reset time. */
	while(!(TIFR0 & (1 << TOV0)));
    SPI_SS_LOW();
    while(ptr < size) {
        SPDR = write_buff[ptr];
        while( ! (SPSR & 1<<SPIF)); /* Busy wait for SPI */
        read_buff[ptr] = SPDR;
        ptr++;
    }
    SPI_SS_HIGH();
	TCNT0 = 0;          /* Reset Timer/Counter */
	TIFR0 |= (1<<TOV0); /* Reset Timer Overflow Flag */
}
