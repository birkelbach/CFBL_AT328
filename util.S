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

.extern main
.extern init_can

/* This section is the bootloader function jump table.
   Starting at the start of the .text section + 2 is the 
   jmp for the first function.  This is so that code
   in the application portion of ROM can use these
   functions without worrying if they have moved within
   the bootloader */
.section .init1
    jmp     start
    jmp     int0_isr
    jmp     init_spi
    jmp     spi_write
	jmp     init_can
    jmp     can_read
    jmp     can_send
    jmp     can_mode
    jmp     can_mask
    jmp     can_filter

.section .init2
start:
    /* Initialize the Stack Pointer */
    ldi     r16,lo8(RAMEND)
    out     _SFR_IO_ADDR(SPL),r16
    ldi     r16,hi8(RAMEND)
    out     _SFR_IO_ADDR(SPH),r16
    /* Clear the zero register */
    clr     R1

.section .init9
    /* Let's get started */
    rjmp main


.section .text
.global start_app
start_app:
    /* Reset the interrupt vector table to start at 0x0000
	   and clear interrupts.  Then jump to 0x0000. */
	cli
    ldi r16, (1<<IVCE)
	out _SFR_IO_ADDR(MCUCR), r16 
    ldi r16, 0
	out _SFR_IO_ADDR(MCUCR), r16 
	/* Jump to the start of the program */
    jmp 0x0000
