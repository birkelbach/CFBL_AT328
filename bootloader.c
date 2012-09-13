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
 *
 * This program is designed to be loaded into the bootloader section
 * of the AVR.  This is done by reassigning the .text section to
 * the starting address of the boot section with the linker.
 *
 * The .bootloader section is not used because it causes too many
 * problems with where the linker puts functions.  Reassigning the
 * .text section works much better.  It is therefore unwise to try
 * and combine this code with any application code.
 */
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "mcp2515.h"
#include <util/delay_basic.h>
#include "bootloader.h"
#include "can.h"
#include "util.h"
//#include <stdio.h> /* Debug only!!! */


static inline void init(void);
void spi_write(uint8_t *write_buff, uint8_t *read_buff, uint8_t size);
void init_can(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);

/* Global Variables */
uint8_t can_iflag;


void
int0_isr(void) {
    can_iflag = 1;
	EIFR = 0x00;
	reti();
}

/* This is a busy wait UART send function.
   It's mainly for debugging */
void
uart_write(char *write_buff, int size)
{
    int ptr=0;
    while(ptr < size) {
        while(!(UCSR0A & (1<<UDRE0)));
        UDR0 = write_buff[ptr];
        ptr++;
    }
}

/* Initialize the Serial port to 9600,8,N,1
   Used for debugging should be removed for
   production unless there is enough space
   to leave it for application use in the BL
   section of the program.
*/
void
init_serial(void)
{
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); /* Set 1 Stop bit no parity */
    /* 9600 BAUD @ 11.0592 MHz */
    UBRR0H = 0;
    UBRR0L = 71;
}

/* Sets the port pins to the proper directions and initializes
   the registers for the SPI port */
void
init_spi()
{
    unsigned char x;

    /* Set MOSI, SCK and SS output, all others input */
    SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);
    SPI_PORT |= (1<<SPI_SS); /* Set the SS pin high to disable slave */
    /* Enable SPI, Master, set clock rate fck/16 */
    SPCR |= (1<<SPE)|(1<<MSTR);
    SPCR |= (1<<SPR0)|(0<<SPIE);
    /* This should clear any lingering interrupts? The book
     * says to do this. */
    x = SPSR;
    x = SPDR;
}




/* Sets up the MCP2515 chip.  
   Sets the CNFx registers according to the arguments.
   Turns on all Rx and Tx interrupts.
   Receive Standard frames only.
   Put the chip in 'Normal' mode.

   This function is exported with the jump table.  See
   boot_util.h and util.S.
*/
void
init_can(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3)
{
    uint8_t wb[8];
    uint8_t rb[8];

    /* Reset the MCP2515 */
    wb[0]=CAN_RESET;
    spi_write(wb,rb,1);
    _delay_loop_1(SPI_DELAY); /* Delay for CS to be high on MCP2515 */
    
    /* Sets up the Bit timing and interrupts in the MCP2515 */
    wb[0]=CAN_WRITE;
    wb[1]=CAN_CNF3;
    wb[2]=cnf3;
    wb[3]=cnf2;
    wb[4]=cnf1;
    //wb[5]=0x1F; /* Turn on all TX and RX interrupts - CANINTE */
	wb[5]=0x03;  /* Turn on RX interrupt bits - CANINTE */
	spi_write(wb,rb,6);
    _delay_loop_1(SPI_DELAY); /* Delay for CS to be high on MCP2515 */

    wb[1]=CAN_RXB0CTRL;
	wb[2]=0x60; /* Set to Rx Standard Frames only - RXB0CTRL */
	wb[3]=0x60; /* Set to Rx Standard Frames only - RXB1CTRL */
	spi_write(wb,rb,4);
    _delay_loop_1(SPI_DELAY); /* Delay for CS to be high on MCP2515 */

    /* Put the chip in Normal Mode */
    wb[1]=CAN_CANCTRL;
    wb[2]=0x00;
    spi_write(wb,rb,3);
    _delay_loop_1(SPI_DELAY); /* Delay for CS to be high on MCP2515 */

}

/* Calls the initialization routines */
static inline void
init(void)
{ 
    uint8_t cnf1=0x03, cnf2=0xb6, cnf3=0x04;
	uint8_t can_speed = 0;

	init_spi();

	/* Set the CAN speed.  The values for 125k are the defaults so 0 is ignored
	   and bad values also result in 125k */
    can_speed = eeprom_read_byte(EE_CAN_SPEED);
	if(can_speed==BITRATE_250) cnf1=0x01;      /* 250kbps */
	else if(can_speed==BITRATE_500) cnf1=0x00; /* 500kbps */
	else if(can_speed==BITRATE_1000) { cnf1=0x00; cnf2=0x92; cnf3=0x02; } /* 1Mbps */
    /* Initialize the MCP2515 */
	init_can(cnf1, cnf2, cnf3);
	
	init_serial();
	/* We use the 16 bit timer to handle the boot polling delay */
    TCCR1B=0x05; /* Set Timer/Counter 1 to clk/1024 */
	/* Move the Interrupt Vector table to the Bootloader section */
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);
	EICRA = 0x02; /* Set INT0 to falling edge */
    EIMSK = 0x01; /* Turn on the INT0 interrupt */
	sei();
}



/* This is the function that we call periodically during the one
   second startup time to see if we have a bootloader request on
   the CAN Bus. */
void
bload_check(void) {
    cli();
	if(can_iflag) {
        can_iflag=0;
        /* Do some CAN Stuff Here */
    }
    sei();
}

/* This calculates a CRC16 for the program memory starting at 
   address 0x0000 and going up to count-1 */
uint16_t
pgmcrc(uint16_t count) {
    uint16_t carry;
    uint16_t crc = 0xffff;
	uint16_t addr = 0; 

    while(addr != count) {
        int i = 8;

        bload_check();

        crc ^= pgm_read_byte_near(addr++);
        while(i--)
        {
            carry = crc & 0x0001;
            crc >>= 1;
            if (carry) crc ^= 0xA001;
        }
    }
    return crc;
}

/* Main Program Routine */

int
main(void)
{
    uint8_t poll_result;
	uint16_t can_id, pgm_crc, count, cmp_crc;
	uint8_t can_data[8], crcgood=0;
    
	init();
	uart_write("Start\n", 6);

    /* Find the firmware size and checksum */
	count      = pgm_read_word_near(PGM_LENGTH);
    cmp_crc    = pgm_read_word_near(PGM_CRC);

	/* Retrieve the Program Checksum */
	pgm_crc = pgmcrc(count);
	/* If it matches then set the good flag */
	if(pgm_crc == cmp_crc)
	    crcgood=1;

	/* This timer expires at roughly one second after startup */
	while(TCNT1 <= 0x2B00) /* Run this for about a second */
        bload_check();
    TCNT1 = 0x0000;
    
	if(crcgood) {
	   start_app(); /* When we go here we ain't never comin' back */
    }
	

	while(1) { /* For testing we'll run forever */


        _delay_loop_1(SPI_DELAY); /* Delay for CS to be high on MCP2515 */

/*		poll_result = can_poll();
		if(poll_result & 0x01) {
		    uart_write("R0\n",3);
            can_read(0, &can_id, can_data);
        } else if(poll_result & 0x02) {
		    uart_write("R1\n",3);
		    can_read(1, &can_id, can_data);
        }
*/
	}	
 	
    
    /* End of bootloader reset to applicaiton */
    start_app();
}



/* CODE SNIPPITS 

	for(n=0; n<64; n++) {
        boot_page_fill(n*2, n);
		boot_page_erase(0x0000);
        boot_spm_busy_wait(); 	
        boot_page_write(0x0000);
	    boot_spm_busy_wait(); 
    }

              Byte addr, data 
    eeprom_write_byte(3, 0x44);

*/
    
