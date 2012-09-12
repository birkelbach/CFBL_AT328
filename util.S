#include <avr/io.h>

.extern main
.extern init_can

/* This section is the bootloader function jump table.
   Starting at the start of the .text section + 1 is the 
   jmp for the first function.  This is so that code
   in the application portion of ROM can use these
   functions without worrying if they have moved within
   the bootloader */
.section .init1
    jmp     start
    jmp     int0_isr
	jmp     init_can

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
	   and clear interrupts. */
	cli
    ldi r16, (1<<IVCE)
	out _SFR_IO_ADDR(MCUCR), r16 
    ldi r16, 0
	out _SFR_IO_ADDR(MCUCR), r16 
	/* Jump to the start of the program */
    jmp 0x0000
