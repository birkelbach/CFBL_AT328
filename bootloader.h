#ifndef _BOOTLOADER_H
#define _BOOTLOADER_H

#define SPI_SS   PB2
#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCK  PB5
#define SPI_DDR  DDRB
#define SPI_PORT PORTB
#define SPI_SS_LOW()  (SPI_PORT &= ~(1<<SPI_SS))
#define SPI_SS_HIGH() (SPI_PORT |= (1<<SPI_SS))
#define SPI_DELAY 24

// General Definitions
#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) ((x) & (BIT(y)))
#define BITCLEAR(x,y) !BITSET((x), (y))
#define BITSSET(x,y) (((x) & (y)) == (y))
#define BITSCLEAR(x,y) (((x) & (y)) == 0)
//#define BITVAL(x,y) (((x)>>(y)) & 1) 

// EEPROM Data Locations 
#define EE_CAN_SPEED 0x00
#define EE_NODE_ID   0x01
#define EE_BAUD      0x02

#endif
