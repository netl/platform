#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "nes.h"

void nessetup()
{
	DDRC=(0<<DATA)|(1<<PULSE)|(1<<LATCH);//0b011;
	PORTC=(1<<PULSE); //HIGH
}

void nesread()
{
	uint8_t i;
	//reset shift registers
	PORTC|=(1<<LATCH); //HIGH
	_delay_us(NESDELAY);
	PORTC&=~(1<<LATCH); //LOW
	//insert bits into variable
	for(i=0;i<8;i++)
	{
		nes=(nes<<1)|((PINC>>(DATA))&0x1);
		//clock next bit in
		_delay_us(NESDELAY);
		PORTC|=(1<<PULSE); //HIGH
		_delay_us(NESDELAY);
		PORTC&=~(1<<PULSE); //LOW
	}
}

