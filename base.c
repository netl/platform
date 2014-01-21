#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include "nes.h"

//void drive(uint8_t dir);
void nesdrive();
void move(int8_t* dir, int8_t* spd);

void nesdrive()	//translates input from nes controller to simple commands for driving the motors
{
	static int8_t dir=0,spd=0;
	uint8_t turning=0,moving=0;
	//set directions according to input from nes controller
	if((~nes>>3)&1) //up
	{
		if(spd<127) spd++;
		moving=1;
	}
	if((~nes>>2)&1) //down
	{
		if(spd>-127) spd--;
		moving=1;
	}
	if((~nes>>1)&1) //left
	{
		if(dir>-127) dir--;
		turning=1;
	}
	if(~nes&1) //right
	{
		if(dir<127) dir++;
		turning=1;
	}

	//slow things down if no input
	if(moving==0)
	{
		if(spd>0) spd--;
		else if(spd<0) spd++;
	}
	if(turning==0)
	{
		if(dir>0) dir--;
		else if(dir<0) dir++;
	}
	move(&dir,&spd);	
}

void move(int8_t* dir, int8_t* spd) //drive motors
{
	static int  lmotor,rmotor;
	//set motor speeds according to direction and speed

	if(*dir==0)
	{
		lmotor=*spd;
		rmotor=*spd;
	}
	else if(*spd>=0)	//set motor speeds while driving forwards
	{
		if(*dir<0)	//turning left
		{
			lmotor=*spd+*dir;	//set left motor speed and direction
			rmotor=*spd*(127+*dir)/127-*dir;	//limit right motor maximum speed
		}
		else if(*dir>0)	//turning right
		{
			rmotor=*spd-*dir;	//set right motor speed and direction
			lmotor=*spd*(127-*dir)/127+*dir;	//limit left motor maximum speed
		}
	}
	else	//backwards
	{
		if(*dir<0)	//turning left
		{
			rmotor=*spd-*dir;	//set left motor speed and direction
			lmotor=*spd*(127+*dir)/127+*dir;	//limit right motor maximum speed speed
		}
		else if(*dir>0)	//turning right
		{
			lmotor=*spd+*dir;	//set right motor speed and direction
			rmotor=*spd*(127-*dir)/127-*dir;	//limit left motor maximum speed speed
		}
	}
	
	//set h-bridge directions
	if(lmotor>0) PORTD=(PORTD&(~3<<2))|(0b01<<2);	//right motor forwards
	else if(lmotor<0) PORTD=(PORTD&(~3<<2))|(0b10<<2);	//reverse
	else PORTD&=~3<<2;	//stopped

	if(rmotor>0) PORTD=(PORTD&(~3))|0b01;	//left motor forwards
	else if(rmotor<0) PORTD=(PORTD&(~3))|0b10;	//reverse
	else PORTD&=~3;	//stopped

	//write speeds into pwm registers
	OCR1A=abs(2*rmotor);
	OCR1B=abs(2*lmotor);
}

int main(void)
{
	nessetup();
	
	//motor setup
	DDRB=0xff;	//portb as output
	PORTB=0xff;	//all high
	DDRD=0x0f;	//motor pins as output
	TCCR1A = (1<<COM1B1)|(1<<COM1A1)|(1<<WGM10);	//fast pwm
	TCCR1B = (1<<CS10)|(1<<WGM12);	//no prescale
	
	//interrupts
	TCCR0 |= (1<<CS01)|(1<<CS00);	//timer/counter0 with 64 prescale
	TIMSK |= (1<<TOIE0);	//enable interrupt for timer/counter0 overflow
	sei();	//enable interrupts
	
	nesread();	//read nes controller before beginning to drive
	while(1)
	{
	 nesdrive();
	 _delay_ms(1);	//add delay to make the robot accelerate/decelerate slower
	}

}

SIGNAL(TIMER0_OVF_vect)
{
	nesread(); //read the nes controller
}
