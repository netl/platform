#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include "nes.h"

uint8_t nesdrive();
void sensordrive();
void move(int8_t dir, int8_t spd);

uint8_t sensor_data;

uint8_t nesdrive()	//translates input from nes controller to simple commands for driving the motors
{
	static int8_t dir=0,spd=0;
	uint8_t turning=0,moving=0;
	//set directions according to input from nes controller
	if((~nes>>3)&1) //up
	{
		spd=127;
		//if(spd<127) spd++;
		moving=1;
	}
	if((~nes>>2)&1) //down
	{
		spd=-127;
		//if(spd>-127) spd--;
		moving=1;
	}
	if((~nes>>1)&1) //left
	{
		dir=-127;
		//if(dir>-127) dir--;
		turning=1;
	}
	if(~nes&1) //right
	{
		dir=127;
		//if(dir<127) dir++;
		turning=1;
	}
	if(~nes>>4&1) //SART
	{
		return(0);
	}

	//slow things down if no input
	if(moving==0)
	{
		spd=0;
	}
	if(turning==0)
	{	
		dir=0;
	}
	move(dir,spd);
	return(1);
}
void sensordrive()
{
	uint8_t sensor=(PIND>>4)&0xf;
	int8_t spd, dir;
	
	switch (sensor)
	{
		case 0b0110:
			spd=127;
			dir=0;
			break;
		case 0b0100:	//Left
			spd=127;
			dir=-64;
			break;
		case 0b1100:
			spd=80;
			dir=-64;
			break;
		case 0b1000:
			spd=0;
			dir=-127;
			break;
		case 0b1110:
			spd=0;
			dir=-127;
			break;
		case 0b0010:	//Right
			spd=127;
			dir=64;
			break;
		case 0b0011:
			spd=80;
			dir=64;
			break;
		case 0b0001:
			spd=0;
			dir=127;
			break;
		case 0b0111:
			spd=0;
			dir=127;
			break;
		case 0b0000:	//lost line. gun it.
			spd=80;
			dir=0;
			break;
		case 0b1111:	//most likely a cossing. gun it.
			spd=127;
			dir=0;
			break;
		default:
			spd=0;
			dir=0;
			break;

	}
	move(dir,spd);
}

void move(int8_t dir, int8_t spd) //drive motors
{
	static int  lmotor,rmotor;
 	static int8_t current_dir=0, current_speed=0;
        //ramp motor speeds
        if(current_speed>spd)
                current_speed --;
        else if(current_speed<spd)
                current_speed ++;
        if(current_dir>dir)
                current_dir --;
        else if(current_dir<dir)
                current_dir ++;

	//set motor speeds according to direction and speed
	 if(current_dir==0)
        {
                lmotor=current_speed;
                rmotor=current_speed;
        }
        else if(current_speed>=0)       //set motor speeds while driving forwards
        {
                if(current_dir<0)       //turning left
                {
                        lmotor=current_speed+current_dir;       //set left motor speed and direction
                        rmotor=current_speed*(127+current_dir)/127-current_dir;        //limit right motor maximum speed
                }
                else if(current_dir>0)  //turning right
                {
                        rmotor=current_speed-current_dir;       //set right motor speed and direction
                        lmotor=current_speed*(127-current_dir)/127+current_dir;        //limit left motor maximum speed
                }
        }
        else    //backwards
        {
                if(current_dir<0)       //turning left
                {
                        rmotor=current_speed-current_dir;       //set left motor speed and direction
                        lmotor=current_speed;        //limit right motor maximum speed speed
                }
                else if(current_dir>0)  //turning right
                {
                        lmotor=current_speed+current_dir;       //set right motor speed and direction
                        rmotor=current_speed;        //limit left motor maximum speed speed
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
	DDRB = 0b100110;	//PB0 button, PB5 as output PB1&2 pwm
	PORTB=0b011001; //PB1&2 must not be touched(pwm output)!!
	DDRD=0x0f;	//motor pins as output and line detector pins as input
	PORTD= 0xf0;	//pull-up on detector pins
	TCCR1A = (1<<COM1B1)|(1<<COM1A1)|(1<<WGM10);	//fast pwm
	TCCR1B = (1<<CS10)|(1<<WGM12);	//no prescale
	
	//interrupts
	TCCR0 |= (1<<CS01)|(1<<CS00);	//timer/counter0 with 64 prescale
	TIMSK |= (1<<TOIE0);	//enable interrupt for timer/counter0 overflow
	sei();	//enable interrupts
	
	nesread();	//read nes controller before beginning to drive
	while (nesdrive())
		_delay_ms(1);
	while(PINB&1);	//wait for button to be pressed
	while(1)
	{
		sensordrive();
		_delay_ms(1);
	}
}

SIGNAL(TIMER0_OVF_vect)
{
	if(PIND&0xf0)
		PORTB=PORTB|(1<<5);
	else
		PORTB= PORTB&~(1<<5);
	nesread(); //read the nes controller
}
