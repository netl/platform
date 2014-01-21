#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include "nes.h"

//void drive(uint8_t dir);
void nesdrive();
void autopilot();
void move(int8_t* dir, int8_t* spd);

uint8_t sensor_data;

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

void autopilot()
{
    // 0: right-most sensor
    // 1: second from right
    // 2: etc

    // value 1: sensor detects black
    // value 0: sensors detects non-black things

    // to read one bit
    // sensor_data = (PINB >> 0)&1;
    // sensor_data = (PINB >> 1)&1;

    // To read first 4 bits and ignore last 4
    sensor_data = PINB&0x0f;

    // To check if first field is 1

    uint8_t dir = 0;
    uint8_t speed = 5;

    // Write the status of the 2nd sensor to PORTB bit 5
    // PORTB = ((sensor_data >> 2)&1) <<5|PORTB&~(1<<5);
    if (sensor_data)
        PORTB = 1<<5|PORTB&~(1<<5);
    else
        PORTB = 0<<5|PORTB&~(1<<5);

    if ((sensor_data >> 0)&1)
    {
        // Right-most sensor detects a line - must turn
        dir = 127;
        speed = 0;
        move(&dir,&speed);
    }
    else if ((sensor_data >> 3)&1)
    {
        // Left-most sensor detects a line - must turn
        dir = -127;
        speed = 0;
        move(&dir, &speed);
    }
    else if ( ((sensor_data >> 1)&1) && (sensor_data >> 2)&1 )
    {
        // Both middle sensors detect a line, go forwards
        dir = 0;
        move(&dir, &speed);
    }
    else if ( ((sensor_data >> 1)&1) && !(sensor_data >> 2)&1 )
    {
        // Second sensor from right detects a line, but the other middle one does not
        // Turn slightly to right
        dir = 20;
        move(&dir, &speed);
    }
    else if ( !((sensor_data >> 1)&1) && (sensor_data >> 2)&1)
    {
        // Second sensor from left detects a line, but the other middle one does not
        // Turn slightly to left
        dir = -20;
        move(&dir, &speed);
    }
    
}

void move(int8_t* dir, int8_t* spd) //drive motors
{
	static int  lmotor,rmotor;
    static int current_dir = 0;
    static int current_speed = 0;
	//set motor speeds according to direction and speed

    if (current_dir < *dir)
        current_dir++;
    else if (current_dir > *dir)
        current_dir--;

    if (current_speed < *spd)
        current_speed++;
    else if (current_speed < *spd)
        current_speed--;

	if(current_dir==0)
	{
		lmotor=current_speed;
		rmotor=current_speed;
	}
	else if(current_speed>=0)	//set motor speeds while driving forwards
	{
		if(current_dir<0)	//turning left
		{
			lmotor=current_speed+current_dir;	//set left motor speed and direction
			rmotor=current_speed*(127+current_dir)/127-current_dir;	//limit right motor maximum speed
		}
		else if(current_dir>0)	//turning right
		{
			rmotor=current_speed-current_dir;	//set right motor speed and direction
			lmotor=current_speed*(127-current_dir)/127+current_dir;	//limit left motor maximum speed
		}
	}
	else	//backwards
	{
		if(current_dir<0)	//turning left
		{
			rmotor=current_speed-current_dir;	//set left motor speed and direction
			lmotor=current_speed*(127+current_dir)/127+current_dir;	//limit right motor maximum speed speed
		}
		else if(current_dir>0)	//turning right
		{
			lmotor=current_speed+current_dir;	//set right motor speed and direction
			rmotor=current_speed*(127-current_dir)/127-current_dir;	//limit left motor maximum speed speed
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
	//DDRB=0x00;	//portb5 as output
    DDRB = 0b100000;
	PORTB=0b011111; //all high
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
	 autopilot();
	 _delay_ms(1);	//add delay to make the robot accelerate/decelerate slower
	}

}

SIGNAL(TIMER0_OVF_vect)
{
	//nesread(); //read the nes controller
}
