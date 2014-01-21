#ifndef NES_H_INCLUDED
#define NES_H_INCLUDED

#define PULSE 0
#define LATCH 1
#define DATA 2
#define NESDELAY 70 //microseconds

volatile uint8_t nes;
void nessetup();
void nesread();
void nesdebug();
#endif
