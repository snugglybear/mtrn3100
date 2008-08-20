#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <typedefs.h>
#include <timing.h>

#define MAX_PWM	0xA0	//not set

#define T0PS	0x02	//timer0 prescale value 0x05=/1024
#define T1PS	0x02	//timer1 prescale value 0x02=/8
#define T2PS	0x03	//timer2 prescale value 0x03=/32


#include <wdt.h>

#include <ultrasonic.h>

#include <ADC.h>

#include <serial.h>


#ifndef __TYPES__
#include <avr/interrupt.h>

	#define __TYPES__
	typedef unsigned char  u08;
	typedef signed char  s08;
	typedef unsigned int  u16;
	typedef signed int  s16;
	typedef unsigned long  u32;
	typedef signed long  s32;

#endif
