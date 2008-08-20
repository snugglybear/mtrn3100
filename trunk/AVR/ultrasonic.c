#include <timing.h>
#include <typedefs.h>
#include <ultrasonic.h>

extern volatile u16 time_ms;

void ping(void) {
	TCCR1B &= 0xF8;			//stop timer1
	TCNT1 = 0;				//clear timer1
	TCCR1B |= (1<<6);		//capture on rising edge
	TIFR1 = (1<<5);		//clear interrupt by writing 1 (0 in other locations)
	
	USPORT |= (1<<USTRIG);		//output high, trigger ultrasonic sensor
	tinywait(20);				//wait a bit, datasheet says 10us minimum pulse
	USPORT &= ~(1<<USTRIG);	//output low
	
	TIMSK1 |= (1<<5);		//enable interrupt on capture
}
