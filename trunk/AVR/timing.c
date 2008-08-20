#include <timing.h>

extern volatile u16 time_ms;

void timing_init0(u08 div) {
	TCCR0B = div&0x07;	//set prescaler (1 8 64 256 1024 ext.fal ext.ris)
	TIMSK0 |= 0x01;	//enable timer0 overflow interrupt
}

void timing_init2(u08 div) {
	TCCR2B = div&0x07;	//set prescaler (1 8 32 64 128 256 1024)
	TIMSK2 |= 0x01;	//enable timer2 overflow interrupt
}

void wait(u08 i) {	//approx ?us per count
	for (; i>0 ;i--)
		asm("nop");
}

void tinywait(u08 i) {	//short delay
	for (; i>0 ;i--)
		asm("nop");
}

void wait_ms(u16 delay) {
	u16 temp = time_ms + delay;
	
	while (time_ms != temp);
	asm("nop");
}
/*
SIGNAL(SIG_OVERFLOW0) {	//not interruptable
	time_ms++;
}
SIGNAL(SIG_OVERFLOW2) {	//not interruptable
	time_ms++;
}
*/
