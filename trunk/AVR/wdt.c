#include <wdt.h>

void WDT_init(void){
	//Reset Watchdog timer
	asm("wdr");
	//Start timed sequence
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	// Set new prescaler (time-out) for 1s
	WDTCSR = (1<<WDE)|(1<<WDP2)|(1<<WDP1);
}

void WDT_clear(void){
	//Reset Watchdog timer
	asm("wdr");
}
