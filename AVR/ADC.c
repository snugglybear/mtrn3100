#include <ADC.h>

void ADC_init(u08 reference) {
	ADMUX = reference<<6;	//AVCC reference, channel 0
	ADCSRA = 0x8F;	//ADC enabled, interrupts on*/
}

void ADC_startconv(u08 channel) {
	ADMUX &= 0xF0;		//don't change voltage reference
	ADMUX |= channel;	//update channel
	ADCSRA |= 0x40;
}
/*
SIGNAL(SIG_ADC) {	//not interruptable
	//something
}*/
