#include <avr/io.h>
#include <typedefs.h>

#define REF_AVCC 0x01
#define REF_256 0x03

#define ADCIR		0	//ADC IR connected to ADC0

void ADC_init(u08);
void ADC_startconv(u08);

