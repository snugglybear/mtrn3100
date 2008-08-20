//#include <main.h>

#define USDDR	DDRB	//ultrasonic sensor trigger port (for data direction setting)
#define USPORT	PORTB	//ultrasonic sensor trigger port
#define USTRIG	1		//ultrasonic sensor trigger pin

void ping(void);		//begin ultrasonic measurement
