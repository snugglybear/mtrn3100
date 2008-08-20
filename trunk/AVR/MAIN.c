/*! \file MAIN.c \Main program for compiling to AVR. */
//*****************************************************************************
//
// File Name	: 'MAIN.c'
// Title		: MAIN C program for compiling ATMEGA88
// Author		: Pascal Stang - Copyright (C) 2003
// Created		: Nov 2005 Daniel Watman
// Revised		: Mar 2007 Phil Sammons
// Revised		: Jan 2008 Hang Xu
// Revised		: Jan 2008 Mark Whitty
// Revised		: Jul 2008 Edward
// Developed at : UNSW
// Website		: http://cmr.mech.unsw.edu.au
// Version		: 1.0
// Target MCU	: Atmel AVR Series
//
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

// Define the ID of the UGV (0x00, 0x10,..., 0x70)
#define UGVID 0x00

#include <MAIN.h>

/*

Timer usage:
TIMER0:	Output compare 0A/0B for motor PWM
TIMER1:	Input capture 1A for ultrasonic measurement	
TIMER2:	1.024ms overflow for general timing
		Output compare 2A for servo pulse generation

In the sample Matlab code,
motorA (speed1) is the right motor,
motorB (speed2) is the left motor.

Pin usage:
PORTB:
	7	H-bridge 3A (Direction control for motor B)
	6	H-bridge 2A (Direction control for motor A)
	5	SCK  (for programming)
	4	MISO (for programming)
	3	MOSI (for programming)
	2	LED for debugging
	1	ultrasonic sensor (trigger pin)
	0	ultrasonic sensor (echo pin)
PORTC:
	6	RESET (also for programming)
	5	No Connection
	4	No Connection
	3	No Connection
	2	No Connection
	1	No Connection
	0	A/D for IR Range Finder
PORTD:
	7	H-bridge 1A (Direction control for motor A)
	6	H-bridge 1,2EN (Speed control for motor A)
	5   H-bridge 3,4EN (Speed control for motor B)
	4	H-bridge 4A (Direction control for motor B)
	3	No Connection
	2	Servo
	1	Serial TX
	0	Serial RX
*/

// everything that changes in an ISR must be declared as volatile!
// while developing its easier just to make everyting volatile
volatile u08 serialflg 	= 0x00;		//Serial flags
volatile u08 motorflg	= 0x00;		//motor flags
volatile u08 ultraflg	= 0x00;		//ultrasonic flags
volatile u08 adcflg 	= 0x00;		//Accelerometer flags
volatile u08 servoflg 	= 0x00;		//servo flags

volatile u16 nextupdate=0, time_ms=0; 		//use for general timing
volatile u08 time_ms20 = 0, time_ms60 = 0; 	//time_ms20 = time_ms%20; time_ms60 = time_ms%60;
//this is only a problem if you want to time something longer than this
volatile u16 us_range = 0;					//range measured by US
volatile u16 us_range_mm = 0;				//range measured by US in mm

//used for reading from A/D ports
volatile u16 analogout 	= 0;
volatile u16 analogin 	= 0;
volatile u08 analogLow 	= 0;
volatile u08 analogHigh = 0;

//used for catching analog converter when it fails to read
volatile u32 wdtimer 	= 0;					//Watchdog Timer for A/D
volatile u32 wdcounter 	= 0;					//Watchdog Counter for A/D

// IR Range
volatile u16 ir_range = 0x200;
volatile s16 temp_ADC = 0x200;


volatile u08 motorA, motorB, servoA;			// Output to motor/servo
volatile u08 direction 	= 0x00;					// Direction for the motors
volatile s16 control_temp[4];					// temp to receive before new input arrives 

volatile u08 i = 0;

volatile char cSREG;


msg TXmsg, RXmsg;			//serial comm
circ_buf TXbuf, RXbuf;		//serial comm

//Declare functions
void do_adc(void);
void do_motors(void);
void do_servo(void);
void do_ultrasonic(void);
void do_serial(void);

int main (void) {

	//PIN SETUP
	//All pins are inputs by default. Set DDR to 1 for output pins
	//Outputting 1 to an input pin sets a pullup resistor on that pin

//*******	
// Initialisation routines performed on startup or reset are below here
//*******

	// Set default values for the receieved serial commands
	control_temp[0] = 0x0000;	// Default Direction: Stop
	control_temp[1] = 0x0080;	// Default Speed for Motor A: 50%
	control_temp[2] = 0x0080;	// Default Speed for Motor B: 50%
	control_temp[3] = 0x0080;	// Default Servo Position: 50%

	serial_init();				//Initialise Serial Stuff

	DDRB |= (1<<2);				//Set PORTB pin 2 to output for LED for debug/heartbeat
	DDRB |= (1<<6);				//Set PORTB pin 6 to output for H-bridge 2A
	DDRB |= (1<<7);				//Set PORTB pin 7 to output for H-bridge 3A
	DDRD |= (1<<2);				//Set PORTD pin 2 to output for Servo
	DDRD |= (1<<4);				//Set PORTD pin 4 to output for H-bridge 4A	
	DDRD |= (1<<5);				//Set PORTD pin 5 to output for H-bridge 3,4EN
	DDRD |= (1<<6);				//Set PORTD pin 6 to output for H-bridge 1,2EN
	DDRD |= (1<<7);				//Set PORTD pin 7 to output for H-bridge 1A

	USDDR |= (1<<USTRIG);		//Set ultrasonic sensor trigger pin 1 PORT B as output

	ADC_init(1);				//Initialise ADC
	adcflg |= 0x81;				//Set flag for IMU

	WDT_init();					//Initialise Watchdog Timer and change the Watchdog Time-out to 1s

	//INTERRUPTS AND TIMER SETUP
	//If you enable an interrupt but don't have an ISR for it, Bad Things(tm) will happen
	//ISRs are down the bottom after the main() function

	// Set Timer 2 for timing and servo pulses
	timing_init2(T2PS);	//timer2 prescale setup/interrupt enable	
	OCR2A = 0x80;		//set initial the servo pos
	OCR2B = 0x80;		//set initial the servo pos

	// Set Timer 0 for PWM for motors
	TCCR0A = 0xA3;
	TCCR0B = 0xA3;
	OCR0A = 0x01;
	OCR0B = 0x01;

	// Enable interrupts only after initialisation is complete!
	sei();			//enable interrupts
	nextupdate = 100;

	motorA = 0;
	motorB = 0;

	time_ms = 0;		//your time starts...NOW!

//****************************/
//MAIN WHILE LOOP
//****************************/
	while (1) {
		do_serial();
		do_servo();
		do_motors();
		do_ultrasonic();
		do_adc();
		
		// Reset Watchdog timer if it goes to the end of the while loop.
		// AVR will reset if it doesn't reset the Watchdog timer for >1s
		WDT_clear();	// Reset Watchdog timer
	}
}

void do_serial(void){
	// Send control board status ----------------------------------
	if (serialflg & 0x01){			//Send via serial
		TXmsg.id = (UGVID+1);
		TXmsg.size = 9;
		TXmsg.data[0] = direction;
		TXmsg.data[1] = motorA; 
		TXmsg.data[2] = motorB;
		TXmsg.data[3] = servoA;
		TXmsg.data[4] = (u08)(us_range_mm & 0xFF);	// US low byte
		TXmsg.data[5] = (u08)(us_range_mm>>8);		// US high byte
		TXmsg.data[6] = (u08)(ir_range & 0xFF);		// IR low byte
		TXmsg.data[7] = (u08)(ir_range >> 8);		// IR high byte
		TXmsg.data[8] = (u08)(OCR2A & 0xFF);		// For Debug, you can change it
		qmsg(&TXbuf, &TXmsg);

		serialflg &= ~0x01;
	}
	// Receive ------------------------------------
	else if (serialflg & 0x80){					//new serial byte received
		if(getmsg(&RXbuf, &RXmsg)) {	//check if full message correctly received and do stuff with it
// NOTE: SERIAL BUFSIZE (in typedefs.h) is 64 bytes
			if (RXmsg.id == (UGVID+1)) {
				control_temp[0] = RXmsg.data[0];	// Direction
				control_temp[1] = RXmsg.data[1];	// Motor A Speed
				control_temp[2] = RXmsg.data[2];	// Motor B Speed
				control_temp[3] = RXmsg.data[3];	// Servo A
			}
		}
		serialflg &= ~0x80;    // reset serial flag
	}
	else{
		while (inbuf(&TXbuf)) {
			tinywait(10);
			serial_byte(getbuf(&TXbuf));
		}
	}
}

void do_motors(void){
	if (motorflg & 0x01){
		us_range_mm = us_range/6;	// Calculate us range in mm

		// Change direction based on the received command
		direction = control_temp[0];

		// Change motors speeds based on the received command
		motorA = control_temp[1];
		motorB = control_temp[2];
		
		//********************************************//
		//Add your on-board obstacle avoidance code here
		//********************************************//

		//********************************************//
		//End of your on-board obstacle avoidance code
		//********************************************//


		// Change motor directions
		if (direction & 0x01){	// Check if 1A in the direction command is high
			PORTD |= (1<<7);}	// Turn on PD7
		else{
			PORTD &= ~(1<<7);}	// Turn off PD7

		if (direction & 0x02){	// Check if 2A in the direction command is high
			PORTB |= (1<<6);}	// Turn on PB6
		else{
			PORTB &= ~(1<<6);}	// Turn off PB6

		if (direction & 0x04){	// Check if 3A in the direction command is high
			PORTB |= (1<<7);}	// Turn on PB7
		else{
			PORTB &= ~(1<<7);}	// Turn off PB7

		if (direction & 0x08){	// Check if 4A in the direction command is high
			PORTD |= (1<<4);}	// Turn on PD4
		else{
			PORTD &= ~(1<<4);}	// Turn off PD4

		// Change motor speed
		if (motorA == 0){
			motorA++;}		// Go funny if it is 0
		else if(motorA > 0xA0){
			motorA = 0xA0;}	//  Limit the max speed/voltage of/to motor
		if (motorB == 0){
			motorB++;}		// Go funny if it is 0
		else if(motorB > 0xA0){
			motorB = 0xA0;}	//  Limit the max speed/voltage of/to motor
		
		OCR0A = motorA;
		OCR0B = motorB;

		// Clear flag
		motorflg &= ~0x01;
	}
}

void do_servo(void){
	//update servo position
	if (servoflg & 0x01) {		
		servoA = control_temp[3];

		if (servoA == 0){
			servoA++;	// Go funny if it is 0
		}
		//clear flag
		servoflg &= ~0x01;
	}
}

void do_adc(void){
	if (adcflg & 0x80){				//read from ADC
		if(adcflg & 0x01){
			ADC_startconv(ADCIR);	//Gyro Rate
		}
		adcflg &= ~0x80;
		//return adcflg;
	}
}


void do_ultrasonic(void){
	if (ultraflg & 0x01) {		//PING ultrasonic
		ping();					//send ping
		ultraflg &= ~0x01;		//clear flag
	}
}


//INTERRUPT SERVICE ROUTINES
//interrupts possible:
//SIG_OVERFLOW0
//SIG_OVERFLOW2
//SIG_INPUT_CAPTURE1
//SIG_OUTPUT_COMPARE2B



// DO SOMETHING TO THE MILLISECOND INTURRUPT ROUTINE
ISR(TIMER2_OVF_vect) {				//1.024ms "tick". Runs each time timer2 overflows.
	if (time_ms20 == 2) {			// Time critical issue go first
		TCNT2 = 0x00;
		servoflg |= 0x02;
	}	
	else if (time_ms20 == 0){	
		OCR2A = servoA;
		if(OCR2A == 0)OCR2A++;		//goes funny if set to 00
	}

	else if (time_ms20 == 1) {		//25 milliseconds (40Hz)
									//note: servo pulses are usually between 30-50Hz repetition rate
		PORTD |= (1<<2);			//turn on servo
		TIMSK2 |= 0x03;				//enable timer2 overflow interrupt and OC2 interruptsf set to 00
	}
	else if (time_ms20 == 3) {
		servoflg &= ~0x02;			//clear flag
		PORTD &= ~(1<<2);			//Make sure the servo pulse go down after 2 ms
		//servoflg |= 0x10;			//wait for interrupt
		//servoflg |= 0x20;			//wait for interrupt
	}

	if (time_ms60 == 27) {			//move servos and do motor control
		servoflg |= 0x01;			//set the flag so it will be done in the main program
		motorflg |= 0x01;
	}
	else if (time_ms60 == 7) {		//Do Ultrasonic
		ultraflg |= 0x01;			//set the flag so it will be done in the main program
	}
	else if (time_ms60 == 47) {
		serialflg |= 0x01;			//Send serial
	}	

	if (time_ms%500 == 0){
		PORTB ^= (1<<2);			//Heartbeat LED flash every s
	}
	time_ms++;						//increment the time
	time_ms20 = time_ms%20;
	time_ms60 = time_ms%60;

	if (time_ms == 60000){
		time_ms = 0;				//Reset time_ms every min
	}	
}

// SERVO INTERRUPT SERVICE ROUTINES

ISR(TIMER2_COMPA_vect) {
	if (servoflg & 0x02) {
		PORTD &= ~(1<<2);		//set servo1 low
		TIMSK2 &= ~(0x02);		//disable interrupt A
	}
}

ISR(USART_RX_vect) {
	tobuf(&RXbuf, UDR0);
	serialflg |= 0x80;  		// changes flow in main while loop
}

// INTERRUPT FOR ULTRASONIC

ISR(TIMER1_CAPT_vect) {			//input capture event
	if (TCCR1B & (1<<6)) {		//rising edge captured
		TCCR1B |= T1PS;			//start timer1
		TCCR1B &= ~(1<<6);		//capture on falling edge
		TIFR1 = (1<<5);			//clear interrupt by writing 1 (0 in other locations)
	}
	else {						//falling edge captured
		us_range = ICR1;		//store measured pulse width
		TIMSK1 &= ~(1<<5);		//disable interrupt on capture
	}
}

// INTERRUPT FOR IMU

ISR(ADC_vect) {	//not interruptable
	analogLow = ADCL;
	analogHigh = ADCH;

	if(adcflg & 0x01){
		temp_ADC = analogHigh <<8;								// Get high byte of the analog input
		temp_ADC |= analogLow;									// Get low byte of the analog input
		ir_range = (temp_ADC + (ir_range<<3) - ir_range)>>3;	// Filter the analog input
		adcflg &= ~0x01;	//clear flag
		adcflg |= 0x01;   	//set next (You need to set another flag if you have more than 1 ADC)
		wdtimer = 0;
	}
	adcflg |= 0x80;
}
