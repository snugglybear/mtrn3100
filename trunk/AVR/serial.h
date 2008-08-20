#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <typedefs.h>

#define	START	0x0A
#define BAUD	38400
#define FOSC	8000000
#define MYUBRR	(((FOSC/16)/BAUD)-1)

void serial_init(void);
void serial_byte(u08);
void serial_string(u08*);
u08 inbuf(circ_buf*);			//check amount of data in buffer
void tobuf(circ_buf*, u08);	//add byte to buffer
u08 getbuf(circ_buf*);			//get last byte in buffer
u08 lookbuf(circ_buf*, u08);	//look at byte in buffer
void incbuf(circ_buf*, u08);	//increment buffer position
u08 getmsg(circ_buf*, msg*);	//extract messages from buffer
void qmsg(circ_buf*, msg*);	//add message to transmit queue

//u08 serial_getmsg(void);		//assembles a message from serial data
//void CAN2serial(can_msg*);		//converts a CAN message and sends over serial
//void serial2CAN(u08*,can_msg*);	//converts a serial message to a CAN msg
