#include <serial.h>

/* file originally written by Daniel Watman and modified by Mark Whitty for UGV and MAVSTAR purposes
* Contains commands for sending and receiving serial messages to a specific protocol:
* first byte is start byte (0x0A)
* second byte is Packet/message ID (user specified)
* third byte is number of bytes of data only in message
* fourth byte and following bytes contain 8 bit data. 16 bit data needs to be split up and recombined if necessary.
* final byte is a checksum of everything else in the packet - just a simple sum. Overflowing of checksum ignored.
*
* BUFSIZE defined in typedefs.h
*/ 

extern u08 msgflg, serbyte, serstat, sermsg[12];

void serial_init() {											// TXD pin as output															// Turn on the transmission and reception circuitry
	UCSR0A |=  (1 << U2X0);										// Enable double speed
	UCSR0C |=  (1 << UCSZ00) | (1 << UCSZ01); 					// Use 8-bit character sizes
//   	UBRR0L = MYUBRR; 										// Load lower 8-bits of the baud rate value into the low byte of the UBRR register
//   	UBRR0H = (MYUBRR >> 8); 								// Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRR0L = 25;												// 25 or 26. Try to change this value if wireless communication is not reliable
	UCSR0B |=  (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); 	// Enable the USART Receive Complete interrupt (USART_RXC)
}

void serial_byte(u08 data) {
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void serial_string(u08 *ptr) {
	while (*ptr) 
		serial_byte(*ptr++);
}

inline u08 inbuf(circ_buf *buf) {
	return (buf->head-buf->tail)&(BUFSIZE-1);
}

void tobuf(circ_buf *buf, u08 data) {
	buf->data[buf->head++] = data;	//add to buffer
	buf->head &= (BUFSIZE-1);		//keep within buffer limits
	if (buf->head==buf->tail) {	//buffer full
		buf->tail++;				//overwrite oldest data
		buf->tail &= (BUFSIZE-1);	//keep within buffer limits
	}
}

u08 getbuf(circ_buf *buf) {
	u08 tmp;
	
	if (inbuf(buf)) {
		tmp = buf->data[buf->tail++];
		buf->tail &= (BUFSIZE-1);	//     within buffer limits
	}
	else tmp = 0;
	
	return tmp;
}

u08 lookbuf(circ_buf *buf, u08 n) {
	u08 tmp = buf->tail + n;
	tmp &= (BUFSIZE-1);		//keep within buffer limits
	return buf->data[tmp];
}

void incbuf(circ_buf *buf, u08 n) {
	u08 tmp;
	
	if (n >= inbuf(buf)) tmp = buf->tail+inbuf(buf);
	else {
		tmp = buf->tail + n;
		tmp &= (BUFSIZE-1);		//keep within buffer limits
	}
	buf->tail = tmp;
}

u08 getmsg(circ_buf *buf, msg *newmsg) {
	u08 i;
	u08 chksum, chksum2;
	msg tmpmsg;
	for (i=0;(lookbuf(buf,i) != START)&&(i<inbuf(buf));i++)
		;
	incbuf(buf,i);	//move tail to oldest start byte

	if (inbuf(buf) >= 5) {	//enough bytes to be worth testing (strt,type,sze,dat,chk)

		tmpmsg.id = lookbuf(buf, 1);
		tmpmsg.size = lookbuf(buf, 2);
		if (tmpmsg.size > MAX_CONTROL_BOARD_PACKET_SIZE) {		//size shouldnt be that big, must be error (***adjust if necessary***)
			incbuf(buf, 1);			//increment tail to go to next possible start
			return 0;
		}
		
		if (inbuf(buf) >= (tmpmsg.size+4)) {	//whole packet (if size ok)
			chksum = lookbuf(buf, tmpmsg.size+3);
			chksum2 = 0;
			for (i=0; i < (tmpmsg.size+2); i++)	//calcuate checksum
				chksum2 += lookbuf(buf, i+1);
			if (chksum==chksum2) {	//checksum matches, copy data to structure
				for (i=0; i < (tmpmsg.size); i++)
					tmpmsg.data[i] = lookbuf(buf, i+3);
				incbuf(buf, tmpmsg.size+4);			//increment tail to go to next possible start
				*newmsg = tmpmsg;
				return 1;	//valid data
			}
			else {	//checksum doesnt match
				incbuf(buf, 1);			//increment tail to go to next possible start
			}
		}
	}
	return 0;
}

void qmsg(circ_buf *TXbuf, msg *m) {
	u08 i, chksum;
	chksum = m->id + m->size;
	
	tobuf(TXbuf, START);
	tobuf(TXbuf, m->id);
	//tobuf(TXbuf, 0);	//for CAN program compatibility
	tobuf(TXbuf, m->size);
	for (i=0;i < m->size;i++) {
		tobuf(TXbuf, m->data[i]);
		chksum += m->data[i];
	}
	tobuf(TXbuf, chksum);
//	tobuf(TXbuf, chksum);
//	tobuf(TXbuf, chksum);
}

void CAN2serial(can_msg* msg) {
	u08 chksum, i;

	chksum = (msg->id >> 8);
	chksum += (msg->id & 0xFF);
	chksum += msg->size;
	for (i=msg->size; i !=0;)
		chksum += msg->data[--i];
		
	serial_byte(START);
	serial_byte(msg->id & 0xFF);	//id low byte
	serial_byte(msg->id >> 8);	//id high byte
	serial_byte(msg->size);		//size
	for (i = 0; i != msg->size; i++)
		serial_byte(msg->data[i]);	//data bytes
	serial_byte(chksum);		//checksum
}

void serial2CAN(u08* data, can_msg* msg) {
	u08 i;
	u16 temp;

    msg->size = data[2];

	temp = data[1]<<8;
	msg->id = (temp + data[0]) & 0x07FF;

	for (i=0; i < data[2]; i++)
        msg->data[i] = data[i+3];
}

