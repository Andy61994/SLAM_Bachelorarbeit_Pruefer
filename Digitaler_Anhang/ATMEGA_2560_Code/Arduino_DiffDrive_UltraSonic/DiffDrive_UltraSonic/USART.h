/*
 * USART.h
 *
 * Created: 19.11.2020 10:20:07
 *  Author: Andy
 */ 


#ifndef USART_H_
#define USART_H_

#define USART_0 0
#define USART_1 1
#define USART_2 2
#define USART_3 3

#define BAUD_9600 9600
#define BAUD_57600 57600
#define BAUD_115200 115200


#define X_OFF_RECEIVED_Event 2
#define X_OFF_TRANSMITTED_Event 4

#define QUEUE_CAPACITY 64

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "queue.h"
#include "events.h"

typedef uint8_t usart_num;
typedef uint8_t register_flag_8;
typedef volatile uint8_t* register_address_8;
typedef uint16_t register_flag_16;
typedef volatile uint16_t* register_address_16;

typedef struct
{
	register_address_8 UDRn; // USART Data Register
	register_address_8 UCSRnB; // USART Control and Status Register B
	register_flag_8 UDRIEn; // USART Data Register Empty Interrupt Enable Flag
	Queue* TransmitQueue;
	Queue* ReceiveQueue;
	uint8_t FlowControl;
	uint8_t full;
	event_type eventStore;
}USART;

USART usarts[4];

Queue receiveQueues[4]; 
Queue transmitQueues[4];

void setUSARTRegisters(USART* _usartN, 
						register_address_8 _UDRn, 
						register_address_8 _UCSRnB, 
						register_flag_8 _UDRIEn, 
						register_flag_8 RXENn, 
						register_flag_8 TXENn, 
						register_flag_8 RXCIEn, 
						register_address_16 _UBRRn, 
						uint32_t _baudRate)
{
	_usartN->UDRn = _UDRn;
	_usartN->UCSRnB = _UCSRnB;
	_usartN->UDRIEn = _UDRIEn;
	*(_usartN->UCSRnB) |= (1<<RXENn) | (1<<TXENn) | (1<<RXCIEn);
	
	if(_baudRate == BAUD_115200)
	{
		*_UBRRn = (((F_CPU/(_baudRate*8UL)))-1);
		UCSR0A |= (1<<U2X0);
	}
	else
	{
		*_UBRRn = (((F_CPU/(_baudRate*16UL)))-1);
	}
	
}

USART* USART_Init(usart_num usartN, uint32_t baudRate, char* _receiveQueueArray, char* _transmitQueueArray){
		
	switch (usartN)
	{
	case USART_0:
		setUSARTRegisters(&usarts[usartN], &UDR0, &UCSR0B, UDRIE0, RXEN0, TXEN0, RXCIE0, &UBRR0, baudRate);
		break;
		
	case USART_1:
		setUSARTRegisters(&usarts[usartN], &UDR1, &UCSR1B, UDRIE1, RXEN1, TXEN1, RXCIE1, &UBRR1, baudRate);
		break;
		
	case USART_2:
		setUSARTRegisters(&usarts[usartN], &UDR2, &UCSR2B, UDRIE2, RXEN2, TXEN2, RXCIE2, &UBRR2, baudRate);
		break;
	
	case USART_3:
		setUSARTRegisters(&usarts[usartN], &UDR3, &UCSR3B, UDRIE3, RXEN3, TXEN3, RXCIE3, &UBRR3, baudRate);
		break;
	}
	
	usarts[usartN].ReceiveQueue = &receiveQueues[usartN];
	usarts[usartN].TransmitQueue = &transmitQueues[usartN];
	
	usarts[usartN].eventStore = createEventStore();
		
	initQueue(usarts[usartN].ReceiveQueue, _receiveQueueArray, QUEUE_CAPACITY);
	initQueue(usarts[usartN].TransmitQueue, _transmitQueueArray, QUEUE_CAPACITY);
	
	usarts[usartN].full = 0;
	usarts[usartN].FlowControl = 0;
	
	return &usarts[usartN];
}

// send a char
void USART_Transmit(unsigned char data, USART* usart){
	while (queueIsFull(usart->TransmitQueue))
	{
		//usart->full = 1;
		//dequeue(&transmitQueue);
		//enqeue('X', &transmitQueue);
	}
	enqeue(data, usart->TransmitQueue);
	*(usart->UCSRnB) |= (1<<usart->UDRIEn);
}

int16_t USART_Receive(USART* usart){
	if(!queueIsEmpty(usart->ReceiveQueue))
	{
		int16_t receiveData = dequeue(usart->ReceiveQueue);
		if(queueUsed(usart->ReceiveQueue) < 10 && eventIsSet(X_OFF_TRANSMITTED_Event, &(usart->eventStore)))
		{
			 // send Xon
			//USART_Transmit('E');	//send to show buffer is empty
			if(usart->FlowControl == 1)
			{
				USART_Transmit(0x11, usart);
				clearEvent(X_OFF_TRANSMITTED_Event, &(usart->eventStore));
			}	
		}
		return receiveData;
	}
	else return -1;
}

// send an int
void USART_SendInt(int data, USART* usart)
{
	char dataChar[32];
	sprintf(dataChar, "%d", data);
	
	for(int f=0;dataChar[f]!='\0';f++)
	{
		USART_Transmit(dataChar[f], usart);
	}
}

void USART_SendString(char *data, USART* usart)
{
	for(int f=0;data[f]!='\0';f++)
	{
		USART_Transmit(data[f], usart);
	}
}

void USART_SendFlashString(char *data, USART* usart)
{
	for(char* f = data; pgm_read_byte(f) != '\0'; f++)
	{
		USART_Transmit(pgm_read_byte(f), usart);
	}
}

void USART_ResetTerminal(USART* usart)
{
	USART_Transmit(27, usart);
	USART_Transmit('c', usart);
}

void USART_ClearLine(USART* usart)
{
	USART_Transmit(27, usart);
	USART_SendString((char*)"[2K", usart);
}

void UsartCursorSafe(USART* usart)
{
	USART_Transmit(27, usart);
	USART_SendString((char*)"7", usart);
}

void UsartCursorRestore(USART* usart)
{
	USART_Transmit(27, usart);
	USART_SendString((char*)"8", usart);
}

typedef uint8_t data_register; // USART Transmit Data Buffer Register and USART Receive Data Buffer Register

void USART_RECEIVE_ISR(USART* usartN)
{
	/*if(eventIsSet(X_OFF_TRANSMITTED_Event))
	{
		USART_Transmit('W');
	}*/ //to find out how many chars are arriving after Xoff was transmitted
	char receiveData = *(usartN->UDRn);
	if(receiveData == 0x13 && usartN->FlowControl == 1)
	{
		setEvent(X_OFF_RECEIVED_Event, &(usartN->eventStore));
	}
	else if(receiveData == 0x11 && usartN->FlowControl == 1)
	{
		clearEvent(X_OFF_RECEIVED_Event, &(usartN->eventStore));
	}
	else
	{
		enqeue(receiveData, usartN->ReceiveQueue);
		if(queueUsed(usartN->ReceiveQueue) > 80 && !eventIsSet(X_OFF_TRANSMITTED_Event, &(usartN->eventStore)))
		{		
			usartN->full = 1;
			USART_Transmit(0x13, usartN); // send Xoff
			//USART_Transmit('F'); //send to show buffer is full
			setEvent(X_OFF_TRANSMITTED_Event, &(usartN->eventStore));
		}
	}
}

void USART_TRANSMIT_ISR(USART* usartN)
{
	*(usartN->UDRn) = dequeue(usartN->TransmitQueue);
	
	if(queueIsEmpty(usartN->TransmitQueue))
	{
		*(usartN->UCSRnB) &= ~(1<<usartN->UDRIEn);
	}
}


ISR(USART0_RX_vect)
{
	USART_RECEIVE_ISR(&usarts[0]);
}

ISR(USART0_UDRE_vect)
{
	USART_TRANSMIT_ISR(&usarts[0]);
}

ISR(USART1_RX_vect)
{
	USART_RECEIVE_ISR(&usarts[1]);
}

ISR(USART1_UDRE_vect)
{
	USART_TRANSMIT_ISR(&usarts[1]);
}

ISR(USART2_RX_vect)
{
	USART_RECEIVE_ISR(&usarts[2]);
}

ISR(USART2_UDRE_vect)
{
	USART_TRANSMIT_ISR(&usarts[2]);
}

ISR(USART3_RX_vect)
{
	USART_RECEIVE_ISR(&usarts[3]);
}

ISR(USART3_UDRE_vect)
{
	USART_TRANSMIT_ISR(&usarts[3]);
}



#endif /* USART_H_ */