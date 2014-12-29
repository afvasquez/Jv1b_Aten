
/* BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER. */


#include <stdlib.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "serial.h"
#include "ledUtil.h"

#define serBAUD_DIV_CONSTANT			( ( unsigned long ) 16 )

/* Constants for writing to UCSRB. */
#define serRX_INT_ENABLE				( ( unsigned char ) 0x80 )
#define serRX_ENABLE					( ( unsigned char ) 0x10 )
#define serTX_ENABLE					( ( unsigned char ) 0x08 )
#define serTX_INT_ENABLE				( ( unsigned char ) 0x20 )

/* Constants for writing to UCSRC. */
#define serUCSRC_SELECT					( ( unsigned char ) 0x80 )
#define serEIGHT_DATA_BITS				( ( unsigned char ) 0x06 )

QueueHandle_t xRxedChars; 
QueueHandle_t xCharsForTx; 

#define vInterruptOn()										\
{															\
	unsigned char ucByte;								\
															\
	ucByte = UCSR0B;											\
	ucByte |= serTX_INT_ENABLE;								\
	UCSR0B = ucByte;											\
}																				
/*-----------------------------------------------------------*/

#define vInterruptOff()										\
{															\
	unsigned char ucInByte;								\
															\
	ucInByte = UCSR0B;										\
	ucInByte &= ~serTX_INT_ENABLE;							\
	UCSR0B = ucInByte;										\
}
/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
unsigned long ulBaudRateCounter;
unsigned char ucByte;

	portENTER_CRITICAL();
	{
		/* Create the queues used by the com test task. */
		xRxedChars = xQueueCreate( uxQueueLength, sizeof( char ) );
		xCharsForTx = xQueueCreate( uxQueueLength, sizeof( char ) );

		/* Calculate the baud rate register value from the equation in the
		data sheet. */
		ulBaudRateCounter = ( configCPU_CLOCK_HZ / ( serBAUD_DIV_CONSTANT * ulWantedBaud ) ) - ( unsigned long ) 1;

		/* Set the baud rate. */	
		ucByte = ( unsigned char ) ( ulBaudRateCounter & ( unsigned long ) 0xff );	
		UBRR0L = ucByte;

		ulBaudRateCounter >>= ( unsigned long ) 8;
		ucByte = ( unsigned char ) ( ulBaudRateCounter & ( unsigned long ) 0xff );	
		UBRR0H = ucByte;

		/* Enable the Rx interrupt.  The Tx interrupt will get enabled
		later. Also enable the Rx and Tx. */
		//UCSR0B = ( serRX_INT_ENABLE | serRX_ENABLE | serTX_ENABLE );

		/* Set the data bits to 8. */
		//UCSR0C = ( serUCSRC_SELECT | serEIGHT_DATA_BITS );
		UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);	// 8-bit data -- 1-Stop Bit automatically set
		UCSR0B = _BV(TXEN0) | _BV(RXCIE0) | _BV(RXEN0);	// Enable TX
	}
	portEXIT_CRITICAL();
	
	/* Unlike other ports, this serial code does not allow for more than one
	com port.  We therefore don't return a pointer to a port structure and can
	instead just return NULL. */
	return NULL;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* Only one port is supported. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
	/* Only one port is supported. */
	( void ) pxPort;

	//vParTestSetLED((portBASE_TYPE) 7, pdTRUE);
	/* Return false if after the block time there is no room on the Tx queue. */
	if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) != pdPASS )
	{
		//vParTestSetLED((portBASE_TYPE) 7, pdTRUE);
		return pdFAIL;
	}
	
	//vParTestSetLED((portBASE_TYPE) 7, pdTRUE);
	vInterruptOn();
	

	return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
unsigned char ucByte;

	/* The parameter is not used. */
	( void ) xPort;

	/* Turn off the interrupts.  We may also want to delete the queues and/or
	re-install the original ISR. */

	portENTER_CRITICAL();
	{
		vInterruptOff();
		ucByte = UCSR0B;
		ucByte &= ~serRX_INT_ENABLE;
		UCSR0B = ucByte;
	}
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

//signed char lastChar = 0x00;
SIGNAL( USART_RX_vect )
{
signed char cChar;
signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/* Get the character and post it on the queue of Rxed characters.
	If the post causes a task to wake force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
	cChar = UDR0;

	// Anti spam logic
	//if () // If the current character is not a spammed character from the list, send to queue
	//{
	//}

	xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken != pdFALSE )
	{
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

SIGNAL( USART_UDRE_vect )
{
signed char cChar, cTaskWoken;

	if( xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
	{
			UDR0 = cChar;
	}
	else
	{
		/* Queue empty, nothing to send. */
		vInterruptOff();
	}
}

