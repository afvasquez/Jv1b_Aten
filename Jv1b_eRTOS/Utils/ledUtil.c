/*
 * ledUtil.c
 *
 * Created: 12/17/2014 3:54:28 PM
 *  Author: avasquez
 */ 


#include "FreeRTOS.h"
#include "task.h"
#include "ledUtil.h"

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

#define partstALL_BITS_OUTPUT			( ( unsigned char ) 0xff )	// Useful Variable Declarations
#define partstALL_OUTPUTS_OFF			( ( unsigned char ) 0x00 )	
#define partstMAX_OUTPUT_LED			( ( unsigned char ) 7 )
#define portB_DIRECTIONS				( ( unsigned char ) 0b00000000 )
#define portC_DIRECTIONS				( ( unsigned char ) 0b00111111 )
#define portC_INIT_OUTPUT				( ( unsigned char ) 0b00000111 )
#define portD_DIRECTIONS				( ( unsigned char ) 0b00011100 )

// Variable definitions
static volatile unsigned char ucCurrentOutputValue = partstALL_OUTPUTS_OFF; /*lint !e956 File scope parameters okay here. */
	// Defining the drive table

/*-----------------------------------------------------------*/

void vParTestInitialise( void )
{
	ucCurrentOutputValue = partstALL_OUTPUTS_OFF;
	
	// Set MOTOR_DRIVE Circuitry to the right value as to not short anything out
	DDRC = portC_DIRECTIONS;
	PORTC = portC_INIT_OUTPUT;
	
	// Set the necessary ports on PD
	DDRD = portD_DIRECTIONS;
	
	/* Set the right ports of PB to be outputs */
	// Set the right ports to be outputs with local binary values
	DDRB = portB_DIRECTIONS; 
	PORTB = partstALL_OUTPUTS_OFF; // Turn off both outputs
	
}
/*-----------------------------------------------------------*/

void vParTestSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue )
{
unsigned char ucBit = ( unsigned char ) 1;

	if( uxLED <= partstMAX_OUTPUT_LED )
	{
		ucBit <<= uxLED;	

		vTaskSuspendAll();
		{
			if( xValue == pdTRUE )
			{
				ucCurrentOutputValue |= ucBit;
			}
			else
			{
				ucBit ^= ( unsigned char ) 0xff;
				ucCurrentOutputValue &= ucBit;
			}

			PORTD = ucCurrentOutputValue;
		}
		xTaskResumeAll();
	}
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned portBASE_TYPE uxLED )
{
unsigned char ucBit;

	if( uxLED <= partstMAX_OUTPUT_LED )
	{
		ucBit = ( ( unsigned char ) 1 ) << uxLED;

		vTaskSuspendAll();
		{
			if( ucCurrentOutputValue & ucBit )
			{
				ucCurrentOutputValue &= ~ucBit;
			}
			else
			{
				ucCurrentOutputValue |= ucBit;
			}

			PORTD = ucCurrentOutputValue;
		}
		xTaskResumeAll();			
	}
}