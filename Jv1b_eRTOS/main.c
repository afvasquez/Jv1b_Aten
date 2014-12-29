/*
 * main.c
 *
 * Created: 11/22/2014 11:39:43 AM
 *  Author: avasquez
 */ 
 
 // Task Priorities
 #define mainLED_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
 #define mainMOTOR_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
 #define mainCOM_TEST_BAUD_RATE			( ( unsigned long ) 9600 )
 
 /* An address in the EEPROM used to count resets.  This is used to check that
 the demo application is not unexpectedly resetting. */
 #define mainRESET_COUNT_ADDRESS			( ( void * ) 0x50 )
 
 /* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

 // Board testing - Include files
	// Blinking LED files
#include "ledUtil.h"
#include "flash.h"
#include "uartUtil.h"
 
 // Motor control includes 
#include "Motor.h" 

#include <avr/eeprom.h>
 
 // Prototype of the ApplicationIdleHook
 void vApplicationTickHook( void );
 
 /*
 * Called on boot to increment a count stored in the EEPROM.  This is used to
 * ensure the CPU does not reset unexpectedly.
 */
static void prvIncrementResetCount( void );
 
 //////////////////////////////////////////////////////////////////////////
 // MAIN
 //////////////////////////////////////////////////////////////////////////
 int main (void)
 {
	// prvIncrementResetCount();
	 
	vParTestInitialise();
	motor_setup();
	
	// Set the initial value of the pin....
	//vParTestSetLED((portBASE_TYPE) 6, pdFALSE);
	
	// Initialize the LED tasks
	// Initialize LED task rate
	ledTaskRate = LED_TASK_RATE_BASE;
	
	vStartLEDFlashTasks( mainLED_TASK_PRIORITY );
	vStartMotorTask( mainMOTOR_TASK_PRIORITY );
	vStartSerialTask( mainLED_TASK_PRIORITY, mainCOM_TEST_BAUD_RATE, 20 );
	
	
	/* Start the RTOS scheduler, this function should not return as it causes the execution
        context to change from main() to one of the created tasks. */
	vTaskStartScheduler();
	return 0;
 }

 void vApplicationTickHook( void )
 {
	clkElapsed++; // Increment the timer
 }
 
 static void prvIncrementResetCount( void )
 {
	 unsigned char ucCount;

	 eeprom_read_block( &ucCount, mainRESET_COUNT_ADDRESS, sizeof( ucCount ) );
	 ucCount++;
	 eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount );
 }
 