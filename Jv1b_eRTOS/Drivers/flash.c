/**
 * This version of flash .c is for use on systems that have limited stack space
 * and no display facilities.  The complete version can be found in the 
 * Demo/Common/Full directory.
 * 
 * Three tasks are created, each of which flash an LED at a different rate.  The first 
 * LED flashes every 200ms, the second every 400ms, the third every 600ms.
 *
 * The LED flash tasks provide instant visual feedback.  They show that the scheduler 
 * is still operational.
 *
 */


#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "ledUtil.h"
#include "flash.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 1 )
#define ledFLASH_RATE_BASE	( ( TickType_t ) 333 )

/* Variable used by the created tasks to calculate the LED number to use, and
the rate at which they should flash the LED. */
TaskHandle_t ledTaskHandle;
TickType_t ledTaskRate;

/* The task that is created two times. */
static portTASK_FUNCTION_PROTO( vLEDFlashTask, pvParameters );

/*-----------------------------------------------------------*/

void vStartLEDFlashTasks( UBaseType_t uxPriority ) {
	// Create the LED blinking task
	xTaskCreate( vLEDFlashTask, "L", ledSTACK_SIZE * 2 , NULL, uxPriority, &ledTaskHandle );
}
/*-----------------------------------------------------------*/

static portTASK_FUNCTION( vLEDFlashTask, pvParameters )
{
TickType_t xLastFlashTime;

	/* We need to initialize xLastFlashTime prior to the first call to 
	vTaskDelayUntil(). */
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
		vTaskDelayUntil( &xLastFlashTime, ledTaskRate );
		vParTestToggleLED( 2 );	// In our application, we only make use of the second LED on the PORT
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

