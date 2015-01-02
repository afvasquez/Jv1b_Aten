/*
 * usart_task.c
 *
 * Created: 11/22/2014 11:44:15 PM
 *  Author: avasquez
 */ 

/* Scheduler include files. */
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "FreeRTOS.h"
#include "task.h"

/* Demo program include files. */
#include "ledUtil.h"
#include "serial.h"
#include "uartUtil.h"
#include "Motor.h"

/* The sequence transmitted is from comFIRST_BYTE to and including comLAST_BYTE. */
#define comFIRST_BYTE				( 'A' )
#define comLAST_BYTE				( 'D' )
#define comBUFFER_LEN				( ( UBaseType_t ) ( comLAST_BYTE - comFIRST_BYTE ) + ( UBaseType_t ) 1 )
#define menLINE_SIZE_HEADER			( 55  )
#define menLINE_SIZE_BODY			( 41  )
#define menCLEAR					(  8  )
#define menTextReady				(  4  )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( TickType_t ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( TickType_t ) 0xffff )

/* Handle to the com port used by both tasks. */
static xComPortHandle xPort = NULL; // Necessary to be able to send messages through the port
TaskHandle_t comTaskHandle;

/* The transmit task as described at the top of the file. */
static portTASK_FUNCTION( vRxTask, pvParameters );
void fnPrintMotorSettings(void);

//void fnPrintString(char* lnMessage, int lnSize); // Function Prototype
char txMode = 'N'; // Create and initialize a com port directive of NO_MODE
const char txClearReset[] PROGMEM = "\e[2J\e[H";
const char txStartMessA[] PROGMEM = "##################################\n";
const char txStartMessB[] PROGMEM = "### BastianRTOS: ProMat Distro ###\n";
const char txStartMessC[] PROGMEM = "### ATmega328P Port by Andres  ###\n";
const char txStartMessD[] PROGMEM = "###      December 2014         ###\n";
const char txStartMessE[] PROGMEM = "### Select from the options    ###\n";
const char txStartMessF[] PROGMEM = "### Press R to run the motor\n";
const char txStartMessG[] PROGMEM = "### Press S to stop the motor\n";
const char txStartMessH[] PROGMEM = "### Press + to increase the speed\n";
const char txStartMessI[] PROGMEM = "### Press - to decrease the speed\n";
//char txWater[5];
// This next variable holds the header to the UI. 

//////////////////////////////////////////////////////////////////////////
// Start the necessary tasks in the following function
void vStartSerialTask( UBaseType_t uxPriority, uint32_t ulBaudRate, UBaseType_t uxLED)
{	
	// Store the LED and Initialize the USART port - THIS was modded so that the LED is not needed
	(void) uxLED;
	xSerialPortInitMinimal(ulBaudRate, comBUFFER_LEN);
	
	// Start the Task
	xTaskCreate(vRxTask,"R",configMINIMAL_STACK_SIZE * 2,NULL,uxPriority, &comTaskHandle );
}

//////////////////////////////////////////////////////////////////////////
// Rx Task
char buffer[34];
static portTASK_FUNCTION( vRxTask, pvParameters )
{
	// Just to stop compiler warnings.
	( void ) pvParameters;
	//UBaseType_t ubtWM;	// Only Necesary when debugging through COM port
	
	/*fnPrintString(txClearReset);
	fnPrintString(txStartMessA);
	fnPrintString(txStartMessA);
	fnPrintString(txStartMessB);
	fnPrintString(txStartMessC);
	fnPrintString(txStartMessD);
	fnPrintString(txStartMessE);
	fnPrintString(txStartMessA);
	fnPrintString(txStartMessA);
	fnPrintString(txStartMessF);
	fnPrintString(txStartMessG);
	fnPrintString(txStartMessH);
	fnPrintString(txStartMessI); */
	
	signed char rxChar;
		
	for( ;; )
	{
		// This is can also be the Rx code to interface with the board
		// Block until the first byte is received
		if (xSerialGetChar(xPort, &rxChar, comRX_BLOCK_TIME))
		{
			// DO the following ONLY if the motor is present and functional
			if ( intPhase < 7 && intPhase > 0 ) {
				if ( rxChar == 'r' || rxChar == 'R' ) {	// If the character was R-r (RUN)
					if ( txMode == 'N' ) {	// If the Motor is at Rest
						txMode = 'r';	// Switch Motor mode to RAMP up
						intOCR2B_Int_S = MOTOR_POWER_BASE;
						clkElapsed = 0;	// Reset the RAMP timer
						//vTaskResume( vMotorRamp_Handle );
					} else if ( txMode == 's' ) {	// If the mode was called up to cancel the ramp down
						txMode = 'S';	// Switch the mode to Ramp Up
						intOCR2B_Int_S = OCR2B;
						clkElapsed = 0;	// Reset the RAMP timer
					}
				} else if ( rxChar == 's' || rxChar == 'S' ) {	// If the character was S-s (STOP)
					if ( txMode == 'R' )	{	// If the motor is Running
						txMode = 's';	// Switch the mode to RAMP down
							// Only stop the duration timer if it is actually active
						/*if (xTimerIsTimerActive(durationHandle) == pdTRUE) {    //  Disable Timer
							xTimerStop(durationHandle, 100);
						}*/
						intOCR2B_Int = motPower;
						// Re-calculate the rate-down 
						rate_down = (float)(((motPower - MOTOR_POWER_BASE)/(motRampDown * 100.0)));
						clkElapsed = 0;	// Reset the RAMP timer
						// ####### Resume the Ramp Up Task
						//vTaskResume( vMotorRamp_Handle );
					} else if ( txMode == 'r' ) {
						txMode = 'E';
						intOCR2B_Int = OCR2B;
						clkElapsed = 0;
					}
				}
				
				if (rxChar == 'P') { // Send Stats
					fnPrintWaterMark(); // Send Watermarks
				} else if (rxChar == 'O') { // Export the run-time data when asked for it
					fnPrintMotorSettings();
				} else if (rxChar == '&') { // Update all the necessary values and perform direction change if need be
					// Start by updating all currently-passive data
					
					if (txMode == 'N') { // If the motor is not running, just update
						xSerialGetChar(xPort, &rxChar, comRX_BLOCK_TIME); //  Update the motor-power
						motPower = rxChar;
						// Get the relevant byte
						xSerialGetChar(xPort, &rxChar, comRX_BLOCK_TIME);
						rate_up = (float)(((motPower - MOTOR_POWER_BASE)/(rxChar * 100.0)));
						motRampUp = rxChar;
						xSerialGetChar(xPort, &rxChar, comRX_BLOCK_TIME);
						rate_down = (float)(((motPower - MOTOR_POWER_BASE)/(rxChar * 100.0)));
						motRampDown = rxChar;
						xSerialGetChar(xPort, &rxChar, comRX_BLOCK_TIME);	// Ramp the speed change IF running, else, just change the top power allowed and rates
						motDirDur = rxChar;
						motDir = motDirDur & 0x40;
						motDur = motDirDur & 0x3F;
						xSerialPutChar( xPort, motDirDur, 100 );
					}
				} else if (rxChar == '1') {
					if (txMode == 'N') {
						goal = 190;
						flagLoop = !flagLoop;
					}
				} else if (rxChar == '2') {
					if (txMode == 'N') {
						goal = -190;
						flagLoop = !flagLoop;
					}
				} else if (rxChar == '3') {
					if (txMode == 'N') {
						goal = 190 * 2;
						flagLoop = !flagLoop;
					}
				} else if (rxChar == '4') {
					if (txMode == 'N') {
						goal = -190 * 2;
						flagLoop = !flagLoop;
					}
				}
			} else {	// The motor is either not present OR it is not functional
				PORTC = mtrCW_Rotate[7];	// Clamp the Motor
				txMode = 'N';	// Emphasize Motor Status of not-running
			}
		}
		// Get the available size of the queues
		//fnPrintWaterMark(uxQueueSpacesAvailable(xRxedChars), 5);
		//fnPrintWaterMark(uxQueueSpacesAvailable(xCharsForTx), 6); 
	}	
}

//////////////////////////////////////////////////////////////////////////
// The following is not a task, this is a function that will print the
//     given string on RS232
void fnPrintString(const char *lnMessage)
{
	while ( pgm_read_byte(lnMessage) != 0x00 ) {
		xSerialPutChar( xPort, pgm_read_byte(lnMessage++), 100 );	
	}
}
const char chTaskA[] PROGMEM = "Rx Task\t";
const char chTaskB[] PROGMEM = "LED Task\t";
const char chTaskC[] PROGMEM = "Rmp Task\t";
const char chTaskD[] PROGMEM = "Mot Task\t";
const char chTaskE[] PROGMEM = "RxQ Task\t";
const char chTaskF[] PROGMEM = "TxQ Task\t";
const char chNL[] = "\n";
void fnPrintWaterMark(void) {
	// Print the name 
	// Send Watermark Delimiter
	xSerialPutChar( xPort, '*', 100 );
	xSerialPutChar( xPort, '*', 100 );
	xSerialPutChar( xPort, uxTaskGetStackHighWaterMark(comTaskHandle), 100 );
	xSerialPutChar( xPort, uxTaskGetStackHighWaterMark(ledTaskHandle), 100 );
	//xSerialPutChar( xPort, uxTaskGetStackHighWaterMark(vMotorRamp_Handle), 100 );
	xSerialPutChar( xPort, uxTaskGetStackHighWaterMark(vMotor_Handle), 100 );
	
	xSerialPutChar( xPort, '*', 100 );
	// Send Watermark Delimiter
}

void fnPrintMotorSettings(void) {
	// Print the name
	// Send Watermark Delimiter
	xSerialPutChar( xPort, '&', 100 );
	xSerialPutChar( xPort, '&', 100 );
	xSerialPutChar( xPort, (signed char) motPower, 100 );
	xSerialPutChar( xPort, (signed char) motRampUp, 100 );
	xSerialPutChar( xPort, (signed char) motRampDown, 100 );
	// need to take both data entries and consolidate (MUST COMPLETE)
	xSerialPutChar( xPort, (signed char) motDirDur, 100 );
	
	xSerialPutChar( xPort, '&', 100 );
	// Send Watermark Delimiter
}
