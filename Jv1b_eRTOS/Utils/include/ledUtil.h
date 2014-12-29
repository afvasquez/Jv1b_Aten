/*
 * ledUtil.h
 *
 * Created: 12/17/2014 3:55:06 PM
 *  Author: avasquez
 */ 


#ifndef LEDUTIL_H_
#define LEDUTIL_H_

#define partstDEFAULT_PORT_ADDRESS		( ( uint16_t ) 0x378 )

// Blinking rate constant declaration
#define LED_TASK_RATE_BASE 0x0534
#define LED_TASK_RATE_ERROR 0x0133

// Function Prototypes
void vParTestInitialise( void );								// I/O Port initialization
void vParTestSetLED( UBaseType_t uxLED, BaseType_t xValue );	// Utility function to set a given pin
void vParTestToggleLED( UBaseType_t uxLED );					// Utility function to toggle a given pin
void fnPrintString(const char *lnMessage);				// Utility function to print the string that is being pointed to

extern TaskHandle_t ledTaskHandle;		// LED Task Handle
extern TaskHandle_t comTaskHandle;		// Communication Task Handle
extern TickType_t ledTaskRate;			// LED Blinking Rate Variable
extern char txMode;						// Motor Mode Variable

#endif /* LEDUTIL_H_ */