/*
 * Motor.c
 *
 * Created: 12/5/2014 10:05:35 AM
 *  Author: avasquez
 */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "ledUtil.h"

// Include the necessary pin name libraries 
#include "Motor.h"


// AUX function prototypes
// Motor Driving Task Function Prototype
static portTASK_FUNCTION_PROTO( vMotorDriveTask, pvParameters );
static portTASK_FUNCTION_PROTO( vMotorRampTask, pvParameters );

	// Local declaration of the motor drive handles
TaskHandle_t vMotorRamp_Handle;
TaskHandle_t vMotor_Handle;

// Function Prototypes
void getPhaseCode(void);
void vTimerCallback(TimerHandle_t pxTimer);

// CW Motor Lookup Table
const char mtrCW_Rotate[8] = { 0x3F, 0x26, 0x0D, 0x25, 0x13, 0x16, 0x0B, 0x07 };
const char mtrCCW_Rotate[8] = { 0x3F, 0x0B, 0x16, 0x13, 0x25, 0x0D, 0x26, 0x07 };
// Current Motor Phase status
char intPhase = 0xFF;	// FF being the starting CODE
float intOCR2B_Int;
char intOCR2B_Int_S;
TickType_t clkElapsed;
float rate_up;
float rate_down;
SemaphoreHandle_t motor_semaphore;

// ################### Motor parameters
char motPower = MOTOR_MIN_POWER;
char motRampUp = MOTOR_TIME_DELTA_SCL;
char motRampDown = MOTOR_TIME_DELTA_SCL;
char motDirDur = MOTOR_DIRDUR_DEFAULT;
char motDir = 0x00;
char motDur = 0x05;
char runDur = 0;
TimerHandle_t durationHandle;
	
// Initialization function
void motor_setup(void) {
	PORTC = mtrCW_Rotate[7];
	rate_up = motPower/MOTOR_TIME_DELTA;
	rate_down = rate_up;
	// Check for the First HALL effect sensor state
	getPhaseCode();
	
	// Create the Binary semaphore
	motor_semaphore = xSemaphoreCreateBinary();
	
	
	// ############ Set the motor timer
	//xTimerCreate( "Timer", ( 1000 ), pdTRUE, ( void * ) 1, vTimerCallback );
	durationHandle = xTimerCreate("Timer", 1000, pdTRUE, 1, vTimerCallback);
	
	// Reset the elapsed time (in thousands of a second)
	clkElapsed = 0x0000;	
	
	// Set the necessary features to make the motor run
	// Set the corresponding Fast PWM Pin
	// Enable the Port as an output
	DDRD |= (1 << PORTD3);
	
	// TIMER STUFF
	// Set the FAST PWM as non-inverting
	TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	// Set the Pre-Scaler
	TCCR2B = (1 << CS21);
	
	OCR2B = MOTOR_POWER_BASE; // Duty Cycle - Initialize to the very lowest base speed
	
	// Set the necessary PCINT settings for PB0-2
	PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
	PCMSK0 |= 0b00000111;  // set PCINT0 to trigger an interrupt on state change
}

//////////////////////////////////////////////////////////////////////////
// MOTOR Drive Functions
// 
//////////////////////////////////////////////////////////////////////////
void vStartMotorTask( UBaseType_t uxPriority) {	
	// Start the Motor Function
	xTaskCreate( vMotorDriveTask, "M", configMINIMAL_STACK_SIZE*2, NULL, uxPriority+1, &vMotor_Handle );
	xTaskCreate( vMotorRampTask, "R", configMINIMAL_STACK_SIZE*2, NULL, uxPriority, &vMotorRamp_Handle );
}

static portTASK_FUNCTION( vMotorDriveTask, pvParameters )
{
	char cInputs; // Getting the Input that governs the OVERCURRENT
	UBaseType_t ubtWM;	// Only Necesary when debugging through COM port
	
	// Start into the infinite loop right away
	for (;;)
	{
		// Yield through its dedicated semaphore
		if (xSemaphoreTake(motor_semaphore, portMAX_DELAY))
		{
			cInputs = PIND & 0b10100000; // Get the Over-Current Flag
			if ((txMode == 'R' || txMode == 'r' || txMode == 's' || txMode == 'E' || txMode == 'S' ) && (cInputs & 0b00100000))
			{	// And Check for Overpowering!! -------------------------^
					// The motor is about to run in this routine ( CW Direction Only )
					if (motDir > 0) {
						PORTC = mtrCCW_Rotate[( PINB & 0b00000111 )];
					} else {
						PORTC = mtrCW_Rotate[( PINB & 0b00000111 )];
					}
				} else if ( txMode == 'N' ) {	// If (In any case) the Motor State is N, Clamp down and set base PWM
					PORTC = mtrCW_Rotate[7];
					OCR2B = MOTOR_POWER_BASE;				
				} else {	// OVERPOWERING SCENARIO
					portENTER_CRITICAL();
						vTaskSuspend(vMotorRamp_Handle);
						txMode = 'N'; // Clamped state
						PORTC = mtrCW_Rotate[7];	// CLAMP ASAP!!
						if (xTimerIsTimerActive(durationHandle) == pdTRUE) {
							xTimerStop(durationHandle, 100);
						}
						OCR2B = MOTOR_POWER_BASE;	// Reset to Base Power
						ledTaskRate = LED_TASK_RATE_ERROR;	// Clamp and throw error light
					portEXIT_CRITICAL();
			}
		}	
	}
}

//////////////////////////////////////////////////////////////////////////
// Motor Ramping Task
static portTASK_FUNCTION( vMotorRampTask, pvParameters ) {
	// Initialize the last time this task was woken
	TickType_t rampLastTick;
	const TickType_t xFrequency = 1;	// Ramp Algorithm Frequency when Ramping
	//UBaseType_t ubtWM;	// Only Necesary when debugging through COM port
	
	// ########## Suspend SELF
	vTaskSuspend( NULL );
	
	rampLastTick = xTaskGetTickCount();	// Record the last wake time
	
	for (;;)	// Loop this task FOREVER
	{
		if ( txMode == 'r' || txMode == 'S') // This code does the ramp up
		{
			// Limit the giving of the semaphore to the first entry
			if (((OCR2B < motPower) && (OCR2B < MOTOR_MAX_POWER)) && OCR2B == intOCR2B_Int_S ) { 
				OCR2B = intOCR2B_Int_S + (clkElapsed * rate_up);
				xSemaphoreGive(motor_semaphore);	// Give the Semaphore
				vTaskDelayUntil(&rampLastTick, xFrequency);	// vTaskDelayUntil to the next millisecond
			} else if ((OCR2B < motPower) && (OCR2B < MOTOR_MAX_POWER)) { // (clkElapsed < MOTOR_TIME_DELTA) &&
				OCR2B = intOCR2B_Int_S + (clkElapsed * rate_up);
				vTaskDelayUntil(&rampLastTick, xFrequency);	// vTaskDelayUntil to the next millisecond
			} else {		// If the Ramp just simply timed out
				txMode = 'R';
				// If the motor is already at set speed, suspend the task
				runDur = 0x00;
				xTimerStart(durationHandle, 100);
				vTaskSuspend( NULL );	// Suspend Self
			}
		} else if (txMode == 's' || txMode == 'E') {	// If the motor is being stopped
			if (OCR2B == MOTOR_POWER_BASE) {
				txMode = 'N';
				if (xTimerIsTimerActive(durationHandle) == pdTRUE) {
					xTimerStop(durationHandle, 100);
				}
				vTaskSuspend( NULL );
			} else { 
				OCR2B = (char)(intOCR2B_Int - (clkElapsed * rate_down));
				vTaskDelayUntil(&rampLastTick, xFrequency);	// vTaskDelayUntil to the next millisecond
			}
		} 
		
		
		
		// Instead, sent to COM
		//fnPrintWaterMark(OCR2B, exitMode);
		
		// %%%%%%%%%%% Debugging Purposes
		//ubtWM = uxTaskGetStackHighWaterMark(NULL);
		//fnPrintWaterMark(ubtWM,3);
	}
}

// This function builds the code sent by the motor on the different pins
void getPhaseCode(void) {
	intPhase = PINB & 0b00000111; // Clear the previous status
}

// Software Timer callback
void vTimerCallback(TimerHandle_t pxTimer) {
	// Perform the duration control algorithm
	if (motDur > 0 && txMode == 'R')
	{
		runDur++;
		if (runDur >= motDur)	// If duration is up
		{
			runDur = 0;
			txMode = 's';	// Switch the mode to RAMP down
			
			intOCR2B_Int = motPower;
			// Re-calculate the rate-down
			rate_down = (float)(((motPower - MOTOR_POWER_BASE)/(motRampDown * 100.0)));
			clkElapsed = 0;	// Reset the RAMP timer
			
			
			// ####### Resume the Ramp Up Task
			vTaskResume( vMotorRamp_Handle );
			xTimerStop(durationHandle, 100);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// ISR Functions
ISR (HALL_0_Vect) {
	// Define the woken task variable
	int motor_task_woken = 0;
	
	// Skip if in the "N" State
	if ( txMode != "N" ) {
		// Give the semaphore
		xSemaphoreGiveFromISR(motor_semaphore, &motor_task_woken);
		
		// If the task was woken
		if (motor_task_woken) {
			taskYIELD();
		}
	}
}

