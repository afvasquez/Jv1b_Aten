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
//static portTASK_FUNCTION_PROTO( vMotorRampTask, pvParameters );
static portTASK_FUNCTION( vPIDControlLoop, pvParameters ); 
void pid_motor_command(float val);

	// Local declaration of the motor drive handles
//TaskHandle_t vMotorRamp_Handle;
TaskHandle_t vMotor_Handle;

// Function Prototypes
void getPhaseCode(void);
void vTimerCallback(TimerHandle_t pxTimer);

//////////////////////////////////////////////////////////////////////////
//	PID Motor Control Variables
int measured_value = PID_MEAS_VALUE_INIT;	// Create and initialize measured value (Change only when interrupt)
float Kp = 2.7;	// Create and initialize PID constants based on 20 increment
double Ki = 0.009;   // 0.3
double Kd = 0.001;
//int setpoint = PID_MEAS_VALUE_INIT-((5.0*6.0*4.0)+9);		// Create and initialize setpoint to be the same as the measured value
#define PID_FOLLOWER_INC 2
int setpoint = PID_MEAS_VALUE_INIT;		// Create and initialize setpoint to be the same as the measured value
char flagLoop = 0x00;
int goal = 0;
int error;									// Create error value
int previous_error = 0;		// Create and initialize Previous Error value
float integral = PID_INTEGRAL_INIT;			// Create and initialize integral value
float derivative;								// Create derivative value
float output;								// Create output variable

// CW Motor Lookup Table
const char mtrCW_Rotate[8] = { 0x07, 0x26, 0x0D, 0x25, 0x13, 0x16, 0x0B, 0x3F }; // 07 for no resistance
const char mtrCCW_Rotate[8] = { 0x07, 0x0B, 0x16, 0x13, 0x25, 0x0D, 0x26, 0x3F };
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
//TimerHandle_t durationHandle;
	
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
	//durationHandle = xTimerCreate("Timer", 1000, pdTRUE, 1, vTimerCallback);
	
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
	//xTaskCreate( vMotorRampTask, "R", configMINIMAL_STACK_SIZE*2, NULL, uxPriority, &vMotorRamp_Handle );
	xTaskCreate( vPIDControlLoop, "P", configMINIMAL_STACK_SIZE*3, NULL, uxPriority, NULL );
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
			// if ((txMode == 'R' || txMode == 'r' || txMode == 's' || txMode == 'E' || txMode == 'S' ) && (cInputs & 0b00100000))
			if ((txMode == 'R' || txMode == 'r' || txMode == 's' || txMode == 'E' || txMode == 'S' ))
			{	// And Check for Overpowering!! -------------------------^
					// The motor is about to run in this routine ( CW Direction Only )
					if (OCR2B > 0)
					{
						if (motDir > 0) {
							PORTC = mtrCCW_Rotate[( PINB & 0b00000111 )];
							} else {
							PORTC = mtrCW_Rotate[( PINB & 0b00000111 )];
						}
					} else {
						PORTC = mtrCW_Rotate[0];
					}
				} else if ( txMode == 'N' ) {	// If (In any case) the Motor State is N, Clamp down and set base PWM
					PORTC = mtrCW_Rotate[7];
					OCR2B = MOTOR_POWER_BASE;				
				} else {	// OVERPOWERING SCENARIO
					portENTER_CRITICAL();
						//vTaskSuspend(vMotorRamp_Handle);
						txMode = 'N'; // Clamped state
						PORTC = mtrCW_Rotate[7];	// CLAMP ASAP!!
						/*if (xTimerIsTimerActive(durationHandle) == pdTRUE) {  // Disable timer
							xTimerStop(durationHandle, 100);
						}*/
						OCR2B = MOTOR_POWER_BASE;	// Reset to Base Power
						ledTaskRate = LED_TASK_RATE_ERROR;	// Clamp and throw error light
					portEXIT_CRITICAL();
			}
		}	
	}
}

//////////////////////////////////////////////////////////////////////////
// Motor Ramping Task
/*static portTASK_FUNCTION( vMotorRampTask, pvParameters ) {
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
				//xTimerStart(durationHandle, 100);
				vTaskSuspend( NULL );	// Suspend Self
			}
		} else if (txMode == 's' || txMode == 'E') {	// If the motor is being stopped
			if (OCR2B == MOTOR_POWER_BASE) {
				txMode = 'N';
				/*if (xTimerIsTimerActive(durationHandle) == pdTRUE) {     // Disable timer
					xTimerStop(durationHandle, 100);
				}*/
/*				vTaskSuspend( NULL );
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
}*/

// This function builds the code sent by the motor on the different pins
void getPhaseCode(void) {
	intPhase = PINB & 0b00000111; // Clear the previous status
}

// Software Timer callback
/*
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
} */

//////////////////////////////////////////////////////////////////////////
// Motor Ramping Task
static portTASK_FUNCTION( vPIDControlLoop, pvParameters ) {
	// Initialize the last time this task was woken
	TickType_t rampLastTick;
	
	previous_error = measured_value;
	
	rampLastTick = xTaskGetTickCount();	// Record the last wake time
	for (;;)	// Loop this task FOREVER
	{	// Start the PID control Loop
		if (goal > setpoint) {
			setpoint += PID_FOLLOWER_INC;
			if (goal < setpoint) {
				setpoint = goal;
			}
		} else if (goal < setpoint) {
			setpoint -= PID_FOLLOWER_INC;
			if (goal > setpoint) {
				setpoint = goal;
			}
		} else {
			// Needs to retract
			if (flagLoop) {
				flagLoop = !flagLoop;
				goal = 0;
			} else {
				txMode = 'N';
			}
		}
		
		error = setpoint - measured_value;
		integral = integral + (error * 0.001);	// Assuming that the PID loop is visited every ms
		derivative = (error - previous_error) * 1000;	// Assuming that the PID loop is visited every ms
		output = Kp*error + Ki*integral + Kd*derivative;	// generate output
		pid_motor_command(output);
		xSemaphoreGive(motor_semaphore);	// Give the Semaphore
		//if (output != 0) {
			//
			//xSerialPutChar( NULL, 'H', 100 );
			//if (error < 127.0 && error > -128) {
				//xSerialPutChar( NULL, (signed char) error , 100 );
			//} else {
				//xSerialPutChar( NULL, 0xBE , 100 );
			//}
			//xSerialPutChar( NULL, '\t' , 100 );
			//xSerialPutChar( NULL, (signed char) output , 100 );
			//xSerialPutChar( NULL, 'h', 100 );
		//}
		previous_error = error;
		vTaskDelayUntil(&rampLastTick, 10);	// vTaskDelayUntil to the next millisecond
	}
}

//////////////////////////////////////////////////////////////////////////
// PID Motor Command Function
void pid_motor_command(float val)	{
	// Send the necessary value to the PWM and Motor Drive
	if (val <= -3.0) {	// Set CCW Rotation
		txMode = 'R';
		motDir = 0x40;
		val = (val * -1) * 2.55;
		if (val > 99) {
			OCR2B = 255;
		} else {
			OCR2B = (char) val;
		}
	} else if (val >= 3.0) {
		txMode = 'R';
		motDir = 0x00;
		val = (val * 2.55);
		if (val > 99) {
			OCR2B = 255;
			} else {
			OCR2B = (char) val;
		}
	} else {
		//PORTC = mtrCW_Rotate[7];	// CLAMP
		//txMode = 'N';
		OCR2B = 0x00;	// Turn off the PWM
	}
}

//////////////////////////////////////////////////////////////////////////
// ISR Functions
ISR (HALL_0_Vect) {
	// Define the woken task variable
	int motor_task_woken = 0;
	
	// Re-evaluate measured value
	if (motDir > 0) {
		// Motor is spinning CCW, decrease measured value;
		measured_value--;
	} else if (motDir == 0) {
		// Motor is spinning CW, increase measured value
		measured_value++;
	}	// Ignore change if the belt is moved by an outside force
	
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

