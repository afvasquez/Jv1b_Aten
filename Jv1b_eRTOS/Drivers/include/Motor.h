/*
 * Motor.h
 *
 * Created: 12/5/2014 9:57:38 AM
 *  Author: avasquez
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_
// Motor Control Definitions
#define MOTOR_POWER_BASE 0.0	// Starting power
#define MOTOR_MAX_POWER  255.0
#define MOTOR_MIN_POWER  255.0	// Minimum Running Power
#define MOTOR_TIME_DELTA 3000.0	// Number of time ticks
#define MOTOR_TIME_DELTA_SCL 30 // Default Scaled Ramp Duration
#define MOTOR_DIRDUR_DEFAULT 0x05	// Default direction/duration variable bit8=Direction (0)-For CW bits7-0= Duration Value
#define MOTOR_TIME_STEPS 1		// Ramp-up Time Resolution
#define MOTOR_POWER_STEP 1		// Rate at which the Power is delivered

// Include the necessary libraries
#include <avr/io.h>
#include <avr/interrupt.h>

#include "timers.h"
#include "semphr.h"

// PORTABLE PIN DEFINITIONS
#define MOTOR_OUT_PORT PORTC // PORTC is the motor output port for demo and mtrCW_Rotate table must be coded accordingly
#define HALL_A PINB0
#define HALL_B PINB1
#define HALL_C PINB2  // These pins must be set previously in the Peripherals Library .h file
	// Remember to set the interrupt registers properly in the source code file <<--- IMPORTANT!!!!

#define HALL_0_Vect PCINT0_vect	// Interrupt vector for first set of input pin interrupts
#define HALL_1_Vect PCINT2_vect	// Interrupt vector for second set of input pin interrupts

// Function prototypes
void motor_setup(void);
void vStartMotorTask( UBaseType_t uxPriority);

// CW Motor Lookup Table
extern const char mtrCW_Rotate[8];
extern TickType_t clkElapsed;
extern char intPhase;
	// Global declaration of the motor drive handles
extern TaskHandle_t vMotorRamp_Handle;
extern TaskHandle_t vMotor_Handle;
extern SemaphoreHandle_t motor_semaphore;
extern float intOCR2B_Int;
extern float rate_up;
extern float rate_down;
extern char motPower;
extern char motRampUp;
extern char motRampDown;
extern char motDirDur;
extern char motDur;
extern char motDir;
extern TimerHandle_t durationHandle;

extern char intOCR2B_Int_S;

#endif /* MOTOR_H_ */