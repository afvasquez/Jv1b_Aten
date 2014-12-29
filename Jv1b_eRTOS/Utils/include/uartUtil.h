/*
 * usart_task.h
 *
 * Created: 11/22/2014 11:44:40 PM
 *  Author: avasquez
 */ 


#ifndef USART_TASK_H_
#define USART_TASK_H_

void vStartSerialTask( UBaseType_t uxPriority, uint32_t ulBaudRate, UBaseType_t uxLED);
void fnPrintWaterMark(void);

#endif /* USART_TASK_H_ */