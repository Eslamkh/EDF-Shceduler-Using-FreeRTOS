/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "Queue.h"
#include "semphr.h"



/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
#define TASK1_PERIODICITY						(TickType_t) 50
#define TASK2_PERIODICITY						(TickType_t) 50
#define TASK3_PERIODICITY						(TickType_t) 100
#define TASK4_PERIODICITY						(TickType_t) 20
#define TASK5_PERIODICITY						(TickType_t) 10
#define TASK6_PERIODICITY						(TickType_t) 100

#define QUEUE_LENGTH_MAX								5
#define BUTTON_FALLING_MSG_LENGTH				22
#define BUTTON_RISING_MSG_LENGTH				21
#define RUN_TIME_ANALYSIS_MSG_LENGTH		400

char Button1Falling[BUTTON_FALLING_MSG_LENGTH] = "Button_1 Falling edge\n";
char Button1Rising[BUTTON_RISING_MSG_LENGTH] =  "Button_1 Rising edge\n";
char Button2Falling[BUTTON_FALLING_MSG_LENGTH] = "Button_2 Falling edge\n";
char Button2Rising[BUTTON_RISING_MSG_LENGTH] = "Button_2 Rising edge\n";


#define TASK1_QUEUE_TIMEOUT						( TickType_t )10
#define TASK2_QUEUE_TIMEOUT						( TickType_t )10
#define TASK3_QUEUE_TIMEOUT						( TickType_t )10

TaskHandle_t Task1_Handler = NULL;
TaskHandle_t Task2_Handler = NULL;
TaskHandle_t Task3_Handler = NULL;
TaskHandle_t Task4_Handler = NULL;
TaskHandle_t Task5_Handler = NULL;
TaskHandle_t Task6_Handler = NULL;

int Button1_flag =1,Button2_flag =1;
QueueHandle_t xQueueUART;

typedef struct
{
	int messageLength;
	char *Data;
}MessageQ;
/*
int task_1_in_time =0, task_1_out_time =0,task_1_total_time;
int task_2_in_time =0, task_2_out_time =0,task_2_total_time;
int task_3_in_time =0, task_3_out_time =0,task_3_total_time;
int task_4_in_time =0, task_4_out_time =0,task_4_total_time;
int task_5_in_time =0, task_5_out_time =0,task_5_total_time;
int task_6_in_time =0, task_6_out_time =0,task_6_total_time;
int system_time =0;
int cpu_load =0;
*/
//char runtimeStatesBuffer[RUN_TIME_ANALYSIS_MSG_LENGTH];
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

void Button_1_Monitor()
{
	MessageQ Message;
	volatile pinState_t ButtonState =0;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 ); 
	Message.Data = NULL;
	Message.messageLength =0;
	while(1)
	{
		ButtonState = GPIO_read(PORT_0,PIN8);
		if(( ButtonState == 1) && (Button1_flag == 1))
		{
			Message.Data = Button1Rising;
			Message.messageLength = BUTTON_RISING_MSG_LENGTH;
			Button1_flag =0;
			/*Send to Uart_Reciver task "Rising edge is detected from Button*/
			xQueueSend(xQueueUART,&Message,TASK1_QUEUE_TIMEOUT);
		}
		if( (ButtonState == 0) &&(Button1_flag == 0))
		{
			Message.messageLength = BUTTON_FALLING_MSG_LENGTH;
			Message.Data = Button1Falling;
			Button1_flag =1;
			/*Send to Uart_Reciver task "Falling edge is detected from Button1"*/
			xQueueSend(xQueueUART,&Message,TASK1_QUEUE_TIMEOUT);
		}
		
	  vTaskDelayUntil( &xLastWakeTime, TASK1_PERIODICITY );
		
		/* Idle Task Toggling */
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
	}
	
}

void Button_2_Monitor()
{
	MessageQ Message;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	GPIO_write(PORT_0,PIN9,PIN_IS_LOW);
	Message.messageLength =0;
	Message.Data = NULL;
	while(1)
	{
		if((GPIO_read(PORT_0,PIN9) == 1) &&(Button2_flag == 1))
		{
			Button2_flag =0;
			Message.Data = Button2Rising;
			Message.messageLength = BUTTON_RISING_MSG_LENGTH;
			/*Send to Uart_Reciver task "Rising edge is detected from Button2"*/
			xQueueSend(xQueueUART,&Message,TASK2_QUEUE_TIMEOUT);
		}
		if((GPIO_read(PORT_0,PIN9) == 0) &&(Button2_flag == 0))
		{
			Button2_flag =1;
			Message.Data = Button2Falling;
			Message.messageLength = BUTTON_FALLING_MSG_LENGTH;
			/*Send to Uart_Reciver task "Falling edge is detected from Button2"*/
			xQueueSend(xQueueUART,&Message,TASK2_QUEUE_TIMEOUT);
		}

	  vTaskDelayUntil( &xLastWakeTime, TASK2_PERIODICITY );
		/* Idle Task Toggling */
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
	}
	
}

void Periodic_Transmitter()
{
	MessageQ Message;
	static char runtimeStatesBuffer[RUN_TIME_ANALYSIS_MSG_LENGTH];
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	Message.messageLength= 0;
	Message.Data = NULL;
	while(1)
	{
		vTaskGetRunTimeStats(runtimeStatesBuffer);
		Message.Data = runtimeStatesBuffer;
		Message.messageLength = RUN_TIME_ANALYSIS_MSG_LENGTH;
		/* Send runtimeStatesBuffer to Uart_Reciver task */
		xQueueSend(xQueueUART,&Message,TASK3_QUEUE_TIMEOUT);
		*(Message.Data) = '\n';
		Message.messageLength = 1;
		xQueueSend(xQueueUART,&Message,TASK3_QUEUE_TIMEOUT);
		vTaskDelayUntil( &xLastWakeTime, TASK3_PERIODICITY );
		/* Idle Task Toggling */
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
	}
}

void Uart_Receiver()
{
	MessageQ MessageRecived;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	while(1)
	{
		
		if( xQueueReceive( xQueueUART, &MessageRecived,10) == pdPASS)
		{
				vSerialPutString(MessageRecived.Data,MessageRecived.messageLength);
		}
		vTaskDelayUntil( &xLastWakeTime, TASK4_PERIODICITY );
		/* Idle Task Toggling */
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
	}
}

void Load_1_Simulation()     // execution time = 5ms
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	while(1)
	{
		/* Simulate load */
		int i =0;
		for(i=0; i<33000 ;i++)
		{
			__asm__("NOP");
		}
		vTaskDelayUntil( &xLastWakeTime, TASK5_PERIODICITY );
		/* Idle Task Toggling */
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
	}
}

void Load_2_Simulation()			 // execution time = 12ms
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	while(1)
	{
		/* Simulate load */
		int i =0;
		for(i=0; i<79200 ;i++)
		{
			__asm__("NOP");
		}
		vTaskDelayUntil( &xLastWakeTime, TASK6_PERIODICITY );
		/* Idle Task Toggling */
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
	}
}
/*-----------------------------------------------------------*/


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */

/* Implement Idle Hook*/
void vApplicationIdleHook( void )
{
	GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
}

/* Implement Tick Hook*/
void vApplicationTickHook( void )
{
	//GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
	//GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
}

int main( void )
{
	BaseType_t xReturned;
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();


	/* Create the task, storing the handle. */
    xReturned = xTaskCreatePeriodic(
                    Button_1_Monitor,      /* Function that implements the task. */
                    "Task1",    /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
										TASK1_PERIODICITY,
                    &Task1_Handler );      /* Used to pass out the created task's handle. */
	/* Create the task, storing the handle. */
    xReturned = xTaskCreatePeriodic(
                    Button_2_Monitor,      /* Function that implements the task. */
                    "Task2",    /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
										TASK2_PERIODICITY,
                    &Task2_Handler );      /* Used to pass out the created task's handle. */
	
	/* Create the task, storing the handle. */
    xReturned = xTaskCreatePeriodic(
                    Periodic_Transmitter,      /* Function that implements the task. */
                    "Task3",    /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
										TASK3_PERIODICITY,
                    &Task3_Handler );      /* Used to pass out the created task's handle. */
	/* Create the task, storing the handle. */
    xReturned = xTaskCreatePeriodic(
                    Uart_Receiver,      /* Function that implements the task. */
                    " Task4",    /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
										TASK4_PERIODICITY,
                    &Task4_Handler );      /* Used to pass out the created task's handle. */
										
	/* Create the task, storing the handle. */
    xReturned = xTaskCreatePeriodic(
                    Load_1_Simulation,      /* Function that implements the task. */
                    "Task5",    /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
										TASK5_PERIODICITY,
                    &Task5_Handler );      /* Used to pass out the created task's handle. */
	/* Create the task, storing the handle. */
    xReturned = xTaskCreatePeriodic(
                    Load_2_Simulation,      /* Function that implements the task. */
                    "Task6",    /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
										TASK6_PERIODICITY,
                    &Task6_Handler );      /* Used to pass out the created task's handle. */

	xQueueUART = xQueueCreate( QUEUE_LENGTH_MAX , sizeof( MessageQ ) );								

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


