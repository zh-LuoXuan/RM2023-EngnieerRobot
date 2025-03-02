#include "fine_tuning_task.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "gpio.h" 
//#include "control_task.h"

/*==============================================================*/
#define FINE_TUNING_TASK_PRIO 26
#define FINE_TUNING_STK_SIZE 256
TaskHandle_t FineTuning_Handler;
void fine_tuning_task(void);

/*==============================================================*/

void task_fine_tuning_Create(void)
{
	xTaskCreate((TaskFunction_t)fine_tuning_task,
                (const char *)"fine_tuning_task",
                (uint16_t)FINE_TUNING_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)FINE_TUNING_TASK_PRIO,
                (TaskHandle_t *)&FineTuning_Handler);
}

void fine_tuning_task(void)
{
	static portTickType TickCount; 
	
	TickCount = xTaskGetTickCount();
	
	while(1)
	{
		vTaskDelayUntil(&TickCount, 1);
	}
}














