#include "send_task.h"
#include "RemoteControl.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "gpio.h" 


/*==============================================================*/
#define SEND_TASK_PRIO 26
#define SEND_STK_SIZE  256
TaskHandle_t SendTask_Handler;
void send_task(void);

/*==============================================================*/

void task_Send_Create(void)
{
	xTaskCreate((TaskFunction_t)send_task,
                (const char *)"send_task",
                (uint16_t)SEND_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SEND_TASK_PRIO,
                (TaskHandle_t *)&SendTask_Handler);
}

void send_task(void)
{
	static portTickType TickCount; 
	int key;
	TickCount = xTaskGetTickCount();
	key = TOPMOVE_KEY;
	while(1)
	{
		CAN2_CMD_RC(&rc_ctrl);
	  CAN2_CMD_KEYMOUSE(&rc_ctrl);
	  CAN2_CMD_MOUSEXYZ(&rc_ctrl);
	  CAN2_CMD_IO(key);
		
		vTaskDelayUntil(&TickCount, 1);
	}
}


