#include "sys.h"
#include "delay.h"

#include "IMUTask.h"
#include "pwm.h"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "start_task.h"
#include "usart6.h"
#include "can.h"
#include "CAN_Receive.h"
#include "RemoteControl.h"
RCC_ClocksTypeDef getrccclock;
int main(void)
{	
	System_Init();
	RCC_GetClocksFreq(&getrccclock);
	start_task_Create();
	
	while(1)
	{

	}
}

