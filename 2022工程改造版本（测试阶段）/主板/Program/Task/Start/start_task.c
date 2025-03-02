#include "start_task.h"
#include "sys.h"
#include "delay.h"
#include "gpio.h" 
#include "IMUTask.h"
#include "pwm.h"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usart.h"
#include "can.h"
#include "CAN_Receive.h"
#include "RemoteControl.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "send_task.h"


#define START_TASK_PRIO		31
#define STAET_STK_SIZE 		152
TaskHandle_t Start_Task_Handler;
void start_task(void *pvParameters);


void start_task_Create(void)
{
    xTaskCreate((TaskFunction_t )start_task,
                (const char*    )"start_task",
                (uint16_t       )STAET_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t*  )&Start_Task_Handler);
    vTaskStartScheduler();
}


void System_Init(void)
{
	delay_init(168);
	Key_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	remote_control_init();
	IMU_Init();
//	TIM5_PWM_Init(10000-1,84-1);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz.
//	USART6_Init(115200);
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);//1Mbps
  CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);

	delay_ms(500);
}   


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	  task_IMU_Create();
	  task_Gimbal_Create();
	  task_Chsssis_Create();
	  task_Send_Create();
	
    vTaskDelete(Start_Task_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

