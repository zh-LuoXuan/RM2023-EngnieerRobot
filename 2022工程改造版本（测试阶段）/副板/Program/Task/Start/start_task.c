#include "start_task.h"
#include "sys.h"
#include "delay.h"
#include "Judge_task.h"
#include "IMUTask.h"
#include "pwm.h"
#include "gpio.h"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Time7.h"

#include "usart6.h"
#include "can.h"
#include "CAN_Receive.h"
#include "RemoteControl.h"
#include "function_task.h"
#include "motor_control_task.h"
#include "UI_Task.h"
#include "judge_Task.h"
#include "queue.h"
#include "string.h"
#include "Detect_Task.h"
#include "Clamp_Y_P_R_task.h"

#define START_TASK_PRIO		2
#define STAET_STK_SIZE 		512
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);//1Mbps
  CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
	delay_ms(2000);
	USART6_Init(115200);
	TIM7_BAS_Init(10-1, 8400-1);
//	Led_Init();
  Key_Init();
	Laser_Init();//���⴫��
	Air_Cylinder();//��·
	
	delay_ms(2000);//ȷ���������ϵ�
}   


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	
	  task_Motor_Control_Create();//�������
	  task_function_Create();//ȡ������
//	  task_Mode_Change_Create();//��λ����
	  task_user_UI_Create();
		task_Clamp_Y_P_R_Crate();
//    task_Judge_Create();
    vTaskDelete(Start_Task_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

