#include "Clamp_Y_P_R_task.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "gpio.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "motor_control_task.h"
#include "function_task.h"

#include "user_lib.h"
#include "math.h"
#include "stdlib.h"


#define CLAMP_Y_P_R_TASK_PRIO 25
#define CLAMP_Y_P_R_STK_SIZE 256
TaskHandle_t Clamp_Y_P_RTask_Handler;
void Clamp_Y_P_R_task(void *pvParameters);

/*==============================================================*/
void task_Clamp_Y_P_R_Crate(void)
{
	xTaskCreate((TaskFunction_t)Clamp_Y_P_R_task,
                (const char *)"Clamp_Y_P_R_task",
                (uint16_t)CLAMP_Y_P_R_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CLAMP_Y_P_R_TASK_PRIO,
                (TaskHandle_t *)&Clamp_Y_P_RTask_Handler);
}
/*==============================================================*/
extern Motor_Control_t motor_control;
extern remote_mode_e INPUTMOD;

void Y_P_R_Mode_Set(Motor_Control_t *PYR_control )
{
	if(PYR_control == NULL)
	{
		return ;
	}
	if(INPUTMOD == CONTROL_P_Y_R)
	{
		
		Reomte_Mode_Set();
		
		PYR_control->Clamp_motor_Y.set_angle += (rc_ctrl.rc.ch[0] * CHANNEL_PRESS);
		PYR_control->Clamp_motor.set_angle += (rc_ctrl.rc.ch[1] * CHANNEL_PRESS);
		PYR_control->Clamp_motor_R.set_angle += (rc_ctrl.rc.ch[2] * CHANNEL_PRESS);
		PYR_control->Up_motor[UP_LEFT].set_angle += (rc_ctrl.rc.ch[3] * CHANNEL_PRESS_Up);
		PYR_control->Up_motor[UP_RIGHT].set_angle -= (rc_ctrl.rc.ch[3] * CHANNEL_PRESS_Up);
		
		if(rc_ctrl.rc.ch[4] < -440)
		{
			STRETCH_TOGGLE;
			vTaskDelay(TIME_STAMP_400MS);
		}
		if(rc_ctrl.rc.ch[4] > 440)
		{
			CLAMP_TOGGLE;
			vTaskDelay(TIME_STAMP_400MS);
		}		
	}
	else
	{
		return;
	}
}

void Clamp_Y_P_R_task(void *pvParameters)
{
	while(1)
	{
		Y_P_R_Mode_Set(&motor_control);
		vTaskDelay(1);
	}
}
