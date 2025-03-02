#include "function_task.h"
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

#include "user_lib.h"
#include "math.h"
#include "stdlib.h"

/*==============================================================*/
#define FUNCTION_TASK_PRIO 23
#define FUNCTION_STK_SIZE 256
TaskHandle_t FunctionTask_Handler;
void Function_task(void *pvParameters);

/*==============================================================*/
void task_function_Create(void)
{
	xTaskCreate((TaskFunction_t)Function_task,
                (const char *)"Function_task",
                (uint16_t)FUNCTION_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)FUNCTION_TASK_PRIO,
                (TaskHandle_t *)&FunctionTask_Handler);
}
/*==============================================================*/

extern remote_mode_e INPUTMOD;

extern Motor_Control_t motor_control;

/*==============================================================*/
char static working_state = 0;
/*===========================================================================================================================*/
float L_Up_Appoint[UP_MOVE_COUNT], 
	    R_Up_Appoint[UP_MOVE_COUNT];

float L_Motor_Longs[UP_MOVE_COUNT] = {L_UP_Longs_HIGHTST, L_Up_Longs_Exchang, L_Up_Longs_SILVER, L_Up_Longs_PICK_FLR, L_Up_Longs_PUT, L_Up_Longs_LOWWST};
float R_Motor_Longs[UP_MOVE_COUNT] = {R_UP_Longs_HIGHTST, R_Up_Longs_Exchang, R_Up_Longs_SILVER, R_Up_Longs_PICK_FLR, R_Up_Longs_PUT, R_Up_Longs_LOWWST};
/*===========================================================================================================================*/	
float Turn_Appoint[CLAMP_TURN_COUNT];
float Turn_Longs[CLAMP_TURN_COUNT]={Level_Turn_Longs, floor_Turn_Longs, EXCHANG_GET_Longs, Back_Longs_Min};
/*===========================================================================================================================*/	
float Turn_R_Appoint[CLAMP_TURN_COUNT];
float Turn_R_Longs[CLAMP_TURN_COUNT]={Level_Turn_R_Longs, floor_Turn_R_Longs, EXCHANG_R_GET_Longs, Back_R_Longs_Min};
/*===========================================================================================================================*/

/**
  * @brief  初始化程序
  * @param 	void
  * @retval void
  * @attentionSS
  */
void Function_Control_Init(Motor_Control_t *function_control_init)
{
	for(int i = 0; i < sizeof(L_Up_Appoint)/sizeof(L_Up_Appoint[0]); i++ )
	{
		L_Up_Appoint[i] = function_control_init->Up_motor[UP_LEFT].angle_offset - L_Motor_Longs[i];
		R_Up_Appoint[i] = function_control_init->Up_motor[UP_RIGHT].angle_offset + R_Motor_Longs[i];
	}
	
	for(int i = 0; i < sizeof(Turn_Appoint)/sizeof(Turn_Appoint[0]); i++ )
	{
		Turn_Appoint[i] = function_control_init->Clamp_motor.angle_offset + Turn_Longs[i];
		Turn_R_Appoint[i] = function_control_init->Clamp_motor_R.angle_offset + Turn_R_Longs[i];
	}
	
}


/**
  * @brief  副板功能复位
  * @param  void
  * @retval void
  * @attention 电机气阀复位
  */
void  ALL_RESET(void)
{
	working_state = 0;
	if(motor_control.mode == RE_SET)  
	{
//		motor_control.Up_motor[UP_LEFT].set_angle = RAMP_float(L_Up_Appoint[HIGHTST],motor_control.Up_motor[UP_LEFT].motor_measure->real_ecd,UP_RAMP);
//		motor_control.Up_motor[UP_RIGHT].set_angle = RAMP_float(R_Up_Appoint[HIGHTST],motor_control.Up_motor[UP_RIGHT].motor_measure->real_ecd,UP_RAMP);
//		
//		motor_control.Clamp_motor.set_angle = RAMP_float(Turn_Appoint[FLOOR],motor_control.Clamp_motor.motor_measure->real_ecd,CLAMP_P_RAMP);
//		motor_control.Clamp_motor_Y.set_angle = RAMP_float(Turn_R_Appoint[FLOOR],motor_control.Clamp_motor_Y.motor_measure->real_ecd,CLAMP_Y_RAMP);
//		motor_control.Clamp_motor_R.set_angle = RAMP_float(motor_control.Clamp_motor_R.angle_offset,motor_control.Clamp_motor_R.motor_measure->real_ecd,CLAMP_R_RAMP);
		
		motor_control.Up_motor[UP_LEFT].set_angle = L_Up_Appoint[HIGHTST];
		motor_control.Up_motor[UP_RIGHT].set_angle = R_Up_Appoint[HIGHTST];
		
		motor_control.Clamp_motor.set_angle = Turn_Appoint[FLOOR];
		motor_control.Clamp_motor_Y.set_angle = Turn_R_Appoint[FLOOR];
		motor_control.Clamp_motor_R.set_angle = motor_control.Clamp_motor_R.angle_offset;
		
		vTaskDelay(TIME_STAMP_20MS);
		Air_Cylinder_RESET;
	}
}

/**
  * @brief  夹取金矿
  * @param  电机结构体
  * @retval void
  * @attention 
  */
static void Pickup_GOLD_ORE(Motor_Control_t *clamp_move_Gold)
{
	static char flag = 0;
	if((working_state == 1) || (working_state == 0))
	{
    	if(flag == 0)
			{
				working_state = 1;
				Clamp_Up;
				Clamp_Turn_To_Point(clamp_move_Gold, Turn_Appoint[LEVEL]);
				Clamp_R_Turn_To_Point(clamp_move_Gold, Turn_R_Appoint[LEVEL]);
				vTaskDelay(TIME_STAMP_750MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
			}
			if(flag == 1)
			{
				Clamp_Down;
				vTaskDelay(TIME_STAMP_300MS);
			}
			
			if((flag == 2) && (!IF_KEY_PRESSED_CTRL))
			{
				Frame_Up;
				vTaskDelay(TIME_STAMP_300MS);
			}
			
			if((flag == 2) && IF_KEY_PRESSED_CTRL)
			{
				Clamp_Up;
				vTaskDelay(TIME_STAMP_300MS);
				flag = 0;
			}
			if((flag == 3) && (!IF_KEY_PRESSED_CTRL)) 
			{
				Clamp_R_Turn_To_Point(clamp_move_Gold, Turn_R_Appoint[BACK]);
				Clamp_Turn_To_Point(clamp_move_Gold, Turn_Appoint[BACK]);
				vTaskDelay(TIME_STAMP_500MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				Up_Down_To_Point(clamp_move_Gold, L_Up_Appoint[PUT], R_Up_Appoint[PUT]);
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_Up;
				vTaskDelay(TIME_STAMP_750MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Gold, L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Gold, Turn_Appoint[FLOOR]);
				Clamp_R_Turn_To_Point(clamp_move_Gold, Turn_R_Appoint[FLOOR]);
				vTaskDelay(TIME_STAMP_400MS);
				Frame_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				working_state = 0;
			}
			
			if((flag == 3) && IF_KEY_PRESSED_CTRL)
			{
				Clamp_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_500MS);
				Frame_Down;
				vTaskDelay(TIME_STAMP_750MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_500MS);
				flag = 0;
			}
			
			flag++;
			
			if(flag >= 4)
				flag = 0;
	}
	
	if(working_state != 1 && working_state != 0)
	{
		ALL_RESET();
		flag = 0;
	}
	
}
/**
  * @brief  空接
  * @param  电机结构体
  * @retval void
  * @attention 
  */

char ctr;

static void Alley_Oop_ORE(Motor_Control_t *clamp_move_Alley)
{
  static char flag = 0;
	if((working_state == 2) || (working_state == 0))
	{
			if(flag == 0)
			{
				working_state = 2;
				Clamp_Up;
				Frame_Up;
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Alley, Turn_Appoint[LEVEL]);
				Clamp_R_Turn_To_Point(clamp_move_Alley, Turn_R_Appoint[LEVEL]);
				vTaskDelay(TIME_STAMP_100MS);
			}
			flag++;
			while(flag == 1)
			{
				if(LASER == 0)
				{
					Clamp_Down;
					vTaskDelay(TIME_STAMP_100MS);
					return;
				}
				if((LASER != 0) && ((IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_E) || ((RC_SW1_UP)&&(rc_ctrl.rc.ch[4] > 440))))
				{
					Clamp_Down;
					vTaskDelay(TIME_STAMP_300MS);
					return;
				}
					vTaskDelay(1);
			}
//			while(flag == 1)
//			{
//				static u8 num = 0;
//				while(LASER == 0)
//				{
//					num = 2;
//					vTaskDelay(1);
//				}
//				if(num == 2)
//				{
//					Clamp_Down;
//					num = 0;
//					return;
//				}
//				if((LASER != 0) && ((IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_E) || ((RC_SW1_UP)&&(rc_ctrl.rc.ch[4] > 440))))
//				{
//					Clamp_Down;
//					return;
//				}
//					vTaskDelay(1);
//			}
			  if((flag == 2) && (IF_KEY_PRESSED_CTRL))
				{
					Clamp_Up;
					vTaskDelay(TIME_STAMP_300MS);
					flag = 0;
				}
			
			if((flag == 2) && !IF_KEY_PRESSED_CTRL)
			{
				Clamp_R_Turn_To_Point(clamp_move_Alley, Turn_R_Appoint[BACK]);
				Clamp_Turn_To_Point(clamp_move_Alley, Turn_Appoint[BACK]);
				vTaskDelay(TIME_STAMP_400MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				Up_Down_To_Point(clamp_move_Alley, L_Up_Appoint[PUT], R_Up_Appoint[PUT]);
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_Up;
				vTaskDelay(TIME_STAMP_750MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Alley, L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Alley, Turn_Appoint[FLOOR]);
				Clamp_R_Turn_To_Point(clamp_move_Alley, Turn_R_Appoint[FLOOR]);
				vTaskDelay(TIME_STAMP_400MS);
				Frame_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				working_state = 0;
			}	
			
			if(flag >= 3)
			flag = 0;
	}
	if(working_state != 2 && working_state != 0)
	{
		ALL_RESET();
		flag = 0;
	}
	
}
/**
  * @brief  夹取银矿石
  * @param  电机结构体
  * @retval void
  * @attention 
  */
static void Pickup_Silver_ORE(Motor_Control_t *clamp_move_Silver)
{
	static char flag = 0;
	if((working_state == 3) || (working_state == 0))
	{
			if(flag == 0)
			{
				working_state = 3;
				Clamp_Up;
				Stretch_Up;
				Frame_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Silver, L_Up_Appoint[SILVER], R_Up_Appoint[SILVER]);
				Clamp_Turn_To_Point(clamp_move_Silver, Turn_Appoint[LEVEL]);
				Clamp_R_Turn_To_Point(clamp_move_Silver, Turn_R_Appoint[LEVEL]);
				vTaskDelay(TIME_STAMP_300MS);
			}
			if(flag == 1)
			{
				Clamp_Down;
				vTaskDelay(TIME_STAMP_300MS);
			}	
			if((flag == 2) && (IF_KEY_PRESSED_CTRL))
			{
				Clamp_Up;
				vTaskDelay(TIME_STAMP_300MS);
				flag = 0;
			}
			if((flag == 2) && (!IF_KEY_PRESSED_CTRL))
			{
				Up_Down_To_Point(clamp_move_Silver, L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				vTaskDelay(TIME_STAMP_300MS);
			}
			if((flag == 3) && (IF_KEY_PRESSED_CTRL))
			{
				Clamp_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Silver, L_Up_Appoint[SILVER], R_Up_Appoint[SILVER]);
				vTaskDelay(TIME_STAMP_300MS);
				flag = 0;
			}
			if((flag == 3) && (!IF_KEY_PRESSED_CTRL))
			{
				Clamp_R_Turn_To_Point(clamp_move_Silver, Turn_R_Appoint[BACK]);
				Clamp_Turn_To_Point(clamp_move_Silver, Turn_Appoint[BACK]);
				vTaskDelay(TIME_STAMP_400MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				Up_Down_To_Point(clamp_move_Silver, L_Up_Appoint[PUT], R_Up_Appoint[PUT]);
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_Up;
				vTaskDelay(TIME_STAMP_750MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Silver, L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Silver, Turn_Appoint[FLOOR]);
				Clamp_R_Turn_To_Point(clamp_move_Silver, Turn_R_Appoint[FLOOR]);
				vTaskDelay(TIME_STAMP_400MS);
				Frame_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				working_state = 0;
			}
			flag++;
			
			if(flag >= 4)
				flag = 0;
	}
	if(working_state != 3 && working_state != 0)
	{
		ALL_RESET();
		flag = 0;
	}
}

/**
  * @brief  夹取地面的矿石
  * @param  电机结构体
  * @retval void
  * @attention 
  */
static void Pickup_Floor_ORE(Motor_Control_t *clamp_move_Floor)
{
	static char flag = 0;
	if((working_state == 4) || (working_state == 0))
	{
			if(flag == 0)
			{
				working_state = 4;
				Clamp_Up;
				Stretch_Up;
				Clamp_Turn_To_Point(clamp_move_Floor, Turn_Appoint[FLOOR]);
				Clamp_R_Turn_To_Point(clamp_move_Floor, Turn_R_Appoint[FLOOR]);
				Up_Down_To_Point(clamp_move_Floor,L_Up_Appoint[PICK_FLR], R_Up_Appoint[PICK_FLR]);
				vTaskDelay(TIME_STAMP_100MS);
			}
			if(flag == 1)
			{
				Clamp_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Frame_Up;
				vTaskDelay(TIME_STAMP_500MS);
				Up_Down_To_Point(clamp_move_Floor,L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				Clamp_R_Turn_To_Point(clamp_move_Floor, Turn_R_Appoint[BACK]);
				Clamp_Turn_To_Point(clamp_move_Floor, Turn_Appoint[BACK]);
				vTaskDelay(TIME_STAMP_400MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				Up_Down_To_Point(clamp_move_Floor, L_Up_Appoint[PUT], R_Up_Appoint[PUT]);
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_Up;
				vTaskDelay(TIME_STAMP_750MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Floor, L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_Turn_To_Point(clamp_move_Floor, Turn_Appoint[FLOOR]);
				Clamp_R_Turn_To_Point(clamp_move_Floor, Turn_R_Appoint[FLOOR]);
				vTaskDelay(TIME_STAMP_400MS);
				Frame_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				working_state = 0;
			}
			
			flag++;
			
			if(flag >= 2)
				flag = 0;
	}
	if(working_state != 4 && working_state != 0)
	{
		ALL_RESET();
		flag = 0;
	}
}


/**
  * @brief  兑换最上面的矿石
  * @param  电机结构体
  * @retval void
  * @attention 
  */
static void Exchang_Upp_ORE(Motor_Control_t *clamp_move_Upp)
{
	static char flag = 0;
	if((working_state == 5) || (working_state == 0))
	{
			if(flag == 0)
			{
				working_state = 5;
				Clamp_Up;
				vTaskDelay(TIME_STAMP_750MS);
				Frame_Up;
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_R_Turn_To_Point(clamp_move_Upp, Turn_R_Appoint[EXGET]);
				Up_Down_To_Point(clamp_move_Upp,L_Up_Appoint[LOWWST], R_Up_Appoint[LOWWST]);
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Upp, Turn_Appoint[EXGET]);
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Upp, Turn_Appoint[BACK]);
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Upp,L_Up_Appoint[EXCHANG], R_Up_Appoint[EXCHANG]);
				Clamp_Turn_To_Point(clamp_move_Upp, Turn_Appoint[LEVEL]);
				vTaskDelay(TIME_STAMP_300MS);
			}
			if(flag == 1)
			{
				Clamp_Up;
				vTaskDelay(TIME_STAMP_500MS);
			}
 			if(flag == 2)
			{
//				Push_Up;
				vTaskDelay(TIME_STAMP_300MS);
//				Push_Down;
			}
			if(flag == 3)
			{
				Up_Down_To_Point(clamp_move_Upp,L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				vTaskDelay(TIME_STAMP_400MS);
				Clamp_Turn_To_Point(clamp_move_Upp, Turn_Appoint[FLOOR]);
				Clamp_R_Turn_To_Point(clamp_move_Upp, Turn_R_Appoint[FLOOR]);
				vTaskDelay(TIME_STAMP_400MS);
				Frame_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				working_state = 0;
				
			}
			
			flag++;
			
			if(flag >= 4)
				flag = 0;
	}
  if(working_state != 5 && working_state != 0)
	{
		ALL_RESET();
		flag = 0;
	}
}

/**
  * @brief  兑换最下面的矿石
  * @param  电机结构体
  * @retval void
  * @attention 
  */
static void Exchang_Low_ORE(Motor_Control_t *clamp_move_Low)
{
	static char flag = 0;
  if((working_state == 6) || (working_state == 0))
	{
			if(flag == 0)
			{
				working_state = 6;
				Clamp_Up;
				vTaskDelay(TIME_STAMP_750MS);
				Frame_Up;
				WareHouse_Up;
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_R_Turn_To_Point(clamp_move_Low, Turn_R_Appoint[EXGET]);
				Up_Down_To_Point(clamp_move_Low,L_Up_Appoint[LOWWST], R_Up_Appoint[LOWWST]);
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Low, Turn_Appoint[EXGET]);
				vTaskDelay(TIME_STAMP_500MS);
				Clamp_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Clamp_Turn_To_Point(clamp_move_Low, Turn_Appoint[BACK]);
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Up;
				vTaskDelay(TIME_STAMP_300MS);
				Up_Down_To_Point(clamp_move_Low,L_Up_Appoint[EXCHANG], R_Up_Appoint[EXCHANG]);
				Clamp_Turn_To_Point(clamp_move_Low, Turn_Appoint[LEVEL]);
				vTaskDelay(TIME_STAMP_300MS);
			}
			if(flag == 1)
			{
				Clamp_Up;
				vTaskDelay(TIME_STAMP_300MS);
			}
		if(flag == 2)
			{
//				Push_Up;
				vTaskDelay(TIME_STAMP_300MS);
//				Push_Down;
			}
			if(flag == 3)
			{
				Up_Down_To_Point(clamp_move_Low,L_Up_Appoint[HIGHTST], R_Up_Appoint[HIGHTST]);
				vTaskDelay(TIME_STAMP_400MS);
				WareHouse_Down;
				Clamp_Turn_To_Point(clamp_move_Low, Turn_Appoint[FLOOR]);
				Clamp_R_Turn_To_Point(clamp_move_Low, Turn_R_Appoint[FLOOR]);
				vTaskDelay(TIME_STAMP_400MS);
				Frame_Down;
				vTaskDelay(TIME_STAMP_300MS);
				Stretch_Down;
				vTaskDelay(TIME_STAMP_750MS);
				working_state = 0;
			}
			
			flag++;
			
			if(flag >= 4)
				flag = 0;
	}
	if(working_state != 6 && working_state != 0)
	{
		ALL_RESET();
		flag = 0;
	}
}
/**
  * @brief  控制模式设置
  * @param  void
  * @retval void
  * @attention 
  */
u8 mode_flag=0,funtion_flag=0;
void Function_Mode_Set(Motor_Control_t *chassis_move_control )
{
	if(chassis_move_control == NULL)
	{
		return ;
	}
	Reomte_Mode_Set();
	if(INPUTMOD == RUN_STOP)
	{
		ALL_RESET();
		if(chassis_move_control->last_mode == NORMAL)
		{
			chassis_move_control->mode = RE_SET;
		}
	}
	if(INPUTMOD == REMOTE_INPUT)
	{
		if(((RC_SW1_UP)&&(rc_ctrl.rc.ch[4] > 440))&&((working_state == 2) || (working_state == 0)))
		{
			Alley_Oop_ORE(chassis_move_control);
			vTaskDelay(TIME_STAMP_1MS); 
			funtion_flag=1;
		}

		else if(((RC_SW1_UP) && (rc_ctrl.rc.ch[4] < -440))&&((working_state == 1) || (working_state == 0)))
		{
			Pickup_GOLD_ORE(chassis_move_control);
			vTaskDelay(TIME_STAMP_1MS); 
			funtion_flag=2;
		}	
		
		else if(((RC_SW1_MID) && (rc_ctrl.rc.ch[4] > 440))&&((working_state == 4) || (working_state == 0)))
		{
			Pickup_Floor_ORE(chassis_move_control);
			vTaskDelay(TIME_STAMP_1MS);
			funtion_flag=3;
		}
		
		else if(((RC_SW1_MID) && (rc_ctrl.rc.ch[4] < -440))&&((working_state == 3) || (working_state == 0)))
		{
			Pickup_Silver_ORE(chassis_move_control);
			vTaskDelay(TIME_STAMP_1MS);
			funtion_flag=4;
		}

		else if(((RC_SW1_DOWN) && (rc_ctrl.rc.ch[4] < -440))&&((working_state == 6) || (working_state == 0)))
		{
			Exchang_Low_ORE(chassis_move_control);
			vTaskDelay(TIME_STAMP_1MS);
			funtion_flag=6;
		}
		
		else if(((RC_SW1_DOWN) && (rc_ctrl.rc.ch[4] > 440))&&((working_state == 5) || (working_state == 0)))
		{
			Exchang_Upp_ORE(chassis_move_control);
			vTaskDelay(TIME_STAMP_1MS);
			funtion_flag=5;
		}	
		
	}
	if(INPUTMOD == KEYMOUSE_INPUT)
	{
		static char E_FLAG , R_FLAG , Z_FLAG , X_FLAG , F_FLAG , G_FLAG , B_FLAG , Q_FLAG;

		if(!IF_KEY_PRESSED_E)
			{
				E_FLAG = 1;
			}
		if(((IF_KEY_PRESSED_E) && (IF_KEY_PRESSED_SHIFT)) && ((working_state == 2) || (working_state == 0)) && (E_FLAG == 1))
			{
				E_FLAG = 0;
				Alley_Oop_ORE(chassis_move_control);
			  vTaskDelay(TIME_STAMP_1MS);
			}
		if(((IF_KEY_PRESSED_E) && (!IF_KEY_PRESSED_SHIFT))&&((working_state == 1) || (working_state == 0)) && (E_FLAG == 1))
			{
				E_FLAG = 0;
				Pickup_GOLD_ORE(chassis_move_control);
				vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}
		if(!IF_KEY_PRESSED_R)
			{
				R_FLAG = 1;
			}
		if(((IF_KEY_PRESSED_R) && (IF_KEY_PRESSED_CTRL))&&((working_state == 5) || (working_state == 0)) && (R_FLAG == 1))
			{
				R_FLAG = 0;
				Exchang_Upp_ORE(chassis_move_control);
				vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}
		if(((IF_KEY_PRESSED_R) && (!IF_KEY_PRESSED_CTRL))&&((working_state == 6) || (working_state == 0)) && (R_FLAG == 1))
			{
				R_FLAG = 0;
				Exchang_Low_ORE(chassis_move_control);
				vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}
		if(!IF_KEY_PRESSED_Z)
			{ 
				Z_FLAG = 1;
			}
		if((IF_KEY_PRESSED_Z)&&((working_state == 3) || (working_state == 0))&&(Z_FLAG == 1))
			{
				Z_FLAG = 0;
				Pickup_Silver_ORE(chassis_move_control);
				vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}
		if(!IF_KEY_PRESSED_X)
			{ 
				X_FLAG = 1;
			}
		if((IF_KEY_PRESSED_X)&&((working_state == 4) || (working_state == 0))&&(X_FLAG == 1))
			{
				X_FLAG = 0;
				Pickup_Floor_ORE(chassis_move_control);
				vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}
		if(!IF_KEY_PRESSED_F)
			{ 
				F_FLAG = 1;
			}
		if((IF_KEY_PRESSED_F)&&(F_FLAG == 1))
			{
				F_FLAG = 0;
				HOOK_TOGGLE;
				vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}		
		if(!IF_KEY_PRESSED_G)
			{ 
				G_FLAG = 1;
			}
		if((IF_KEY_PRESSED_G)&&(G_FLAG == 1))
			{
				G_FLAG = 0;
				CARD_TOGGLE;
				vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}
		if(!IF_KEY_PRESSED_B)
			{ 
				B_FLAG = 1;
			}
		if((IF_KEY_PRESSED_B)&&(B_FLAG == 1))
			{
				B_FLAG = 0;
				OBSTACLE_TOGGLE;
			  vTaskDelay(TIME_STAMP_1MS);  //系统延时
			}
		if(!IF_KEY_PRESSED_Q)
			{
				Q_FLAG = 1;
			}
		if((IF_KEY_PRESSED_Q) && (Q_FLAG == 1))
			{
				Q_FLAG = 0;
				CLAMP_TOGGLE;
			  vTaskDelay(TIME_STAMP_50MS);  //系统延时
			}
	}
}

void Reomte_Mode_Set()
{
		if (rc_ctrl.rc.ch[0] == -660 && rc_ctrl.rc.ch[1] == -660 && rc_ctrl.rc.ch[2] == 660 && rc_ctrl.rc.ch[3] == -660) //拨杆内八软件强制复位
    {
        __set_FAULTMASK(1);//关闭所有中断
        NVIC_SystemReset();//软件复位
    }
    
    else if (switch_is_mid(rc_ctrl.rc.s[ModeChannel_L]) && switch_is_up(rc_ctrl.rc.s[ModeChannel_R]))//左中右上
    {
        INPUTMOD = KEYMOUSE_INPUT;//键盘控制
    }
    else if (switch_is_mid(rc_ctrl.rc.s[ModeChannel_R]))//右中
    {
        INPUTMOD = REMOTE_INPUT;//遥控器控制
    }
    else if (switch_is_down(rc_ctrl.rc.s[ModeChannel_L]) && switch_is_down(rc_ctrl.rc.s[ModeChannel_R]))//双下
    {
        INPUTMOD = RUN_STOP;//停止
    }
		else if (switch_is_up(rc_ctrl.rc.s[ModeChannel_L]) && switch_is_up(rc_ctrl.rc.s[ModeChannel_R]))//双上
		{
				INPUTMOD = CONTROL_P_Y_R;
		}

}

void Function_task(void *pvParameters)
{
	while(1)
	{
		ctr = LASER;
		if(motor_control.mode == INIT)
		{
			Function_Control_Init(&motor_control);
		}
//		if((motor_control.mode == NORMAL) || (motor_control.mode == TUNING))
//		{
		 
		  Function_Mode_Set(&motor_control);
//		}
 
		vTaskDelay(1);  //系统延时
	}
}

