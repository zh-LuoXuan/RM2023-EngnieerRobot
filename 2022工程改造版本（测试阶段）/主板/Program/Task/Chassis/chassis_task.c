#include "chassis_task.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

#include "user_lib.h"
#include "math.h"
#include "stdlib.h"

/*==============================================================*/
#define CHASSIS_TASK_PRIO 23
#define CHASSIS_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;
void Chassis_task(void *pvParameters);

/*==============================================================*/

void task_Chsssis_Create(void)
{
	xTaskCreate((TaskFunction_t)Chassis_task,
                (const char *)"Chassisl_task",
                (uint16_t)CHASSIS_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CHASSIS_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
}
/*==============================================================*/

Chassis_Control_t chassis_control;
Chassis_mode_e CHASSIS_MODE = MAINTAIN;

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
int16_t   timeXFron,    timeXBack,    timeYLeft,    timeYRigh;
extern remote_mode_e INPUTMOD;
extern Drive_mode_e DRIVE_MODE;
fp32 steer_age_offset[4] = {CHASSIS_STEER_M1_OFFSET , CHASSIS_STEER_M2_OFFSET , CHASSIS_STEER_M3_OFFSET , CHASSIS_STEER_M4_OFFSET};
 
fp32 chassis_power_spd_pid[3] = {CHASSIS_POWER_SPEED_PID_KP , CHASSIS_POWER_SPEED_PID_KI , CHASSIS_POWER_SPEED_PID_KD};
fp32 chassis_steer_spd_pid[3] = {CHASSIS_STEER_SPEED_PID_KP , CHASSIS_STEER_SPEED_PID_KI , CHASSIS_STEER_SPEED_PID_KD};
fp32 chassis_steer_age_pid[3] = {CHASSIS_STEER_ANGLE_PID_KP , CHASSIS_STEER_ANGLE_PID_KI , CHASSIS_STEER_ANGLE_PID_KD};

fp32 chassis_follow_pid[3] = {CHASSIS_FOLLOW_ANGLE_PID_KP , CHASSIS_FOLLOW_ANGLE_PID_KI , CHASSIS_FOLLOW_ANGLE_PID_KD};

int16_t power_out[4];
int16_t Last_power_out[4];
int16_t steer_out[4];
fp32 speed_set[4];
fp32 last_speed_set[4];

static void Steering_Calc(float vx, float vy, float vw, Chassis_Control_t *Chassis_Calc);


/**
  * @brief  底盘初始化
  * @param  PID输出值
  * @retval void
  */
static void Chassis_Init(Chassis_Control_t *chassis_init)
{
	uint8_t i = 0;
	if (chassis_init == NULL)
    {
        return;
    }
  chassis_init->chassis_angle_gyro_point = get_Gyro_Angle_Point();
	chassis_init->chassis_rc_ctrl = get_remote_control_point();
  chassis_init->Gimbal_yaw_data = get_gimbal_yaw_control_t_point();
	chassis_init->gimbal_offset = chassis_init->Gimbal_yaw_data->angle_offset;
	chassis_init->chassis_vx = 0;
	chassis_init->chassis_vy = 0;
	chassis_init->chassis_vw = 0;
	chassis_init->steer_offset_angle[0]=-91.1f;
	chassis_init->steer_offset_angle[1]=-91.2f;
	chassis_init->steer_offset_angle[2]=120.0f;
	chassis_init->steer_offset_angle[3]=31.0f;
		
	for(i = 0; i < 4; i++)
  {			
		PID_Init(&chassis_init->chassis_power_speed_pid[i] , PID_POSITION , chassis_power_spd_pid , CHASSIS_POWER_SPEED_PID_MAX_OUT , CHASSIS_POWER_SPEED_PID_MAX_IOUT);
		PID_Init(&chassis_init->chassis_steer_speed_pid[i] , PID_POSITION , chassis_steer_spd_pid , CHASSIS_STEER_SPEED_PID_MAX_OUT , CHASSIS_STEER_SPEED_PID_MAX_IOUT);
		PID_Init(&chassis_init->chassis_steer_angle_pid[i] , PID_POSITION , chassis_steer_age_pid , CHASSIS_STEER_ANGLE_PID_MAX_OUT , CHASSIS_STEER_ANGLE_PID_MAX_IOUT);
	}
	
		for(i = 0; i < 4; i++)
  {		
	  chassis_init->chassis_power_motor[i].chassis_motor_measure = get_Power_Motor_Measure_Point(i);
		chassis_init->chassis_steer_motor[i].chassis_motor_measure = get_Steer_Motor_Measure_Point(i);
	}
	
	for(i = 0; i < 4; i++)
  {		
 	  chassis_init->set_power_speed[i] = 0;
		chassis_init->set_steer_angle[i] = steer_age_offset[i];
  }
	
	PID_Init(&chassis_init->follow_gimbal_pid , PID_POSITION , chassis_follow_pid , CHASSIS_FOLLOW_ANGLE_PID_MAX_OUT , CHASSIS_FOLLOW_ANGLE_PID_MAX_IOUT);	
}

/**
  * @brief  底盘掉头
  * @param  车辆掉头和底盘转向
  * @retval void
  * @attention  
  */
void Chassis_Turn(Chassis_Control_t *chassis_turn)
{
	if (IF_KEY_PRESSED_W)
		timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
	if (IF_KEY_PRESSED_S)
		timeXFron = 0;//同理
	if (IF_KEY_PRESSED_D)
		timeYLeft = 0;
	if (IF_KEY_PRESSED_A)
		timeYRigh = 0;
	
	Slope_Chassis_Move_Fron = (int16_t)( CHASSIS_RC_MAX_SPEED_X * Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, 1, 100 ) );

	Slope_Chassis_Move_Back = (int16_t)( -CHASSIS_RC_MAX_SPEED_X * Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, 1, 100) );

	Slope_Chassis_Move_Left = (int16_t)( -CHASSIS_RC_MAX_SPEED_Y * Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYLeft, 1, 100 ) );

	Slope_Chassis_Move_Righ = (int16_t)( CHASSIS_RC_MAX_SPEED_Y * Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYRigh, 1, 100 ) );
			
	if((DRIVE_MODE == NORMAL) && (INPUTMOD == KEYMOUSE_INPUT))
	{
	  chassis_turn->chassis_mode = AHEAD;
		
//		if(fabs(chassis_turn->Gimbal_yaw_data->relative_angle) > 8.0f)
//		{
		  chassis_turn->chassis_vw = PID_Calc(&chassis_turn->follow_gimbal_pid , chassis_turn->Gimbal_yaw_data->relative_angle , 0.0f);
//		}
//		else
//		{
//			chassis_turn->chassis_vw = 0;
//		}
		
		if((IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron, chassis_turn->chassis_vx,1.5f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ, chassis_turn->chassis_vy,1.5f);  //左右计算 
		}
		if((!IF_KEY_PRESSED_SHIFT) && (IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron, chassis_turn->chassis_vx,2.0f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ, chassis_turn->chassis_vy,2.0f);  //左右计算 
		}
		if((!IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron, chassis_turn->chassis_vx,1.0f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ, chassis_turn->chassis_vy,1.0f);  //左右计算 
		}
		if((!IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron, chassis_turn->chassis_vx,2.0f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ, chassis_turn->chassis_vy,2.0f);  //左右计算 
		}
	}
	else if((DRIVE_MODE == TURN_BACK) && (INPUTMOD == KEYMOUSE_INPUT))
	{
		chassis_turn->chassis_mode = BACK;
		
//		if(fabs(chassis_turn->Gimbal_yaw_data->relative_angle) > 8.0f)
//		{
		  chassis_turn->chassis_vw = PID_Calc(&chassis_turn->follow_gimbal_pid , (chassis_turn->Gimbal_yaw_data->relative_angle) , 0.0f);
//		}
//		else
//		{
//			chassis_turn->chassis_vw = 0;
//		}
		if((IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(-(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron), chassis_turn->chassis_vx,1.5f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(-(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ), chassis_turn->chassis_vy,1.5f);  //左右计算 
		}
		if((!IF_KEY_PRESSED_SHIFT) && (IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(-(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron), chassis_turn->chassis_vx,2.0f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(-(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ), chassis_turn->chassis_vy,2.0f);  //左右计算 
		}
		if((!IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(-(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron), chassis_turn->chassis_vx,1.0f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(-(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ), chassis_turn->chassis_vy,1.0f);  //左右计算 
		}
		if((!IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
		{
			chassis_turn->chassis_vx = RAMP_float(-(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron), chassis_turn->chassis_vx,2.0f);  //前后计算
			chassis_turn->chassis_vy = RAMP_float(-(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ), chassis_turn->chassis_vy,2.0f);  //左右计算 
		}
	}
	if(chassis_turn->chassis_mode != chassis_turn->chassis_mode_last)
	{
		CHASSIS_MODE = TURNING;	
	}
	if(CHASSIS_MODE == TURNING)
	{
		chassis_turn->chassis_vw = 0;
		
		if(fabs(chassis_turn->Gimbal_yaw_data->set_angle - chassis_turn->Gimbal_yaw_data->PidFb) < 5.0f)
		{
			CHASSIS_MODE = MAINTAIN;
		}
	}
	chassis_turn->chassis_mode_last = chassis_turn->chassis_mode; 
}

/**
  * @brief  遥控器控制
  * @param  
  * @retval void
  * @attention  
  */
void Remote_Ctrl(Chassis_Control_t *remote_ctrl)
{

	  remote_ctrl->chassis_vw = -RC_Control_CH2 * 0.05;
	  remote_ctrl->chassis_vx = RC_Control_CH1;
	  remote_ctrl->chassis_vy = RC_Control_CH0;
	
	  remote_ctrl->chassis_vw = Constrain_float(remote_ctrl->chassis_vw , remote_ctrl->chassis_vw * 0.6f , remote_ctrl->chassis_vw * 0.85f);
		remote_ctrl->chassis_vx = Constrain_float(remote_ctrl->chassis_vx , -500 , 500);
		remote_ctrl->chassis_vy = Constrain_float(remote_ctrl->chassis_vy , -500 , 500);

}

/**
  * @brief  键鼠控制
  * @param  
  * @retval void
  * @attention  
  */
void Key_Mouse_Ctrl(Chassis_Control_t *key_mouse)
{
	static portTickType  ulCurrentTime = 0;
	static uint32_t  ulDelay = 0;	
	static uint8_t Flag = 1;
	ulCurrentTime = xTaskGetTickCount();//当前系统时间	
	if (ulCurrentTime >= ulDelay)//每3ms变化一次斜坡量
	{ 
		ulDelay = ulCurrentTime + TIME_STAMP_1MS;
		
		Chassis_Turn(key_mouse);
			
			if((IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
			{
				key_mouse->chassis_vx = Constrain_float(key_mouse->chassis_vx , -750 , 750);
				key_mouse->chassis_vy = Constrain_float(key_mouse->chassis_vy , -750 , 750);
			}
			if((!IF_KEY_PRESSED_SHIFT) && (IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
			{
				key_mouse->chassis_vx = Constrain_float(key_mouse->chassis_vx , -250 , 250);
				key_mouse->chassis_vy = Constrain_float(key_mouse->chassis_vy , -250 , 250);
			}
			if((!IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (IF_MOUSE_PRESSED_RIGH))
			{
				key_mouse->chassis_vx = Constrain_float(key_mouse->chassis_vx , -50 , 50);
				key_mouse->chassis_vy = Constrain_float(key_mouse->chassis_vy , -50 , 50);
			}
			if((!IF_KEY_PRESSED_SHIFT) && (!IF_MOUSE_PRESSED_LEFT) && (!IF_MOUSE_PRESSED_RIGH))
			{
				key_mouse->chassis_vx = key_mouse->chassis_vx;
				key_mouse->chassis_vx=Constrain_float(key_mouse->chassis_vx,-500,500);
				key_mouse->chassis_vy = key_mouse->chassis_vy;
				key_mouse->chassis_vy=Constrain_float(key_mouse->chassis_vy,-500,500);
			}
	}
}


/**
  * @brief  底盘模式设置
  * @param  底盘数据结构体，遥控器数据指针
  * @retval void
  */
void Chassis_Mode_Set(Chassis_Control_t *chassis_move_control , remote_mode_e *inputmode )
{
	if(chassis_move_control == NULL)
	{
		return ;
	}
	if(*inputmode == RUN_STOP)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			chassis_move_control->set_power_speed[i] = 0;
			chassis_move_control->set_steer_angle[i] = steer_age_offset[i];
			chassis_move_control->steer_offset_angle[i]=chassis_move_control->chassis_steer_motor[i].chassis_motor_measure->real_ecd;
			chassis_move_control->steer_angle_fdb[i]=chassis_move_control->chassis_steer_motor[i].chassis_motor_measure->real_ecd;
		}
		chassis_move_control->chassis_vw = 0;
	}
	if(*inputmode == REMOTE_INPUT)
	{
    Remote_Ctrl(chassis_move_control);
		Chassis_Romete_Mode(chassis_move_control->chassis_vx , chassis_move_control->chassis_vy , chassis_move_control->chassis_vw , chassis_move_control);
	}
	if(*inputmode == KEYMOUSE_INPUT)
	{
    Key_Mouse_Ctrl(chassis_move_control);
		Chassis_Romete_Mode(chassis_move_control->chassis_vx , chassis_move_control->chassis_vy , chassis_move_control->chassis_vw , chassis_move_control);
  }
}

/**
  * @brief  舵轮的解算
  * @param  遥控器期望
  * @retval 实际速度
  * @attention  主要应用于小陀螺的时的姿态解算，其余的时候不调用此函数 
  */

void Steering_Calc(float vx, float vy, float vw, Chassis_Control_t *Chassis_Calc)
{
	float tan1[4] = {0, 0, 0, 0};
	float rotate_ratio_x[4], 
		           rotate_ratio_y[4], 
	             v_w[4]; 
	
	for (int i = 0; i < 4; i++)
	{
		rotate_ratio_x[i] = vx * RC_CHASSIS_RESPONSE; //获取通道值，或通过其他方式对x方向进行速度赋值（此处建议用通道值）
		
		rotate_ratio_y[i] = vy * RC_CHASSIS_RESPONSE; //获取y轴速度
		
		 v_w[i] = vw * 30; //获取z轴速度
	}


	//对3508的速度进项解算--因为解算的值全都是正值，从而对角度进行判断
	//将cos和sin转换成0.707，防止因为数值过大导致数据溢出（在数据转换后没有测试）。
	Chassis_Calc->set_power_speed[0] = sqrt((rotate_ratio_y[1] - v_w[1] * 0.707f) * (rotate_ratio_y[1] - v_w[1] * 0.707f) + (rotate_ratio_x[1] + v_w[1] * 0.707f) * (rotate_ratio_x[1] + v_w[1] * 0.707f));
	Chassis_Calc->set_power_speed[1] = sqrt((rotate_ratio_y[0] - v_w[0] * 0.707f) * (rotate_ratio_y[0] - v_w[0] * 0.707f) + (rotate_ratio_x[0] - v_w[0] * 0.707f) * (rotate_ratio_x[0] - v_w[0] * 0.707f));
	Chassis_Calc->set_power_speed[2] = sqrt((rotate_ratio_y[3] + v_w[3] * 0.707f) * (rotate_ratio_y[3] + v_w[3] * 0.707f) + (rotate_ratio_x[3] - v_w[3] * 0.707f) * (rotate_ratio_x[3] - v_w[3] * 0.707f));
	Chassis_Calc->set_power_speed[3] = sqrt((rotate_ratio_y[2] + v_w[2] * 0.707f) * (rotate_ratio_y[2] + v_w[2] * 0.707f) + (rotate_ratio_x[2] + v_w[2] * 0.707f) * (rotate_ratio_x[2] + v_w[2] * 0.707f));

#if(CHASSIS_Solution_MODE==1)	
	tan1[0] = atan2((rotate_ratio_y[1] - v_w[1] * 0.707f), (rotate_ratio_x[1] + v_w[1] * 0.707f)) * 57.2957805f;
	tan1[1] = atan2((rotate_ratio_y[0] - v_w[0] * 0.707f), (rotate_ratio_x[0] - v_w[0] * 0.707f)) * 57.2957805f;
	tan1[2] = atan2((rotate_ratio_y[3] + v_w[3] * 0.707f), (rotate_ratio_x[3] - v_w[3] * 0.707f)) * 57.2957805f;
	tan1[3] = atan2((rotate_ratio_y[2] + v_w[2] * 0.707f), (rotate_ratio_x[2] + v_w[2] * 0.707f)) * 57.2957805f;
	if(vw>=0)
	{
		Chassis_Calc->Mode_flag[0]=1;
	  Chassis_Calc->Mode_flag[1]=-1;
	  Chassis_Calc->Mode_flag[2]=-1;
	  Chassis_Calc->Mode_flag[3]=1;
	}
	if(vw<0)
	{
			Chassis_Calc->Mode_flag[0]=-1;
	  Chassis_Calc->Mode_flag[1]=1;
	  Chassis_Calc->Mode_flag[2]=1;
	  Chassis_Calc->Mode_flag[3]=-1;
	
	}



#elif(CHASSIS_Solution_MODE==2)	
	tan1[0] = atan((rotate_ratio_y[1] - v_w[1] * 0.707f), (rotate_ratio_x[1] + v_w[1] * 0.707f)) * 57.2957805f;
	tan1[1] = atan((rotate_ratio_y[0] - v_w[0] * 0.707f), (rotate_ratio_x[0] - v_w[0] * 0.707f)) * 57.2957805f;
	tan1[2] = atan((rotate_ratio_y[3] + v_w[3] * 0.707f), (rotate_ratio_x[3] - v_w[3] * 0.707f)) * 57.2957805f;
	tan1[3] = atan((rotate_ratio_y[2] + v_w[2] * 0.707f), (rotate_ratio_x[2] + v_w[2] * 0.707f)) * 57.2957805f;
	if(vx<0)
	{
    Chassis_Calc->Mode_flag=-1;
	}
	else 
	{
	  Chassis_Calc->Mode_flag=1;
	}
	#endif
	if((vx!=0)||(vy!=0))
	{
			if(Chassis_Calc->chassis_vw==0)
			{
				for (int i = 0; i < 4; i++)
				{
					if(tan1[i]==0||tan1[i]==90||tan1[i]==-90)
					{
						Chassis_Calc->Mode_flag[i] = 1;
					  Chassis_Calc->set_steer_angle[i]=tan1[i];
					}
//					else 
//					{
					//计算舵向电机在解算坐标系中位置         
					 Chassis_Calc->steer_real_angl[i] = Chassis_Calc->chassis_steer_motor[i].chassis_motor_measure->real_ecd - (Chassis_Calc->steer_offset_angle[i]);
						if(Chassis_Calc->steer_real_angl[i]<0)
						{
						Chassis_Calc->steer_real_angl[i]=Chassis_Calc->steer_real_angl[i]+360;
						}
						else if(Chassis_Calc->steer_real_angl[i]>360)
						{
						Chassis_Calc->steer_real_angl[i]=Chassis_Calc->steer_real_angl[i]-360;
						}
//					Chassis_Calc->steer_real_angl[1] = Chassis_Calc->chassis_steer_motor[1].chassis_motor_measure->real_ecd - (Chassis_Calc->steer_offset_angle[1]);
//					Chassis_Calc->steer_real_angl[2] = Chassis_Calc->chassis_steer_motor[2].chassis_motor_measure->real_ecd - (Chassis_Calc->steer_offset_angle[2] );//
//					Chassis_Calc->steer_real_angl[3] = Chassis_Calc->chassis_steer_motor[3].chassis_motor_measure->real_ecd - (Chassis_Calc->steer_offset_angle[3]);
						
					//将解算值转换为0到360的连续值
					if (tan1[i] < 0)
					{
						tan1[i] = tan1[i] + 360;
					}
					//舵向电机应该到达的位置
					Chassis_Calc->steer_should_arrive_angle[i] =tan1[i];
					//计算舵向电机对应的反向角度反向角度
					if (Chassis_Calc->steer_should_arrive_angle[i] >= 180)
					{
						Chassis_Calc->steer_reverse_angle[i] = Chassis_Calc->steer_should_arrive_angle[i] - 180;
					}
					else if (Chassis_Calc->steer_should_arrive_angle[i] < 180)
					{
						Chassis_Calc->steer_reverse_angle[i] = Chassis_Calc->steer_should_arrive_angle[i] + 180;
					}
			//		//通过计算当前角度与应到角度差值判断旋转角度与方向
							if (fabs(Chassis_Calc->steer_real_angl[i] - Chassis_Calc->steer_should_arrive_angle[i]) <= 90)
					{
						//如果旋转角度在正负90度之间则以期望角度旋转
						Chassis_Calc->set_steer_angle[i] = Chassis_Calc->steer_should_arrive_angle[i];
						Chassis_Calc->Mode_flag[i] = 1; // 3508正转
					}
					else
					{
						
			//    	Chassis_Calc->steer_should_arrive_angle[i] = Chassis_Calc->steer_reverse_angle[i];
						//如果旋转角度在90-180之间则往返方向旋转
						
						if (fabs(Chassis_Calc->steer_real_angl[i] - Chassis_Calc->steer_reverse_angle[i]) <= 90)
						{
							Chassis_Calc->set_steer_angle[i] =  Chassis_Calc->steer_reverse_angle[i];
							Chassis_Calc->Mode_flag[i] = -1; // 3508反转
						}
						else if(fabs(Chassis_Calc->steer_real_angl[i]-Chassis_Calc->steer_should_arrive_angle[i] + 360)<90)
						{
							Chassis_Calc->set_steer_angle[i] = Chassis_Calc->steer_should_arrive_angle[i] - 360;
							Chassis_Calc->Mode_flag[i] = 1;
						}
						else if(fabs(Chassis_Calc->steer_real_angl[i]+Chassis_Calc->steer_should_arrive_angle[i] - 360)<=90)
						{
						   Chassis_Calc->set_steer_angle[i]=Chassis_Calc->steer_should_arrive_angle[i]+360;
							Chassis_Calc->Mode_flag[i] = 1;
						}
//						if(Chassis_Calc->set_steer_angle[i]<-180)
//						{
//								 Chassis_Calc->set_steer_angle[i]=Chassis_Calc->set_steer_angle[i]+180;
//						     
//						}

//						if(Chassis_Calc->steer_should_arrive_angle[i]>270)
					}
//				}
//				if(vx>=0||vy>=0)
//				{
//				Chassis_Calc->Mode_flag[i] = 1;				
//				}
//				else if(vx<0||vy<0)
//				{
//				Chassis_Calc->Mode_flag[i] = -1;			
//				}
				Chassis_Calc->steer_angle_fdb[i]=Chassis_Calc->chassis_steer_motor[i].chassis_motor_measure->real_ecd-Chassis_Calc->steer_offset_angle[i];
				}

			}
			else 
			{
			for(int i=0;i<4;i++)
				{
					Chassis_Calc->set_steer_angle[i] = steer_age_offset[i] + tan1[i];
					Chassis_Calc->steer_angle_fdb[i]=Chassis_Calc->chassis_steer_motor[i].chassis_motor_measure->real_ecd;	
				}
			
   
			}
	
	}
  else
	{
	  Chassis_Calc->set_steer_angle[0] = steer_age_offset[0] - 45.0f;
		Chassis_Calc->set_steer_angle[1] = steer_age_offset[1] + 45.0f;
		Chassis_Calc->set_steer_angle[2] = steer_age_offset[2] - 45.0f;
		Chassis_Calc->set_steer_angle[3] = steer_age_offset[3] + 45.0f;
	if(vw>=0)
	{
	 Chassis_Calc->Mode_flag[0]=1;
	 Chassis_Calc->Mode_flag[1]=-1;
	 Chassis_Calc->Mode_flag[2]=-1;
	 Chassis_Calc->Mode_flag[3]=1;
	}
	if(vw<0)
	{
		Chassis_Calc->Mode_flag[0]=-1;
	  Chassis_Calc->Mode_flag[1]=1;
	  Chassis_Calc->Mode_flag[2]=1;
	  Chassis_Calc->Mode_flag[3]=-1;
	}

		for(int i=0;i<4;i++)
		{
			Chassis_Calc->steer_angle_fdb[i]=Chassis_Calc->chassis_steer_motor[i].chassis_motor_measure->real_ecd;	
		}
		
	}
	
}

/**
  * @brief  舵轮的解算2（向书镜修改后版本）
  * @param  遥控器期望
  * @retval 实际速度
  * @attention  
  */
void Chassis_Romete_Mode(float vx,float vy,float vw,Chassis_Control_t *Chassis_Mode)
{
	float tan1[4] = {0, 0, 0, 0};
  fp32 bili = 70;
	float rotate_ratio_x[4], 
		           rotate_ratio_y[4], 
	             v_w[4]; 
	
	for (int i = 0; i < 4; i++)
	{
		rotate_ratio_x[i] = vx * RC_CHASSIS_RESPONSE; //获取通道值，或通过其他方式对x方向进行速度赋值（此处建议用通道值）
		
		rotate_ratio_y[i] = vy * RC_CHASSIS_RESPONSE; //获取y轴速度
		
		 v_w[i] = vw * bili; //获取z轴速度
	}


	//对3508的速度进项解算--因为解算的值全都是正值，从而对角度进行判断
	//将cos和sin转换成0.707，防止因为数值过大导致数据溢出（在数据转换后没有测试）。
	Chassis_Mode->set_power_speed[0] = sqrt((rotate_ratio_y[1] - v_w[1] * 0.707f) * (rotate_ratio_y[1] - v_w[1] * 0.707f) + (rotate_ratio_x[1] + v_w[1] * 0.707f) * (rotate_ratio_x[1] + v_w[1] * 0.707f));
	Chassis_Mode->set_power_speed[1] = sqrt((rotate_ratio_y[0] - v_w[0] * 0.707f) * (rotate_ratio_y[0] - v_w[0] * 0.707f) + (rotate_ratio_x[0] - v_w[0] * 0.707f) * (rotate_ratio_x[0] - v_w[0] * 0.707f));
	Chassis_Mode->set_power_speed[2] = sqrt((rotate_ratio_y[3] + v_w[3] * 0.707f) * (rotate_ratio_y[3] + v_w[3] * 0.707f) + (rotate_ratio_x[3] - v_w[3] * 0.707f) * (rotate_ratio_x[3] - v_w[3] * 0.707f));
	Chassis_Mode->set_power_speed[3] = sqrt((rotate_ratio_y[2] + v_w[2] * 0.707f) * (rotate_ratio_y[2] + v_w[2] * 0.707f) + (rotate_ratio_x[2] + v_w[2] * 0.707f) * (rotate_ratio_x[2] + v_w[2] * 0.707f));

#if(CHASSIS_Solution_MODE==1)	
	tan1[0] = atan2((rotate_ratio_y[1] - v_w[1] * 0.707f), (rotate_ratio_x[1] + v_w[1] * 0.707f)) * 57.2957805f;
	tan1[1] = atan2((rotate_ratio_y[0] - v_w[0] * 0.707f), (rotate_ratio_x[0] - v_w[0] * 0.707f)) * 57.2957805f;
	tan1[2] = atan2((rotate_ratio_y[3] + v_w[3] * 0.707f), (rotate_ratio_x[3] - v_w[3] * 0.707f)) * 57.2957805f;
	tan1[3] = atan2((rotate_ratio_y[2] + v_w[2] * 0.707f), (rotate_ratio_x[2] + v_w[2] * 0.707f)) * 57.2957805f;
	if(vw>=0)
	{
		Chassis_Mode->Mode_flag[0]=1;
	  Chassis_Mode->Mode_flag[1]=-1;
	  Chassis_Mode->Mode_flag[2]=-1;
	  Chassis_Mode->Mode_flag[3]=1;
	}
	if(vw<0)
	{
			Chassis_Mode->Mode_flag[0]=-1;
	  Chassis_Mode->Mode_flag[1]=1;
	  Chassis_Mode->Mode_flag[2]=1;
	  Chassis_Mode->Mode_flag[3]=-1;
	
	}



#elif(CHASSIS_Solution_MODE==2)	
	tan1[0] = atan((rotate_ratio_y[1] - v_w[1] * 0.707f), (rotate_ratio_x[1] + v_w[1] * 0.707f)) * 57.2957805f;
	tan1[1] = atan((rotate_ratio_y[0] - v_w[0] * 0.707f), (rotate_ratio_x[0] - v_w[0] * 0.707f)) * 57.2957805f;
	tan1[2] = atan((rotate_ratio_y[3] + v_w[3] * 0.707f), (rotate_ratio_x[3] - v_w[3] * 0.707f)) * 57.2957805f;
	tan1[3] = atan((rotate_ratio_y[2] + v_w[2] * 0.707f), (rotate_ratio_x[2] + v_w[2] * 0.707f)) * 57.2957805f;
	if(vx<0)
	{
    Chassis_Calc->Mode_flag=-1;
	}
	else 
	{
	  Chassis_Calc->Mode_flag=1;
	}
	#endif
	if((vx!=0)||(vy!=0))
	{
		if(vx > 0 && vw > vx)
		{
			vw = vx - 5;
		}
		if(vx < 0 && vw < vx)
		{
			vw = vx + 5;
		}
		if((vy > 0) && vw > vy)
		{
			vw = vy - 5;
		}
		if((vy < 0) && vw < vy)
		{
			vw = vy + 5;
		}
			for (int i = 0; i < 4; i++)
			{
				//计算舵向电机在解算坐标系中位置         
				Chassis_Mode->steer_real_angl[i] = Chassis_Mode->chassis_steer_motor[i].chassis_motor_measure->real_ecd - (Chassis_Mode->steer_offset_angle[i]);
				
				if(Chassis_Mode->steer_real_angl[i]<0)
				{
				Chassis_Mode->steer_real_angl[i]=Chassis_Mode->steer_real_angl[i]+360;
				}		
				
			 else if(Chassis_Mode->steer_real_angl[i]>360)	
			 {
			 Chassis_Mode->steer_real_angl[i]=Chassis_Mode->steer_real_angl[i]-360;
			 }					 
			 
				//将解算值转换为0到360的连续值
				if (tan1[i] < 0)
				{
					tan1[i] = tan1[i] + 360;
				}
				
				//舵向电机应该到达的位置
				//计算舵向电机对应的反向角度反向角度
				if (tan1[i] >= 180)
				{
					Chassis_Mode->steer_reverse_angle[i] = tan1[i] - 180;
				}
				
				else if (tan1[i] < 180)
				{
					Chassis_Mode->steer_reverse_angle[i] = tan1[i] + 180;
				}
				
				if((tan1[i] < 200) && (tan1[i] > 160))
				{
					tan1[i]=Chassis_Mode->steer_reverse_angle[i];
				}
				
				Chassis_Mode->steer_should_arrive_angle[i] = tan1[i];
				
			 //通过计算当前角度与应到角度差值判断旋转角度与方向
				if (fabs(Chassis_Mode->steer_real_angl[i] - Chassis_Mode->steer_should_arrive_angle[i]) <= 90)
				{
					//如果旋转角度在正负90度之间则以期望角度旋转
					Chassis_Mode->steer_should_rotate_angle[i]=Chassis_Mode->steer_should_arrive_angle[i]-Chassis_Mode->steer_real_angl[i];
//						Chassis_Mode->set_steer_angle[i] = Chassis_Mode->chassis_steer_motor[i].chassis_motor_measure->real_ecd+Chassis_Mode->steer_should_rotate_angle[i];
					Chassis_Mode->Mode_flag[i] = 1; // 3508正转
				}
				
				else
				{
					
//						Chassis_Calc->steer_should_arrive_angle[i] = Chassis_Calc->steer_reverse_angle[i];
					//如果旋转角度在90-180之间则往返方向旋转
					
					if (fabs(Chassis_Mode->steer_real_angl[i] - Chassis_Mode->steer_reverse_angle[i]) <= 90)
					{
						Chassis_Mode->steer_should_rotate_angle[i]=Chassis_Mode->steer_reverse_angle[i]-Chassis_Mode->steer_real_angl[i];
//							Chassis_Mode->set_steer_angle[i] =  Chassis_Mode->steer_reverse_angle[i];
						Chassis_Mode->Mode_flag[i] = -1; // 3508反转
					}
					
					else if(fabs(Chassis_Mode->steer_real_angl[i]-Chassis_Mode->steer_should_arrive_angle[i] + 360)<90)
					{
						Chassis_Mode->steer_should_rotate_angle[i]=(-Chassis_Mode->steer_should_arrive_angle[i] + 360) - Chassis_Mode->steer_real_angl[i];
//							Chassis_Mode->set_steer_angle[i] =Chassis_Mode->chassis_steer_motor[i].chassis_motor_measure->real_ecd -
//                                    							Chassis_Mode->steer_real_angl[i]-Chassis_Mode->steer_should_arrive_angle[i] + 360;
						Chassis_Mode->Mode_flag[i] = 1;
					}
					
					else if(fabs(Chassis_Mode->steer_real_angl[i]+Chassis_Mode->steer_should_arrive_angle[i] - 360)<=90)
					{
						Chassis_Mode->steer_should_rotate_angle[i]=(Chassis_Mode->steer_should_arrive_angle[i] + 360) - Chassis_Mode->steer_real_angl[i];
//						   Chassis_Mode->set_steer_angle[i]=Chassis_Mode->chassis_steer_motor[i].chassis_motor_measure->real_ecd + 
//							                                  Chassis_Mode->steer_real_angl[i]+Chassis_Mode->steer_should_arrive_angle[i] - 360;
						Chassis_Mode->Mode_flag[i] = 1;
					}	
															
				}
				
				if(tan1[i]==Chassis_Mode->steer_reverse_angle[i])
				{
					Chassis_Mode->Mode_flag[i] = -1;
				}
				
				Chassis_Mode->steer_angle_fdb[i]=-Chassis_Mode->steer_should_rotate_angle[i];

				Chassis_Mode->set_steer_angle[i] = 0;
				
//				if(vx>0||vy<0)
//				{
//				Chassis_Mode->Mode_flag[i] = 1;				
//				}
//				else if(vx<0||vy>0)
//				{
//				Chassis_Mode->Mode_flag[i] = -1;			
//				}
//				
			}

	
	}
  else
	{
	  Chassis_Mode->set_steer_angle[0] = steer_age_offset[0] - 45.0f;
		Chassis_Mode->set_steer_angle[1] = steer_age_offset[1] + 45.0f;
		Chassis_Mode->set_steer_angle[2] = steer_age_offset[2] - 45.0f;
		Chassis_Mode->set_steer_angle[3] = steer_age_offset[3] + 45.0f;
		
		if(vw>=0)
		{
		 Chassis_Mode->Mode_flag[0]=1;
		 Chassis_Mode->Mode_flag[1]=-1;
		 Chassis_Mode->Mode_flag[2]=-1;
		 Chassis_Mode->Mode_flag[3]=1;
		}
		
		if(vw<0)
		{
			Chassis_Mode->Mode_flag[0]=-1;
			Chassis_Mode->Mode_flag[1]=1;
			Chassis_Mode->Mode_flag[2]=1;
			Chassis_Mode->Mode_flag[3]=-1;
		}

		for(int i=0;i<4;i++)
		{
			Chassis_Mode->steer_angle_fdb[i]=Chassis_Mode->chassis_steer_motor[i].chassis_motor_measure->real_ecd;	
		}
		
	}

}

/**
  * @brief  电机输出
  * @param  底盘数据结构体
  * @retval void
  * @attention  
  */
void Chassis_Motor_Control(Chassis_Control_t *chassis_motor_control)
{
	
	if (chassis_motor_control == NULL)
	{
			return;
	}
	
	for(uint8_t i = 0; i < 4; i++)
	{
		for(uint8_t i = 0; i < 4; i++)
		PID_Calc(&chassis_motor_control->chassis_power_speed_pid[i] , chassis_motor_control->chassis_power_motor[i].chassis_motor_measure->speed_rpm , chassis_motor_control->set_power_speed[i]*chassis_motor_control->Mode_flag[i]);
		PID_Calc(&chassis_motor_control->chassis_steer_angle_pid[i] , chassis_motor_control->steer_angle_fdb[i], chassis_motor_control->set_steer_angle[i]);
		PID_Calc(&chassis_motor_control->chassis_steer_speed_pid[i] , chassis_motor_control->chassis_steer_motor[i].chassis_motor_measure->speed_rpm , chassis_motor_control->chassis_steer_angle_pid[i].out);
	}
	for(uint8_t i = 0; i < 4; i++)
	{
		power_out[i] = chassis_motor_control->chassis_power_speed_pid[i].out;
		steer_out[i] = chassis_motor_control->chassis_steer_speed_pid[i].out;
	}

	CAN1_CMD_CHASSIS(power_out[0] , power_out[1] , power_out[2] , power_out[3]);
	CAN2_CMD_CONTROL(steer_out[0],steer_out[1] , steer_out[2] , steer_out[3]);
	
	vTaskDelay(1);
}


/**
  * @brief  底盘任务
  * @param  void
  * @retval void
  * @attention  
  */
void Chassis_task(void *pvParameters)
{
	Chassis_Init(&chassis_control);
	while(1)
	{
		Chassis_Mode_Set(&chassis_control ,&INPUTMOD);
		
		Chassis_Motor_Control(&chassis_control);
				
		vTaskDelay(1);  //系统延时
	}
}

