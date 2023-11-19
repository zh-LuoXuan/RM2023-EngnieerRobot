#include "gimbal_task.h"
#include "sys.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "new_pid.h"

#include "chassis_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "gpio.h"
#include "user_lib.h"
#include "kalman.h"
#include "kalman_filter.h"

#include "pwm.h"

/*==============================================================*/
#define GIMBAL_TASK_PRIO 28	
#define GIMBAL_STK_SIZE 512
TaskHandle_t GimbalTask_Handler;
void Gimbal_task(void);

/*==============================================================*/

void task_Gimbal_Create(void)
{
	xTaskCreate((TaskFunction_t)Gimbal_task,
                (const char *)"Gimbal_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GimbalTask_Handler);
}

/*==============================================================*/

GimbalCtrl_t GimbalData;

fp32 pitchAgePid[3] = {GIMBAL_PITCH_ANGLE_PID_KP, GIMBAL_PITCH_ANGLE_PID_KI, GIMBAL_PITCH_ANGLE_PID_KD};
fp32 pitchSpdPid[3] = {GIMBAL_PITCH_SPEED_PID_KP, GIMBAL_PITCH_SPEED_PID_KI, GIMBAL_PITCH_SPEED_PID_KD};

fp32 yawAgePid[3] = {GIMBAL_YAW_ANGLE_PID_KP, GIMBAL_YAW_ANGLE_PID_KI, GIMBAL_YAW_ANGLE_PID_KD};
fp32 yawSpdPid[3] = {GIMBAL_YAW_SPEED_PID_KP, GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD};


static void Gimbal_Init(GimbalCtrl_t* init);
static void Ctrl_Mode_set(GimbalCtrl_t* ctrlMode);
static void Gimbal_State_Set(GimbalCtrl_t* stateSet);
static void Ctrl_Refresh(GimbalCtrl_t* ctrlRefresh);
static void FeedBack_Refresh(GimbalCtrl_t* fdbRefresh);
static void Drive_Mode_Refresh(GimbalCtrl_t* driceFresh);
static void Ctrl_Buff_Refresh(GimbalCtrl_t* buffRefresh);
static void Data_Protect(GimbalCtrl_t* data);
static void Give_current(GimbalCtrl_t* giveCurrent);
static void CanSendData(GimbalCtrl_t* send);


/**
  * @brief  云台任务
  * @param  void
  * @retval void
  * @attention  
  */
void Gimbal_task(void)
{
	static portTickType TickCount; 
	
	TickCount = xTaskGetTickCount();
	
	Gimbal_Init(&GimbalData);
	while(1)
	{	
		Ctrl_Mode_set(&GimbalData);
		Gimbal_State_Set(&GimbalData);
		Ctrl_Refresh(&GimbalData);
		FeedBack_Refresh(&GimbalData);
		Give_current(&GimbalData);
		CAN1_CMD_GIMBAL(GimbalData.yawMotor.Current, GimbalData.pitchMotor.Current);
//		CanSendData(&GimbalData);
		Ctrl_Buff_Refresh(&GimbalData);
		
		vTaskDelay(2);
//		vTaskDelayUntil(&TickCount, 2);
	}
}


/**
  * @brief  云台初始化
  * @param  云台数据结构体指针
  * @retval void
  */
static void Gimbal_Init(GimbalCtrl_t* init)
{
	if(init == NULL)
	{
		return;
	}
#if ISGYRO
	init->gimbal_gyro_point = get_Gyro_Angle_Point();
#endif
	init->gimbal_rc_ctrl = get_remote_control_point();
	init->pitchMotor.GimbalEncoderMeasure = get_Pitch_EncoderProcess_Point();
	init->yawMotor.GimbalEncoderMeasure = get_Yaw_EncoderProcess_Point();
	
	/***************************云台PID初始化************************************/
	New_PID_Init(&init->pitchMotor.AnglePid, 
	             NEW_PID_POSITION, 
	             pitchAgePid, 
	             GIMBAL_PITCH_ANGLE_PID_MAX_OUT, 
	             GIMBAL_PITCH_ANGLE_PID_MAX_IOUT, 
               GIMBAL_PITCH_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             GIMBAL_PITCH_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&init->pitchMotor.SpeedPid, 
	             NEW_PID_POSITION, 
	             pitchSpdPid, 
	             GIMBAL_PITCH_SPEED_PID_MAX_OUT, 
	             GIMBAL_PITCH_SPEED_PID_MAX_IOUT, 
               GIMBAL_PITCH_SPEED_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             GIMBAL_PITCH_SPEED_PID_I_SEPARATION);
							 
  New_PID_Init(&init->yawMotor.AnglePid, 
	             NEW_PID_POSITION, 
	             yawAgePid, 
	             GIMBAL_YAW_ANGLE_PID_MAX_OUT, 
	             GIMBAL_YAW_ANGLE_PID_MAX_IOUT, 
               GIMBAL_YAW_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             GIMBAL_YAW_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&init->yawMotor.SpeedPid, 
	             NEW_PID_POSITION, 
	             yawSpdPid, 
	             GIMBAL_YAW_SPEED_PID_MAX_OUT, 
	             GIMBAL_YAW_SPEED_PID_MAX_IOUT, 
               GIMBAL_YAW_SPEED_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             GIMBAL_YAW_SPEED_PID_I_SEPARATION);
							 
	
	init->pitchMotor.CenterOffset = PITCH_CENTER_OFFSET;
	init->yawMotor.CenterOffset = YAW_CENTER_OFFSET;

	init->CtrlMode = NO_CTRL;
	init->DriveMode = NORMAL;
	init->GimbalMeasure = ENCODER_MEASURE;
	init->GimbalState = GIMBAL_RESET;
	
	init->ISTurn_Flag = 0;
}


/**
  * @brief  控制模式设置
  * @param  云台数据结构体指针
  * @retval void
  */
static void Ctrl_Mode_set(GimbalCtrl_t* ctrlMode)
{
	if(ctrlMode == NULL)
	{
		return;
	}
	if(switch_is_down(ctrlMode->gimbal_rc_ctrl->rc.s[ModeChannel_R]))//右下，无控制模式
	{
		ctrlMode->CtrlMode = NO_CTRL;
	}
	else if(switch_is_mid(ctrlMode->gimbal_rc_ctrl->rc.s[ModeChannel_R])&&switch_is_down(ctrlMode->gimbal_rc_ctrl->rc.s[ModeChannel_L]))//右中左下，遥控器模式
	{
		ctrlMode->CtrlMode = RC_CTRL;
	}
	else if(switch_is_up(ctrlMode->gimbal_rc_ctrl->rc.s[ModeChannel_R]))//右上，PC模式
	{
		ctrlMode->CtrlMode = PC_CTRL;
	}
	else
	{
		ctrlMode->CtrlMode = NO_CTRL;
	}
	Data_Protect(ctrlMode);//数据保护	
}


/**
  * @brief  云台状态设置
  * @param  云台数据结构体指针
  * @retval void
  */
static void Gimbal_State_Set(GimbalCtrl_t* stateSet)
{
	if(stateSet == NULL)
	{
		return;
	}
	switch(stateSet->CtrlMode)
	{
		case NO_CONTROL:
			{
				stateSet->GimbalState = GIMBAL_RESET;
				break;
			}			
		case RC_CONTROL:
		case PC_CONTROL:
			{
      	stateSet->GimbalState = GIMBAL_WORK;
				break;
		  }		
		default:
			break;
	}
	
}


/**
  * @brief  输入量更新
  * @param  云台数据结构体指针
  * @retval void
  */
static void Ctrl_Refresh(GimbalCtrl_t* ctrlRefresh)
{
	if(ctrlRefresh == NULL)
	{
		return;
	}
	
	switch(ctrlRefresh->GimbalState)
	{
		case GIMBAL_FREE:
		case GIMBAL_RESET:
			{
				ctrlRefresh->yawMotor.AngleRef = RAMP_float(ctrlRefresh->yawMotor.CenterOffset, ctrlRefresh->yawMotor.GimbalEncoderMeasure->ecd_value, SLOPE_BEGIN_YAW);
				ctrlRefresh->pitchMotor.AngleRef = PITCH_CENTER_OFFSET;
#if ISGYRO				
				if(fabs(ctrlRefresh->yawMotor.GimbalEncoderMeasure->ecd_value - ctrlRefresh->yawMotor.AngleRef) <= 0.5f)
				{
					ctrlRefresh->yawMotor.AbsoluteAngle = ctrlRefresh->gimbal_gyro_point->YAW;
				}
				if(fabs(ctrlRefresh->pitchMotor.GimbalEncoderMeasure->ecd_value - ctrlRefresh->pitchMotor.AngleRef) <= 0.5f)
				{
					ctrlRefresh->pitchMotor.AbsoluteAngle = ctrlRefresh->gimbal_gyro_point->Pitch;
				}
#else
				ctrlRefresh->yawMotor.AbsoluteAngle = ctrlRefresh->yawMotor.CenterOffset;
				ctrlRefresh->pitchMotor.AbsoluteAngle = PITCH_CENTER_OFFSET;
#endif
				break;
			}
		case GIMBAL_WORK:
			{
				
				if(ctrlRefresh->CtrlMode == RC_CTRL)
				{
          
				  Drive_Mode_Refresh(ctrlRefresh);
					
					ctrlRefresh->pitchMotor.AngleRef += ctrlRefresh->gimbal_rc_ctrl->rc.ch[1] * RC_PITCH_RESPONSE;
					
					ctrlRefresh->pitchMotor.AngleRef = Constrain_float(ctrlRefresh->pitchMotor.AngleRef, 
					                                                   ctrlRefresh->pitchMotor.CenterOffset - PITCH_MIN_OFFSET, 
					                                                   ctrlRefresh->pitchMotor.CenterOffset + PITCH_MAX_OFFSET);
					if(ctrlRefresh->DriveMode == TURN_BACK)
					{
						ctrlRefresh->yawMotor.AngleRef = ctrlRefresh->yawMotor.CenterOffset + YAW_FLIP_ANGLE;
					}
				  if(ctrlRefresh->DriveMode == NORMAL)
					{
						ctrlRefresh->yawMotor.AngleRef = ctrlRefresh->yawMotor.CenterOffset;
					} 
				}
				
				if(ctrlRefresh->CtrlMode == PC_CTRL)
				{					
				  Drive_Mode_Refresh(ctrlRefresh);
					
					if(!IF_MOUSE_PRESSED_LEFT && !IF_MOUSE_PRESSED_RIGH)
					{
					  ctrlRefresh->pitchMotor.AngleRef -= ctrlRefresh->gimbal_rc_ctrl->mouse.y * PC_PITCH_RESPONSE;
					}
					else
					{
						ctrlRefresh->pitchMotor.AngleRef += 0.0f;
					}
				
					ctrlRefresh->pitchMotor.AngleRef = Constrain_float(ctrlRefresh->pitchMotor.AngleRef, 
					                                                   ctrlRefresh->pitchMotor.CenterOffset - PITCH_MIN_OFFSET, 
					                                                   ctrlRefresh->pitchMotor.CenterOffset + PITCH_MAX_OFFSET);
#if ISGYRO	
					if(!IF_MOUSE_PRESSED_LEFT && !IF_MOUSE_PRESSED_RIGH)
					{
					  ctrlRefresh->yawMotor.AngleRef += ctrlRefresh->gimbal_rc_ctrl->mouse.x * PC_YAW_RESPONSE;
					}
					else
					{
					  ctrlRefresh->yawMotor.AngleRef += 0.0f;
				  }
					
					if(ctrlRefresh->DriveMode == TURN_BACK)
					{
					  ctrlRefresh->yawMotor.AngleRef = Constrain_float(ctrlRefresh->yawMotor.AngleRef, 
						                                                (ctrlRefresh->yawMotor.AbsoluteAngle + YAW_FLIP_ANGLE) - YAW_MIN_OFFSET, 
						                                                (ctrlRefresh->yawMotor.AbsoluteAngle + YAW_FLIP_ANGLE) + YAW_MAX_OFFSET);
					}
					if(ctrlRefresh->DriveMode == NORMAL)
					{
					  ctrlRefresh->yawMotor.AngleRef = Constrain_float(ctrlRefresh->yawMotor.AngleRef, 
						                                                 ctrlRefresh->yawMotor.AbsoluteAngle - YAW_MIN_OFFSET, 
						                                                 ctrlRefresh->yawMotor.AbsoluteAngle + YAW_MAX_OFFSET);
					} 
#else
						if(ctrlRefresh->DriveMode == TURN_BACK)
						{
							ctrlRefresh->yawMotor.AngleRef = ctrlRefresh->yawMotor.CenterOffset + YAW_FLIP_ANGLE;
						}
						if(ctrlRefresh->DriveMode == NORMAL)
						{
							ctrlRefresh->yawMotor.AngleRef = ctrlRefresh->yawMotor.CenterOffset;
						} 
#endif
       }
				break;
			}
		default:
			break;
	}
}



/**
  * @brief  反馈量更新
  * @param  云台数据结构体指针
  * @retval void
  */
static void FeedBack_Refresh(GimbalCtrl_t* fdbRefresh)
{
	if(fdbRefresh == NULL)
	{
		return;
	}
	switch(fdbRefresh->GimbalState)
	{
		case GIMBAL_FREE:
		case GIMBAL_WORK:
#if ISGYRO
			{
				fdbRefresh->yawMotor.AngleFdb = fdbRefresh->gimbal_gyro_point->YAW;
				fdbRefresh->yawMotor.SpeedFdb = fdbRefresh->gimbal_gyro_point->V_Z;
				
				fdbRefresh->pitchMotor.AngleFdb = fdbRefresh->gimbal_gyro_point->Pitch;
				fdbRefresh->pitchMotor.SpeedFdb = fdbRefresh->gimbal_gyro_point->V_Y;
				
				break;
			}
#endif
		case GIMBAL_RESET:
			{
				fdbRefresh->yawMotor.AngleFdb = fdbRefresh->yawMotor.GimbalEncoderMeasure->ecd_value;
				fdbRefresh->yawMotor.SpeedFdb = fdbRefresh->yawMotor.GimbalEncoderMeasure->filter_rate;
				
				fdbRefresh->pitchMotor.AngleFdb = fdbRefresh->pitchMotor.GimbalEncoderMeasure->ecd_value;
				fdbRefresh->pitchMotor.SpeedFdb = fdbRefresh->pitchMotor.GimbalEncoderMeasure->filter_rate;
				
				break;
			}
		default:
			break;
	}
}


/**
  * @brief  历史数据刷新
  * @param  云台数据结构体指针
  * @retval void
  */
static void Ctrl_Buff_Refresh(GimbalCtrl_t* buffRefresh)
{
	if(buffRefresh == NULL)
	{
		return;
	}
	buffRefresh->LastGimbalState = buffRefresh->GimbalState;
	buffRefresh->LastCtrlMode = buffRefresh->CtrlMode;
	buffRefresh->LastDriveMode = buffRefresh->DriveMode;
	buffRefresh->LastGimbalMeasure = buffRefresh->GimbalMeasure;
}


/**
  * @brief  云台调头
  * @param  云台数据结构体指针
  * @retval void
  */
static void Drive_Mode_Refresh(GimbalCtrl_t* driceFresh)
{
	if(driceFresh == NULL)
	{
		return;
	}
	
	
	switch(driceFresh->CtrlMode)
	{
		static uint16_t V_Flag = 1;
		case RC_CTRL:
		{
			if(driceFresh->gimbal_rc_ctrl->rc.ch[4] >= 500 && V_Flag)
			{
				V_Flag = 0;
			}
			if(driceFresh->gimbal_rc_ctrl->rc.ch[4] < 500 && !V_Flag)
			{
				driceFresh->ISTurn_Flag = 1;
				V_Flag = 1;
			}
			break;
		}
		case PC_CTRL:
		{
			if(IF_KEY_PRESSED_V && V_Flag)
			{
				V_Flag = 0;
			}
			if(!IF_KEY_PRESSED_V && !V_Flag)
			{
				driceFresh->ISTurn_Flag = 1;
				V_Flag = 1;
			}
			break;
		}
		default:
			break;
	}
	
	if(driceFresh->ISTurn_Flag)
	{
		if(driceFresh->DriveMode == NORMAL)
		{
			driceFresh->yawMotor.AngleRef = driceFresh->yawMotor.AbsoluteAngle + YAW_FLIP_ANGLE;
#if ISGYRO		
			if(fabs(driceFresh->gimbal_gyro_point->YAW - driceFresh->yawMotor.AngleRef) <= 5.0f)
#endif
			{
				driceFresh->ISTurn_Flag = 0;
				driceFresh->DriveMode = TURN_BACK;
			}
	  }
		else if(driceFresh->DriveMode == TURN_BACK)
		{
			driceFresh->yawMotor.AngleRef = driceFresh->yawMotor.AbsoluteAngle;
#if ISGYRO	
			if(fabs(driceFresh->gimbal_gyro_point->YAW - driceFresh->yawMotor.AngleRef) <= 5.0f)
#endif	
			{
				driceFresh->ISTurn_Flag = 0;
				driceFresh->DriveMode = NORMAL;
			}
		}
	}
}


/**
  * @brief  得到发送电流值
  * @param  数据结构体指针
  * @retval void
  */
static void Give_current(GimbalCtrl_t* giveCurrent)
{
	
	if (giveCurrent == NULL)
	{
		 return;
	}
	
/***************************得到pitch轴电流值************************************/	
	New_PID_Calc(&giveCurrent->pitchMotor.AnglePid, 
	              giveCurrent->pitchMotor.AngleFdb,  
	              giveCurrent->pitchMotor.AngleRef);
	New_PID_Calc(&giveCurrent->pitchMotor.SpeedPid, 
	              giveCurrent->pitchMotor.SpeedFdb,  
	              giveCurrent->pitchMotor.AnglePid.out);
	giveCurrent->pitchMotor.Current = giveCurrent->pitchMotor.SpeedPid.out;
	
/***************************得到yaw轴电流值************************************/	
	New_PID_Calc(&giveCurrent->yawMotor.AnglePid, 
	              giveCurrent->yawMotor.AngleFdb,  
	              giveCurrent->yawMotor.AngleRef);
	New_PID_Calc(&giveCurrent->yawMotor.SpeedPid, 
	              giveCurrent->yawMotor.SpeedFdb,  
	              giveCurrent->yawMotor.AnglePid.out);
	giveCurrent->yawMotor.Current = giveCurrent->yawMotor.SpeedPid.out;
}


/**
  * @brief  CAN发送
  * @param  数据结构体指针
  * @retval void
  */
static void CanSendData(GimbalCtrl_t* send)
{
//	send->topMoveKey = TOPMOVE_KEY;
	CAN1_CMD_GIMBAL(send->yawMotor.Current, send->pitchMotor.Current);
}


/**
  * @brief  模式切换数据保护
  * @param  云台数据结构体指针
  * @retval void
  */
static void Data_Protect(GimbalCtrl_t* data)
{
	if(data == NULL)
	{
		return;
	}
#if ISGYRO	
	if(data->LastCtrlMode != data->CtrlMode && data->CtrlMode == GIMBAL_WORK)
	{
		data->yawMotor.AngleRef = data->gimbal_gyro_point->YAW;
		data->pitchMotor.AngleRef = data->gimbal_gyro_point->Pitch;
		data->yawMotor.SpeedRef = data->gimbal_gyro_point->V_Z;
		data->pitchMotor.SpeedRef = data->gimbal_gyro_point->V_Y;
	}
	if(data->LastCtrlMode != data->CtrlMode && data->CtrlMode == GIMBAL_RESET)
	{
		data->yawMotor.AngleRef = data->yawMotor.GimbalEncoderMeasure->ecd_value;
		data->pitchMotor.AngleRef = data->pitchMotor.GimbalEncoderMeasure->ecd_value;
		data->yawMotor.SpeedRef = data->yawMotor.GimbalEncoderMeasure->filter_rate;
		data->pitchMotor.SpeedRef = data->pitchMotor.GimbalEncoderMeasure->filter_rate;
	}
#endif

}


/**
  * @brief  返回云台结构体指针
  * @param  void
  * @retval 
  * @attention  
  */
const GimbalCtrl_t* get_Gimbal_Point(void)
{
  return &GimbalData;
}

