#include "chassis_task.h"
#include "gimbal_task.h"
#include "new_pid.h"
#include "gpio.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "user_lib.h"

/*==============================================================*/
#define CHASSIS_TASK_PRIO 27
#define CHASSIS_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;
void Chassis_task(void);

/*==============================================================*/

void Chsssis_task_Create(void)
{
	xTaskCreate((TaskFunction_t)Chassis_task,
                (const char *)"Chassisl_task",
                (uint16_t)CHASSIS_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CHASSIS_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
}
/*==============================================================*/


fp32 followPid[3] = {CHASSIS_FOLLOW_PID_KP, CHASSIS_FOLLOW_PID_KI, CHASSIS_FOLLOW_PID_KD};


ChassisCtrl_t ChassisData;

fp32 Chassis_SpeedPid[3] = { CHASSIS_SPEED_PID_KP,  CHASSIS_SPEED_PID_KI,  CHASSIS_SPEED_PID_KD};

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
float Chassis_WS_Move_Max = 6600.0f;//底盘前后平移限速
float Chassis_AD_Move_Max = 2200.0f;//底盘左右平移限速
	
int16_t  timeXFron = 0, //W
				 timeXBack = 0, //S
				 timeYLeft = 0, //A
				 timeYRigh = 0; //D
	
uint16_t timeInc_Saltation,
				 timeInc,
				 timeDec;



static void Chassis_Init(ChassisCtrl_t* chassisInit);
static void Ctrl_Mode_Set(ChassisCtrl_t* ctrlMode);
static void Chassis_State_Set(ChassisCtrl_t* stateSet);
static void Control_Refresh(ChassisCtrl_t* ctrlRefresh);
static void Feedback_Refresh(ChassisCtrl_t* FdbRefresh);
static void Control_Buff_Refresh(ChassisCtrl_t* buffRefresh);
static void Mecanum_Calc(ChassisCtrl_t* chassisMec);
static void Key_WASD_Slope(void);
static void Give_Current(ChassisCtrl_t* giveCurrent);
static void CanSend(ChassisCtrl_t* send);
/**
  * @brief  底盘任务
  * @param  void
  * @retval void
  * @attention  
  */
void Chassis_task(void)
{
	static portTickType TickCount; 
	TickCount = xTaskGetTickCount();
	
	Chassis_Init(&ChassisData);
	while(1)
	{
		Ctrl_Mode_Set(&ChassisData);
		Chassis_State_Set(&ChassisData);
		Control_Refresh(&ChassisData);
		Feedback_Refresh(&ChassisData);
		Mecanum_Calc(&ChassisData);
		Give_Current(&ChassisData);
		CanSend(&ChassisData);
		Control_Buff_Refresh(&ChassisData);
		
//		vTaskDelayUntil(&TickCount, 5);
		vTaskDelay(2);
	}
}

/**
  * @brief  底盘初始化
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Chassis_Init(ChassisCtrl_t* chassisInit)
{
	uint8_t i = 0;
	if (chassisInit == NULL)
    {
        return;
    }
  chassisInit->gimbalData = get_Gimbal_Point();
	chassisInit->chassis_rc_ctrl = get_remote_control_point();
	chassisInit->Vx = 0;
	chassisInit->Vy = 0;
	chassisInit->Vw = 0;
		

	for(i = 0; i < 4; i++)
  {			
		chassisInit->ChassisMotor[i].ChassisMotorMeasure = get_Chassis_Measure_Point(i);
		chassisInit->ChassisMotor[i].SpeedRef = 0.0f;
		chassisInit->ChassisMotor[i].SpeedFdb = chassisInit->ChassisMotor[i].ChassisMotorMeasure->speed_rpm;
		chassisInit->ChassisMotor[i].GiveCurrent = 0.0f;
	}
	
	/***************************底盘PID初始化************************************/
	for(i = 0; i < 4; i++)
		{			
			New_PID_Init(&chassisInit->ChassisMotor[i].SpeedPid, 
									 NEW_PID_POSITION,
									 Chassis_SpeedPid, 
									 CHASSIS_SPEED_PID_MAX_OUT, 
									 CHASSIS_SPEED_PID_MAX_IOUT,
									 CHASSIS_SPEED_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 CHASSIS_SPEED_PID_I_SEPARATION);
		}
		
//		
//			    New_PID_Init(&chassisInit->FollowPid, 
//	    	           NEW_PID_POSITION, 
//	    	           followPid, 
//	    	           CHASSIS_FOLLOW_PID_MAX_OUT, 
//	    	           CHASSIS_FOLLOW_PID_MAX_IOUT,
//	    	           CHASSIS_FOLLOW_PID_DEAD_ZONE,
//	    	           0.0f,
//	    	           0.0f,
//	    	           0.0f,
//	    	           CHASSIS_FOLLOW_PID_I_SEPARATION);
#if ISGYRO
	    New_PID_Init(&chassisInit->FollowPid, 
	    	           NEW_PID_POSITION, 
	    	           followPid, 
	    	           CHASSIS_FOLLOW_PID_MAX_OUT, 
	    	           CHASSIS_FOLLOW_PID_MAX_IOUT,
	    	           CHASSIS_FOLLOW_PID_DEAD_ZONE,
	    	           0.0f,
	    	           0.0f,
	    	           0.0f,
	    	           CHASSIS_FOLLOW_PID_I_SEPARATION);
#endif
		
	chassisInit->SpeedMode = SPEED_NORMAL;
	chassisInit->CtrlMode = NO_CONTROL;
	
}



/**
  * @brief  控制模式设置
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Ctrl_Mode_Set(ChassisCtrl_t* ctrlMode)
{
	if (ctrlMode == NULL)
	{
			return;
	}
	if(switch_is_down(ctrlMode->chassis_rc_ctrl->rc.s[ModeChannel_R]))//右下，无控制模式
	{
		ctrlMode->CtrlMode = NO_CONTROL;
	}
	else if(switch_is_mid(ctrlMode->chassis_rc_ctrl->rc.s[ModeChannel_R]))//右中，遥控器模式
	{
		ctrlMode->CtrlMode = RC_CONTROL;
	}
	else if(switch_is_up(ctrlMode->chassis_rc_ctrl->rc.s[ModeChannel_R]))//右上，PC模式
	{
		ctrlMode->CtrlMode = PC_CONTROL;
	}
	else
	{
		;
	}
}


/**
  * @brief  底盘状态设置
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Chassis_State_Set(ChassisCtrl_t* stateSet)
{
	if(stateSet == NULL)
	{
		return ;
	}
	switch(stateSet->CtrlMode)
	{
		case NO_CONTROL:
			{
				stateSet->ChassisState = CHASSIS_FREE;
				break;
			}			
		case RC_CONTROL:
		case PC_CONTROL:
			{
#if ISGYRO
				static u8 Separate_Flag = 0;
        stateSet->ChassisState = CHASSIS_FOLLOW;	
				if(PC_TRIGGER_SEPARATE && !Separate_Flag)
				{
					Separate_Flag = 1;
				}
				if(!PC_TRIGGER_SEPARATE && Separate_Flag)
				{
					stateSet->ChassisState = CHASSIS_SEPARATE;
					Separate_Flag = 0;
				}	
#else
       stateSet->ChassisState = CHASSIS_SEPARATE;		 
#endif			
				if(stateSet->gimbalData->ISTurn_Flag)
				{
					stateSet->ChassisDrive = DRIVE_TURNNING;
				}	
        else
        {
          if(stateSet->gimbalData->DriveMode == NORMAL)
					{
						stateSet->ChassisDrive = DRIVE_NORMAL;
					}
					if(stateSet->gimbalData->DriveMode == TURN_BACK)
					{
						stateSet->ChassisDrive = DRIVE_FLIP;
					}
        }			

				
				if(IF_KEY_PRESSED_CTRL)
				{
					stateSet->SpeedMode = SPEED_SLOW;
				}
				else if(IF_KEY_PRESSED_SHIFT)
				{
					stateSet->SpeedMode = SPEED_FAST;
				}
				else
				{
					stateSet->SpeedMode = SPEED_NORMAL;
				}
				break;
		  }		
		default:
			break;
	}
	
}



/**
  * @brief  控制量更新
  * @param  底盘数据结构体指针
  * @retval void
  */
float angle=0; 
static void Control_Refresh(ChassisCtrl_t* ctrlRefresh)
{
	if(ctrlRefresh == NULL)
	{
		return ;
	}
	
	float k_rc_w = 1;//根据w速度调节前后左右平移移速比
	float Chassis_Standard_Move_Max = STANDARD_MAX_NORMAL;//底盘前后左右平移限速
	float Chassis_Revolve_Move_Max   = Omni_Speed_Max;//左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动
		 
	
	Key_WASD_Slope();
	
	if(fabs(ctrlRefresh->Vw) > 80)//扭头速度越快,前后速度越慢,防止转弯半径过大
		{
			k_rc_w = ( (Chassis_Revolve_Move_Max - fabs(ctrlRefresh->Vw) + 80) * (Chassis_Revolve_Move_Max - fabs(ctrlRefresh->Vw) + 80) )
					/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
		
			k_rc_w = Constrain_float(k_rc_w,0,1);
		}
	else
		{
			k_rc_w = 1;
		}
	
	switch(ctrlRefresh->ChassisState)
	{
		case CHASSIS_FREE:
			{
				ctrlRefresh->Vw = 0.0f;
				ctrlRefresh->Vx = 0.0f;
				ctrlRefresh->Vy = 0.0f;
				break;
			}
		case CHASSIS_FOLLOW:
			{
				if(ctrlRefresh->CtrlMode == RC_CONTROL)
				{
					ctrlRefresh->Vw = New_PID_Calc(&ctrlRefresh->FollowPid, ctrlRefresh->gimbalData->yawMotor.RelativeAngle, 0.0f);
					ctrlRefresh->Vx = fp32_deadline(RC_Control_CH3, -10, 10) * RC_VX_RESPONSE;
					ctrlRefresh->Vy = fp32_deadline(RC_Control_CH2, -10, 10) * RC_VY_RESPONSE;
				}
				else
				{
					if(!IF_MOUSE_PRESSED_LEFT && !IF_MOUSE_PRESSED_RIGH)
					{
						ctrlRefresh->Vw = New_PID_Calc(&ctrlRefresh->FollowPid, ctrlRefresh->gimbalData->yawMotor.RelativeAngle, 0.0f);
					}
					else
					{
						ctrlRefresh->Vw = 0.0f;
					}
						switch(ctrlRefresh->ChassisDrive)
						{
							case DRIVE_NORMAL:
							{
								ctrlRefresh->Vx = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron);
								ctrlRefresh->Vy = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ);
								ctrlRefresh->Vw = ctrlRefresh->chassis_rc_ctrl->mouse.x * PC_VW_RESPONSE;
							}
							case DRIVE_TURNNING:
							{	
								ctrlRefresh->Vw = 0.0f;
								ctrlRefresh->Vx = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron);
								ctrlRefresh->Vy = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ);
								break;
							}
							case DRIVE_FLIP:
							{
								ctrlRefresh->Vx = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron);
								ctrlRefresh->Vy = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ);
								break;
							}
							default:
								break;
						}
				}
				break;
			}
		case CHASSIS_SEPARATE:
			{
				if(ctrlRefresh->CtrlMode == RC_CONTROL)
				{
					ctrlRefresh->Vw = fp32_deadline(RC_Control_CH0, -10, 10) * RC_VW_RESPONSE;
					switch(ctrlRefresh->ChassisDrive)
					{
						case DRIVE_NORMAL:
						case DRIVE_TURNNING:
						{	
					    ctrlRefresh->Vx = fp32_deadline(RC_Control_CH3, -10, 10) * RC_VX_RESPONSE;
					    ctrlRefresh->Vy = fp32_deadline(RC_Control_CH2, -10, 10) * RC_VY_RESPONSE;
							break;
						}
						case DRIVE_FLIP:
						{
					    ctrlRefresh->Vx = -fp32_deadline(RC_Control_CH3, -10, 10) * RC_VX_RESPONSE;
					    ctrlRefresh->Vy = -fp32_deadline(RC_Control_CH2, -10, 10) * RC_VY_RESPONSE;
							break;
						}
						default:
							break;
					}
					
				}
				else
				{
					if(!ctrlRefresh->chassis_rc_ctrl->mouse.press_l && !ctrlRefresh->chassis_rc_ctrl->mouse.press_r)
					{
      				ctrlRefresh->Vw = ctrlRefresh->chassis_rc_ctrl->mouse.x * PC_YAW_RESPONSE;
					}

					switch(ctrlRefresh->ChassisDrive)
					{
						case DRIVE_NORMAL:
						case DRIVE_TURNNING:
						{	
							ctrlRefresh->Vx = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_w;
							ctrlRefresh->Vy = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_w;
							break;
						}
						case DRIVE_FLIP:
						{
							ctrlRefresh->Vx = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_w;
							ctrlRefresh->Vy = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_w;
							break;
						}
						default:
							break;
					}
				}
				break;
			}
		default:
				break;
	}
				
}


/**
  * @brief  反馈量更新
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Feedback_Refresh(ChassisCtrl_t* FdbRefresh)
{
	if(FdbRefresh == NULL)
	{
		return ;
	}
	
	uint16_t i = 0;
	for(i = 0; i < 4; i++)
	{
		FdbRefresh->ChassisMotor[i].SpeedFdb = FdbRefresh->ChassisMotor[i].ChassisMotorMeasure->speed_rpm;
	}
		
}



/**
  * @brief  历史数据刷新
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Control_Buff_Refresh(ChassisCtrl_t* buffRefresh)
{
	if(buffRefresh == NULL)
	{
		return ;
	}
	
	buffRefresh->LastChassisState = buffRefresh->ChassisState;
	buffRefresh->LastCtrlMode = buffRefresh->CtrlMode;
	buffRefresh->LastSpeedMode = buffRefresh->SpeedMode;
	buffRefresh->LastChassisDrive = buffRefresh->ChassisDrive;
}

/**
  * @brief  底盘移动按键斜坡量计算
  * @param  void
  * @retval void
  */
static void Key_WASD_Slope(void)
{
	static portTickType  ulCurrentTime = 0;
	static uint32_t  ulDelay = 0;
	
	ulCurrentTime = xTaskGetTickCount();
	
	timeInc_Saltation = TIME_INC_SALTATION;
	timeInc = TIME_INC_NORMAL;
	timeDec = TIME_DEC_NORMAL;
	
	if(ulCurrentTime >= ulDelay)
		{
			ulDelay = ulCurrentTime + KEY_RAMP_DELAY_INC;
			if(IF_KEY_PRESSED_W)
				{
					timeXBack = 0;//按下前进,后退斜坡归零
				}
			if(IF_KEY_PRESSED_S)
				{
					timeXFron = 0;
				}
			if(IF_KEY_PRESSED_A)
				{
					timeYRigh = 0;
				}
			if(IF_KEY_PRESSED_D)
				{
					timeYLeft = 0;
				}
			//计算前后斜坡量
			Slope_Chassis_Move_Fron = (int16_t)(Chassis_WS_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc_Saltation, timeDec ));

			Slope_Chassis_Move_Back = (int16_t)(-Chassis_WS_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc_Saltation, timeDec ));

			//左右的增加斜坡跟前后不一样,别搞错
			Slope_Chassis_Move_Left = (int16_t)(-Chassis_AD_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYLeft, timeInc/1, timeDec ));

			Slope_Chassis_Move_Righ = (int16_t)(Chassis_AD_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYRigh, timeInc/1, timeDec ));
		}
				
}    


/**
  * @brief  麦克纳姆轮全向运动
  * @param  
  * @retval void
  */
static void Mecanum_Calc(ChassisCtrl_t* chassisMec)
{
	if(chassisMec == NULL)
	{
		return;
	}
		static uint8_t i = 0;
	  static fp32 wheel_rpm[4] = {0.0f};
    static fp32 rotate_ratio_fr;//前右
    static fp32 rotate_ratio_fl;//前左
    static fp32 rotate_ratio_bl;//后左
    static fp32 rotate_ratio_br;//后右
		
    static fp32 wheel_rpm_ratio;
		static fp32 rotate_x_offset;
		static fp32 rotate_y_offset;
		
		static fp32 vx, 
							  vy, 
							  vw;
		
		vx = chassisMec->Vx;
		vy = chassisMec->Vy;
		vw = chassisMec->Vw;
		
#if ISGIMBALCENTER 
		rotate_x_offset = 0.0f;
		rotate_y_offset = 0.0f;
#else
		rotate_x_offset = GIMBAL_X_OFFSET;
		rotate_y_offset = GIMBAL_Y_OFFSET;
#endif
		
    wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO); //25.157
    rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - rotate_x_offset + rotate_y_offset) / RADIAN_COEF; //6.63
		rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - rotate_x_offset - rotate_y_offset) / RADIAN_COEF;
		rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + rotate_x_offset - rotate_y_offset) / RADIAN_COEF;
		rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + rotate_x_offset + rotate_y_offset) / RADIAN_COEF;
		
    wheel_rpm[RIGH_FRON] = (-vx + vy + vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[LEFT_FRON] = (+vx + vy + vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[LEFT_BACK] = (+vx - vy + vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[RIGH_BACK] = (-vx - vy + vw * rotate_ratio_br) * wheel_rpm_ratio;
		
		if(chassisMec->SpeedMode == SPEED_FAST)
		{
			for(i = 0; i < 4; i++)
			{
				wheel_rpm[i] = Constrain_float(wheel_rpm[i], -CHASSIS_FAST_SPEED_MAX, CHASSIS_FAST_SPEED_MAX);
				memcpy(&chassisMec->ChassisMotor[i].SpeedRef, &wheel_rpm[i], sizeof(float));
			}
		}
		else if(chassisMec->SpeedMode == SPEED_SLOW)
		{
			for(i = 0; i < 4; i++)
			{
				wheel_rpm[i] = Constrain_float(wheel_rpm[i], -CHASSIS_SLOW_SPEED_MAX, CHASSIS_SLOW_SPEED_MAX);
				memcpy(&chassisMec->ChassisMotor[i].SpeedRef, &wheel_rpm[i], sizeof(float));
			}
		}
		else
		{
			for(i = 0; i < 4; i++)
			{
				wheel_rpm[i] = Constrain_float(wheel_rpm[i], -CHASSIS_NORMAL_SPEED_MAX, CHASSIS_NORMAL_SPEED_MAX);
				memcpy(&chassisMec->ChassisMotor[i].SpeedRef, &wheel_rpm[i], sizeof(float));
			}
		}
}

/**
* @brief  得到发送电流值
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Give_Current(ChassisCtrl_t* giveCurrent)
{
	uint8_t i = 0;
	
	if (giveCurrent == NULL)
	{
		 return;
	}
	
/***************************得到底盘电流值************************************/	
	for(i = 0; i < 4; i++)
	{
		New_PID_Calc(&giveCurrent->ChassisMotor[i].SpeedPid,
		             giveCurrent->ChassisMotor[i].SpeedFdb,
		             giveCurrent->ChassisMotor[i].SpeedRef);
	 giveCurrent->ChassisMotor[i].GiveCurrent = giveCurrent->ChassisMotor[i].SpeedPid.out;
	}
	
}


/**
  * @brief  CAN发送
  * @param  数据结构体指针
  * @retval void
  */
static void CanSend(ChassisCtrl_t* send)
{

	if(send->ChassisState == CHASSIS_FREE)
	{
		CAN1_CMD_CHASSIS(0, 0, 0, 0);
	}
	else
	{
		CAN1_CMD_CHASSIS(send->ChassisMotor[0].GiveCurrent,  \
		                 send->ChassisMotor[1].GiveCurrent,  \
		                 send->ChassisMotor[2].GiveCurrent,  \
		                 send->ChassisMotor[3].GiveCurrent);
				
//		CAN1_CMD_CHASSIS(0, 0, 0, 0);
	}
//	CAN1_CMD_GIMBAL(send->gimbalData->yawMotor.Current, send->gimbalData->pitchMotor.Current);
	
}

/**
  * @brief  返回底盘结构体指针
  * @param  void
  * @retval 
  * @attention  
  */
const ChassisCtrl_t* get_Chassis_Point(void)
{
  return &ChassisData;
}

