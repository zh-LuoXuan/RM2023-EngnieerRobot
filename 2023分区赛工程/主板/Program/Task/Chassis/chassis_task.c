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

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
float Chassis_WS_Move_Max = 4400.0f;//����ǰ��ƽ������
float Chassis_AD_Move_Max = 1100.0f;//��������ƽ������
	
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
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention  
  */
void Chassis_task(void)
{
	static portTickType TickCount; 
	TickCount = xTaskGetTickCount();
	int key;
	key = TOPMOVE_KEY;
	
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
		
		vTaskDelay(2);
//		vTaskDelayUntil(&TickCount, 2);
	}
}

/**
  * @brief  ���̳�ʼ��
  * @param  �������ݽṹ��ָ��
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
		
	for(i = 0; i < 4; i++)
  {			
		chassisInit->ChassisMotor[i].ChassisMotorMeasure = get_Chassis_Measure_Point(i);
		chassisInit->ChassisMotor[i].SpeedRef = 0.0f;
		chassisInit->ChassisMotor[i].SpeedFdb = chassisInit->ChassisMotor[i].ChassisMotorMeasure->speed_rpm;
		chassisInit->ChassisMotor[i].GiveCurrent = 0.0f;
	}
	
	/***************************����PID��ʼ��************************************/
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
					New_PID_Init(&chassisInit->FOLLOWPid, 
											 NEW_PID_POSITION,
											 followPid, 
											 CHASSIS_SPEED_PID_MAX_OUT, 
											 CHASSIS_SPEED_PID_MAX_IOUT,
											 CHASSIS_SPEED_PID_DEAD_ZONE,
											 0.0f,
											 0.0f,
											 0.0f,
											 CHASSIS_SPEED_PID_I_SEPARATION);
	chassisInit->SpeedMode = SPEED_NORMAL;
	chassisInit->CtrlMode = NO_CONTROL;
	
}



/**
  * @brief  ����ģʽ����
  * @param  �������ݽṹ��ָ��
  * @retval void
  */
static void Ctrl_Mode_Set(ChassisCtrl_t* ctrlMode)
{
	if (ctrlMode == NULL)
	{
			return;
	}
	if(switch_is_down(ctrlMode->chassis_rc_ctrl->rc.s[ModeChannel_R]))//���£��޿���ģʽ
	{
		ctrlMode->CtrlMode = NO_CONTROL;
	}
	else if(switch_is_mid(ctrlMode->chassis_rc_ctrl->rc.s[ModeChannel_R]))//���У�ң����ģʽ
	{
		ctrlMode->CtrlMode = RC_CONTROL;
	}
	else if(switch_is_up(ctrlMode->chassis_rc_ctrl->rc.s[ModeChannel_R]))//���ϣ�PCģʽ
	{
		ctrlMode->CtrlMode = PC_CONTROL;
	}
	else
	{
		;
	}
}


/**
  * @brief  ����״̬����
  * @param  �������ݽṹ��ָ��
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
				static uint16_t G_Flag = 1;
        stateSet->ChassisState = CHASSIS_FOLLOW;	
				if(IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_G && G_Flag)
				{
					G_Flag = 0;
				}
				if(!IF_KEY_PRESSED_CTRL && !IF_KEY_PRESSED_G && !G_Flag)
				{
					stateSet->ChassisState = CHASSIS_SEPARATE;
					G_Flag = 1;
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
  * @brief  ����������
  * @param  �������ݽṹ��ָ��
  * @retval void
  */
float angle=0; 
static void Control_Refresh(ChassisCtrl_t* ctrlRefresh)
{
	if(ctrlRefresh == NULL)
	{
		return ;
	}
	
	Key_WASD_Slope();
	
	switch(ctrlRefresh->ChassisState)
	{
		case CHASSIS_FREE:
		case CHASSIS_LOCK:
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
						angle=RAMP_float(ctrlRefresh->chassis_rc_ctrl->mouse.x * 2.0f,angle,1.0f);
						ctrlRefresh->Vw = New_PID_Calc(&ctrlRefresh->FOLLOWPid, 0.0f,angle );

					switch(ctrlRefresh->ChassisDrive)
					{
						case DRIVE_NORMAL:
						case DRIVE_TURNNING:
						{	
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
		default:
				break;
	}
				
}


/**
  * @brief  ����������
  * @param  �������ݽṹ��ָ��
  * @retval void
  */
static void Feedback_Refresh(ChassisCtrl_t* FdbRefresh)
{
	if(FdbRefresh == NULL)
	{
		return ;
	}
	
	uint16_t i = 0;
	
	switch(FdbRefresh->ChassisState)
	{
		case CHASSIS_FREE:
		case CHASSIS_LOCK:
		case CHASSIS_FOLLOW:
		case CHASSIS_SEPARATE:
			{
				for(i = 0; i < 4; i++)
				{
				  FdbRefresh->ChassisMotor[i].SpeedFdb = FdbRefresh->ChassisMotor[i].ChassisMotorMeasure->speed_rpm;
				}
				break;
			}
		default:
				break;
	}
				
}



/**
  * @brief  ��ʷ����ˢ��
  * @param  �������ݽṹ��ָ��
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
  * @brief  �����ƶ�����б��������
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
					timeXBack = 0;//����ǰ��,����б�¹���
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
			//����ǰ��б����
			Slope_Chassis_Move_Fron = (int16_t)(Chassis_WS_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc_Saltation, timeDec ));

			Slope_Chassis_Move_Back = (int16_t)(-Chassis_WS_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc_Saltation, timeDec ));

			//���ҵ�����б�¸�ǰ��һ��,����
			Slope_Chassis_Move_Left = (int16_t)(-Chassis_AD_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYLeft, timeInc/1, timeDec ));

			Slope_Chassis_Move_Righ = (int16_t)(Chassis_AD_Move_Max * Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYRigh, timeInc/1, timeDec ));
		}
				
}    


/**
  * @brief  �����ķ��ȫ���˶�
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
    static fp32 rotate_ratio_fr;//ǰ��
    static fp32 rotate_ratio_fl;//ǰ��
    static fp32 rotate_ratio_bl;//����
    static fp32 rotate_ratio_br;//����
		
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
				wheel_rpm[i] = Constrain_int32_t(wheel_rpm[i], -CHASSIS_FAST_SPEED_MAX, CHASSIS_FAST_SPEED_MAX);
				memcpy(&chassisMec->ChassisMotor[i].SpeedRef, &wheel_rpm[i], sizeof(float));
			}
		}
		else if(chassisMec->SpeedMode == SPEED_SLOW)
		{
			for(i = 0; i < 4; i++)
			{
				wheel_rpm[i] = Constrain_int32_t(wheel_rpm[i], -CHASSIS_SLOW_SPEED_MAX, CHASSIS_SLOW_SPEED_MAX);
				memcpy(&chassisMec->ChassisMotor[i].SpeedRef, &wheel_rpm[i], sizeof(float));
			}
		}
		else
		{
			for(i = 0; i < 4; i++)
			{
				wheel_rpm[i] = Constrain_int32_t(wheel_rpm[i], -CHASSIS_NORMAL_SPEED_MAX, CHASSIS_NORMAL_SPEED_MAX);
				memcpy(&chassisMec->ChassisMotor[i].SpeedRef, &wheel_rpm[i], sizeof(float));
			}
		}
}

/**
* @brief  �õ����͵���ֵ
  * @param  �������ݽṹ��ָ��
  * @retval void
  */
static void Give_Current(ChassisCtrl_t* giveCurrent)
{
	uint8_t i = 0;
	
	if (giveCurrent == NULL)
	{
		 return;
	}
	
/***************************�õ����̵���ֵ************************************/	
	for(i = 0; i < 4; i++)
	{
//	 giveCurrent->ChassisMotor[i].SpeedSet = RAMP_float(giveCurrent->ChassisMotor[i].SpeedRef,giveCurrent->ChassisMotor[i].SpeedFdb,8);
//   New_PID_Calc(&giveCurrent->ChassisMotor[i].SpeedPid,
//		             giveCurrent->ChassisMotor[i].SpeedFdb,
//		             giveCurrent->ChassisMotor[i].SpeedSet);
		New_PID_Calc(&giveCurrent->ChassisMotor[i].SpeedPid,
		             giveCurrent->ChassisMotor[i].SpeedFdb,
		             giveCurrent->ChassisMotor[i].SpeedRef);
	 giveCurrent->ChassisMotor[i].GiveCurrent = giveCurrent->ChassisMotor[i].SpeedPid.out;
	}
	
}


/**
  * @brief  CAN����
  * @param  ���ݽṹ��ָ��
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
	}
	
}

/**
  * @brief  ���ص��̽ṹ��ָ��
  * @param  void
  * @retval 
  * @attention  
  */
const ChassisCtrl_t* get_Chassis_Point(void)
{
  return &ChassisData;
}

