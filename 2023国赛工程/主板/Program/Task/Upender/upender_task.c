#include "upender_task.h"
#include "gpio.h"
/*==============================================================*/
#define UPENDER_TASK_PRIO 29
#define UPENDER_STK_SIZE 512
TaskHandle_t UpenderTask_Handler;
void upender_task(void);

/*==============================================================*/

void task_Upender_Create(void)
{
	xTaskCreate((TaskFunction_t)upender_task,
                (const char *)"upender_task",
                (uint16_t)UPENDER_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)UPENDER_TASK_PRIO,
                (TaskHandle_t *)&UpenderTask_Handler);
}

UpenderCtrl_t UpenderData;


fp32 upenderSpeedPid[3] = {UPENDER_SPEED_PID_KP, UPENDER_SPEED_PID_KI, UPENDER_SPEED_PID_KD};

static void Upender_Init(UpenderCtrl_t* init);
static void Ctrl_Mode_Set(UpenderCtrl_t* ctrlMode);
static void Control_Ref(UpenderCtrl_t* ctrlRef);
static void FeedBack_Refresh(UpenderCtrl_t* feedbackRefresh);
static void Control_Buff_Refresh(UpenderCtrl_t* buffRefresh);
static void GiveCurrent(UpenderCtrl_t* giveCurrent);
static void canSend(UpenderCtrl_t* send);

void upender_task(void)
{
	static portTickType TickCount; 
	
	TickCount = xTaskGetTickCount();
	
	Upender_Init(&UpenderData);
	while(1)
	{
    Ctrl_Mode_Set(&UpenderData);
		FeedBack_Refresh(&UpenderData);
		Control_Ref(&UpenderData);
		GiveCurrent(&UpenderData);
		canSend(&UpenderData);
		Control_Buff_Refresh(&UpenderData);
		
		vTaskDelayUntil(&TickCount, 2);
	}
}


/**
  * @brief  矿石翻转机构初始化
  * @param  矿石翻转机构数据结构体指针
  * @retval void
  */
static void Upender_Init(UpenderCtrl_t* init)
{
	if(init == NULL)
	{
		return;
	}
  uint8_t i = 0;
	
/***************************获取数据指针************************************/
	
	init->upender_rc_ctrl = get_remote_control_point();
	for(i = 0; i < 2; i++)
	{
	  init->upenderMotor[i].UpenderEncoderMeasure = get_Upender_EncoderProcess_Point(i);
		init->upenderMotor[i].SpeedFdb = init->upenderMotor[i].UpenderEncoderMeasure->filter_rate;
		init->upenderMotor[i].SpeedRef = 0.0f;
	}
/***************************翻转矿石PID初始化************************************/
	for(i = 0; i < 2; i++)
		{			
			New_PID_Init(&init->upenderMotor[i].SpeedPid, 
											 NEW_PID_POSITION,
											 upenderSpeedPid, 
											 UPENDER_SPEED_PID_MAX_OUT, 
											 UPENDER_SPEED_PID_MAX_IOUT,
											 UPENDER_SPEED_PID_DEAD_ZONE,
											 0.0f,
											 0.0f,
											 0.0f,
											 UPENDER_SPEED_PID_I_SEPARATION);
		}
		
	init->CtrlMode = NO_CTRL_UPENDER;
	
}

/**
  * @brief  控制模式设置
  * @param  矿石翻转机构数据结构体指针
  * @retval void
  */
static void Ctrl_Mode_Set(UpenderCtrl_t* ctrlMode)
{
	if (ctrlMode == NULL)
	{
			return;
	}
	if(switch_is_down(ctrlMode->upender_rc_ctrl->rc.s[ModeChannel_R]))//右下，无控制模式
	{
		ctrlMode->CtrlMode = NO_CTRL_UPENDER;
	}
	else if(switch_is_mid(ctrlMode->upender_rc_ctrl->rc.s[ModeChannel_R]))//右中，遥控器模式
	{
		ctrlMode->CtrlMode = RC_CTRL_UPENDER;
	}
	else if(switch_is_up(ctrlMode->upender_rc_ctrl->rc.s[ModeChannel_R]))//右上，PC模式
	{
		ctrlMode->CtrlMode = PC_CTRL_UPENDER;
	}
	else
	{
		;
	}
}


/**
  * @brief  控制量更新
  * @param  矿石翻转机构数据结构体指针
  * @retval void
  */
static void Control_Ref(UpenderCtrl_t* ctrlRef)
{
	if(ctrlRef == NULL)
	{
		return ;
	}
	
	static uint8_t i = 0;
	
	switch(ctrlRef->CtrlMode)
	{
		case NO_CTRL_UPENDER: 
			{
				VACUUM_OFF;	
        PRESSURE_OFF;	
				CAN2_CMD_UPENDER(0, 0);
			}
		case RC_CTRL_UPENDER:
				
//				for(i = 0; i < 2; i++)
//				{
//				  ctrlRef->upenderMotor[i].SpeedRef = 0.0f;
//				}
				break;
		case PC_CTRL_UPENDER:
		  {
//				if(IF_KEY_PRESSED_F)
//				{
//					vTaskDelay(5);
//					if(IF_KEY_PRESSED_F)
//					{
//						VACUUM_ON;
//						PRESSURE_OFF;
//					}
//				
//				}
//				else if(IF_KEY_PRESSED_G)
//				{
//					vTaskDelay(5);
//					if(IF_KEY_PRESSED_G)
//					{
//						VACUUM_OFF;
//						PRESSURE_ON;
//					}
				if(ctrlRef->upender_rc_ctrl->rc.ch[4] > 500)
				{
						VACUUM_ON;
						PRESSURE_OFF;			
//            ctrlRef->upenderMotor[FRONT_MOTOR].SpeedRef = CTRL_UPENDER_SPEED;					
				}
				else if(ctrlRef->upender_rc_ctrl->rc.ch[4] < -500)
				{
						VACUUM_OFF;
						PRESSURE_ON;	
					  vTaskDelay(1000);
						PRESSURE_OFF;			
//					  ctrlRef->upenderMotor[FRONT_MOTOR].SpeedRef = 0;
				}
/*=========================================矿石前滚======================================*/
				if(IF_KEY_PRESSED_Z && !IF_KEY_PRESSED_CTRL)
				{
					ctrlRef->upenderMotor[FRONT_MOTOR].SpeedRef = -CTRL_UPENDER_SPEED;
					ctrlRef->upenderMotor[BACK_MOTOR].SpeedRef = -CTRL_UPENDER_SPEED;
				}
/*=========================================矿石后滚======================================*/
				else if(IF_KEY_PRESSED_Z && IF_KEY_PRESSED_CTRL)
				{
					ctrlRef->upenderMotor[FRONT_MOTOR].SpeedRef = CTRL_UPENDER_SPEED;
					ctrlRef->upenderMotor[BACK_MOTOR].SpeedRef = CTRL_UPENDER_SPEED;
				}
/*=========================================矿石上移======================================*/
				else if(IF_KEY_PRESSED_X && !IF_KEY_PRESSED_CTRL)
				{
					ctrlRef->upenderMotor[FRONT_MOTOR].SpeedRef = CTRL_UPENDER_SPEED;
					ctrlRef->upenderMotor[BACK_MOTOR].SpeedRef = -CTRL_UPENDER_SPEED;
				}
/*=========================================矿石下移======================================*/
				else if(IF_KEY_PRESSED_X && IF_KEY_PRESSED_CTRL)
				{
					ctrlRef->upenderMotor[FRONT_MOTOR].SpeedRef = -CTRL_UPENDER_SPEED;
					ctrlRef->upenderMotor[BACK_MOTOR].SpeedRef = CTRL_UPENDER_SPEED;
				}
				else
				{
					for(i = 0; i < 2; i++)
					{
						ctrlRef->upenderMotor[i].SpeedRef = 0.0f;
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
  * @param  矿石翻转机构数据结构体指针
  * @retval void
  */
static void FeedBack_Refresh(UpenderCtrl_t* feedbackRefresh)
{
	if(feedbackRefresh == NULL)
	{
		return ;
	}
	
	static uint8_t i = 0;
	
	switch(feedbackRefresh->CtrlMode)
	{
		case NO_CTRL_UPENDER:
		case RC_CTRL_UPENDER:
		case PC_CTRL_UPENDER:
			{
				for(i = 0; i < 2; i++)
				{
				  feedbackRefresh->upenderMotor[i].SpeedFdb = feedbackRefresh->upenderMotor[i].UpenderEncoderMeasure->filter_rate;
				}
				break;
			}
		default:
			break;
	}
}

/**
  * @brief  历史数据刷新
  * @param  矿石翻转机构数据结构体指针
  * @retval void
  */
static void Control_Buff_Refresh(UpenderCtrl_t* buffRefresh)
{
	if(buffRefresh == NULL)
	{
		return ;
	}
	buffRefresh->LastCtrlMode = buffRefresh->CtrlMode;
}


/**
* @brief  得到发送电流值
  * @param  底盘数据结构体指针
  * @retval void
  */
static void GiveCurrent(UpenderCtrl_t* giveCurrent)
{
	uint8_t i = 0;
	
	if (giveCurrent == NULL)
	{
		 return;
	}

/***************************得到翻矿机构电流值************************************/	
	for(i = 0; i < 2; i++)
	{
   New_PID_Calc(&giveCurrent->upenderMotor[i].SpeedPid,
		             giveCurrent->upenderMotor[i].SpeedFdb,
		             giveCurrent->upenderMotor[i].SpeedRef);
	 giveCurrent->upenderMotor[i].Current = giveCurrent->upenderMotor[i].SpeedPid.out;
	}

}


/**
  * @brief  CAN发送
  * @param  数据结构体指针
  * @retval void
  */
static void canSend(UpenderCtrl_t* send)
{
	CAN2_CMD_UPENDER(send->upenderMotor[0].Current, send->upenderMotor[1].Current);
//	CAN2_CMD_UPENDER(0, 0);
}


/**
  * @brief  返回矿石翻转机构结构体指针
  * @param  void
  * @retval 
  * @attention  
  */
const UpenderCtrl_t* get_Upender_Point(void)
{
  return &UpenderData;
}

