#include "control_task.h"
#include "FreeRTOSConfig.h"
#include "motor_control_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "RemoteControl.h"
#include "time7.h"
#include "user_lib.h"


/*==============================================================*/
#define CONTROL_TASK_PRIO 30
#define CONTROL_STK_SIZE 1024
TaskHandle_t ControlTask_Handler;
void Control_task(void);

/*==============================================================*/

void task_control_Create(void)
{
	xTaskCreate((TaskFunction_t)Control_task,
                (const char *)"Control_task",
                (uint16_t)CONTROL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CONTROL_TASK_PRIO,
                (TaskHandle_t *)&ControlTask_Handler);
}
/*==============================================================*/


ControlTask_t ctrlTask;
int UI_Flag = 0;

fp32 L_Up_Appoint[UP_COUNT];//左抬升点位
fp32 R_Up_Appoint[UP_COUNT];//右抬升点位
fp32 SucPitch_Appoint[SUC_PITCH_COUNT];//吸盘PITCH轴点位
fp32 SucRoll_Appoint[SUC_ROLL_COUNT];//吸盘ROLL轴点位


fp32 L_Up_Motor_Longs[UP_COUNT] = {L_UP_LONGS_HIGHTST, L_Up_LONGS_LOWEST};
fp32 R_Up_Motor_Longs[UP_COUNT] = {R_UP_LONGS_HIGHTST, R_Up_LONGS_LOWEST};
fp32 SucPitch_Motor_Longs[SUC_PITCH_COUNT] = {SUC_PITCH_LONGS_FRONT, SUC_PITCH_LONGS_DOWN, SUC_PITCH_LONGS_BACK};
fp32 SucRoll_Motor_Longs[SUC_ROLL_COUNT] = {SUC_ROLL_LONGS_RES, SUC_ROLL_LONGS_READY};

static void Ctrl_Init(ControlTask_t* init);
static void CtrlMode_Set(ControlTask_t* ModeSet);
static void feedBack_Set(ControlTask_t* FdbSet);
static void WorkState_Set(ControlTask_t* stateSet);
static void Ctrl_Buff_Refresh(ControlTask_t* buffRefresh);
static void Get_Motor_Appoint(ControlTask_t* getAppiont);
static void Motor_Reset(ControlTask_t* motorRes);
static void Get_Ore_Gold_Fir(ControlTask_t* getGold);
static void Get_Ore_Gold_Sec(ControlTask_t* getGold);
static void Get_Ore_Silver_Fir(ControlTask_t* getSilver);
static void Get_Ore_Silver_Sec(ControlTask_t* getSilver);
static void Catch_Ore(ControlTask_t* CatchGold);
static void Exchange_Ore(ControlTask_t* exchangeOre);
static void Fine_Tuning(ControlTask_t* fineTuning);
void SucRoll_Reset(ControlTask_t* sucRollTurn);
void SucPitch_Reset(ControlTask_t* sucPitchTurn);

void Control_task(void)
{
	static portTickType TickCount; 
	TickCount = xTaskGetTickCount();
	Ctrl_Init(&ctrlTask);
	while(1)
	{
		CtrlMode_Set(&ctrlTask);
		WorkState_Set(&ctrlTask);
		feedBack_Set(&ctrlTask);
		Ctrl_Buff_Refresh(&ctrlTask);
		PID_Output(&ctrlTask.motorCtrl);
		vTaskDelay(1);
	}
}


/**
  * @brief  结构体初始化
  * @param  数据结构体指针
  * @retval void
  * @attention 
  */
static void Ctrl_Init(ControlTask_t* init)
{
	if(init == NULL)
	{
		return;
	}
	init->Input_Point = get_remote_control_point();
	init->ctrlMode = CTRL_NULL;
	init->runState = RUN_RESET;
	init->workStste = DO_NOTHING;
//	init->UI_Flag = 0;
	motorInit(&init->motorCtrl);
}

/**
  * @brief  获取电机预设目标位置
  * @param  数据结构体指针
  * @retval void
  * @attention 
  */
static void Get_Motor_Appoint(ControlTask_t* getAppiont)
{
	if(getAppiont == NULL)
	{
		return;
	}
	uint8_t i = 0;

	for(i = 0; i < UP_COUNT; i++)
	{
		L_Up_Appoint[i] = getAppiont->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset + L_Up_Motor_Longs[i];
		R_Up_Appoint[i] = getAppiont->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset + R_Up_Motor_Longs[i];
	}
	for(i = 0; i < SUC_PITCH_COUNT; i++)
	{
		SucPitch_Appoint[i] = getAppiont->motorCtrl.SucPitchMotor.CenterOffset + SucPitch_Motor_Longs[i];
	}
	for(i = 0; i < SUC_ROLL_COUNT; i++)
	{
		SucRoll_Appoint[i] = getAppiont->motorCtrl.SucRollMotor.CenterOffset + SucRoll_Motor_Longs[i];
	}
}


/**
  * @brief  所有电机复位
  * @param  数据结构体指针
  * @retval void
  * @attention 
  */
static void Motor_Reset(ControlTask_t* motorRes)
{
	if(motorRes == NULL)
	{
		return;
	}
	u8 i = 0;
	static u8 Unres_Flag = OFF;
	IMPLICATION(OFF);
	if(IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_X && !Unres_Flag)
	{
		Unres_Flag = ON;
	}
	if(!IF_KEY_PRESSED_CTRL && !IF_KEY_PRESSED_SHIFT && !IF_KEY_PRESSED_X && Unres_Flag)
	{
		Unres_Flag = OFF;
		motorRes->motorCtrl.UpMotor[LEFT_MOTOR].SpeedRef = 0;
	  motorRes->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset = motorRes->motorCtrl.UpMotor[LEFT_MOTOR].motor_measure->real_ecd;
		motorRes->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef = motorRes->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset;
		
		motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].SpeedRef = 0;
	  motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset = motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].motor_measure->real_ecd;
	  motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef = motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset;
		
	  motorRes->motorCtrl.SucPitchMotor.CenterOffset = motorRes->motorCtrl.SucPitchMotor.motor_measure->real_ecd;
		motorRes->motorCtrl.SucPitchMotor.AngleRef = motorRes->motorCtrl.SucPitchMotor.CenterOffset;
		
	  motorRes->motorCtrl.SucRollMotor.CenterOffset = motorRes->motorCtrl.SucRollMotor.motor_measure->real_ecd;
		motorRes->motorCtrl.SucRollMotor.AngleRef = motorRes->motorCtrl.SucRollMotor.CenterOffset;
		
		Get_Motor_Appoint(motorRes);
		motorRes->motorCtrl.ifSpid = true;
		motorRes->runState = RUN_NORMAL;
	}
	if(motorRes->runState == RUN_RESET)
	{
		if((motorRes->motorCtrl.UpMotor[LEFT_MOTOR].ResFlag) && \
		   (motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].ResFlag) && \
	     (motorRes->motorCtrl.SucPitchMotor.ResFlag) && \
	     (motorRes->motorCtrl.SucRollMotor.ResFlag))
	 	   {
	 	  	motorRes->runState = RUN_TURNING;
	 	   }
			
		 if((motorRes->motorCtrl.UpMotor[LEFT_MOTOR].ResFlag) && \
		   (motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].ResFlag))	 
	   {
	   	 if(!IF_SUCTION_PITCH_KEY)
	       {
	       	motorRes->motorCtrl.SucPitchMotor.AngleRef--;
	       }
	     if(IF_SUCTION_PITCH_KEY)
	       {
	       	motorRes->motorCtrl.SucPitchMotor.AngleRef -= 0;
	       	motorRes->motorCtrl.SucPitchMotor.CenterOffset = motorRes->motorCtrl.SucPitchMotor.motor_measure->real_ecd;
		    	motorRes->motorCtrl.SucPitchMotor.AngleRef = motorRes->motorCtrl.SucPitchMotor.CenterOffset;
	       	motorRes->motorCtrl.SucPitchMotor.ResFlag = true;
	       }
	   }
	 if(!IF_UP_KEY_L)
	   {
	   	motorRes->motorCtrl.UpMotor[LEFT_MOTOR].SpeedRef = 800;
	   }
	 if(IF_UP_KEY_L)
	   {
	   	motorRes->motorCtrl.UpMotor[LEFT_MOTOR].SpeedRef = 0;
	   	motorRes->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset = motorRes->motorCtrl.UpMotor[LEFT_MOTOR].motor_measure->real_ecd;
			motorRes->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef = motorRes->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset;
	   	motorRes->motorCtrl.UpMotor[LEFT_MOTOR].ResFlag = true;
	   }
	 if(!IF_UP_KEY_R)
	   {
	   	motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].SpeedRef = -800;
	   }
	 if(IF_UP_KEY_R)
	   {
	   	motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].SpeedRef = 0;
	   	motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset = motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].motor_measure->real_ecd;	 
			motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef = motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset;
	   	motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].ResFlag = true;
	   }
	 if(!IF_SUCTION_ROLL_KEY)
	   {
	   	motorRes->motorCtrl.SucRollMotor.AngleRef--;
	   }
	 if(IF_SUCTION_ROLL_KEY)
	   {
	   	motorRes->motorCtrl.SucRollMotor.AngleRef -= 0;
	   	motorRes->motorCtrl.SucRollMotor.CenterOffset = motorRes->motorCtrl.SucRollMotor.motor_measure->real_ecd;
			motorRes->motorCtrl.SucRollMotor.AngleRef = motorRes->motorCtrl.SucRollMotor.CenterOffset;
	   	motorRes->motorCtrl.SucRollMotor.ResFlag = true;
	   }
	}
	
	if(motorRes->runState == RUN_TURNING)
	{
		 for(i = 0; i < 2; i++)
		{
			motorRes->motorCtrl.UpMotor[i].Amplitude_Flag = true;
		}
		motorRes->motorCtrl.SucPitchMotor.Amplitude_Flag = true;
		motorRes->motorCtrl.SucRollMotor.Amplitude_Flag = true;
		
		motorRes->motorCtrl.ifSpid = true;
		
	  motorRes->motorCtrl.SucRollMotor.CenterOffset = motorRes->motorCtrl.SucRollMotor.CenterOffset + 15 * REDUCTION_RATIO_2006;
		motorRes->motorCtrl.SucRollMotor.AngleRef = motorRes->motorCtrl.SucRollMotor.CenterOffset;
	 	Get_Motor_Appoint(motorRes);
		motorRes->motorCtrl.SucPitchMotor.AngleRef = SucPitch_Appoint[BACK];
		motorRes->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[LOWEST];
		motorRes->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[LOWEST];
		motorRes->runState = RUN_NORMAL;
	}
	
}



/**
  * @brief  控制模式设置
  * @param  数据结构体指针
  * @retval void
  * @attention 
  */
static void CtrlMode_Set(ControlTask_t* ModeSet)
{
	if(ModeSet == NULL)
	{
		return;
	}
	switch(ModeSet->runState)
	{
	  case RUN_RESET:
		case RUN_TURNING:
		{
			Motor_Reset(ModeSet);
			break;
		}
		case RUN_NORMAL:
		{
			if (rc_ctrl.rc.ch[0] == -660 && rc_ctrl.rc.ch[1] == -660 && rc_ctrl.rc.ch[2] == 660 && rc_ctrl.rc.ch[3] == -660) //拨杆内八软件强制复位
	    {
	    	__set_FAULTMASK(1);//关闭所有中断
	    	NVIC_SystemReset();//软件复位
				ModeSet->runState = RUN_RESET;
	    }
	    if(switch_is_down(ModeSet->Input_Point->rc.s[ModeChannel_R]))//右下，无控制模式
	    {
	    	ModeSet->ctrlMode = CTRL_NULL;
	    }
	    else if(switch_is_mid(ModeSet->Input_Point->rc.s[ModeChannel_R]))//右中，遥控器模式
	    {
	    	ModeSet->ctrlMode = CTRL_RC;
	    }
	    else if(switch_is_up(ModeSet->Input_Point->rc.s[ModeChannel_L]) && switch_is_up(ModeSet->Input_Point->rc.s[ModeChannel_R]))//左上右上，微调模式
	    {
	    	ModeSet->ctrlMode = CTRL_FINETUNING;
	    }
	    else if(switch_is_up(ModeSet->Input_Point->rc.s[ModeChannel_R]))//右上非左上 PC模式
	    {
	    	ModeSet->ctrlMode = CTRL_PC;
	    }
	    else
	    {
	    	;
	    }
			break;
		}
		default:
			break;
 }
}


/**
  * @brief  工作状态设置
  * @param  数据结构体指针
  * @retval void
  */
static void WorkState_Set(ControlTask_t* stateSet)
{
	if(stateSet == NULL)
	{
		return;
	}
	
	switch(stateSet->ctrlMode)
	{
		case CTRL_NULL:
		{
			break;
		}
		case CTRL_RC:
		{
			static u8 rc_gold_Fir = OFF,
				        rc_gold_Sec = OFF,
                rc_catch = OFF,
					      rc_selver_Fir = OFF,
					      rc_selver_Sec = OFF,
					      rc_exchange = OFF;
			//ORE_GOLD
			if((switch_is_up(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] >= CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_GOLD_FIR) && !rc_gold_Fir)
			{
				rc_gold_Fir = ON;
			}
			if((switch_is_up(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] < CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_GOLD_FIR) && rc_gold_Fir)
			{
				rc_gold_Fir = OFF;
				Get_Ore_Gold_Fir(stateSet);
			}
			//ORE_CATCH
			if((switch_is_up(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] <= -CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_CATCH) && !rc_catch)
			{
				rc_catch = ON;
			}
			if((switch_is_up(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] > -CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_CATCH) && rc_catch)
			{
				rc_catch = OFF;
				Catch_Ore(stateSet);
			}
			//ORE_SILVER
			if((switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] >= CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_SILVER_FIR) && !rc_selver_Fir)
			{
				rc_selver_Fir = ON;
			}
			if((switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] < CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_SILVER_FIR) && rc_selver_Fir)
			{
				rc_selver_Fir = OFF;
				Get_Ore_Silver_Fir(stateSet);
			}
			//ORE_EXCHANGE
			if((switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] <= -CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_EXCHANGE) && !rc_exchange)
			{
				rc_exchange = ON;
			}
			if((switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_L]) && stateSet->Input_Point->rc.ch[4] > -CH4_TRIGGER_VAL) && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_EXCHANGE) && rc_exchange)
			{
				rc_exchange = OFF;
				Exchange_Ore(stateSet);
			}
			
			break;
		}
		case CTRL_FINETUNING:
		{
			Fine_Tuning(stateSet);
		}
		case CTRL_PC:
		{
      static u8 pc_gold_Fir = OFF,
					      pc_gold_Sec = OFF,
			          pc_catch = OFF,
					      pc_selver_Fir = OFF,
					      pc_selver_Sec = OFF,
					      pc_exchange = OFF;
			//ORE_GOLD
			if(PC_TRIGGER_GOLD && !IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_GOLD_FIR) && !pc_gold_Fir)
			{
				pc_gold_Fir = ON;
			}
			if(!PC_TRIGGER_GOLD && !IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_GOLD_FIR) && pc_gold_Fir)
			{
				pc_gold_Fir = OFF;
				Get_Ore_Gold_Fir(stateSet);
			}			
			
			//ORE_CATCH
			if(IF_KEY_PRESSED_E && IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_CATCH) && !pc_catch)
			{
				pc_catch = ON;
			}
			if(!IF_KEY_PRESSED_E && !IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_CATCH) && pc_catch)
			{
				pc_catch = OFF;
				Catch_Ore(stateSet);
			}
			//ORE_SILVER
			if(PC_TRIGGER_SILVER && !IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_SILVER_FIR) && !pc_selver_Fir)
			{
				pc_selver_Fir = ON;
			}
			if(!PC_TRIGGER_SILVER && !IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_SILVER_FIR) && pc_selver_Fir)
			{
				pc_selver_Fir = OFF;
				Get_Ore_Silver_Fir(stateSet);
			}
			if(PC_TRIGGER_SILVER && IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_SILVER_SEC) && !pc_selver_Sec)
			{
				pc_selver_Sec = ON;
			}
			if(!PC_TRIGGER_SILVER && !IF_KEY_PRESSED_CTRL && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_SILVER_SEC) && pc_selver_Sec)
			{
				pc_selver_Sec = OFF;
				Get_Ore_Silver_Sec(stateSet);
			}
			
			//ORE_EXCHANGE
			if(PC_TRIGGER_EXCHANGE && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_EXCHANGE) && !pc_exchange)
			{
				pc_exchange = ON;
			}
			if(!PC_TRIGGER_EXCHANGE && (stateSet->workStste == DO_NOTHING || stateSet->workStste == ORE_EXCHANGE) && pc_exchange)
			{
				pc_exchange = OFF;
				Exchange_Ore(stateSet);
			}
			break;
		}
		default:
			break;
	}
	
}



/**
  * @brief  反馈量更新
  * @param  数据结构体指针
  * @retval void
  */
static void feedBack_Set(ControlTask_t* FdbSet)
{
	if(FdbSet == NULL)
	{
		return ;
	}
	
	uint16_t i = 0;

	for(i = 0; i < 2; i++)
	{
		FdbSet->motorCtrl.UpMotor[i].AngleFdb = FdbSet->motorCtrl.UpMotor[i].motor_measure->real_ecd;
		FdbSet->motorCtrl.UpMotor[i].SpeedFdb = FdbSet->motorCtrl.UpMotor[i].motor_measure->speed_rpm;
	}
		
	FdbSet->motorCtrl.SucPitchMotor.AngleFdb = FdbSet->motorCtrl.SucPitchMotor.motor_measure->real_ecd;
	FdbSet->motorCtrl.SucPitchMotor.SpeedFdb = FdbSet->motorCtrl.SucPitchMotor.motor_measure->speed_rpm;

	FdbSet->motorCtrl.SucRollMotor.AngleFdb = FdbSet->motorCtrl.SucRollMotor.motor_measure->real_ecd;
	FdbSet->motorCtrl.SucRollMotor.SpeedFdb = FdbSet->motorCtrl.SucRollMotor.motor_measure->speed_rpm;
	
}


/**
  * @brief  历史数据刷新
  * @param  数据结构体指针
  * @retval void
  */
static void Ctrl_Buff_Refresh(ControlTask_t* buffRefresh)
{
	if(buffRefresh == NULL)
	{
		return ;
	}
	buffRefresh->LastctrlMode = buffRefresh->ctrlMode;
	buffRefresh->LastworkStste = buffRefresh->workStste;
	buffRefresh->LastrunState = buffRefresh->runState;
}

/**
  * @brief  获取第一块金矿石
  * @param  数据结构体指针
  * @retval void
  */
static void Get_Ore_Gold_Fir(ControlTask_t* getGold)
{
	if(getGold == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	if(process_Flag == 0)
	{
	  getGold->workStste = ORE_GOLD_FIR;
		UI_Flag = 1;
		CLAMP(ON);
		vTaskDelay(TIME_STAMP_500MS);
		CLAMP_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		MAGNETIC(ON);
		vTaskDelay(TIME_STAMP_100MS);
	}
	else if(process_Flag == 1)
	{
		CLAMP(OFF);
		vTaskDelay(TIME_STAMP_1000MS);
		FRAME(ON);
		vTaskDelay(TIME_STAMP_500MS);
//		SucPitch_Turn(&getGold->motorCtrl, SucPitch_Appoint[DOWN]);
//		vTaskDelay(TIME_STAMP_500MS);
		MAGNETIC(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		CLAMP_STRETCH(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(OFF);
		vTaskDelay(TIME_STAMP_100MS);
	}
	else if(process_Flag == 2)
	{
//		SUCTION_STRETCH(ON);
//		vTaskDelay(TIME_STAMP_500MS);
//		FRAME(OFF);
//		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		CLAMP(ON);
		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(OFF);
		vTaskDelay(TIME_STAMP_500MS);
//		SucPitch_Reset(getGold);
//		vTaskDelay(TIME_STAMP_500MS);
		getGold->workStste = DO_NOTHING;
		UI_Flag = 0;
	}
	process_Flag++;
	
	if(process_Flag >= 3)
	{
			process_Flag = 0;
	}
}

/**
  * @brief  获取第二块金矿石
  * @param  数据结构体指针
  * @retval void
  */
static void Get_Ore_Gold_Sec(ControlTask_t* getGold)
{
	if(getGold == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	if(process_Flag == 0)
	{
	  getGold->workStste = ORE_GOLD_SEC;
		UI_Flag = 2;
		CLAMP(ON);
		vTaskDelay(TIME_STAMP_500MS);
		MAGNETIC(ON);
	}
	else if(process_Flag == 1)
	{
		CLAMP_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		CLAMP(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(ON);
		vTaskDelay(TIME_STAMP_500MS);
		CLAMP_STRETCH(OFF);
		vTaskDelay(TIME_STAMP_100MS);
	}
	else if(process_Flag == 2)
	{
		SUCTION_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(OFF);
		getGold->workStste = DO_NOTHING;
		UI_Flag = 0;
	}
	process_Flag++;
	
	if(process_Flag >= 3)
	{
			process_Flag = 0;
	}
}

/**
  * @brief  获取第一块银矿石
  * @param  数据结构体指针
  * @retval void
  */
static void Get_Ore_Silver_Fir(ControlTask_t* getSilver)
{
	if(getSilver == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	if(process_Flag == 0)
	{
	  getSilver->workStste = ORE_SILVER_FIR;
		UI_Flag = 3;
		CLAMP(ON);
		CLAMP_STRETCH(ON);
		vTaskDelay(TIME_STAMP_100MS);
		MAGNETIC(ON);
		vTaskDelay(TIME_STAMP_100MS);
		CLAMP_STRETCH(OFF);
	}
	else if(process_Flag == 1)
	{
		CLAMP(OFF);
		vTaskDelay(TIME_STAMP_1000MS);
		FRAME(ON);
		vTaskDelay(TIME_STAMP_1000MS);
//		SucPitch_Turn(&getSilver->motorCtrl, SucPitch_Appoint[DOWN]);
//		vTaskDelay(TIME_STAMP_500MS);
		MAGNETIC(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(OFF);
		vTaskDelay(TIME_STAMP_100MS);
	}
	else if(process_Flag == 2)
	{
//		SUCTION_STRETCH(ON);
//		vTaskDelay(TIME_STAMP_500MS);
//		FRAME(OFF);
//		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		CLAMP(ON);
		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(OFF);
//		vTaskDelay(TIME_STAMP_500MS);
//		SucPitch_Reset(getSilver);
		vTaskDelay(TIME_STAMP_500MS);
		getSilver->workStste = DO_NOTHING;
		UI_Flag = 0;
	}
	process_Flag++;
	
	if(process_Flag >= 3)
	{
			process_Flag = 0;
	}
}

/**
  * @brief  获取第二块银矿石
  * @param  数据结构体指针
  * @retval void
  */
static void Get_Ore_Silver_Sec(ControlTask_t* getSilver)
{
	if(getSilver == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	if(process_Flag == 0)
	{
	  getSilver->workStste = ORE_SILVER_SEC;
		UI_Flag = 4;
		CLAMP(ON);
		CLAMP_STRETCH(ON);
		vTaskDelay(TIME_STAMP_100MS);
		MAGNETIC(ON);
		vTaskDelay(TIME_STAMP_100MS);
		CLAMP_STRETCH(OFF);
	}
	else if(process_Flag == 1)
	{
		CLAMP(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(ON);
//		vTaskDelay(TIME_STAMP_500MS);
//		SucPitch_Turn(&getSilver->motorCtrl, SucPitch_Appoint[DOWN]);
		vTaskDelay(TIME_STAMP_500MS);
	}
	else if(process_Flag == 2)
	{
//		SUCTION_STRETCH(ON);
//		vTaskDelay(TIME_STAMP_500MS);	
		SUCTION_STRETCH(ON);
		vTaskDelay(TIME_STAMP_100MS);
		FRAME(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		
//		SucPitch_Reset(getSilver);
//		vTaskDelay(TIME_STAMP_500MS);
		getSilver->workStste = DO_NOTHING;
		UI_Flag = 0;
	}
	process_Flag++;
	
	if(process_Flag >= 3)
	{
			process_Flag = 0;
	}
}
/**
  * @brief  空接
  * @param  数据结构体指针
  * @retval void
  */
static void Catch_Ore(ControlTask_t* CatchGold)
{
	if(CatchGold == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	CatchGold->workStste = ORE_CATCH;
	UI_Flag = 5;
	if(process_Flag == 0)
	{
		CatchGold->workStste = ORE_CATCH;
		CLAMP(ON);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(ON);
		vTaskDelay(TIME_STAMP_1000MS);
		CLAMP_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		MAGNETIC(ON);
		vTaskDelay(TIME_STAMP_500MS);
		Up_Down_To_Point(&CatchGold->motorCtrl, L_Up_Appoint[HIGHEST], R_Up_Appoint[HIGHEST]);
		vTaskDelay(TIME_STAMP_500MS);
	}
	process_Flag++;
	while(process_Flag == 1)
	{
		if(IF_INFRARAD)
		{
		 CLAMP(OFF);
		 vTaskDelay(TIME_STAMP_500MS);
		 Up_Down_To_Point(&CatchGold->motorCtrl, L_Up_Appoint[LOWEST], R_Up_Appoint[LOWEST]);
		 vTaskDelay(TIME_STAMP_500MS);
		 return;
		}
		if(!IF_INFRARAD )
		{
			if((IF_KEY_PRESSED_E && IF_KEY_PRESSED_CTRL))
			{
		    CLAMP(OFF);
		    vTaskDelay(TIME_STAMP_500MS);
				Up_Down_To_Point(&CatchGold->motorCtrl, L_Up_Appoint[LOWEST], R_Up_Appoint[LOWEST]);
		    vTaskDelay(TIME_STAMP_500MS);
		    return;
		  }
		  else if((switch_is_up(CatchGold->Input_Point->rc.s[ModeChannel_L]) && CatchGold->Input_Point->rc.ch[4] <= -CH4_TRIGGER_VAL))
			{ 
				CLAMP(OFF);
	   	  vTaskDelay(TIME_STAMP_500MS);
				Up_Down_To_Point(&CatchGold->motorCtrl, L_Up_Appoint[LOWEST], R_Up_Appoint[LOWEST]);
		    vTaskDelay(TIME_STAMP_500MS);
		    return;
			}
		}
		PID_Output(&ctrlTask.motorCtrl);
		vTaskDelay(1);
	 }
	if(process_Flag == 2)
	{
		CLAMP_STRETCH(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		MAGNETIC(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(OFF);
	}
	else if(process_Flag == 3)
	{
		SUCTION_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		CLAMP(ON);
		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(OFF);
		
		process_Flag++;
		CatchGold->workStste = DO_NOTHING;
		UI_Flag = 0;
	}

	if(process_Flag >= 4)
	{
			process_Flag = 0;
	}
}


/**
  * @brief  兑换矿石
  * @param  数据结构体指针
  * @retval void
  */
static void Exchange_Ore(ControlTask_t* exchangeOre)
{
	if(exchangeOre == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	if(process_Flag == 0)
	{
	  exchangeOre->workStste = ORE_EXCHANGE;
		UI_Flag = 6;
		IMPLICATION(ON);
		vTaskDelay(TIME_STAMP_2000MS);
		FRAME(ON);
		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(ON);
		vTaskDelay(TIME_STAMP_500MS);
		SucPitch_Turn(&exchangeOre->motorCtrl, SucPitch_Appoint[FRONT]);
		vTaskDelay(TIME_STAMP_500MS);
		SucRoll_Turn(&exchangeOre->motorCtrl, SucRoll_Appoint[READY]);
		vTaskDelay(TIME_STAMP_500MS);
    MAGNETIC(OFF); 
	}
	else if(process_Flag == 1)
	{
		IMPLICATION(OFF);
	}
	else if(process_Flag == 2 )
	{	
//		SucPitch_Turn(&exchangeOre->motorCtrl, SucPitch_Appoint[DOWN]);
//		vTaskDelay(TIME_STAMP_100MS);
		SucRoll_Turn(&exchangeOre->motorCtrl, SucRoll_Appoint[RES]);
		vTaskDelay(TIME_STAMP_50MS);
		SucRoll_Reset(exchangeOre);
		vTaskDelay(TIME_STAMP_500MS);
		SucPitch_Turn(&exchangeOre->motorCtrl, SucPitch_Appoint[BACK]);
		vTaskDelay(TIME_STAMP_500MS);
		FRAME(OFF);
		vTaskDelay(TIME_STAMP_500MS);
		Up_Down_To_Point(&exchangeOre->motorCtrl, L_Up_Appoint[LOWEST], R_Up_Appoint[LOWEST]);
		CLAMP(ON);
		vTaskDelay(TIME_STAMP_500MS);
		SUCTION_STRETCH(OFF);
		vTaskDelay(TIME_STAMP_500MS);
//		SucPitch_Reset(exchangeOre);
		exchangeOre->workStste = DO_NOTHING;
		UI_Flag = 0;
	}
	process_Flag++;
	
	if(process_Flag >= 3)
	{
			process_Flag = 0;
	}
}

/**
  * @brief  吸盘Roll复位
  * @param  电机结构体
  * @retval void
  * @attention 
  */
void SucRoll_Reset(ControlTask_t* sucRollTurn)
{
	uint8_t i = 0;
	sucRollTurn->motorCtrl.SucRollMotor.Amplitude_Flag = false;
	while(!IF_SUCTION_ROLL_KEY)
	{
		sucRollTurn->motorCtrl.SucRollMotor.AngleRef--;
    PID_Output(&sucRollTurn->motorCtrl);
		vTaskDelay(TIME_STAMP_1MS);//系统延时	
	}
	sucRollTurn->motorCtrl.SucRollMotor.CenterOffset = sucRollTurn->motorCtrl.SucRollMotor.motor_measure->real_ecd;
	sucRollTurn->motorCtrl.SucRollMotor.CenterOffset = sucRollTurn->motorCtrl.SucRollMotor.CenterOffset + 15 * REDUCTION_RATIO_2006;
	sucRollTurn->motorCtrl.SucRollMotor.AngleRef = sucRollTurn->motorCtrl.SucRollMotor.CenterOffset;
	for(i = 0; i < SUC_ROLL_COUNT; i++)
	{
		SucRoll_Appoint[i] = sucRollTurn->motorCtrl.SucRollMotor.CenterOffset + SucRoll_Motor_Longs[i];
	}
	sucRollTurn->motorCtrl.SucRollMotor.Amplitude_Flag = true;
		
}

/**
  * @brief  吸盘Pitch复位
  * @param  电机结构体
  * @retval void
  * @attention 
  */
void SucPitch_Reset(ControlTask_t* sucPitchTurn)
{
	uint8_t i = 0;
	uint8_t flag_sucPitchTurn;
	sucPitchTurn->motorCtrl.SucRollMotor.Amplitude_Flag = false;
	while(!IF_SUCTION_PITCH_KEY)
	{
		sucPitchTurn->motorCtrl.SucPitchMotor.AngleRef--;
    PID_Output(&sucPitchTurn->motorCtrl);
		vTaskDelay(TIME_STAMP_1MS);//系统延时	
	}
	sucPitchTurn->motorCtrl.SucPitchMotor.CenterOffset = sucPitchTurn->motorCtrl.SucPitchMotor.motor_measure->real_ecd;
	sucPitchTurn->motorCtrl.SucPitchMotor.AngleRef = sucPitchTurn->motorCtrl.SucPitchMotor.CenterOffset;
	for(i = 0; i < SUC_PITCH_COUNT; i++)
	{
		SucPitch_Appoint[i] = sucPitchTurn->motorCtrl.SucPitchMotor.CenterOffset + SucPitch_Motor_Longs[i];
	}
	sucPitchTurn->motorCtrl.SucPitchMotor.Amplitude_Flag = true;	
	if(IF_SUCTION_PITCH_KEY)
	{
		SUCTION_STRETCH(OFF);
		vTaskDelay(TIME_STAMP_500MS);//系统延时	
	}
}


/**
  * @brief  手动微调控制
  * @param  数据结构体指针
  * @retval void
  */
static void Fine_Tuning(ControlTask_t* fineTuning)
{
	if(fineTuning == NULL)
	{
		return ;
	}
	static u8 Rc_imp_Flag = OFF,
		        Pc_imp_Flag = OFF,
	          Rc_clamp_Flag = OFF,
	          Pc_gimup_Flag = OFF,
	          Pc_magnetic_Flag = OFF,
	          Pc_suction_Stretch_Flag = OFF,
		        uesr_Flag = OFF;
	
//	static u8 f_Flag = OFF,
//					  w_Flag = OFF,
//					  a_Flag = OFF,
//					  s_Flag = OFF,
//					  d_Flag = OFF;
//	
//	if(IF_KEY_PRESSED_F && !f_Flag)
//	{
//		f_Flag = ON;
//	}
//	if(!IF_KEY_PRESSED_F && f_Flag)
//	{
//		f_Flag = OFF;
//		CLAMP_TOGGLE;
//	}
//	
//	if(IF_KEY_PRESSED_W && !w_Flag)
//	{
//		w_Flag = ON;
//	}
//	if(!IF_KEY_PRESSED_W && w_Flag)
//	{
//		w_Flag = OFF;
//		FRAME_TOGGLE;
//	}
//	
//	if(IF_KEY_PRESSED_A && !a_Flag)
//	{
//		a_Flag = ON;
//	}
//	if(!IF_KEY_PRESSED_A && a_Flag)
//	{
//		a_Flag = OFF;
//		CLAMP_STRETCH_TOGGLE;
//	}

//	if(IF_KEY_PRESSED_S && !s_Flag)
//	{
//		s_Flag = ON;
//	}
//	if(!IF_KEY_PRESSED_S && s_Flag)
//	{
//		s_Flag = OFF;
//		MAGNETIC_TOGGLE;
//	}
//	
//	if(IF_KEY_PRESSED_D && !d_Flag)
//	{
//		d_Flag = ON;
//	}
//	if(!IF_KEY_PRESSED_D && d_Flag)
//	{
//		d_Flag = OFF;
//		SUCTION_STRETCH_TOGGLE;
//	}
	
	
/*============================================涵道开关============================================*/      
	if(fineTuning->Input_Point->mouse.press_r && !Pc_imp_Flag)
	{
		Pc_imp_Flag = ON;
		Rc_imp_Flag = OFF;
	}
	if(!fineTuning->Input_Point->mouse.press_r && Pc_imp_Flag)
	{
		Pc_imp_Flag = OFF;
		IMPLICATION_TOGGLE;
	}
	
	if(fineTuning->Input_Point->rc.ch[4] >= CH4_TRIGGER_VAL  && !Rc_imp_Flag)
	{
		Rc_imp_Flag = ON;
		Pc_imp_Flag = OFF;
	}
	if(fineTuning->Input_Point->rc.ch[4] < CH4_TRIGGER_VAL && Rc_imp_Flag)
	{
		Rc_imp_Flag = OFF;
		IMPLICATION_TOGGLE;
	}
	
/*============================================云台升降============================================*/ 
	if(IF_KEY_PRESSED_B && !Pc_gimup_Flag)
	{
		Pc_gimup_Flag = ON;
	}
	if(!IF_KEY_PRESSED_B && Pc_gimup_Flag)
	{
		Pc_gimup_Flag = OFF;
		GIMBAL_UP_TOGGLE;
	}

	
/*============================================夹爪气缸============================================*/ 
	if(fineTuning->Input_Point->rc.ch[4] <= -CH4_TRIGGER_VAL  && !Rc_clamp_Flag)
	{
		Rc_clamp_Flag = ON;
	}
	if(fineTuning->Input_Point->rc.ch[4] > -CH4_TRIGGER_VAL && Rc_clamp_Flag)
	{
		Rc_clamp_Flag = OFF;
		CLAMP_TOGGLE;
	}
	
/*============================================磁耦气缸============================================*/ 	
	if(IF_KEY_PRESSED_F && !Pc_magnetic_Flag)
	{
		Pc_magnetic_Flag = ON;
	}
	if(!IF_KEY_PRESSED_F && Pc_magnetic_Flag)
	{
		Pc_magnetic_Flag = OFF;
		MAGNETIC_TOGGLE;
	}
	

/*============================================吸盘前伸气缸============================================*/ 		
	if(IF_KEY_PRESSED_G && !Pc_suction_Stretch_Flag)
	{
		Pc_suction_Stretch_Flag = ON;
	}
	if(!IF_KEY_PRESSED_G && Pc_suction_Stretch_Flag)
	{
		Pc_suction_Stretch_Flag = OFF;
		SUCTION_STRETCH_TOGGLE;
	}
	
	
/*============================================用户手动确认复位点============================================*/ 
	if(IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_Z && !uesr_Flag)
	{
		uesr_Flag = ON;
	}
	if(!IF_KEY_PRESSED_CTRL && !IF_KEY_PRESSED_SHIFT && !IF_KEY_PRESSED_Z && uesr_Flag)
	{
		uesr_Flag = OFF;
	  fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset = fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].motor_measure->real_ecd;
		fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef = fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset;
		
	  fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset = fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].motor_measure->real_ecd;
	  fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef = fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset;
		
	  fineTuning->motorCtrl.SucPitchMotor.CenterOffset = fineTuning->motorCtrl.SucPitchMotor.motor_measure->real_ecd;
		fineTuning->motorCtrl.SucPitchMotor.AngleRef = fineTuning->motorCtrl.SucPitchMotor.CenterOffset;
		
	  fineTuning->motorCtrl.SucRollMotor.CenterOffset = fineTuning->motorCtrl.SucRollMotor.motor_measure->real_ecd;
		fineTuning->motorCtrl.SucRollMotor.AngleRef = fineTuning->motorCtrl.SucRollMotor.CenterOffset;			

		fineTuning->motorCtrl.ifSpid = true;
		Get_Motor_Appoint(fineTuning);
		fineTuning->runState = RUN_TURNING;
	}
	  
/*============================================手动调整对矿位置============================================*/	
	if(fineTuning->Input_Point->mouse.press_l)
	{
	  fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef -= fineTuning->Input_Point->mouse.y * PC_UP_RATIO;
	  fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef += fineTuning->Input_Point->mouse.y * PC_UP_RATIO;
	}
	if(fineTuning->Input_Point->mouse.press_r)
	{
	  fineTuning->motorCtrl.SucRollMotor.AngleRef += fineTuning->Input_Point->mouse.x * PC_SUC_ROLL_RATIO;
	}
	
	fineTuning->motorCtrl.SucPitchMotor.AngleRef += fineTuning->Input_Point->mouse.z * PC_SUC_PITCH_RATIO;
	  
	if(!(fineTuning->Input_Point->mouse.press_l) && !(fineTuning->Input_Point->mouse.press_r))
	{
	  fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef += fineTuning->Input_Point->rc.ch[3] * RC_UP_RATIO;
	  fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef -= fineTuning->Input_Point->rc.ch[3] * RC_UP_RATIO;
	  
	  fineTuning->motorCtrl.SucPitchMotor.AngleRef += fineTuning->Input_Point->rc.ch[1] * RC_SUC_PITCH_RATIO;
	  
	  fineTuning->motorCtrl.SucRollMotor.AngleRef += fineTuning->Input_Point->rc.ch[0] * RC_SUC_ROLL_RATIO;
	}
	
//	if(fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].Amplitude_Flag && fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].Amplitude_Flag)
//	{
//		fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef = Constrain_float(fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].AngleRef, \
//																																				(fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset + L_Up_LONGS_LOWEST), \
//																																				(fineTuning->motorCtrl.UpMotor[LEFT_MOTOR].CenterOffset + L_UP_LONGS_HIGHTST));
//		fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef = Constrain_float(fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].AngleRef,  \
//																																				 (fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset + R_UP_LONGS_HIGHTST), \
//																																				 (fineTuning->motorCtrl.UpMotor[RIGHT_MOTOR].CenterOffset + R_Up_LONGS_LOWEST));
//	}
//	if(fineTuning->motorCtrl.SucPitchMotor.Amplitude_Flag)
//	{
//		fineTuning->motorCtrl.SucPitchMotor.AngleRef = Constrain_float(fineTuning->motorCtrl.SucPitchMotor.AngleRef,  \
//																																	(fineTuning->motorCtrl.SucPitchMotor.CenterOffset), \
//																																	(fineTuning->motorCtrl.SucPitchMotor.CenterOffset + (210 *REDUCTION_RATIO_3508)));
//	}
//	if(fineTuning->motorCtrl.SucRollMotor.Amplitude_Flag)
//	{
//		fineTuning->motorCtrl.SucRollMotor.AngleRef = Constrain_float(fineTuning->motorCtrl.SucRollMotor.AngleRef,  \
//																																	(fineTuning->motorCtrl.SucRollMotor.CenterOffset), \
//																																	(fineTuning->motorCtrl.SucRollMotor.CenterOffset + ((180 + 60) * REDUCTION_RATIO_2006)));
//	}
	
}



/**
  * @brief  返回结构体变量地址
  * @param  void
  * @retval const ControlTask_t*
  * @attention 
  */
const ControlTask_t* get_Makemoney_Point(void)
{
	return &ctrlTask;
}

