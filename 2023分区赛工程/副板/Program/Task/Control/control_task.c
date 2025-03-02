#include "control_task.h"
#include "FreeRTOSConfig.h"
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


ControlTask_t makeMoney;

u8 pid_flag=0;
int rest_flag=1;

fp32 L_Up_Appoint[UP_COUNT];
fp32 R_Up_Appoint[UP_COUNT];
fp32 L_Stretch_Appoint[STRETCH_COUNT];
fp32 R_Stretch_Appoint[STRETCH_COUNT];
fp32 L_TopMove_Appoint[TOPMOVE_COUNT];
fp32 R_TopMove_Appoint[TOPMOVE_COUNT];
fp32 L_Rescue_Appoint[RESCUE_COUNT];
fp32 R_Rescue_Appoint[RESCUE_COUNT];
fp32 TopPitch_Appoint[TOP_PITCH_COUNT];
fp32 TopRoll_Appoint[TOP_ROLL_COUNT];

fp32 L_Up_Motor_Longs[UP_COUNT] = {L_UP_LONGS_HIGHTST, L_Up_LONGS_GOLD,	L_Up_LONGS_SILVER, L_Up_LONGS_Exchang, L_Up_LONGS_LOWEST};
fp32 R_Up_Motor_Longs[UP_COUNT] = {R_UP_LONGS_HIGHTST, R_Up_LONGS_GOLD,	R_Up_LONGS_SILVER, R_Up_LONGS_Exchang, R_Up_LONGS_LOWEST};
fp32 L_Stretch_Motor_Longs[STRETCH_COUNT] = {L_STRETCH_LONGS_LONGEST, L_STRETCH_LONGS_GOLD,	L_STRETCH_LONGS_SILVER, L_STRETCH_LONGS_Exchang, L_STRETCH_LONGS_SHORTEST};
fp32 R_Stretch_Motor_Longs[STRETCH_COUNT] = {R_STRETCH_LONGS_LONGEST, R_STRETCH_LONGS_GOLD,	R_STRETCH_LONGS_SILVER, R_STRETCH_LONGS_Exchang, R_STRETCH_LONGS_SHORTEST};
fp32 L_TopMove_Motor_Longs[TOPMOVE_COUNT] = {L_TOPMOVE_LONGS_FURTHEST, L_TOPMOVE_LONGS_CLOSEST};
fp32 R_TopMove_Motor_Longs[TOPMOVE_COUNT] = {R_TOPMOVE_LONGS_FURTHEST, R_TOPMOVE_LONGS_CLOSEST};
fp32 TopPitch_Motor_Longs[TOP_PITCH_COUNT] = {TOP_PITCH_LONGS_UP, TOP_PITCH_LONGS_FRONT, TOP_PITCH_LONGS_DOWN, TOP_PITCH_LONGS_BACK};
fp32 TopRoll_Motor_Longs[TOP_ROLL_COUNT] = {TOP_ROLL_LONGS_RES};
fp32 L_Rescue_Motor_Longs[RESCUE_COUNT] = {L_RESCUE_LONGS_ON, L_RESCUE_LONGS_OFF};
fp32 R_Rescue_Motor_Longs[RESCUE_COUNT] = {R_RESCUE_LONGS_ON, R_RESCUE_LONGS_OFF};


fp32 topPitchAgePid[3] = {TOP_PITCH_ANGLE_PID_KP, TOP_PITCH_ANGLE_PID_KI, TOP_PITCH_ANGLE_PID_KD};
fp32 topPitchSpdPid[3] = {TOP_PITCH_SPEED_PID_KP, TOP_PITCH_SPEED_PID_KI, TOP_PITCH_SPEED_PID_KD};
fp32 topRollAgePid[3] = {TOP_ROLL_ANGLE_PID_KP, TOP_ROLL_ANGLE_PID_KI, TOP_ROLL_ANGLE_PID_KD};
fp32 topRollSpdPid[3] = {TOP_ROLL_SPEED_PID_KP, TOP_ROLL_SPEED_PID_KI, TOP_ROLL_SPEED_PID_KD};
fp32 upAgePid[3] = {UP_ANGLE_PID_KP, UP_ANGLE_PID_KI, UP_ANGLE_PID_KD};
fp32 upSpdPid[3] = {UP_SPEED_PID_KP, UP_SPEED_PID_KI, UP_SPEED_PID_KD};
fp32 stretchAgePid[3] = {STRETCH_ANGLE_PID_KP, STRETCH_ANGLE_PID_KI, STRETCH_ANGLE_PID_KD};
fp32 stretchSpdPid[3] = {STRETCH_SPEED_PID_KP, STRETCH_SPEED_PID_KI, STRETCH_SPEED_PID_KD};
fp32 rescueAgePid[3] = {RESCUE_ANGLE_PID_KP, RESCUE_ANGLE_PID_KI, RESCUE_ANGLE_PID_KD};
fp32 rescueSpdPid[3] = {RESCUE_SPEED_PID_KP, RESCUE_SPEED_PID_KI, RESCUE_SPEED_PID_KD};
fp32 topMoveAgePid[3] = {TOPMOVE_ANGLE_PID_KP, TOPMOVE_ANGLE_PID_KI, TOPMOVE_ANGLE_PID_KD};
fp32 topMoveSpdPid[3] = {TOPMOVE_SPEED_PID_KP, TOPMOVE_SPEED_PID_KI, TOPMOVE_SPEED_PID_KD};

static void Ctrl_Init(ControlTask_t* init);
static void CtrlMode_Set(ControlTask_t* ModeSet);
static void makeMoney_Work_Set(ControlTask_t* stateSet);
static void feedBack_Set(ControlTask_t* FdbSet);
static void Ctrl_Buff_Refresh(ControlTask_t* buffRefresh);
static void Get_Motor_Appoint(ControlTask_t* getAppiont);
static void Motor_Reset(ControlTask_t* motorRes);
static void Do_Nothing(ControlTask_t* doNothing);
static void Get_Ore_Gold(ControlTask_t* getGold);
static void Get_Ore_Silver(ControlTask_t* getSilver);
static void Catch_Ore_Gold(ControlTask_t* CatchGold);
static void Exchange_Ore(ControlTask_t* exchangeOre);
static void Fine_Tuning(ControlTask_t* fineTuning);
static void Motor_Constrain(ControlTask_t* Constrain);
static void give_current(ControlTask_t* giveCurrent);

void Control_task(void)
{
	static portTickType TickCount; 
	TickCount = xTaskGetTickCount();
	Ctrl_Init(&makeMoney);
	Get_Motor_Appoint(&makeMoney);

	while(1)
	{
		
		if(rest_flag==1)//上电复位
		{
			Motor_Reset(&makeMoney);
		}
		else
		{
		 CtrlMode_Set(&makeMoney);//控制模式设置
		}
		makeMoney_Work_Set(&makeMoney);
		feedBack_Set(&makeMoney);//反馈更新
		give_current(&makeMoney);
		
		if((makeMoney.ctrlMode == CTRL_NULL)&&(rest_flag==0))//给上电复位电流
		{
			CAN1_CMD_0x200(0,0,0,0);
	  	CAN2_CMD_STRETCH(0,0);  
		}
		else
		{
			CAN1_CMD_0x200(makeMoney.UpMotor[0].Current, makeMoney.UpMotor[1].Current,makeMoney.TopMoveMotor[0].Current, makeMoney.TopMoveMotor[1].Current);
		  CAN2_CMD_STRETCH(makeMoney.StretchMotor[0].SpeedPid.out,makeMoney.StretchMotor[1].SpeedPid.out);			
		}
		Ctrl_Buff_Refresh(&makeMoney);
		
		vTaskDelay(1);
//		vTaskDelayUntil(&TickCount, 1);
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
	
	uint8_t i = 0;
/***************************获取数据指针************************************/	
	init->Input_Point = get_remote_control_point();
	init->TopPitchMotor.motor_measure = get_TopPitch_EncoderProcess_Point();
	init->TopRollMotor.motor_measure = get_TopRoll_EncoderProcess_Point();
	init->topMoveKeyPoint = get_topMoveKey_Point();
	for(i = 0; i < 2; i++)
	{
		init->StretchMotor[i].motor_measure = get_Stretch_EncoderProcess_Point(i);
		init->TopMoveMotor[i].motor_measure = get_TopMove_EncoderProcess_Point(i);
		init->UpMotor[i].motor_measure = get_Up_EncoderProcess_Point(i);	
	}
	vTaskDelay(10);
/***************************初始化反馈数据************************************/		
	init->TopPitchMotor.AngleFdb = init->TopPitchMotor.motor_measure->real_ecd;
	init->TopPitchMotor.SpeedFdb = init->TopPitchMotor.motor_measure->speed_rpm;
	init->TopRollMotor.AngleFdb = init->TopRollMotor.motor_measure->real_ecd;
	init->TopRollMotor.SpeedFdb = init->TopRollMotor.motor_measure->speed_rpm;
	for(i = 0; i < 2; i++)
	{

		init->StretchMotor[i].AngleFdb = init->StretchMotor[i].motor_measure->real_ecd;
		init->StretchMotor[i].SpeedFdb = init->StretchMotor[i].motor_measure->speed_rpm;
		
		init->TopMoveMotor[i].AngleFdb = init->TopMoveMotor[i].motor_measure->real_ecd;
		init->TopMoveMotor[i].SpeedFdb = init->TopMoveMotor[i].motor_measure->speed_rpm;
		
		init->UpMotor[i].AngleFdb = init->UpMotor[i].motor_measure->real_ecd;		
		init->UpMotor[i].SpeedFdb = init->UpMotor[i].motor_measure->speed_rpm;	
	}
	
	/***************************初始化归中数据************************************/		
	init->TopPitchMotor.CenterOffset = init->TopPitchMotor.motor_measure->real_ecd;
	init->TopRollMotor.CenterOffset = init->TopRollMotor.motor_measure->real_ecd;
	for(i = 0; i < 2; i++)
	{
		
		init->StretchMotor[i].CenterOffset = init->StretchMotor[i].motor_measure->real_ecd;
		
		init->TopMoveMotor[i].CenterOffset = init->TopMoveMotor[i].motor_measure->real_ecd;
		
		init->UpMotor[i].CenterOffset = init->UpMotor[i].motor_measure->real_ecd;
	}
	
	/***************************初始化期望数据************************************/		
	init->TopPitchMotor.AngleRef = init->TopPitchMotor.CenterOffset;
	init->TopRollMotor.AngleRef = init->TopRollMotor.CenterOffset;
	for(i = 0; i < 2; i++)
	{
		init->RescueMotor[i].AngleRef = init->RescueMotor[i].CenterOffset;
		init->StretchMotor[i].AngleRef = init->StretchMotor[i].CenterOffset;																														 
		init->TopMoveMotor[i].AngleRef = init->TopMoveMotor[i].CenterOffset;
		init->UpMotor[i].AngleRef = init->UpMotor[i].CenterOffset;
	}
	
	/*************************吸盘pitch轴PID初始化************************************/
	New_PID_Init(&init->TopPitchMotor.AnglePid, 
	             NEW_PID_POSITION, 
	             topPitchAgePid, 
	             TOP_PITCH_ANGLE_PID_MAX_OUT, 
	             TOP_PITCH_ANGLE_PID_MAX_IOUT, 
               TOP_PITCH_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             TOP_PITCH_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&init->TopPitchMotor.SpeedPid, 
	             NEW_PID_POSITION, 
	             topPitchSpdPid, 
	             TOP_PITCH_SPEED_PID_MAX_OUT, 
	             TOP_PITCH_SPEED_PID_MAX_IOUT, 
               TOP_PITCH_SPEED_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             TOP_PITCH_SPEED_PID_I_SEPARATION);
							 
/*************************吸盘roll轴PID初始化************************************/
	New_PID_Init(&init->TopRollMotor.AnglePid, 
	             NEW_PID_POSITION, 
	             topRollAgePid, 
	             TOP_ROLL_ANGLE_PID_MAX_OUT, 
	             TOP_ROLL_ANGLE_PID_MAX_IOUT, 
               TOP_ROLL_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             TOP_ROLL_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&init->TopRollMotor.SpeedPid, 
	             NEW_PID_POSITION, 
	             topRollSpdPid, 
	             TOP_ROLL_SPEED_PID_MAX_OUT, 
	             TOP_ROLL_SPEED_PID_MAX_IOUT, 
               TOP_ROLL_SPEED_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             TOP_ROLL_SPEED_PID_I_SEPARATION);
							 

	for(i = 0; i < 2; i++)
		{	
/*************************抬升PID初始化************************************/
			
			New_PID_Init(&init->UpMotor[i].AnglePid, 
									 NEW_PID_POSITION, 
									 upAgePid, 
									 UP_ANGLE_PID_MAX_OUT, 
									 UP_ANGLE_PID_MAX_IOUT, 
									 UP_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 UP_ANGLE_PID_I_SEPARATION);
			
			
			New_PID_Init(&init->UpMotor[i].SpeedPid, 
									 NEW_PID_POSITION, 
									 upSpdPid, 
									 UP_SPEED_PID_MAX_OUT, 
									 UP_SPEED_PID_MAX_IOUT, 
									 UP_SPEED_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 UP_SPEED_PID_I_SEPARATION);
			 
/*************************前伸PID初始化************************************/
			
			New_PID_Init(&init->StretchMotor[i].AnglePid, 
									 NEW_PID_POSITION, 
									 stretchAgePid, 
									 STRETCH_ANGLE_PID_MAX_OUT, 
									 STRETCH_ANGLE_PID_MAX_IOUT, 
									 STRETCH_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 STRETCH_ANGLE_PID_I_SEPARATION);
			
			
			New_PID_Init(&init->StretchMotor[i].SpeedPid, 
									 NEW_PID_POSITION, 
									 stretchSpdPid, 
									 STRETCH_SPEED_PID_MAX_OUT, 
									 STRETCH_SPEED_PID_MAX_IOUT, 
									 STRETCH_SPEED_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 STRETCH_SPEED_PID_I_SEPARATION);
			
					 
/*************************吸盘移动PID初始化************************************/

			New_PID_Init(&init->TopMoveMotor[i].AnglePid, 
									 NEW_PID_POSITION, 
									 topMoveAgePid, 
									 TOPMOVE_ANGLE_PID_MAX_OUT, 
									 TOPMOVE_ANGLE_PID_MAX_IOUT, 
									 TOPMOVE_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 TOPMOVE_ANGLE_PID_I_SEPARATION);


			New_PID_Init(&init->TopMoveMotor[i].SpeedPid, 
									 NEW_PID_POSITION, 
									 topMoveSpdPid, 
									 TOPMOVE_SPEED_PID_MAX_OUT, 
									 TOPMOVE_SPEED_PID_MAX_IOUT, 
									 TOPMOVE_SPEED_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 TOPMOVE_SPEED_PID_I_SEPARATION);
				 
	 }				 
	
	/***************************复位标志初始化************************************/
	init->TopPitchMotor.ResFlag = 0;
	init->TopRollMotor.ResFlag = 0;
	for(i = 0; i < 2; i++)
	{
		init->StretchMotor[i].ResFlag = 0;
		init->TopMoveMotor[i].ResFlag = 0;
		init->UpMotor[i].ResFlag = 0;
	}
	init->ctrlMode = CTRL_NULL;
	init->makeMoney = DO_RESET;
	init->VacuumFlag = OFF;
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
		L_Up_Appoint[i] = getAppiont->UpMotor[LEFT_MOTOR].CenterOffset + L_Up_Motor_Longs[i];
		R_Up_Appoint[i] = getAppiont->UpMotor[RIGHT_MOTOR].CenterOffset + R_Up_Motor_Longs[i];
	}
	for(i = 0; i < STRETCH_COUNT; i++)
	{
		L_Stretch_Appoint[i] = getAppiont->StretchMotor[LEFT_MOTOR].CenterOffset + L_Stretch_Motor_Longs[i];
		R_Stretch_Appoint[i] = getAppiont->StretchMotor[RIGHT_MOTOR].CenterOffset + R_Stretch_Motor_Longs[i];
	}
	for(i = 0; i < TOPMOVE_COUNT; i++)
	{
		L_TopMove_Appoint[i] = getAppiont->TopMoveMotor[LEFT_MOTOR].CenterOffset + L_TopMove_Motor_Longs[i];
		R_TopMove_Appoint[i] = getAppiont->TopMoveMotor[RIGHT_MOTOR].CenterOffset + R_TopMove_Motor_Longs[i];
	}
	for(i = 0; i < TOP_PITCH_COUNT; i++)
	{
		TopPitch_Appoint[i] = getAppiont->TopPitchMotor.CenterOffset + TopPitch_Motor_Longs[i];
	}
	for(i = 0; i < TOP_ROLL_COUNT; i++)
	{
		TopRoll_Appoint[i] = getAppiont->TopRollMotor.CenterOffset + TopRoll_Motor_Longs[i];
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
/*********************复位完成后得到行程**********************************/
	if((motorRes->UpMotor[LEFT_MOTOR].ResFlag) && \
		 (motorRes->UpMotor[RIGHT_MOTOR].ResFlag) && \
		 (motorRes->StretchMotor[LEFT_MOTOR].ResFlag) && \
		 (motorRes->StretchMotor[RIGHT_MOTOR].ResFlag)	&&  \
	   (motorRes->TopPitchMotor.ResFlag))  
		{
			Get_Motor_Appoint(motorRes);
			motorRes->makeMoney = DO_NOTHING;
			motorRes->workStste = NONE;
			rest_flag=0;
		}
		
		
//			if((L_STRETCH_KEY == 1)&&(R_STRETCH_KEY == 1)&&(L_UP_KEY == 1)&&(R_UP_KEY == 1)&&(TOP_PITCH_KEY == 1))                                                                          //&& (motorRes->TopRollMotor.ResFlag)	
//		{
//			Get_Motor_Appoint(motorRes);
//			motorRes->makeMoney = DO_NOTHING;
//			motorRes->workStste = NONE;
//			rest_flag=0;
//		}
		
	if((L_STRETCH_KEY == 1)&&(R_STRETCH_KEY == 1))
	{
	if(L_UP_KEY != 1)//1   没按下
	{
motorRes->UpMotor[LEFT_MOTOR].ResFlag = 0;		
		motorRes->UpMotor[LEFT_MOTOR].SpeedRef = -2000;
	}
	if(L_UP_KEY == 1)//
	{
		motorRes->UpMotor[LEFT_MOTOR].SpeedRef = 0;
	
		motorRes->UpMotor[LEFT_MOTOR].CenterOffset = motorRes->UpMotor[LEFT_MOTOR].motor_measure->real_ecd;
		motorRes->UpMotor[LEFT_MOTOR].ResFlag = 1;
	}
	if(R_UP_KEY != 1)
	{
		motorRes->UpMotor[RIGHT_MOTOR].SpeedRef = 2000;
		motorRes->UpMotor[RIGHT_MOTOR].ResFlag = 0;
	}
	if(R_UP_KEY == 1)
	{
		motorRes->UpMotor[RIGHT_MOTOR].SpeedRef = 0;
		motorRes->UpMotor[RIGHT_MOTOR].CenterOffset = motorRes->UpMotor[RIGHT_MOTOR].motor_measure->real_ecd;
		motorRes->UpMotor[RIGHT_MOTOR].ResFlag = 1;
	}
}
/*******************************************************/	
//	if(motorRes->topMoveKeyPoint!= 0)
//	{
//		motorRes->TopMoveMotor[LEFT_MOTOR].SpeedRef = 10;
//		motorRes->TopMoveMotor[RIGHT_MOTOR].SpeedRef = 10;

//	}
//	if(motorRes->topMoveKeyPoint == 0)
//	{
//		motorRes->TopMoveMotor[LEFT_MOTOR].SpeedRef = 0;
//		motorRes->TopMoveMotor[RIGHT_MOTOR].SpeedRef = 0;
//		for(i = 0; i < 2; i++)
//		{
//			motorRes->TopMoveMotor[i].CenterOffset = motorRes->TopMoveMotor[i].motor_measure->real_ecd;
//			motorRes->TopMoveMotor[i].ResFlag = 1;
//		}
//	}	
//	
///*******************************************************/
	
	if(L_STRETCH_KEY != 1)//前伸
	{
		motorRes->StretchMotor[LEFT_MOTOR].SpeedRef = 2000;
		motorRes->StretchMotor[LEFT_MOTOR].ResFlag = 0;
	}
	if(L_STRETCH_KEY == 1)
	{
		motorRes->StretchMotor[LEFT_MOTOR].SpeedRef = 0;
		motorRes->StretchMotor[LEFT_MOTOR].CenterOffset = motorRes->StretchMotor[LEFT_MOTOR].motor_measure->real_ecd;
		motorRes->StretchMotor[LEFT_MOTOR].ResFlag = 1;
	}
	if(R_STRETCH_KEY != 1)
	{
		motorRes->StretchMotor[RIGHT_MOTOR].SpeedRef = -2000;
				motorRes->StretchMotor[RIGHT_MOTOR].ResFlag = 0;

	}
	if(R_STRETCH_KEY == 1)
	{
		motorRes->StretchMotor[RIGHT_MOTOR].SpeedRef = 0;
		motorRes->StretchMotor[RIGHT_MOTOR].CenterOffset = motorRes->StretchMotor[RIGHT_MOTOR].motor_measure->real_ecd;
		motorRes->StretchMotor[RIGHT_MOTOR].ResFlag = 1;
	}

	if(TOP_PITCH_KEY != 1)
	{
			motorRes->TopPitchMotor.AngleRef = motorRes->TopPitchMotor.AngleRef+1;//逐渐累加到微动开关
			motorRes->TopPitchMotor.ResFlag = 0;
	}
	if(TOP_PITCH_KEY == 1)
	{	
		motorRes->TopPitchMotor.CenterOffset=motorRes->TopPitchMotor.AngleFdb;
		motorRes->TopPitchMotor.AngleRef = motorRes->TopPitchMotor.CenterOffset;
		motorRes->TopPitchMotor.ResFlag = 1;
	}	
	

/*******************************************************/		
//	if((motorRes->UpMotor[LEFT_MOTOR].ResFlag) && (motorRes->UpMotor[RIGHT_MOTOR].ResFlag) && \
//		 (motorRes->StretchMotor[LEFT_MOTOR].ResFlag) && (motorRes->StretchMotor[RIGHT_MOTOR].ResFlag) && \
//		 (motorRes->TopMoveMotor[LEFT_MOTOR].ResFlag) && (motorRes->TopMoveMotor[RIGHT_MOTOR].ResFlag) && \
//		 (motorRes->RescueMotor[LEFT_MOTOR].ResFlag) && (motorRes->RescueMotor[RIGHT_MOTOR].ResFlag) &&  
//		 (motorRes->TopPitchMotor.ResFlag) && (motorRes->TopRollMotor.ResFlag))
	
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
	if (rc_ctrl.rc.ch[0] == -660 && rc_ctrl.rc.ch[1] == -660 && rc_ctrl.rc.ch[2] == 660 && rc_ctrl.rc.ch[3] == -660) //拨杆内八软件强制复位
	{
			__set_FAULTMASK(1);//关闭所有中断
			NVIC_SystemReset();//软件复位
		Motor_Reset(ModeSet);
	}
	if(switch_is_down(ModeSet->Input_Point->rc.s[ModeChannel_L]) &&switch_is_down(ModeSet->Input_Point->rc.s[ModeChannel_R]))//无控制模式
	{
		ModeSet->ctrlMode = CTRL_NULL;
	}
	else if(switch_is_mid(ModeSet->Input_Point->rc.s[ModeChannel_R]))//右中，遥控器模式
	{
		ModeSet->ctrlMode = CTRL_RC;
	}
	else if( switch_is_mid(ModeSet->Input_Point->rc.s[ModeChannel_L])&& switch_is_up(ModeSet->Input_Point->rc.s[ModeChannel_R]))//左中右上 PC模式
	{
		ModeSet->ctrlMode = CTRL_PC;
	}
	else if(switch_is_up(ModeSet->Input_Point->rc.s[ModeChannel_L]) && switch_is_up(ModeSet->Input_Point->rc.s[ModeChannel_R]))//左上右上，微调模式
	{
		pid_flag=0;
		ModeSet->ctrlMode = CTRL_FINETUNING;
	}
	else if(switch_is_mid(ModeSet->Input_Point->rc.s[ModeChannel_L]) && switch_is_down(ModeSet->Input_Point->rc.s[ModeChannel_R]))//左中右下，复位
	{
	ModeSet->ctrlMode = CTRL_RC;//更改为遥控器模式
	pid_flag=0;
	Motor_Reset(ModeSet);
	}
	else
	{
		;
	}
	

	
}


/**
  * @brief  搞钱工作设置
  * @param  云台数据结构体指针
  * @retval void
  */
int GOLD_flag=0,GLOD_state=0;
int Silver_flag=0,Silver_state=0;
int EXCHANGE_flag=0,EXCHANGE_state=0;
int CATCH_flag=0,CATCH_state=0;
uint8_t e_Flag = 1, 
			  r_Flag = 1, 
				q_Flag = 1,
		   	b_Flag = 1,
			  c_Flag = 1;
static void makeMoney_Work_Set(ControlTask_t* stateSet)
{
	if(stateSet == NULL)
	{
		return;
	}
	/************************************左上右中通道4上拨取金矿**************************************************/
	if (switch_is_up(stateSet->Input_Point->rc.s[ModeChannel_L])&&switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_R]) &&stateSet->Input_Point->rc.ch[4] < -600 \
		&&(stateSet->workStste == NONE || stateSet->workStste == GOLD))
	{
	GOLD_flag=1;
	}
	if(GOLD_flag==1&&(stateSet->Input_Point->rc.ch[4]>-10&&stateSet->Input_Point->rc.ch[4]<10))
	{
		GOLD_flag=0;
    GLOD_state=1;
	}
	
		/************************************左上右中通道4下拨取银矿**************************************************/
	if (switch_is_up(stateSet->Input_Point->rc.s[ModeChannel_L])&&switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_R]) &&stateSet->Input_Point->rc.ch[4] > 600 \
		 &&(stateSet->workStste == NONE || stateSet->workStste ==SILVER ))
	{
	Silver_flag=1;
		
}
	if(Silver_flag==1&&(stateSet->Input_Point->rc.ch[4]>-10&&stateSet->Input_Point->rc.ch[4]<10))
	{
		Silver_flag=0;
	Silver_state=1;
	}	
	
	/************************************左中右中通道4上拨兑换矿石**************************************************/
	if (switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_L])&&switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_R]) &&stateSet->Input_Point->rc.ch[4] < -600 \
		&&(stateSet->workStste == NONE || stateSet->workStste == EXCHANGE))
	{
	EXCHANGE_flag=1;
	
}
		if(EXCHANGE_flag==1&&(stateSet->Input_Point->rc.ch[4]>-10&&stateSet->Input_Point->rc.ch[4]<10))
	{
		EXCHANGE_flag=0;
	EXCHANGE_state=1;
	}
	/************************************左中右中通道4下拨空接**************************************************/
	
	if (switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_L])&&switch_is_mid(stateSet->Input_Point->rc.s[ModeChannel_R]) &&stateSet->Input_Point->rc.ch[4]  > 600 \
		 &&(stateSet->workStste == NONE || stateSet->workStste ==CATCH))
	{
	CATCH_flag=1;
}
		if(CATCH_flag==1&&(stateSet->Input_Point->rc.ch[4]>-10&&stateSet->Input_Point->rc.ch[4]<10))
	{
		CATCH_flag=0;
	CATCH_state=1;
	}


	switch(stateSet->ctrlMode)
	{
		case CTRL_NULL:
			{
				break;
			}			
		case CTRL_RC:
			{

				if(GLOD_state==1)
					{ 
						pid_flag=1;
						
						stateSet->makeMoney = ORE_GOLD;
						
						Get_Ore_Gold(stateSet);
						GLOD_state=0;
						vTaskDelay(100);
					}
					
						if(Silver_state==1)
					{ 
						pid_flag=1;
						
						stateSet->makeMoney = ORE_SILVER;
						
						Get_Ore_Silver(stateSet);
						Silver_state=0;
						vTaskDelay(100);
					}
					if(EXCHANGE_state==1)
					{ 
						pid_flag=1;
						
						stateSet->makeMoney =ORE_EXCHANGE;
						
						Exchange_Ore(stateSet);
						EXCHANGE_state=0;
						vTaskDelay(100);
					}
					
					if(CATCH_state==1)
					{ 
						pid_flag=1;
						
						stateSet->makeMoney = ORE_CATCH;
						
						Catch_Ore_Gold(stateSet);
						CATCH_state=0;
						vTaskDelay(100);
					}
					
					
			
				else
					{

					}
      	break;
		  }		
		case CTRL_PC:
			{
				   
				if(IF_KEY_PRESSED_SHIFT  )//SHIFT+E取金矿
				{
				if(IF_KEY_PRESSED_E)
					{
					
						e_Flag = 0;
					}
			
				if((!IF_KEY_PRESSED_E && e_Flag==0) && (stateSet->workStste == NONE || stateSet->workStste == GOLD))
					{
						
						
           pid_flag=1;
						stateSet->makeMoney = ORE_GOLD;
						Get_Ore_Gold(stateSet);
						vTaskDelay(100);
				
				  e_Flag = 1;
					q_Flag = 1;
					b_Flag = 1;
					r_Flag = 1;
					}
		
				if(IF_KEY_PRESSED_Q)//SHIFT+Q取银矿
					{
						q_Flag = 0;
					}
			
				if((!IF_KEY_PRESSED_Q && q_Flag==0) && (stateSet->workStste == NONE || stateSet->workStste == SILVER))
					{
							
            pid_flag=1;
						stateSet->makeMoney = ORE_SILVER;
				  	Get_Ore_Silver(stateSet);
						vTaskDelay(100);
				    e_Flag = 1;
					  q_Flag = 1;
					  b_Flag = 1;
					  r_Flag = 1;
					}	
					
					
					if(IF_KEY_PRESSED_R)//SHIFT+R兑换矿石
					{
						r_Flag = 0;
					}
			
				if((!IF_KEY_PRESSED_R && r_Flag==0) && (stateSet->workStste == NONE || stateSet->workStste == EXCHANGE))
					{
							
            pid_flag=1;
						stateSet->makeMoney = ORE_EXCHANGE;
    				Exchange_Ore(stateSet);
						vTaskDelay(100);
				    e_Flag = 1;
					  q_Flag = 1;
					  b_Flag = 1;
					  r_Flag = 1;
					}	
					
					
					
						if(IF_KEY_PRESSED_B)//SHIFT+B空接
					{
					
						b_Flag = 0;
					}
			
				if((!IF_KEY_PRESSED_B && b_Flag==0) && (stateSet->workStste == NONE || stateSet->workStste ==  CATCH))
					{
							
            pid_flag=1;
						stateSet->makeMoney = ORE_CATCH;
						Catch_Ore_Gold(stateSet);
						vTaskDelay(100);
				    e_Flag = 1;
					  q_Flag = 1;
					  b_Flag = 1;
					  r_Flag = 1;
					}	
					
		
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
//	Motor_Constrain(stateSet);//限幅
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
	
	FdbSet->TopPitchMotor.AngleFdb = FdbSet->TopPitchMotor.motor_measure->real_ecd;
	FdbSet->TopPitchMotor.SpeedFdb = FdbSet->TopPitchMotor.motor_measure->speed_rpm;
	
	FdbSet->TopRollMotor.AngleFdb = FdbSet->TopRollMotor.motor_measure->real_ecd;
	FdbSet->TopRollMotor.SpeedFdb = FdbSet->TopRollMotor.motor_measure->speed_rpm;
	
	for(i = 0; i < 2; i++)
	{
		FdbSet->UpMotor[i].AngleFdb = FdbSet->UpMotor[i].motor_measure->real_ecd;
		FdbSet->UpMotor[i].SpeedFdb = FdbSet->UpMotor[i].motor_measure->speed_rpm;
		
		FdbSet->StretchMotor[i].AngleFdb = FdbSet->StretchMotor[i].motor_measure->real_ecd;
		FdbSet->StretchMotor[i].SpeedFdb = FdbSet->StretchMotor[i].motor_measure->speed_rpm;
		
		FdbSet->TopMoveMotor[i].AngleFdb = FdbSet->TopMoveMotor[i].motor_measure->real_ecd;
		FdbSet->TopMoveMotor[i].SpeedFdb = FdbSet->TopMoveMotor[i].motor_measure->speed_rpm;
		
		FdbSet->RescueMotor[i].AngleFdb = FdbSet->RescueMotor[i].motor_measure->real_ecd;
		FdbSet->RescueMotor[i].SpeedFdb = FdbSet->RescueMotor[i].motor_measure->speed_rpm;
	}
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
	buffRefresh->LastmakeMoney = buffRefresh->makeMoney;
	buffRefresh->LastworkStste = buffRefresh->workStste;
}

/**
  * @brief  平时状态
  * @param  数据结构体指针
  * @retval void
  */
static void Do_Nothing(ControlTask_t* doNothing)
{
	if(doNothing == NULL)
	{
		return ;
	}

	doNothing->workStste = NONE;

	doNothing->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[LOWEST];
	doNothing->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[LOWEST];
	
	doNothing->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[SHORTEST];
	doNothing->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[SHORTEST];
	
	doNothing->TopMoveMotor[LEFT_MOTOR].AngleRef = L_TopMove_Appoint[FURTHEST];
	doNothing->TopMoveMotor[RIGHT_MOTOR].AngleRef = R_TopMove_Appoint[FURTHEST];
	
	doNothing->RescueMotor[LEFT_MOTOR].AngleRef = L_Rescue_Appoint[OFF_POS];
	doNothing->RescueMotor[RIGHT_MOTOR].AngleRef = R_Rescue_Appoint[OFF_POS];
	
	doNothing->TopPitchMotor.AngleRef = TopPitch_Appoint[FRONT];
	
	doNothing->TopRollMotor.AngleRef = TopRoll_Appoint[RES];
	
	doNothing->VacuumFlag = OFF;
}


/**
  * @brief  获取金矿石
  * @param  数据结构体指针
  * @retval void
  */


static void Get_Ore_Gold(ControlTask_t* getGold)
{
	if(getGold == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	if(getGold->workStste == NONE )//复位后过程标志位清零
	{
	process_Flag = 0;
	}
		getGold->workStste = GOLD;

	if((getGold->workStste == NONE || getGold->workStste == GOLD))//防止两个同级一键模式中出现冲突
	{
	if(process_Flag == 0)
	{
    getGold->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[UP_GOLD];
		getGold->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[UP_GOLD];
		getGold->TopPitchMotor.AngleRef = TopPitch_Appoint[FRONT];
		
	}
		else if(process_Flag == 1)
			
	{
		getGold->VacuumFlag = ON;
		getGold->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[STRETCH_GOLD];
		getGold->StretchMotor[RIGHT_MOTOR].AngleRef =R_Stretch_Appoint[STRETCH_GOLD];
	
	}
	else if(process_Flag == 2)
	{

		getGold->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[HIGHEST];
		getGold->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[HIGHEST];
	
	}
	else if(process_Flag == 3)
	{
		
		getGold->TopPitchMotor.AngleRef = TopPitch_Appoint[UP];
	}
	else if(process_Flag == 4)
	{
	  getGold->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[SHORTEST];
		getGold->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[SHORTEST];
	}
	
	else if(process_Flag == 5)
	{
		
		getGold->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[LOWEST];
		getGold->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[LOWEST];
		
	
	}
//	vTaskDelay(50);
	process_Flag ++;
	if(process_Flag >= 6)
	{	
		process_Flag = 0;
		getGold->makeMoney=DO_NOTHING;
		getGold->workStste = NONE;
	}
}
}
/**
  * @brief  获取银矿石
  * @param  数据结构体指针
  * @retval void
  */
static void Get_Ore_Silver(ControlTask_t* getSilver)
{
	if(getSilver == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
		if(getSilver->workStste == NONE )//复位后过程标志位清零
	{
	process_Flag = 0;
	}
	getSilver->workStste = SILVER;
	if(getSilver->workStste == NONE || getSilver->workStste == SILVER)//防止两个同级一键模式中出现冲突
	{
		if(process_Flag == 0)
		{ 
			
			
			getSilver->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[UP_SILVER];
			getSilver->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[UP_SILVER];		
				
			getSilver->VacuumFlag = ON;
		}
		else if(process_Flag == 1)
		{
		getSilver->TopPitchMotor.AngleRef = TopPitch_Appoint[DOWN];	
		
		}
		else if(process_Flag == 2)
		{
			getSilver->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[STRETCH_SILVER];
			getSilver->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[STRETCH_SILVER];
		
		}
		else if(process_Flag == 3)
		{
		  getSilver->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[HIGHEST];
			getSilver->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[HIGHEST];
		}
		else if(process_Flag == 4)
		{

			getSilver->TopPitchMotor.AngleRef = TopPitch_Appoint[FRONT];
		}

		else if(process_Flag == 5)
		{
			getSilver->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[SHORTEST];
			getSilver->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[SHORTEST];

		}
		else if(process_Flag == 6)
		{
      getSilver->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[UP_SILVER];
			getSilver->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[UP_SILVER];	

		}

	
		
		else if(process_Flag == 7)
		{
	
	   getSilver->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[HIGHEST];
			getSilver->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[HIGHEST];	
		}
		
		else if(process_Flag == 8)
		{
	getSilver->TopPitchMotor.AngleRef = TopPitch_Appoint[UP];
	 
		}
		
	 
   else if(process_Flag == 9)
		{
		getSilver->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[SHORTEST];
		getSilver->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[SHORTEST];
		}		
		 else if(process_Flag == 10)
		{
	    getSilver->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[LOWEST];
			getSilver->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[LOWEST];	
		}		
		
		process_Flag ++;
		
		if(process_Flag >= 11)
		{
			process_Flag = 0;
			
	  	getSilver->makeMoney=DO_NOTHING;
			getSilver->workStste = NONE;
		}
  }
}


/**
  * @brief  空接
  * @param  数据结构体指针
  * @retval void
  */
int gg=0;
static void Catch_Ore_Gold(ControlTask_t* CatchGold)
{
	if(CatchGold == NULL)
	{
		return ;
	}
	static u8 process_Flag = 0;
	if(CatchGold->workStste == NONE )//复位后过程标志位清零
	{
	process_Flag = 0;
	}
	CatchGold->workStste = CATCH;
	if((CatchGold->workStste == NONE || CatchGold->workStste == CATCH))//防止两个同级一键模式中出现冲突
	{
	if(process_Flag == 0)
	{
		gg=1;
		CatchGold->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[HIGHEST];
		CatchGold->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[HIGHEST];
		
		CatchGold->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[LONGEST];
		CatchGold->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[LONGEST];
		
		CatchGold->TopPitchMotor.AngleRef = TopPitch_Appoint[UP];
		
		CatchGold->VacuumFlag = ON;
	}
	if(process_Flag == 1)
	{

		CatchGold->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[SHORTEST];
		CatchGold->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[SHORTEST];
		
	}
		if(process_Flag == 2)
	{
		CatchGold->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[LOWEST];
		CatchGold->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[LOWEST];
		
		CatchGold->TopMoveMotor[LEFT_MOTOR].AngleRef = L_TopMove_Appoint[CLOSEST];
		CatchGold->TopMoveMotor[RIGHT_MOTOR].AngleRef = R_TopMove_Appoint[CLOSEST];
		
	}

	process_Flag ++;
	if(process_Flag >= 3)
	{
		process_Flag = 0;
   
		CatchGold->makeMoney=DO_NOTHING;/*******/
CatchGold->workStste = NONE;
	}
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
	if(exchangeOre->workStste == NONE )//复位后过程标志位清零
	{
	process_Flag = 0;
	}
	exchangeOre->workStste = EXCHANGE;
	if((exchangeOre->workStste == NONE || exchangeOre->workStste == EXCHANGE))//防止两个同级一键模式中出现冲突
	{
	if(process_Flag == 0)
	{
		
		exchangeOre->VacuumFlag = ON;
		
		exchangeOre->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[UP_EXCHANGE];
		exchangeOre->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[UP_EXCHANGE];
		
		exchangeOre->TopMoveMotor[LEFT_MOTOR].AngleRef = L_TopMove_Appoint[CLOSEST];
		exchangeOre->TopMoveMotor[RIGHT_MOTOR].AngleRef = R_TopMove_Appoint[CLOSEST];
		
		exchangeOre->TopPitchMotor.AngleRef = TopPitch_Appoint[FRONT];
		
	}
	if(process_Flag == 1)
	{
		exchangeOre->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[STRETCH_EXCHANGE];
		exchangeOre->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[STRETCH_EXCHANGE];
		
		exchangeOre->TopMoveMotor[LEFT_MOTOR].AngleRef = L_TopMove_Appoint[FURTHEST];
		exchangeOre->TopMoveMotor[RIGHT_MOTOR].AngleRef = R_TopMove_Appoint[FURTHEST];
		
	}
	if(process_Flag == 2)
	{
		exchangeOre->VacuumFlag = OFF;
	}
	if(process_Flag == 3)
	{

		
		exchangeOre->StretchMotor[LEFT_MOTOR].AngleRef = L_Stretch_Appoint[SHORTEST];
		exchangeOre->StretchMotor[RIGHT_MOTOR].AngleRef = R_Stretch_Appoint[SHORTEST];
		
	}
		if(process_Flag == 4)
	{
		exchangeOre->UpMotor[LEFT_MOTOR].AngleRef = L_Up_Appoint[LOWEST];
		exchangeOre->UpMotor[RIGHT_MOTOR].AngleRef = R_Up_Appoint[LOWEST];
		
	
	}
		
	  process_Flag ++;
	if(process_Flag >= 5)
	{
		process_Flag = 0;
		
		exchangeOre->makeMoney=DO_NOTHING;/*******/
		exchangeOre->workStste = NONE;
	}
}

}

/**
  * @brief  执行机构微调控制
  * @param  数据结构体指针
  * @retval void
  */
static void Fine_Tuning(ControlTask_t* fineTuning)
{
	if(fineTuning == NULL)
	{
		return ;
	}

	if(IF_MOUSE_PRESSED_LEFT)
	{
		fineTuning->UpMotor[LEFT_MOTOR].AngleRef += fineTuning->Input_Point->mouse.z * UP_FINETUNING_RATIO;
		fineTuning->UpMotor[RIGHT_MOTOR].AngleRef -= fineTuning->Input_Point->mouse.z * UP_FINETUNING_RATIO;
		fineTuning->TopPitchMotor.AngleRef += fineTuning->Input_Point->mouse.x * TOP_PITCH_FINETUNING_RATIO;
		fineTuning->TopRollMotor.AngleRef += fineTuning->Input_Point->mouse.y * TOP_ROLL_FINETUNING_RATIO;
	}
	if(IF_MOUSE_PRESSED_RIGH)
	{
		fineTuning->StretchMotor[LEFT_MOTOR].AngleRef += fineTuning->Input_Point->mouse.z * STRETCH_FINETUNING_RATIO;
		fineTuning->StretchMotor[RIGHT_MOTOR].AngleRef -= fineTuning->Input_Point->mouse.z * STRETCH_FINETUNING_RATIO;
	}
	
}


/**
  * @brief  电机限幅
  * @param  数据结构体指针
  * @retval void
  */
static void Motor_Constrain(ControlTask_t* Constrain)
{
	if(Constrain == NULL)
	{
		return ;
	}
	Constrain->UpMotor[LEFT_MOTOR].AngleRef = Constrain_float(Constrain->UpMotor[LEFT_MOTOR].AngleRef, L_Up_Appoint[LOWEST], L_Up_Appoint[HIGHEST]);
	Constrain->UpMotor[RIGHT_MOTOR].AngleRef = Constrain_float(Constrain->UpMotor[RIGHT_MOTOR].AngleRef, R_Up_Appoint[LOWEST], R_Up_Appoint[HIGHEST]);
	Constrain->TopPitchMotor.AngleRef = Constrain_float(Constrain->TopPitchMotor.AngleRef, TopPitch_Appoint[DOWN], TopPitch_Appoint[BACK]);
	Constrain->TopRollMotor.AngleRef = Constrain_float(Constrain->TopRollMotor.AngleRef, TopRoll_Appoint[RES] - (45 * REDUCTION_RATIO_2006), TopRoll_Appoint[RES] + (45 * REDUCTION_RATIO_2006));
	Constrain->StretchMotor[LEFT_MOTOR].AngleRef = Constrain_float(Constrain->StretchMotor[LEFT_MOTOR].AngleRef, L_Stretch_Appoint[SHORTEST], L_Stretch_Appoint[LONGEST]);
	Constrain->StretchMotor[RIGHT_MOTOR].AngleRef = Constrain_float(Constrain->StretchMotor[RIGHT_MOTOR].AngleRef, R_Stretch_Appoint[SHORTEST], R_Stretch_Appoint[LONGEST]);
}


/**
* @brief  得到发送电流值
  * @param  底盘数据结构体指针
  * @retval void
  */
extern int s_pidflag;
static void give_current(ControlTask_t* giveCurrent)
{
	uint8_t i = 0;
	
	if (giveCurrent == NULL)
	{
		 return;
	}
	
 /***************************得到吸盘pitch轴电流值************************************/
		New_PID_Calc(&giveCurrent->TopPitchMotor.AnglePid, 
									giveCurrent->TopPitchMotor.AngleFdb,  
									giveCurrent->TopPitchMotor.AngleRef);
		New_PID_Calc(&giveCurrent->TopPitchMotor.SpeedPid, 
									giveCurrent->TopPitchMotor.SpeedFdb,  
									giveCurrent->TopPitchMotor.AnglePid.out);
		giveCurrent->TopPitchMotor.Current = giveCurrent->TopPitchMotor.SpeedPid.out;
		
	 /***************************得到吸盘roll轴电流值************************************/	
		New_PID_Calc(&giveCurrent->TopRollMotor.AnglePid, 
									giveCurrent->TopRollMotor.AngleFdb,  
									giveCurrent->TopRollMotor.AngleRef);
		New_PID_Calc(&giveCurrent->TopRollMotor.SpeedPid, 
									giveCurrent->TopRollMotor.SpeedFdb,  
									giveCurrent->TopRollMotor.AnglePid.out);
		giveCurrent->TopRollMotor.Current = giveCurrent->TopRollMotor.SpeedPid.out;
		
		for(i = 0; i < 2; i++)
		{
	 /***************************得到抬升机构电流值************************************/	
			if(	pid_flag==1)
{
			New_PID_Calc(&giveCurrent->UpMotor[i].AnglePid, 
										giveCurrent->UpMotor[i].AngleFdb,  
										giveCurrent->UpMotor[i].AngleRef);
			New_PID_Calc(&giveCurrent->UpMotor[i].SpeedPid,
										giveCurrent->UpMotor[i].SpeedFdb,
										giveCurrent->UpMotor[i].AnglePid.out);
}
else{
			New_PID_Calc(&giveCurrent->UpMotor[i].SpeedPid,
										giveCurrent->UpMotor[i].SpeedFdb,
										giveCurrent->UpMotor[i].SpeedRef);
	
}
		giveCurrent->UpMotor[i].Current = giveCurrent->UpMotor[i].SpeedPid.out;
	 /***************************得到前伸机构电流值************************************/	
if(	pid_flag==1)
{
			New_PID_Calc(&giveCurrent->StretchMotor[i].AnglePid, 
										giveCurrent->StretchMotor[i].AngleFdb,  
										giveCurrent->StretchMotor[i].AngleRef);
			New_PID_Calc(&giveCurrent->StretchMotor[i].SpeedPid,
										giveCurrent->StretchMotor[i].SpeedFdb,
										giveCurrent->StretchMotor[i].AnglePid.out);
}
else
{
			New_PID_Calc(&giveCurrent->StretchMotor[i].SpeedPid,
										giveCurrent->StretchMotor[i].SpeedFdb,
										giveCurrent->StretchMotor[i].SpeedRef);
}
			giveCurrent->StretchMotor[i].Current = giveCurrent->StretchMotor[i].SpeedPid.out;	

	 /***************************得到吸盘移动机构电流值************************************/	
			New_PID_Calc(&giveCurrent->TopMoveMotor[i].AnglePid, 
										giveCurrent->TopMoveMotor[i].AngleFdb,  
										giveCurrent->TopMoveMotor[i].AngleRef);
			New_PID_Calc(&giveCurrent->TopMoveMotor[i].SpeedPid,
										giveCurrent->TopMoveMotor[i].SpeedFdb,
										giveCurrent->TopMoveMotor[i].AnglePid.out);
			giveCurrent->TopMoveMotor[i].Current = giveCurrent->TopMoveMotor[i].SpeedPid.out;			
	 }
}


	


/**
  * @brief  返回结构体变量地址
  * @param  void
  * @retval const ControlTask_t*
  * @attention 
  */
const ControlTask_t* get_Makemoney_Point(void)
{
	return &makeMoney;
}

