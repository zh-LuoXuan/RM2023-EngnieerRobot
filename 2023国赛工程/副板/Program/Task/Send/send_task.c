#include "send_task.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "gpio.h" 


/*==============================================================*/
#define SEND_TASK_PRIO 31
#define SEND_STK_SIZE 1024
TaskHandle_t SendTask_Handler;
void send_task(void);

/*==============================================================*/

void task_Send_Create(void)
{
	xTaskCreate((TaskFunction_t)send_task,
                (const char *)"send_task",
                (uint16_t)SEND_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SEND_TASK_PRIO,
                (TaskHandle_t *)&SendTask_Handler);
}

SendData_t sendData;

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


static void Sent_Init(SendData_t* init);
static void give_current(SendData_t* giveCurrent);
static void CanSendData(SendData_t* send);


void send_task(void)
{
	static portTickType TickCount; 
	
	TickCount = xTaskGetTickCount();
	
	Sent_Init(&sendData);
	while(1)
	{
		give_current(&sendData);
		CanSendData(&sendData);
		
		vTaskDelayUntil(&TickCount, 1);
	}
}


/**
  * @brief  初始化
  * @param  数据结构体指针
  * @retval void
  */
static void Sent_Init(SendData_t* init)
{
	if(init == NULL)
	{
		return;
	}
  uint8_t i = 0;
	
/***************************获取数据指针************************************/
	init->makeMoney_Point = get_Makemoney_Point();

/*************************吸盘pitch轴PID初始化************************************/
	New_PID_Init(&init->topPitchAnglePid, 
	             NEW_PID_POSITION, 
	             topPitchAgePid, 
	             TOP_PITCH_ANGLE_PID_MAX_OUT, 
	             TOP_PITCH_ANGLE_PID_MAX_IOUT, 
               TOP_PITCH_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             TOP_PITCH_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&init->topPitchSpeedPid, 
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
	New_PID_Init(&init->topRollAnglePid, 
	             NEW_PID_POSITION, 
	             topRollAgePid, 
	             TOP_ROLL_ANGLE_PID_MAX_OUT, 
	             TOP_ROLL_ANGLE_PID_MAX_IOUT, 
               TOP_ROLL_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             TOP_ROLL_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&init->topRollSpeedPid, 
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
			New_PID_Init(&init->upAnglePid[i], 
									 NEW_PID_POSITION, 
									 upAgePid, 
									 UP_ANGLE_PID_MAX_OUT, 
									 UP_ANGLE_PID_MAX_IOUT, 
									 UP_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 UP_ANGLE_PID_I_SEPARATION);
			
			New_PID_Init(&init->upSpeedPid[i], 
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
			New_PID_Init(&init->stretchAnglePid[i], 
									 NEW_PID_POSITION, 
									 stretchAgePid, 
									 STRETCH_ANGLE_PID_MAX_OUT, 
									 STRETCH_ANGLE_PID_MAX_IOUT, 
									 STRETCH_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 STRETCH_ANGLE_PID_I_SEPARATION);
			
			New_PID_Init(&init->stretchSpeedPid[i], 
									 NEW_PID_POSITION, 
									 stretchSpdPid, 
									 STRETCH_SPEED_PID_MAX_OUT, 
									 STRETCH_SPEED_PID_MAX_IOUT, 
									 STRETCH_SPEED_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 STRETCH_SPEED_PID_I_SEPARATION);
									 
/*************************救援PID初始化************************************/
			New_PID_Init(&init->rescueAnglePid[i], 
									 NEW_PID_POSITION, 
									 rescueAgePid, 
									 RESCUE_ANGLE_PID_MAX_OUT, 
									 RESCUE_ANGLE_PID_MAX_IOUT, 
									 RESCUE_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 RESCUE_ANGLE_PID_I_SEPARATION);
			
			New_PID_Init(&init->rescueSpeedPid[i], 
									 NEW_PID_POSITION, 
									 rescueSpdPid, 
									 RESCUE_SPEED_PID_MAX_OUT, 
									 RESCUE_SPEED_PID_MAX_IOUT, 
									 RESCUE_SPEED_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 RESCUE_SPEED_PID_I_SEPARATION);
									 
/*************************吸盘移动PID初始化************************************/
			New_PID_Init(&init->topMoveAnglePid[i], 
									 NEW_PID_POSITION, 
									 topMoveAgePid, 
									 TOPMOVE_ANGLE_PID_MAX_OUT, 
									 TOPMOVE_ANGLE_PID_MAX_IOUT, 
									 TOPMOVE_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 TOPMOVE_ANGLE_PID_I_SEPARATION);
			
			New_PID_Init(&init->topMoveSpeedPid[i], 
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
}
/**
* @brief  得到发送电流值
  * @param  底盘数据结构体指针
  * @retval void
  */
static void give_current(SendData_t* giveCurrent)
{
	uint8_t i = 0;
	
	if (giveCurrent == NULL)
	{
		 return;
	}
//	if(giveCurrent->makeMoney_Point->makeMoney == DO_RESET)
//	{
//	 /***************************得到吸盘pitch轴电流值************************************/
//		New_PID_Calc(&giveCurrent->topPitchSpeedPid, 
//	              giveCurrent->makeMoney_Point->TopPitchMotor.SpeedFdb,  
//	              giveCurrent->makeMoney_Point->TopPitchMotor.SpeedRef);
//	  giveCurrent->topPitchCurrent = giveCurrent->topPitchSpeedPid.out;
//		
//	 /***************************得到吸盘roll轴电流值************************************/	
//		New_PID_Calc(&giveCurrent->topRollSpeedPid, 
//									giveCurrent->makeMoney_Point->TopRollMotor.SpeedFdb,  
//									giveCurrent->makeMoney_Point->TopRollMotor.SpeedRef);
//		giveCurrent->topRollCurrent = giveCurrent->topRollSpeedPid.out;
//		
//		for(i = 0; i < 2; i++)
//		{
//	 /***************************得到抬升机构电流值************************************/	
//			New_PID_Calc(&giveCurrent->upSpeedPid[i],
//										giveCurrent->makeMoney_Point->UpMotor[i].SpeedFdb,
//										giveCurrent->makeMoney_Point->UpMotor[i].SpeedRef);
//			giveCurrent->upCurrent[i] = giveCurrent->upSpeedPid[i].out;
//			
//	 /***************************得到前伸机构电流值************************************/	
//			New_PID_Calc(&giveCurrent->stretchSpeedPid[i],
//										giveCurrent->makeMoney_Point->StretchMotor[i].SpeedFdb,
//										giveCurrent->makeMoney_Point->StretchMotor[i].SpeedRef);
//			giveCurrent->stretchCurrent[i] = giveCurrent->stretchSpeedPid[i].out;		

//	/***************************得到救援机构电流值************************************/	
//			New_PID_Calc(&giveCurrent->rescueSpeedPid[i],
//										giveCurrent->makeMoney_Point->RescueMotor[i].SpeedFdb,
//										giveCurrent->makeMoney_Point->RescueMotor[i].SpeedRef);
//			giveCurrent->rescueCurrent[i] = giveCurrent->rescueSpeedPid[i].out;
//			
//	 /***************************得到吸盘移动机构电流值************************************/	
//			New_PID_Calc(&giveCurrent->topMoveSpeedPid[i],
//										giveCurrent->makeMoney_Point->TopMoveMotor[i].SpeedFdb,
//										giveCurrent->makeMoney_Point->TopMoveMotor[i].SpeedRef);
//			giveCurrent->topMoveCurrent[i] = giveCurrent->topMoveSpeedPid[i].out;		
//			
//		}
//	}
//	else
	{
	 /***************************得到吸盘pitch轴电流值************************************/	
		New_PID_Calc(&giveCurrent->topPitchAnglePid, 
									giveCurrent->makeMoney_Point->TopPitchMotor.AngleFdb,  
									giveCurrent->makeMoney_Point->TopPitchMotor.AngleRef);
		New_PID_Calc(&giveCurrent->topPitchSpeedPid, 
									giveCurrent->makeMoney_Point->TopPitchMotor.SpeedFdb,  
									giveCurrent->topPitchAnglePid.out);
		giveCurrent->topPitchCurrent = giveCurrent->topPitchSpeedPid.out;
		
	 /***************************得到吸盘roll轴电流值************************************/	
		New_PID_Calc(&giveCurrent->topRollAnglePid, 
									giveCurrent->makeMoney_Point->TopRollMotor.AngleFdb,  
									giveCurrent->makeMoney_Point->TopRollMotor.AngleRef);
		New_PID_Calc(&giveCurrent->topRollSpeedPid, 
									giveCurrent->makeMoney_Point->TopRollMotor.SpeedFdb,  
									giveCurrent->topRollAnglePid.out);
		giveCurrent->topRollCurrent = giveCurrent->topRollSpeedPid.out;
		

		for(i = 0; i < 2; i++)
		{
	 /***************************得到抬升机构电流值************************************/	
			New_PID_Calc(&giveCurrent->upAnglePid[i], 
										giveCurrent->makeMoney_Point->UpMotor[i].AngleFdb,  
										giveCurrent->makeMoney_Point->UpMotor[i].AngleRef);
			New_PID_Calc(&giveCurrent->upSpeedPid[i],
										giveCurrent->makeMoney_Point->UpMotor[i].SpeedFdb,
										giveCurrent->upAnglePid[i].out);
			giveCurrent->upCurrent[i] = giveCurrent->upSpeedPid[i].out;
			
	 /***************************得到前伸机构电流值************************************/	
			New_PID_Calc(&giveCurrent->stretchAnglePid[i], 
										giveCurrent->makeMoney_Point->StretchMotor[i].AngleFdb,  
										giveCurrent->makeMoney_Point->StretchMotor[i].AngleRef);
			New_PID_Calc(&giveCurrent->stretchSpeedPid[i],
										giveCurrent->makeMoney_Point->StretchMotor[i].SpeedFdb,
										giveCurrent->stretchAnglePid[i].out);
			giveCurrent->stretchCurrent[i] = giveCurrent->stretchSpeedPid[i].out;		

	/***************************得到救援机构电流值************************************/	
			New_PID_Calc(&giveCurrent->rescueAnglePid[i], 
										giveCurrent->makeMoney_Point->RescueMotor[i].AngleFdb,  
										giveCurrent->makeMoney_Point->RescueMotor[i].AngleRef);
			New_PID_Calc(&giveCurrent->rescueSpeedPid[i],
										giveCurrent->makeMoney_Point->RescueMotor[i].SpeedFdb,
										giveCurrent->rescueAnglePid[i].out);
			giveCurrent->rescueCurrent[i] = giveCurrent->rescueSpeedPid[i].out;
			
	 /***************************得到吸盘移动机构电流值************************************/	
			New_PID_Calc(&giveCurrent->topMoveAnglePid[i], 
										giveCurrent->makeMoney_Point->TopMoveMotor[i].AngleFdb,  
										giveCurrent->makeMoney_Point->TopMoveMotor[i].AngleRef);
			New_PID_Calc(&giveCurrent->topMoveSpeedPid[i],
										giveCurrent->makeMoney_Point->TopMoveMotor[i].SpeedFdb,
										giveCurrent->topMoveAnglePid[i].out);
			giveCurrent->topMoveCurrent[i] = giveCurrent->topMoveSpeedPid[i].out;		
			
		}
	}


}

/**
* @brief  CAN发送
  * @param  数据结构体指针
  * @retval void
  */
static void CanSendData(SendData_t* send)
{
	CAN1_CMD_UP(send->upCurrent[0], send->upCurrent[1]);
	CAN1_CMD_TOPMOVE(send->topMoveCurrent[0], send->topMoveCurrent[1]);
	CAN1_CMD_TOP_PITCH(send->topPitchCurrent);
	CAN1_CMD_TOP_ROLL(send->topRollCurrent);
	CAN2_CMD_STRETCH(send->stretchCurrent[0], send->stretchCurrent[1]);
	CAN2_CMD_RESCUE(send->rescueCurrent[0], send->rescueCurrent[1]);
	CAN2_CMD_VACUUM(send->makeMoney_Point->VacuumFlag);
		
}
