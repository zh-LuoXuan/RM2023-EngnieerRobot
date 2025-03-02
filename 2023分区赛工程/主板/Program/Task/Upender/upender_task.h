#ifndef __UPENDER_TASH_H
#define __UPENDER_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "Encoder_process.h"
#include "gimbal_task.h"
#include "new_pid.h"
#include "RemoteControl.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "judgement_info.h"


#define FRONT_MOTOR  0
#define BACK_MOTOR   1

/*************************************翻矿**************************************/
//初始化底盘 速度环 PID参数
#define  UPENDER_SPEED_PID_KP		10.0f	
#define  UPENDER_SPEED_PID_KI		0.0f	
#define  UPENDER_SPEED_PID_KD		0.0f	
#define  UPENDER_SPEED_PID_MAX_OUT	  8000.0f   //最大输出
#define  UPENDER_SPEED_PID_MAX_IOUT   1000.0f    //最大积分输出
#define  UPENDER_SPEED_PID_DEAD_ZONE   0.0f      //死区
#define  UPENDER_SPEED_PID_I_SEPARATION   0.0f   //积分分离

#define CTRL_UPENDER_SPEED (200.0f)

typedef enum 
{
	NO_CTRL_UPENDER = 0, //无控制
	RC_CTRL_UPENDER,  //遥控器控制
	PC_CTRL_UPENDER,  //PC控制
	JUDGE_CTRL_UPENDER,  //自定义控制器控制
} eUpenderCtrl;


typedef struct
{
	const MotorMeasure_t* UpenderMotorMeasure;
	const EncoderProcess_t* UpenderEncoderMeasure;
	
	fp32 SpeedRef;
	fp32 SpeedFdb;
	fp32 Current;
	newPidTypeDef SpeedPid;

} UpenderMotor_t;


typedef struct
{
  const RC_ctrl_t* upender_rc_ctrl;		//遥控输入
	
	eUpenderCtrl CtrlMode;
	eUpenderCtrl LastCtrlMode;
	
	UpenderMotor_t upenderMotor[2];	
	
} UpenderCtrl_t;



void task_Upender_Create(void);
const UpenderCtrl_t* get_Upender_Point(void);

#endif

