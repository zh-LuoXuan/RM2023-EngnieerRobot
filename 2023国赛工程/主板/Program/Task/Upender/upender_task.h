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

/*************************************����**************************************/
//��ʼ������ �ٶȻ� PID����
#define  UPENDER_SPEED_PID_KP		10.0f	
#define  UPENDER_SPEED_PID_KI		0.0f	
#define  UPENDER_SPEED_PID_KD		0.0f	
#define  UPENDER_SPEED_PID_MAX_OUT	  8000.0f   //������
#define  UPENDER_SPEED_PID_MAX_IOUT   1000.0f    //���������
#define  UPENDER_SPEED_PID_DEAD_ZONE   0.0f      //����
#define  UPENDER_SPEED_PID_I_SEPARATION   0.0f   //���ַ���

#define CTRL_UPENDER_SPEED (200.0f)

typedef enum 
{
	NO_CTRL_UPENDER = 0, //�޿���
	RC_CTRL_UPENDER,  //ң��������
	PC_CTRL_UPENDER,  //PC����
	JUDGE_CTRL_UPENDER,  //�Զ������������
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
  const RC_ctrl_t* upender_rc_ctrl;		//ң������
	
	eUpenderCtrl CtrlMode;
	eUpenderCtrl LastCtrlMode;
	
	UpenderMotor_t upenderMotor[2];	
	
} UpenderCtrl_t;



void task_Upender_Create(void);
const UpenderCtrl_t* get_Upender_Point(void);

#endif

