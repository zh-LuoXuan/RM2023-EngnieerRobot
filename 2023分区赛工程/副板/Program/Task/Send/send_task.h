#ifndef _SEND_TASK_H
#define _SEND_TASK_H

#include "new_pid.h"
#include "control_task.h"

/*===================================��ʼ��PID����===================================*/

/*************************************����pitch��**************************************/
//��ʼ������pitch �ǶȻ� PID����
#define TOP_PITCH_ANGLE_PID_KP		(0.0f)	
#define TOP_PITCH_ANGLE_PID_KI		(0.0f)	
#define TOP_PITCH_ANGLE_PID_KD		(0.0f)	
#define TOP_PITCH_ANGLE_PID_MAX_OUT	 (8000.0f)    //������
#define TOP_PITCH_ANGLE_PID_MAX_IOUT  (1000.0f)   //���������
#define TOP_PITCH_ANGLE_PID_DEAD_ZONE (0.0f)      //����
#define TOP_PITCH_ANGLE_PID_I_SEPARATION (0.0f)   //���ַ���

//��ʼ������pitch �ٶȻ� PID����
#define TOP_PITCH_SPEED_PID_KP		(0.0f)
#define TOP_PITCH_SPEED_PID_KI		(0.0f)
#define TOP_PITCH_SPEED_PID_KD		(0.0f)	
#define TOP_PITCH_SPEED_PID_MAX_OUT	 (16000.0f)   //������
#define TOP_PITCH_SPEED_PID_MAX_IOUT  (1000.0f)   //���������
#define TOP_PITCH_SPEED_PID_DEAD_ZONE (0.0f)      //����
#define TOP_PITCH_SPEED_PID_I_SEPARATION (0.0f)   //���ַ���

/*************************************����roll��**************************************/
//��ʼ������roll �ǶȻ� PID����
#define TOP_ROLL_ANGLE_PID_KP		(0.0f)	
#define TOP_ROLL_ANGLE_PID_KI		(0.0f)	
#define TOP_ROLL_ANGLE_PID_KD		(0.0f)	
#define TOP_ROLL_ANGLE_PID_MAX_OUT	 (8000.0f)   //������
#define TOP_ROLL_ANGLE_PID_MAX_IOUT  (1000.0f)   //���������
#define TOP_ROLL_ANGLE_PID_DEAD_ZONE (0.0f)      //����
#define TOP_ROLL_ANGLE_PID_I_SEPARATION (0.0f)   //���ַ���

//��ʼ������roll �ٶȻ� PID����
#define TOP_ROLL_SPEED_PID_KP		(0.0f)
#define TOP_ROLL_SPEED_PID_KI		(0.0f)
#define TOP_ROLL_SPEED_PID_KD		(0.0f)	
#define TOP_ROLL_SPEED_PID_MAX_OUT	 (16000.0f)  //������
#define TOP_ROLL_SPEED_PID_MAX_IOUT  (1000.0f)   //���������
#define TOP_ROLL_SPEED_PID_DEAD_ZONE (0.0f)      //����
#define TOP_ROLL_SPEED_PID_I_SEPARATION (0.0f)   //���ַ���

/*************************************̧��**************************************/
//��ʼ��̧�� �ǶȻ� PID����
#define  UP_ANGLE_PID_KP		(0.0f)	
#define  UP_ANGLE_PID_KI		(0.0f)	
#define  UP_ANGLE_PID_KD		(0.0f)	
#define  UP_ANGLE_PID_MAX_OUT	   (16000.0f)  //������
#define  UP_ANGLE_PID_MAX_IOUT   (1000.0f)   //���������
#define  UP_ANGLE_PID_DEAD_ZONE  (0.0f)      //����
#define  UP_ANGLE_PID_I_SEPARATION  (0.0f)   //���ַ���

//��ʼ��̧�� �ٶȻ� PID����
#define  UP_SPEED_PID_KP		(0.0f)	
#define  UP_SPEED_PID_KI		(0.0f)	
#define  UP_SPEED_PID_KD		(0.0f)	
#define  UP_SPEED_PID_MAX_OUT	   (16000.0f)  //������
#define  UP_SPEED_PID_MAX_IOUT   (1000.0f)   //���������
#define  UP_SPEED_PID_DEAD_ZONE  (0.0f)      //����
#define  UP_SPEED_PID_I_SEPARATION  (0.0f)   //���ַ���

/*************************************ǰ��**************************************/
//��ʼ��ǰ�� �ǶȻ� PID����
#define  STRETCH_ANGLE_PID_KP		(0.0f)	
#define  STRETCH_ANGLE_PID_KI		(0.0f)	
#define  STRETCH_ANGLE_PID_KD		(0.0f)	
#define  STRETCH_ANGLE_PID_MAX_OUT	   (16000.0f)  //������
#define  STRETCH_ANGLE_PID_MAX_IOUT   (1000.0f)   //���������
#define  STRETCH_ANGLE_PID_DEAD_ZONE  (0.0f)      //����
#define  STRETCH_ANGLE_PID_I_SEPARATION  (0.0f)   //���ַ���

//��ʼ��ǰ�� �ٶȻ� PID����
#define  STRETCH_SPEED_PID_KP		(0.0f)	
#define  STRETCH_SPEED_PID_KI		(0.0f)	
#define  STRETCH_SPEED_PID_KD		(0.0f)	
#define  STRETCH_SPEED_PID_MAX_OUT	   (16000.0f)  //������
#define  STRETCH_SPEED_PID_MAX_IOUT   (1000.0f)   //���������
#define  STRETCH_SPEED_PID_DEAD_ZONE  (0.0f)      //����
#define  STRETCH_SPEED_PID_I_SEPARATION  (0.0f)   //���ַ���

/*************************************��Ԯ**************************************/
//��ʼ����Ԯ �ǶȻ� PID����
#define  RESCUE_ANGLE_PID_KP		(0.0f)	
#define  RESCUE_ANGLE_PID_KI		(0.0f)	
#define  RESCUE_ANGLE_PID_KD		(0.0f)	
#define  RESCUE_ANGLE_PID_MAX_OUT	   (16000.0f)  //������
#define  RESCUE_ANGLE_PID_MAX_IOUT   (1000.0f)   //���������
#define  RESCUE_ANGLE_PID_DEAD_ZONE  (0.0f)      //����
#define  RESCUE_ANGLE_PID_I_SEPARATION  (0.0f)   //���ַ���

//��ʼ����Ԯ �ٶȻ� PID����
#define  RESCUE_SPEED_PID_KP		(0.0f)	
#define  RESCUE_SPEED_PID_KI		(0.0f)	
#define  RESCUE_SPEED_PID_KD		(0.0f)	
#define  RESCUE_SPEED_PID_MAX_OUT	   (16000.0f)  //������
#define  RESCUE_SPEED_PID_MAX_IOUT   (1000.0f)   //���������
#define  RESCUE_SPEED_PID_DEAD_ZONE  (0.0f)      //����
#define  RESCUE_SPEED_PID_I_SEPARATION  (0.0f)   //���ַ���

/*************************************�����ƶ�**************************************/
//��ʼ�������ƶ� �ǶȻ� PID����
#define  TOPMOVE_ANGLE_PID_KP		(0.0f)	
#define  TOPMOVE_ANGLE_PID_KI		(0.0f)	
#define  TOPMOVE_ANGLE_PID_KD		(0.0f)	
#define  TOPMOVE_ANGLE_PID_MAX_OUT	   (16000.0f)  //������
#define  TOPMOVE_ANGLE_PID_MAX_IOUT   (1000.0f)   //���������
#define  TOPMOVE_ANGLE_PID_DEAD_ZONE  (0.0f)      //����
#define  TOPMOVE_ANGLE_PID_I_SEPARATION  (0.0f)   //���ַ���

//��ʼ�������ƶ� �ٶȻ� PID����
#define  TOPMOVE_SPEED_PID_KP		(0.0f)	
#define  TOPMOVE_SPEED_PID_KI		(0.0f)	
#define  TOPMOVE_SPEED_PID_KD		(0.0f)	
#define  TOPMOVE_SPEED_PID_MAX_OUT	   (16000.0f)  //������
#define  TOPMOVE_SPEED_PID_MAX_IOUT   (1000.0f)   //���������
#define  TOPMOVE_SPEED_PID_DEAD_ZONE  (0.0f)      //����
#define  TOPMOVE_SPEED_PID_I_SEPARATION  (0.0f)   //���ַ���


typedef struct
{
	const ControlTask_t* makeMoney_Point;
	
	newPidTypeDef topMoveAnglePid[2];
	newPidTypeDef topMoveSpeedPid[2];
	
	newPidTypeDef rescueAnglePid[2];
	newPidTypeDef rescueSpeedPid[2];
	
	newPidTypeDef upAnglePid[2];
	newPidTypeDef upSpeedPid[2];
	
	newPidTypeDef stretchAnglePid[2];
	newPidTypeDef stretchSpeedPid[2];
	
	newPidTypeDef topPitchAnglePid;
	newPidTypeDef topPitchSpeedPid;
	
	newPidTypeDef topRollAnglePid;
	newPidTypeDef topRollSpeedPid;

	
	fp32 topMoveCurrent[2];
	fp32 rescueCurrent[2];
	fp32 upCurrent[2];
	fp32 stretchCurrent[2];
	fp32 topPitchCurrent;
	fp32 topRollCurrent;
	
	
} SendData_t;



void task_Send_Create(void);

#endif

