#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "sys.h"
#include "CAN_Receive.h"
#include "new_pid.h"
#include "RemoteControl.h"
//#include "Encoder_process.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include <stdint.h>
#include <stdbool.h>


//�Ƿ�ʹ��������
#define ISGYRO     0

/*************************************��̨**************************************/

//��ʼ��pitch �ǶȻ� PID����
#define GIMBAL_PITCH_ANGLE_PID_KP		3.5f	
#define GIMBAL_PITCH_ANGLE_PID_KI		0.0f	
#define GIMBAL_PITCH_ANGLE_PID_KD		0.0f	
#define GIMBAL_PITCH_ANGLE_PID_MAX_OUT	10000.0f
#define GIMBAL_PITCH_ANGLE_PID_MAX_IOUT	1000.0f
#define GIMBAL_PITCH_ANGLE_PID_DEAD_ZONE (0.0f)
#define GIMBAL_PITCH_ANGLE_PID_I_SEPARATION (0.0f)

//��ʼ��pitch �ٶȻ� PID�����Լ� PID���������������
#define GIMBAL_PITCH_SPEED_PID_KP		2.5f	
#define GIMBAL_PITCH_SPEED_PID_KI		0.0f	
#define GIMBAL_PITCH_SPEED_PID_KD	  135.0f	
#define GIMBAL_PITCH_SPEED_PID_MAX_OUT	16000.0f
#define GIMBAL_PITCH_SPEED_PID_MAX_IOUT 1000.0f
#define GIMBAL_PITCH_SPEED_PID_DEAD_ZONE (0.0f)
#define GIMBAL_PITCH_SPEED_PID_I_SEPARATION (0.0f)

//��ʼ��yaw �ǶȻ� PID����
#define GIMBAL_YAW_ANGLE_PID_KP		5.0f	
#define GIMBAL_YAW_ANGLE_PID_KI		0.0f	
#define GIMBAL_YAW_ANGLE_PID_KD		0.0f	
#define GIMBAL_YAW_ANGLE_PID_MAX_OUT	8000.0f
#define GIMBAL_YAW_ANGLE_PID_MAX_IOUT	1000.0f
#define GIMBAL_YAW_ANGLE_PID_DEAD_ZONE (0.0f)
#define GIMBAL_YAW_ANGLE_PID_I_SEPARATION (0.0f)

//��ʼ��yaw �ٶȻ� PID�����Լ� PID���������������
#define GIMBAL_YAW_SPEED_PID_KP		10.0f
#define GIMBAL_YAW_SPEED_PID_KI		0.0f
#define GIMBAL_YAW_SPEED_PID_KD		0.0f	
#define GIMBAL_YAW_SPEED_PID_MAX_OUT	8000.0f
#define GIMBAL_YAW_SPEED_PID_MAX_IOUT 1000.0f
#define GIMBAL_YAW_SPEED_PID_DEAD_ZONE (0.0f)
#define GIMBAL_YAW_SPEED_PID_I_SEPARATION (0.0f)


//pitch �޷�ƫ����
#define PITCH_MAX_OFFSET			(1500)
#define PITCH_MIN_OFFSET			(1500)

//yaw �޷�ƫ����
#define YAW_MAX_OFFSET			(0.0f)
#define YAW_MIN_OFFSET			(0.0f)

//����ֵ
#define PITCH_CENTER_OFFSET		(2521)
#define YAW_CENTER_OFFSET		  (2593)

#define RC_PITCH_RESPONSE  0.0045f  //ң����������Ӧ
#define RC_YAW_RESPONSE    0.0001f
#define PC_PITCH_RESPONSE  0.01f  //ң����������Ӧ
#define PC_YAW_RESPONSE    0.005f

//��̨��ʼ��б����
#define SLOPE_BEGIN_PITCH		30
#define SLOPE_BEGIN_YAW			0.5


//��ͷ��̨��ͷ��ת�Ƕ�
#define YAW_FLIP_ANGLE		(4096)


typedef enum
{
	NORMAL = 0,
	TURN_BACK,
	
} eDriveMode;

typedef enum
{
	NO_CTRL = 0,
	RC_CTRL, 
	PC_CTRL,
	JUDGE_CTRL,
	
} eGimbalCtrl;

typedef enum
{
	GYRO_MEASURE = 0,
	ENCODER_MEASURE,

} eMeasure;

typedef enum
{
	GIMBAL_RESET = 0,
	GIMBAL_FREE,
	GIMBAL_WORK,

} eGimbalState;


typedef struct
{
	const MotorMeasure_t* GimbalMotorMeasure;
	const EncoderProcess_t* GimbalEncoderMeasure;	
	
	fp32 AbsoluteAngle;
	fp32 RelativeAngle;
	fp32 CenterOffset;
	fp32 AngleRef;
	fp32 SpeedRef;
	fp32 AngleFdb;
	fp32 SpeedFdb;
	fp32 Current;
	
	newPidTypeDef AnglePid;	//����ǶȻ�PID
	newPidTypeDef SpeedPid;	//����ٶȻ�PID
	
} GimbalMotor_t;


typedef struct
{
  const RC_ctrl_t* gimbal_rc_ctrl;		//ң������
	const Angular_Handle* gimbal_gyro_point;
	
	eDriveMode DriveMode;
	eDriveMode LastDriveMode;
	
	eMeasure GimbalMeasure;
	eMeasure LastGimbalMeasure;
	
	eGimbalCtrl CtrlMode;
	eGimbalCtrl LastCtrlMode;
	
	eGimbalState GimbalState;
	eGimbalState LastGimbalState;
	
	GimbalMotor_t pitchMotor;	//pitch�����ز���
	GimbalMotor_t yawMotor;		//yaw�����ز���
	
	bool ISTurn_Flag;
	int topMoveKey;
		
} GimbalCtrl_t;



void task_Gimbal_Create(void);
const GimbalCtrl_t* get_Gimbal_Point(void);

#endif

