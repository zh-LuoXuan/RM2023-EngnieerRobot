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


//是否使用陀螺仪
#define ISGYRO     0

/*************************************云台**************************************/

//初始化pitch 角度环 PID参数
#define GIMBAL_PITCH_ANGLE_PID_KP		20//10//15//15//3.5f	
#define GIMBAL_PITCH_ANGLE_PID_KI		0.0f	
#define GIMBAL_PITCH_ANGLE_PID_KD		1//0.1//20.0f	
#define GIMBAL_PITCH_ANGLE_PID_MAX_OUT	1500.0f
#define GIMBAL_PITCH_ANGLE_PID_MAX_IOUT	0.0f
#define GIMBAL_PITCH_ANGLE_PID_DEAD_ZONE (0.0f)
#define GIMBAL_PITCH_ANGLE_PID_I_SEPARATION (0.0f)

//初始化pitch 速度环 PID参数以及 PID最大输出，积分输出
#define GIMBAL_PITCH_SPEED_PID_KP		30//8//20//70//30//2.5f	
#define GIMBAL_PITCH_SPEED_PID_KI		0.0f//0.05f	
#define GIMBAL_PITCH_SPEED_PID_KD	 200//0// 135.0f	
#define GIMBAL_PITCH_SPEED_PID_MAX_OUT	8000.0f
#define GIMBAL_PITCH_SPEED_PID_MAX_IOUT 1000.0f
#define GIMBAL_PITCH_SPEED_PID_DEAD_ZONE (0.0f)
#define GIMBAL_PITCH_SPEED_PID_I_SEPARATION (0.0f)

//初始化yaw 角度环 PID参数
#define GIMBAL_YAW_ANGLE_PID_KP	20//	5.0f	
#define GIMBAL_YAW_ANGLE_PID_KI		0.0f	
#define GIMBAL_YAW_ANGLE_PID_KD		0.0f	
#define GIMBAL_YAW_ANGLE_PID_MAX_OUT	8000.0f
#define GIMBAL_YAW_ANGLE_PID_MAX_IOUT	1000.0f
#define GIMBAL_YAW_ANGLE_PID_DEAD_ZONE (0.0f)
#define GIMBAL_YAW_ANGLE_PID_I_SEPARATION (0.0f)

//初始化yaw 速度环 PID参数以及 PID最大输出，积分输出
#define GIMBAL_YAW_SPEED_PID_KP		10//10//10.0f
#define GIMBAL_YAW_SPEED_PID_KI		0.0f
#define GIMBAL_YAW_SPEED_PID_KD		0.0f	
#define GIMBAL_YAW_SPEED_PID_MAX_OUT	8000.0f
#define GIMBAL_YAW_SPEED_PID_MAX_IOUT 1000.0f
#define GIMBAL_YAW_SPEED_PID_DEAD_ZONE (0.0f)
#define GIMBAL_YAW_SPEED_PID_I_SEPARATION (0.0f)


//pitch 限幅偏移量
#define PITCH_MAX_OFFSET			(45.0f)
#define PITCH_MIN_OFFSET			(45.0f)

//yaw 限幅偏移量
#define YAW_MAX_OFFSET			(0.0f)
#define YAW_MIN_OFFSET			(0.0f)

//归中值
#define PITCH_CENTER_OFFSET		(-50.0f)
#define YAW_CENTER_OFFSET		  (123.0f)

#define RC_PITCH_RESPONSE  0.008f//0.0001//0.0045f  //遥控器控制响应
#define RC_YAW_RESPONSE    0.00003f
#define PC_PITCH_RESPONSE  0.05f  //遥控器控制响应
#define PC_YAW_RESPONSE    1.5f

//云台初始化斜坡量
#define SLOPE_BEGIN_PITCH		30
#define SLOPE_BEGIN_YAW			0.5


//调头云台调头旋转角度
#define YAW_FLIP_ANGLE		(180.0f)
#define YAW_HALF_ANGLE		(90.0f)

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
	
	newPidTypeDef AnglePid;	//电机角度环PID
	newPidTypeDef SpeedPid;	//电机速度环PID
	
} GimbalMotor_t;


typedef struct
{
  const RC_ctrl_t* gimbal_rc_ctrl;		//遥控输入
	const Angular_Handle* gimbal_gyro_point;
	
	eDriveMode DriveMode;
	eDriveMode LastDriveMode;
	
	eMeasure GimbalMeasure;
	eMeasure LastGimbalMeasure;
	
	eGimbalCtrl CtrlMode;
	eGimbalCtrl LastCtrlMode;
	
	eGimbalState GimbalState;
	eGimbalState LastGimbalState;
	
	GimbalMotor_t pitchMotor;	//pitch电机相关参数
	GimbalMotor_t yawMotor;		//yaw电机相关参数
	bool HalfTurn_flag;
	bool ISTurn_Flag;
	int topMoveKey;
		
} GimbalCtrl_t;


void task_Gimbal_Create(void);
const GimbalCtrl_t* get_Gimbal_Point(void);

#endif

