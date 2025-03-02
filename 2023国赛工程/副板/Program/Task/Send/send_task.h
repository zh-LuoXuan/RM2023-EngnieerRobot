#ifndef _SEND_TASK_H
#define _SEND_TASK_H

#include "new_pid.h"
#include "control_task.h"

/*===================================初始化PID参数===================================*/

/*************************************吸盘pitch轴**************************************/
//初始化吸盘pitch 角度环 PID参数
#define TOP_PITCH_ANGLE_PID_KP		(0.0f)	
#define TOP_PITCH_ANGLE_PID_KI		(0.0f)	
#define TOP_PITCH_ANGLE_PID_KD		(0.0f)	
#define TOP_PITCH_ANGLE_PID_MAX_OUT	 (8000.0f)    //最大输出
#define TOP_PITCH_ANGLE_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_PITCH_ANGLE_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_PITCH_ANGLE_PID_I_SEPARATION (0.0f)   //积分分离

//初始化吸盘pitch 速度环 PID参数
#define TOP_PITCH_SPEED_PID_KP		(0.0f)
#define TOP_PITCH_SPEED_PID_KI		(0.0f)
#define TOP_PITCH_SPEED_PID_KD		(0.0f)	
#define TOP_PITCH_SPEED_PID_MAX_OUT	 (16000.0f)   //最大输出
#define TOP_PITCH_SPEED_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_PITCH_SPEED_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_PITCH_SPEED_PID_I_SEPARATION (0.0f)   //积分分离

/*************************************吸盘roll轴**************************************/
//初始化吸盘roll 角度环 PID参数
#define TOP_ROLL_ANGLE_PID_KP		(0.0f)	
#define TOP_ROLL_ANGLE_PID_KI		(0.0f)	
#define TOP_ROLL_ANGLE_PID_KD		(0.0f)	
#define TOP_ROLL_ANGLE_PID_MAX_OUT	 (8000.0f)   //最大输出
#define TOP_ROLL_ANGLE_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_ROLL_ANGLE_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_ROLL_ANGLE_PID_I_SEPARATION (0.0f)   //积分分离

//初始化吸盘roll 速度环 PID参数
#define TOP_ROLL_SPEED_PID_KP		(0.0f)
#define TOP_ROLL_SPEED_PID_KI		(0.0f)
#define TOP_ROLL_SPEED_PID_KD		(0.0f)	
#define TOP_ROLL_SPEED_PID_MAX_OUT	 (16000.0f)  //最大输出
#define TOP_ROLL_SPEED_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_ROLL_SPEED_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_ROLL_SPEED_PID_I_SEPARATION (0.0f)   //积分分离

/*************************************抬升**************************************/
//初始化抬升 角度环 PID参数
#define  UP_ANGLE_PID_KP		(0.0f)	
#define  UP_ANGLE_PID_KI		(0.0f)	
#define  UP_ANGLE_PID_KD		(0.0f)	
#define  UP_ANGLE_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  UP_ANGLE_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  UP_ANGLE_PID_DEAD_ZONE  (0.0f)      //死区
#define  UP_ANGLE_PID_I_SEPARATION  (0.0f)   //积分分离

//初始化抬升 速度环 PID参数
#define  UP_SPEED_PID_KP		(0.0f)	
#define  UP_SPEED_PID_KI		(0.0f)	
#define  UP_SPEED_PID_KD		(0.0f)	
#define  UP_SPEED_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  UP_SPEED_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  UP_SPEED_PID_DEAD_ZONE  (0.0f)      //死区
#define  UP_SPEED_PID_I_SEPARATION  (0.0f)   //积分分离

/*************************************前伸**************************************/
//初始化前伸 角度环 PID参数
#define  STRETCH_ANGLE_PID_KP		(0.0f)	
#define  STRETCH_ANGLE_PID_KI		(0.0f)	
#define  STRETCH_ANGLE_PID_KD		(0.0f)	
#define  STRETCH_ANGLE_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  STRETCH_ANGLE_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  STRETCH_ANGLE_PID_DEAD_ZONE  (0.0f)      //死区
#define  STRETCH_ANGLE_PID_I_SEPARATION  (0.0f)   //积分分离

//初始化前伸 速度环 PID参数
#define  STRETCH_SPEED_PID_KP		(0.0f)	
#define  STRETCH_SPEED_PID_KI		(0.0f)	
#define  STRETCH_SPEED_PID_KD		(0.0f)	
#define  STRETCH_SPEED_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  STRETCH_SPEED_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  STRETCH_SPEED_PID_DEAD_ZONE  (0.0f)      //死区
#define  STRETCH_SPEED_PID_I_SEPARATION  (0.0f)   //积分分离

/*************************************救援**************************************/
//初始化救援 角度环 PID参数
#define  RESCUE_ANGLE_PID_KP		(0.0f)	
#define  RESCUE_ANGLE_PID_KI		(0.0f)	
#define  RESCUE_ANGLE_PID_KD		(0.0f)	
#define  RESCUE_ANGLE_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  RESCUE_ANGLE_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  RESCUE_ANGLE_PID_DEAD_ZONE  (0.0f)      //死区
#define  RESCUE_ANGLE_PID_I_SEPARATION  (0.0f)   //积分分离

//初始化救援 速度环 PID参数
#define  RESCUE_SPEED_PID_KP		(0.0f)	
#define  RESCUE_SPEED_PID_KI		(0.0f)	
#define  RESCUE_SPEED_PID_KD		(0.0f)	
#define  RESCUE_SPEED_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  RESCUE_SPEED_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  RESCUE_SPEED_PID_DEAD_ZONE  (0.0f)      //死区
#define  RESCUE_SPEED_PID_I_SEPARATION  (0.0f)   //积分分离

/*************************************吸盘移动**************************************/
//初始化吸盘移动 角度环 PID参数
#define  TOPMOVE_ANGLE_PID_KP		(0.0f)	
#define  TOPMOVE_ANGLE_PID_KI		(0.0f)	
#define  TOPMOVE_ANGLE_PID_KD		(0.0f)	
#define  TOPMOVE_ANGLE_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  TOPMOVE_ANGLE_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  TOPMOVE_ANGLE_PID_DEAD_ZONE  (0.0f)      //死区
#define  TOPMOVE_ANGLE_PID_I_SEPARATION  (0.0f)   //积分分离

//初始化吸盘移动 速度环 PID参数
#define  TOPMOVE_SPEED_PID_KP		(0.0f)	
#define  TOPMOVE_SPEED_PID_KI		(0.0f)	
#define  TOPMOVE_SPEED_PID_KD		(0.0f)	
#define  TOPMOVE_SPEED_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  TOPMOVE_SPEED_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  TOPMOVE_SPEED_PID_DEAD_ZONE  (0.0f)      //死区
#define  TOPMOVE_SPEED_PID_I_SEPARATION  (0.0f)   //积分分离


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

