#ifndef __MOTOR_CONTROL_TASH_H
#define __MOTOR_CONTROL_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

/*************************************初始化PID参数**************************************/


//初始化抬升电机 角度环 PID参数以及 PID最大输出，积分输出
#define  UP_ANGLE_PID_KP		5.0f	
#define  UP_ANGLE_PID_KI		0.0f	
#define  UP_ANGLE_PID_KD		1.0f	
#define  UP_ANGLE_PID_MAX_OUT	2000.0f
#define  UP_ANGLE_PID_MAX_IOUT	1000.0f
//初始化抬升电机 速度环 PID参数以及 PID最大输出，积分输出
#define  UP_SPEED_PID_KP		12.0f	
#define  UP_SPEED_PID_KI		0.0f	
#define  UP_SPEED_PID_KD		0.0f
#define  UP_SPEED_PID_MAX_OUT	18000.0f
#define  UP_SPEED_PID_MAX_IOUT 1000.0f

//初始化夹手PITCH电机 角度环 PID参数以及 PID最大输出，积分输出
#define  CLAMP_ANGLE_PID_KP		8.50f	
#define  CLAMP_ANGLE_PID_KI		0.0f	
#define  CLAMP_ANGLE_PID_KD		3.0f	
#define  CLAMP_ANGLE_PID_MAX_OUT	1200.0f
#define  CLAMP_ANGLE_PID_MAX_IOUT	800.0f

//初始化夹手PITCH电机 速度环 PID参数以及 PID最大输出，积分输出
#define  CLAMP_SPEED_PID_KP		12.50f	
#define  CLAMP_SPEED_PID_KI		0.0f	
#define  CLAMP_SPEED_PID_KD		0.0f	
#define  CLAMP_SPEED_PID_MAX_OUT	18000.0f
#define  CLAMP_SPEED_PID_MAX_IOUT 1200.0f
//初始化夹手YAW电机 角度环 PID参数以及 PID最大输出，积分输出
#define  CLAMP_Y_ANGLE_PID_KP		2.0f	
#define  CLAMP_Y_ANGLE_PID_KI		0.005f	
#define  CLAMP_Y_ANGLE_PID_KD		0.0f	
#define  CLAMP_Y_ANGLE_PID_MAX_OUT	1100.0f
#define  CLAMP_Y_ANGLE_PID_MAX_IOUT	100.0f
//初始化夹手YAW电机 速度环 PID参数以及 PID最大输出，积分输出
#define  CLAMP_Y_SPEED_PID_KP		11.0f	
#define  CLAMP_Y_SPEED_PID_KI		0.001f	
#define  CLAMP_Y_SPEED_PID_KD		0.0f	
#define  CLAMP_Y_SPEED_PID_MAX_OUT	7500.0f
#define  CLAMP_Y_SPEED_PID_MAX_IOUT 300.0f
//初始化夹手ROLL电机 角度环 PID参数以及 PID最大输出，积分输出
#define  CLAMP_R_ANGLE_PID_KP		6.0f	
#define  CLAMP_R_ANGLE_PID_KI		0.0f	
#define  CLAMP_R_ANGLE_PID_KD		0.0f	
#define  CLAMP_R_ANGLE_PID_MAX_OUT	1250.0f
#define  CLAMP_R_ANGLE_PID_MAX_IOUT	700.0f
//初始化夹手ROLL电机 速度环 PID参数以及 PID最大输出，积分输出
#define  CLAMP_R_SPEED_PID_KP		9.0f	
#define  CLAMP_R_SPEED_PID_KI		0.001f	
#define  CLAMP_R_SPEED_PID_KD		8.0f	
#define  CLAMP_R_SPEED_PID_MAX_OUT	10000.0f
#define  CLAMP_R_SPEED_PID_MAX_IOUT 1000.0f

/*********************************************************************************************/

#define MOTOR_SPEED_LIMIT	10000.f

#define RC_CHASSIS_RESPONSE  10.f  //遥控器控制响应
#define ANGLE_TO_RAD    0.01745f

#define REDUCTION_RATIO_3508  (19.0f)
#define REDUCTION_RATIO_2006  (36.0f)

#define UP_RAMP 90
#define CLAMP_P_RAMP 40
#define CLAMP_Y_RAMP 40
#define CLAMP_R_RAMP 40

typedef enum
{
	KEY_CONTROL,
	RC_CONTROL,
	STOP 
	
} Motor_mode_e;

typedef enum
{
	INIT,
	NORMAL,
	RE_SET,
	TUNING
} Control_e;

typedef enum
{
	UP_LEFT = 0,
	UP_RIGHT 
	
} eUpMotorList;


typedef struct
{
	const motor_measure_t *motor_measure;
	fp32 angle_offset;
	fp32 set_angle;
	
} Motor_t;

typedef struct
{
	const Angular_Handle *motor_angle_gyro_point;
	const RC_ctrl_t *motor_rc_ctrl;		//遥控输入
	const remote_mode_e *Control_mode;  //输入模式
	u8* Can_Key_data;

	Control_e mode;
	Control_e last_mode;
	
	Motor_t Up_motor[2];
	Motor_t Clamp_motor;
	Motor_t Clamp_motor_Y;
	Motor_t Clamp_motor_R;
 
	PidTypeDef Up_motor_angle_pid[2];	
	PidTypeDef Up_motor_speed_pid[2];	
	PidTypeDef Clamp_motor_angle_pid;
	PidTypeDef Clamp_motor_speed_pid;
	
	PidTypeDef Clamp_motor_Y_angle_pid;
	PidTypeDef Clamp_motor_Y_speed_pid;
	PidTypeDef Clamp_motor_R_angle_pid;
	PidTypeDef Clamp_motor_R_speed_pid;
	
	
} Motor_Control_t;


void task_Motor_Control_Create(void);
void task_Mode_Change_Create(void);
//void Mode_Change(void);
void Motor_Init(Motor_Control_t *motor_init);
void Clamp_Turn_To_Point(Motor_Control_t *Motor_Clamp_Turn, fp32 Location);
void Clamp_Y_Turn_To_Point(Motor_Control_t *Motor_Clamp_Y_Turn, fp32 Location);
void Clamp_R_Turn_To_Point(Motor_Control_t *Motor_Clamp_R_Turn, fp32 Location);
void Up_Down_To_Point(Motor_Control_t *Motor_Up_Down, fp32 L_Location, fp32 R_Location);

#endif

