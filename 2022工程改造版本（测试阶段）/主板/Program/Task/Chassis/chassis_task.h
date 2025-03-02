#ifndef __CHASSIS_TASH_H
#define __CHASSIS_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "RemoteControl.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

/*************************************初始化PID参数**************************************/


//初始化6020 角度环 PID参数以及 PID最大输出，积分输出
#define  CHASSIS_STEER_ANGLE_PID_KP		25.0f	
#define  CHASSIS_STEER_ANGLE_PID_KI		0.0f	
#define  CHASSIS_STEER_ANGLE_PID_KD		0.0f	
#define  CHASSIS_STEER_ANGLE_PID_MAX_OUT	1000.0f
#define  CHASSIS_STEER_ANGLE_PID_MAX_IOUT	1000.0f
//初始化6020 速度环 PID参数以及 PID最大输出，积分输出
#define  CHASSIS_STEER_SPEED_PID_KP		150.0f	
#define  CHASSIS_STEER_SPEED_PID_KI		0.001f	
#define  CHASSIS_STEER_SPEED_PID_KD		0.0f	
#define  CHASSIS_STEER_SPEED_PID_MAX_OUT	30000.0f
#define  CHASSIS_STEER_SPEED_PID_MAX_IOUT 30000.0f

//初始化3508 速度环 PID参数以及 PID最大输出，积分输出
#define  CHASSIS_POWER_SPEED_PID_KP		20.0f	
#define  CHASSIS_POWER_SPEED_PID_KI		0.0f	
#define  CHASSIS_POWER_SPEED_PID_KD		0.0f	
#define  CHASSIS_POWER_SPEED_PID_MAX_OUT	16000.0f
#define  CHASSIS_POWER_SPEED_PID_MAX_IOUT 1000.0f

//跟随 PID参数以及 PID最大输出，积分输出
#define  CHASSIS_FOLLOW_ANGLE_PID_KP		1.0f	
#define  CHASSIS_FOLLOW_ANGLE_PID_KI		0.0f	
#define  CHASSIS_FOLLOW_ANGLE_PID_KD		0.0f	
#define  CHASSIS_FOLLOW_ANGLE_PID_MAX_OUT	1000.0f
#define  CHASSIS_FOLLOW_ANGLE_PID_MAX_IOUT	1000.0f


#define  CHASSIS_STEER_M1_OFFSET  (-2177.f/ECD_RATE)
#define  CHASSIS_STEER_M2_OFFSET  (-1933.f/ECD_RATE)
#define  CHASSIS_STEER_M3_OFFSET  (2672.f/ECD_RATE)
#define  CHASSIS_STEER_M4_OFFSET  (716.f/ECD_RATE)

#define CHASSIS_STEER_SPEED_LIMIT		2000.f
#define CHASSIS_POWER_SPEED_LIMIT	 16000.f

#define CHASSIS_RC_MAX_SPEED_X   CHASSIS_POWER_SPEED_LIMIT
#define CHASSIS_RC_MAX_SPEED_Y   CHASSIS_POWER_SPEED_LIMIT

#define RC_CHASSIS_RESPONSE  10.0f  //遥控器控制响应
#define ANGLE_TO_RAD    0.01745f
#define CHASSIS_Solution_MODE       1

#define TIME_STAMP_1MS 1

typedef enum
{
	NOR,
	PARK
} MOVE_e;
typedef enum
{
	MAINTAIN,
	TURNING
} Chassis_mode_e;

typedef enum
{
	AHEAD,
	BACK
} Chassis_move_e;

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	
} Chassis_Motor_t;

typedef struct
{
	const Angular_Handle *chassis_angle_gyro_point;
	const RC_ctrl_t *chassis_rc_ctrl;		//遥控输入
	const remote_mode_e *Control_mode;  //输入模式
	const Gimbal_Motor_t *Gimbal_yaw_data;
	Chassis_move_e chassis_mode;               //底盘控制状态机
	Chassis_move_e chassis_mode_last;          //底盘上次控制状态机	
  
	MOVE_e move_mode;
	
	Chassis_Motor_t chassis_power_motor[4];
	Chassis_Motor_t chassis_steer_motor[4];
    
	PidTypeDef chassis_steer_angle_pid[4];	//pitch电机角度环PID
	PidTypeDef chassis_steer_speed_pid[4];	//pitch电机速度环PID
	PidTypeDef chassis_power_speed_pid[4];	//pitch电机速度环PID
	PidTypeDef follow_gimbal_pid;
	
	int Mode_flag[4];
	fp32 steer_should_rotate_angle[4];
	fp32 steer_reverse_angle[4];
	fp32 power_put[4];
	fp32 steer_angle_fdb[4];
	fp32 set_steer_angle[4];
	fp32 steer_offset_angle[4];
	fp32 steer_real_angl[4];
	fp32 steer_should_arrive_angle[4];
  fp32 set_power_speed[4];	
	fp32 chassis_vx;		
	fp32 chassis_vy;		
	fp32 chassis_vw;
	fp32 gimbal_offset;
	fp32 Dir_flag;
 
} Chassis_Control_t;

void task_Chsssis_Create(void);
void Chassis_Romete_Mode(float vx,float vy,float vw,Chassis_Control_t *Chassis_Mode);
#endif

