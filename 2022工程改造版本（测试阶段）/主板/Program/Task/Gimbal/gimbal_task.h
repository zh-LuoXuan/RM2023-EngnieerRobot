#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"


//pitch����������
#define GIMBAL_PITCH_MOTOR_ECD_MAX		8195.0f
#define GIMBAL_PITCH_MOTOR_ECD_MIN		0.0f

//yaw����������
#define GIMBAL_YAW_MOTOR_ECD_MAX		8195.0f
#define GIMBAL_YAW_MOTOR_ECD_MIN		0.0f


/*************************************��ʼ��PID����**************************************/
//��ʼ��pitch �ǶȻ� PID����
#define GIMBAL_PITCH_ANGLE_PID_KP		75.0f	
#define GIMBAL_PITCH_ANGLE_PID_KI		0.005f	
#define GIMBAL_PITCH_ANGLE_PID_KD		0.0f	
#define GIMBAL_PITCH_ANGLE_PID_MAX_OUT	8000.0f
#define GIMBAL_PITCH_ANGLE_PID_MAX_IOUT	1000.0f
//��ʼ��pitch �ٶȻ� PID�����Լ� PID���������������
#define GIMBAL_PITCH_SPEED_PID_KP		1.35f	
#define GIMBAL_PITCH_SPEED_PID_KI		0.0f	
#define GIMBAL_PITCH_SPEED_PID_KD	  60.0f	
#define GIMBAL_PITCH_SPEED_PID_MAX_OUT	16000.0f
#define GIMBAL_PITCH_SPEED_PID_MAX_IOUT 16000.0f
//��ʼ��yaw �ǶȻ� PID����
#define GIMBAL_YAW_ANGLE_PID_KP		150.0f	
#define GIMBAL_YAW_ANGLE_PID_KI		0.005f	
#define GIMBAL_YAW_ANGLE_PID_KD		0.0f	
#define GIMBAL_YAW_ANGLE_PID_MAX_OUT	8000.0f
#define GIMBAL_YAW_ANGLE_PID_MAX_IOUT	1000.0f
//��ʼ��yaw �ٶȻ� PID�����Լ� PID���������������
#define GIMBAL_YAW_SPEED_PID_KP		1.36f
#define GIMBAL_YAW_SPEED_PID_KI		0.0f
#define GIMBAL_YAW_SPEED_PID_KD		140.0f	
#define GIMBAL_YAW_SPEED_PID_MAX_OUT	16000.0f
#define GIMBAL_YAW_SPEED_PID_MAX_IOUT 16000.0f

//pitch �޷�
#define GIMBAL_PITCH_ANGLE_MAX			(GIMBAL_PITCH_ANGLE_OFFSET + 75.f)
#define GIMBAL_PITCH_ANGLE_MIN			(GIMBAL_PITCH_ANGLE_OFFSET - 75.f)

//yaw �޷�
#define GIMBAL_YAW_ANGLE_MAX			(GIMBAL_YAW_ANGLE_OFFSET + 130.f)
#define GIMBAL_YAW_ANGLE_MIN			(GIMBAL_YAW_ANGLE_OFFSET - 45.f)

//����ֵ
#define GIMBAL_PITCH_ANGLE_OFFSET		175.8f
#define GIMBAL_YAW_ANGLE_OFFSET		-2.5f

#define RC_GAMBAL_RESPONSE  0.001f  //ң����������Ӧ
//#define RC_YAW_RESPONSE  0.11363f

//�Ƿ�ʹ��������
#define ISGYRO      1
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
	
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;

typedef struct
{
	fp32 max_yaw;
	fp32 min_yaw;
	fp32 max_pitch;
	fp32 min_pitch;
	uint16_t max_yaw_ecd;
	uint16_t min_yaw_ecd;
	uint16_t max_pitch_ecd;
	uint16_t min_pitch_ecd;
	uint8_t step;
	
} Gimbal_Cali_t;



typedef enum
{
	NORMAL,
	TURN_BACK
	
} Drive_mode_e;

typedef enum
{
	GIMBAL_NONE,
	GIMBAL_RC, //��̨�����Ǿ��ԽǶȿ���
	GIMBAL_RC_PC,
	GIMBAL_AUTO, //��̨�������ֵ��ԽǶȿ���
} Gimbal_Mode_e;

typedef struct
{
	const motor_measure_t *gimbal_motor_measure;
	
	fp32 absolute_angle;
	fp32 relative_angle;
	fp32 angle_offset;
	fp32 set_angle;
  fp32 last_angle;
	fp32 PidFb;
	fp32 last_ecd;
	fp32 last_spd;
	fp32 speed_fdb;
	fp32 angle_fdb;

} Gimbal_Motor_t;

typedef struct
{
  const RC_ctrl_t *gimbal_rc_ctrl;		//ң������
	const Angular_Handle *gimbal_angle_gyro_point;
	
	//��̨ģʽ
	Gimbal_Mode_e gimbal_mode;	//��̨ģʽ
	Gimbal_Mode_e gimbal_mode_last;
	remote_mode_e Gimbal_Last_mode;
	remote_mode_e Gimbal_now_mode;
	//��̨���
	Gimbal_Motor_t gimbal_pitch_motor;		//pitch�����ز���
	Gimbal_Motor_t gimbal_yaw_motor;		//yaw�����ز���
	
	
	PidTypeDef gimbal_pitch_angle_pid;	//pitch����ǶȻ�PID
	PidTypeDef gimbal_pitch_speed_pid;	//pitch����ٶȻ�PID
	PidTypeDef gimbal_yaw_angle_pid;	//yaw����ǶȻ�PID
	PidTypeDef gimbal_yaw_speed_pid;	//yaw����ٶȻ�PID
		
} Gimbal_Control_t;

void task_Gimbal_Create(void);
extern const Gimbal_Motor_t *get_gimbal_yaw_control_t_point(void);


#endif

