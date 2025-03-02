#ifndef __MOTOR_CONTROL_TASH_H
#define __MOTOR_CONTROL_TASH_H
#include "CAN_Receive.h"
#include "new_pid.h"
#include <stdint.h>
#include <stdbool.h>


#define LEFT_MOTOR	    0
#define RIGHT_MOTOR	    1

/*======================================================================*/
#define    TIME_STAMP_1MS        1
#define    TIME_STAMP_2MS        2
#define    TIME_STAMP_4MS        4
#define    TIME_STAMP_10MS      10
#define    TIME_STAMP_20MS      20
#define    TIME_STAMP_30MS      30
#define    TIME_STAMP_40MS      40
#define    TIME_STAMP_50MS      50
#define    TIME_STAMP_60MS      60
#define    TIME_STAMP_80MS      80
#define    TIME_STAMP_100MS    100
#define    TIME_STAMP_150MS    150
#define    TIME_STAMP_200MS    200
#define    TIME_STAMP_250MS    250
#define    TIME_STAMP_300MS    300
#define    TIME_STAMP_400MS    400
#define    TIME_STAMP_500MS    500
#define    TIME_STAMP_750MS    750
#define    TIME_STAMP_1000MS  1000
#define    TIME_STAMP_1500MS  1500
#define    TIME_STAMP_2000MS  2000
#define    TIME_STAMP_2500MS	2500
#define    TIME_STAMP_3000MS	3000
#define    TIME_STAMP_10S     10000

/*************************************����pitch��**************************************/
//��ʼ������pitch �ǶȻ� PID����
#define SUC_PITCH_ANGLE_PID_KP		(9.0f)	
#define SUC_PITCH_ANGLE_PID_KI		(0.0f)	
#define SUC_PITCH_ANGLE_PID_KD		(0.0f)	
#define SUC_PITCH_ANGLE_PID_MAX_OUT	 (600.0f)    //������
#define SUC_PITCH_ANGLE_PID_MAX_IOUT  (1000.0f)   //���������
#define SUC_PITCH_ANGLE_PID_DEAD_ZONE (0.0f)      //����
#define SUC_PITCH_ANGLE_PID_I_SEPARATION (0.0f)   //���ַ���

//��ʼ������pitch �ٶȻ� PID����
#define SUC_PITCH_SPEED_PID_KP		(18.0f)
#define SUC_PITCH_SPEED_PID_KI		(0.0f)
#define SUC_PITCH_SPEED_PID_KD		(0.0f)	
#define SUC_PITCH_SPEED_PID_MAX_OUT	 (8000.0f)   //������
#define SUC_PITCH_SPEED_PID_MAX_IOUT  (1000.0f)   //���������
#define SUC_PITCH_SPEED_PID_DEAD_ZONE (0.0f)      //����
#define SUC_PITCH_SPEED_PID_I_SEPARATION (0.0f)   //���ַ���

/*************************************����roll��**************************************/
//��ʼ������roll �ǶȻ� PID����
#define SUC_ROLL_ANGLE_PID_KP		(8.0f)	
#define SUC_ROLL_ANGLE_PID_KI		(0.0f)	
#define SUC_ROLL_ANGLE_PID_KD		(0.0f)	
#define SUC_ROLL_ANGLE_PID_MAX_OUT	 (1500.0f)   //������
#define SUC_ROLL_ANGLE_PID_MAX_IOUT  (1000.0f)   //���������
#define SUC_ROLL_ANGLE_PID_DEAD_ZONE (0.0f)      //����
#define SUC_ROLL_ANGLE_PID_I_SEPARATION (0.0f)   //���ַ���

//��ʼ������roll �ٶȻ� PID����
#define SUC_ROLL_SPEED_PID_KP		(10.0f)
#define SUC_ROLL_SPEED_PID_KI		(0.0f)
#define SUC_ROLL_SPEED_PID_KD		(0.0f)	
#define SUC_ROLL_SPEED_PID_MAX_OUT	 (5000.0f)  //������
#define SUC_ROLL_SPEED_PID_MAX_IOUT  (1000.0f)   //���������
#define SUC_ROLL_SPEED_PID_DEAD_ZONE (0.0f)      //����
#define SUC_ROLL_SPEED_PID_I_SEPARATION (0.0f)   //���ַ���

/*************************************�ڿ������**************************************/
//��ʼ��̧�� �ǶȻ� PID����
#define  UP_ANGLE_PID_KP		(8.0f)	
#define  UP_ANGLE_PID_KI		(0.0f)	
#define  UP_ANGLE_PID_KD		(0.0f)	
#define  UP_ANGLE_PID_MAX_OUT	   (1000.0f)  //������
#define  UP_ANGLE_PID_MAX_IOUT   (1000.0f)   //���������
#define  UP_ANGLE_PID_DEAD_ZONE  (0.0f)      //����
#define  UP_ANGLE_PID_I_SEPARATION  (0.0f)   //���ַ���

//��ʼ��̧�� �ٶȻ� PID����
#define  UP_SPEED_PID_KP		(10.0f)	
#define  UP_SPEED_PID_KI		(0.0f)	
#define  UP_SPEED_PID_KD		(0.0f)	
#define  UP_SPEED_PID_MAX_OUT	   (8000.0f)  //������
#define  UP_SPEED_PID_MAX_IOUT   (1000.0f)   //���������
#define  UP_SPEED_PID_DEAD_ZONE  (0.0f)      //����
#define  UP_SPEED_PID_I_SEPARATION  (0.0f)   //���ַ���

/************************************************************************************/

typedef struct
{
	const MotorMeasure_t* motor_measure;
	
	fp32 CenterOffset;  //����ֵ
	fp32 AngleRef;   //�Ƕ�����
	fp32 AngleFdb;    //�Ƕȷ�����û���ϣ�
	fp32 SpeedRef;   //�ٶ�����
	fp32 SpeedFdb;   //�ٶȷ���
	fp32 Current;   //����ֵ
	newPidTypeDef AnglePid;
	newPidTypeDef SpeedPid;
	bool ResFlag;  //��λ��
  bool Amplitude_Flag;
	
} MOTOR_t;

typedef struct 
{
	bool ifSpid;
	MOTOR_t UpMotor[2];
	MOTOR_t SucPitchMotor;
	MOTOR_t SucYawMotor;
	MOTOR_t SucRollMotor;
	
}MotorControl_t;

void task_Motor_Control_Create(void);
void motorInit(MotorControl_t* Mctrl);
void PID_Output(MotorControl_t *Output_Calc);
void SucPitch_Turn(MotorControl_t* sucPitchTurn, fp32 Location);
void SucRoll_Turn(MotorControl_t* sucRollTurn, fp32 Location);
void Up_Down_To_Point(MotorControl_t* Motor_Up_Down, fp32 L_Location, fp32 R_Location);

#endif

