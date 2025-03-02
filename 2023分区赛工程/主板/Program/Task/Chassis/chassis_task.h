#ifndef __CHASSIS_TASH_H
#define __CHASSIS_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "Encoder_process.h"
#include "gimbal_task.h"
#include "new_pid.h"
#include "RemoteControl.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "judgement_info.h"

/*************************************��̨�Ƿ�װ**************************************/

#define ISGIMBAL 1

/*************************************��̨�Ƿ��ڵ�������**************************************/
#define ISGIMBALCENTER 0

/*************************************����б����ʱ����**************************************/
#define KEY_RAMP_DELAY_INC  1

//���� PID�����Լ� PID���������������
#if ISGIMBAL 

#define  CHASSIS_FOLLOW_PID_KP  3.5f	
#define  CHASSIS_FOLLOW_PID_KI	0.0f	
#define  CHASSIS_FOLLOW_PID_KD	0.0f	
#define  CHASSIS_FOLLOW_PID_MAX_OUT	  8000.0f
#define  CHASSIS_FOLLOW_PID_MAX_IOUT	1000.0f   //������
#define  CHASSIS_FOLLOW_PID_MAX_IOUT  1000.0f   //���������
#define  CHASSIS_FOLLOW_PID_DEAD_ZONE   0.0f    //����
#define  CHASSIS_FOLLOW_PID_I_SEPARATION  0.0f  //���ַ���

#endif

/*************************************����**************************************/
//��ʼ������ �ٶȻ� PID����
#define  CHASSIS_SPEED_PID_KP		3.5f	
#define  CHASSIS_SPEED_PID_KI		0.0f	
#define  CHASSIS_SPEED_PID_KD		0.0f	
#define  CHASSIS_SPEED_PID_MAX_OUT	  16000.0f   //������
#define  CHASSIS_SPEED_PID_MAX_IOUT   1000.0f    //���������
#define  CHASSIS_SPEED_PID_DEAD_ZONE   0.0f      //����
#define  CHASSIS_SPEED_PID_I_SEPARATION   0.0f   //���ַ���

/*************************************�ٶ��޷�**************************************/
#define CHASSIS_NORMAL_SPEED_MAX   6600.0f
#define CHASSIS_FAST_SPEED_MAX   8000.0f
#define CHASSIS_SLOW_SPEED_MAX   4500.0f


/*************************************����������**************************************/
//ң����������Ӧ
#define RC_VW_RESPONSE  0.4f   
#define RC_VX_RESPONSE  4.0f  
#define RC_VY_RESPONSE  4.0f 

//PC������Ӧ
#define PC_VW_RESPONSE  0.5f   
#define PC_VX_RESPONSE  10.0f  
#define PC_VY_RESPONSE  10.0f 
/*************************************������ز���**************************************/

#define WHEELBASE 						 (400.0f) //ǰ����ࣨmm��
#define WHEELTRACK             (360.0f) //�����־ࣨmm��
#define GIMBAL_X_OFFSET        (0.0f) //��̨��Ե������ĵ�ǰ��ƫ����
#define GIMBAL_Y_OFFSET        (0.0f) //��̨��Ե������ĵ�����ƫ����
#define RADIAN_COEF        		 (57.3f) //����ϵ��
#define PERIMETER              (478)  //�����ܳ���mm��
#define CHASSIS_DECELE_RATIO   (1.0f/19.0f)  //������ٱ�
#define ANGLE_TO_RAD           (1/RADIAN_COEF) //�Ƕ���ת��Ϊ������


/**************************��ͬģʽ��,б�º�����Ӧ��ʱ��ֵ,һ������ͨ�ٶȾ���*************************/
#define    TIME_INC_NORMAL           8  //6//4	  //����б��,Խ�������ٶ�Խ��,���ʱ��Խ��
#define    TIME_DEC_NORMAL           75 //180        //����б��,Խ���С�ٶ�Խ��(һ��Ҫ��INC��һ��,�����ɿ������ܸ���Ϊ0,̫�������ɵ���ͣ������ʱ����Ծ)
#define    TIME_INC_SALTATION        1//ͻȻ����������ٶȱ仯����
#define    TIME_INC_SHIFT			 25
#define	   TIME_DEC_SHIFT			 45
#define    TIME_SALTATION_SHFIT      5//ͻȻ����������ٶȱ仯����
#define    TIME_INC_SLOW			 10
#define	   TIME_DEC_SLWO			 20
#define    TIME_SALTATION_SLOW       1//ͻȻ����������ٶȱ仯����


typedef enum
{
	RIGH_FRON = 0,  // ��ǰ
	LEFT_FRON,  		// ��ǰ
	LEFT_BACK,  		// ���
	RIGH_BACK,  		// �Һ�
	
} eChassisWheel;

typedef enum 
{
	NO_CONTROL = 0, //�޿���
	RC_CONTROL,  //ң��������
	PC_CONTROL,  //PC����
	JUDGE_CONTROL,  //�Զ������������
} eChassisCtrl;


typedef enum 
{
	CHASSIS_FREE = 0, //��������
	CHASSIS_LOCK,  //��������
	CHASSIS_FOLLOW, //���̸�����̨
	CHASSIS_SEPARATE, //������̨����
	
} eChassisState;

typedef enum 
{
	DRIVE_NORMAL = 0, //������ʻ
	DRIVE_TURNNING, //��ʻ����任��
	DRIVE_FLIP, //��ͷ��ת��ʻ
	
} eChassisDrive;

typedef enum
{
	SPEED_NORMAL = 0,//��ͨģʽ
	SPEED_SLOW,//����ģʽ
	SPEED_FAST,//����ģʽ

} eChassisSpeed;

typedef struct
{
	const MotorMeasure_t* ChassisMotorMeasure;
	const EncoderProcess_t* ChassisEncoderMeasure;
	fp32 SpeedRef;
	fp32 SpeedSet;
	fp32 GiveCurrent; 
	fp32 SpeedFdb; 
	newPidTypeDef SpeedPid;
	
} MotorTypeDef;

typedef struct
{
	const RC_ctrl_t* chassis_rc_ctrl;		//ң������
	const GimbalCtrl_t* gimbalData;
	
	eChassisCtrl CtrlMode;
	eChassisCtrl LastCtrlMode;
	
	eChassisState ChassisState;
	eChassisState LastChassisState;
	
	eChassisDrive ChassisDrive;
	eChassisDrive LastChassisDrive;
	
	eChassisSpeed SpeedMode;
	eChassisSpeed LastSpeedMode;
	newPidTypeDef FOLLOWPid;
	MotorTypeDef ChassisMotor[4];
  
	fp32 Vx;		
	fp32 Vy;		
	fp32 Vw;
	
#if ISGIMBAL 
	newPidTypeDef FollowPid;
#endif
 
} ChassisCtrl_t;


void Chsssis_task_Create(void);
const ChassisCtrl_t* get_Chassis_Point(void);

#endif

