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

/*==========================================云台是否安装==========================================*/
#define ISGIMBAL 1

/*==========================================云台是否在底盘中心==========================================*/
#define ISGIMBALCENTER 0

/*==========================================按键斜坡延时增量==========================================*/
#define KEY_RAMP_DELAY_INC  1

//跟随 PID参数以及 PID最大输出，积分输出
#if ISGIMBAL 

#define  CHASSIS_FOLLOW_PID_KP  3.5f	
#define  CHASSIS_FOLLOW_PID_KI	0.0f	
#define  CHASSIS_FOLLOW_PID_KD	0.0f	
#define  CHASSIS_FOLLOW_PID_MAX_OUT	  8000.0f
#define  CHASSIS_FOLLOW_PID_MAX_IOUT	1000.0f   //最大输出
#define  CHASSIS_FOLLOW_PID_MAX_IOUT  1000.0f   //最大积分输出
#define  CHASSIS_FOLLOW_PID_DEAD_ZONE   0.0f    //死区
#define  CHASSIS_FOLLOW_PID_I_SEPARATION  0.0f  //积分分离

#endif

/*==========================================底盘==========================================*/
//初始化底盘 速度环 PID参数
#define  CHASSIS_SPEED_PID_KP		5.0f	
#define  CHASSIS_SPEED_PID_KI		0.0f	
#define  CHASSIS_SPEED_PID_KD		0.0f	
#define  CHASSIS_SPEED_PID_MAX_OUT	  16000.0f   //最大输出
#define  CHASSIS_SPEED_PID_MAX_IOUT   1000.0f    //最大积分输出
#define  CHASSIS_SPEED_PID_DEAD_ZONE   0.0f      //死区
#define  CHASSIS_SPEED_PID_I_SEPARATION   0.0f   //积分分离

/*==========================================速度限幅==========================================*/
#define CHASSIS_NORMAL_SPEED_MAX   6600.0f
#define CHASSIS_FAST_SPEED_MAX    8000.0f
#define CHASSIS_SLOW_SPEED_MAX    800.0f


/*==========================================控制灵敏度==========================================*/
//遥控器控制响应
#define RC_VW_RESPONSE  0.4f   
#define RC_VX_RESPONSE  4.0f  
#define RC_VY_RESPONSE  3.0f 

//PC控制响应
#define PC_VW_RESPONSE  0.8f   
#define PC_VX_RESPONSE  10.0f  
#define PC_VY_RESPONSE  10.0f 
/*==========================================底盘相关参数==========================================*/

#define WHEELBASE 						 (400.0f) //前后轴距（mm）
#define WHEELTRACK             (360.0f) //左右轮距（mm）
#define GIMBAL_X_OFFSET        (0.0f) //云台相对底盘中心的前后偏移量
#define GIMBAL_Y_OFFSET        (0.0f) //云台相对底盘中心的左右偏移量
#define RADIAN_COEF        		 (57.3f) //弧度系数
#define PERIMETER              (478)  //轮子周长（mm）
#define CHASSIS_DECELE_RATIO   (1.0f/19.0f)  //电机减速比
#define ANGLE_TO_RAD           (1/RADIAN_COEF) //角度制转换为弧度制



/*==========================================底盘运动参数==========================================*/

#define 	Omni_Speed_Max            1000//9000     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
#define		STANDARD_MAX_NORMAL       6000//9000     //平地开车最快速度，防止摇杆比例*660超过这个值
#define   STANDARD_MAX_SLOW         800//5000//6000//4000//3600	  //手动爬坡模式下水平移动速度
#define		REVOLVE_MAX_NORMAL        660//9000     //平地扭头最快速度
#define   STANDARD_MAX_SZUPUP       6000//5000//6000//4000//3600	  //手动爬坡模式下水平移动速度


/*==========================================不同模式下,斜坡函数对应的时间值,一般用普通速度就行==========================================*/
#define    TIME_INC_NORMAL           8  //6//4	  //键盘斜坡,越大增加速度越快,完成时间越短
#define    TIME_DEC_NORMAL           75 //180        //键盘斜坡,越大减小速度越快(一般要比INC大一点,这样松开键盘能更快为0,太大则会造成底盘停下来的时候跳跃)

#define    TIME_INC_SALTATION        1//突然变速情况下速度变化快慢
#define    TIME_INC_SHIFT			 25
#define	   TIME_DEC_SHIFT			 45
#define    TIME_SALTATION_SHFIT      5//突然变速情况下速度变化快慢
#define    TIME_INC_SLOW			 10
#define	   TIME_DEC_SLWO			 20
#define    TIME_SALTATION_SLOW       1//突然变速情况下速度变化快慢

/*==========================================PC触发============================================*/
#define PC_TRIGGER_SEPARATE         (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_B)




typedef enum
{
	RIGH_FRON = 0,  // 左前
	LEFT_FRON,  		// 右前
	LEFT_BACK,  		// 左后
	RIGH_BACK,  		// 右后
	
} eChassisWheel;

typedef enum 
{
	NO_CONTROL = 0, //无控制
	RC_CONTROL,  //遥控器控制
	PC_CONTROL,  //PC控制
	JUDGE_CONTROL,  //自定义控制器控制
	
} eChassisCtrl;


typedef enum 
{
	CHASSIS_FREE = 0, //底盘无力
	CHASSIS_FOLLOW, //底盘跟随云台
	CHASSIS_SEPARATE, //底盘云台分离
	
} eChassisState;

typedef enum 
{
	DRIVE_NORMAL = 0, //正常行驶
	DRIVE_TURNNING, //行驶方向变换中
	DRIVE_FLIP, //车头反转行驶
	
} eChassisDrive;

typedef enum
{
	SPEED_NORMAL = 0,//普通
	SPEED_SLOW,//低速
	SPEED_FAST,//高速

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
	newPidTypeDef FollowPid;
} MotorTypeDef;

typedef struct
{
	const RC_ctrl_t* chassis_rc_ctrl;		//遥控输入
	const GimbalCtrl_t* gimbalData;
	
	eChassisCtrl CtrlMode;
	eChassisCtrl LastCtrlMode;
	
	eChassisState ChassisState;
	eChassisState LastChassisState;
	
	eChassisDrive ChassisDrive;
	eChassisDrive LastChassisDrive;
	
	eChassisSpeed SpeedMode;
	eChassisSpeed LastSpeedMode;

	MotorTypeDef ChassisMotor[4];
//	newPidTypeDef FollowPid;
	
	
#if ISGIMBAL 
	newPidTypeDef FollowPid;
#endif
//   
	fp32 Vx;		
	fp32 Vy;		
	fp32 Vw;

} ChassisCtrl_t;


void Chsssis_task_Create(void);
const ChassisCtrl_t* get_Chassis_Point(void);

#endif

