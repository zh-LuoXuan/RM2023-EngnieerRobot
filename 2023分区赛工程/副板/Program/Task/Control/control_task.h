#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "stm32f4xx.h"  
#include "gpio.h" 
#include "CAN_Receive.h"
#include "new_pid.h"
#include <stdint.h>
#include <stdbool.h>

#define LEFT_MOTOR	    0
#define RIGHT_MOTOR	    1


/*************************************吸盘pitch轴**************************************/
//初始化吸盘pitch 角度环 PID参数
#define TOP_PITCH_ANGLE_PID_KP		(5.0f)	
#define TOP_PITCH_ANGLE_PID_KI		(0.0f)	
#define TOP_PITCH_ANGLE_PID_KD		(0.0f)	
#define TOP_PITCH_ANGLE_PID_MAX_OUT	 (1000.0f)    //最大输出
#define TOP_PITCH_ANGLE_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_PITCH_ANGLE_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_PITCH_ANGLE_PID_I_SEPARATION (0.0f)   //积分分离

//初始化吸盘pitch 速度环 PID参数
#define TOP_PITCH_SPEED_PID_KP		(10.0f)

#define TOP_PITCH_SPEED_PID_KI		(0.0f)
#define TOP_PITCH_SPEED_PID_KD		(0.0f)	
#define TOP_PITCH_SPEED_PID_MAX_OUT	 (5000.0f)   //最大输出
#define TOP_PITCH_SPEED_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_PITCH_SPEED_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_PITCH_SPEED_PID_I_SEPARATION (0.0f)   //积分分离

/*************************************吸盘roll轴**************************************/
//初始化吸盘roll 角度环 PID参数
#define TOP_ROLL_ANGLE_PID_KP		(8.0f)	
#define TOP_ROLL_ANGLE_PID_KI		(0.0f)	
#define TOP_ROLL_ANGLE_PID_KD		(0.0f)	
#define TOP_ROLL_ANGLE_PID_MAX_OUT	 (1000.0f)   //最大输出
#define TOP_ROLL_ANGLE_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_ROLL_ANGLE_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_ROLL_ANGLE_PID_I_SEPARATION (0.0f)   //积分分离

//初始化吸盘roll 速度环 PID参数
#define TOP_ROLL_SPEED_PID_KP		(20.0f)
#define TOP_ROLL_SPEED_PID_KI		(0.0f)
#define TOP_ROLL_SPEED_PID_KD		(0.0f)	
#define TOP_ROLL_SPEED_PID_MAX_OUT	 (5000.0f)  //最大输出
#define TOP_ROLL_SPEED_PID_MAX_IOUT  (1000.0f)   //最大积分输出
#define TOP_ROLL_SPEED_PID_DEAD_ZONE (0.0f)      //死区
#define TOP_ROLL_SPEED_PID_I_SEPARATION (0.0f)   //积分分离

/*************************************抬升**************************************/
//初始化抬升 角度环 PID参数
#define  UP_ANGLE_PID_KP		3.0f//(1.0f)	
#define  UP_ANGLE_PID_KI		(0.0f)	
#define  UP_ANGLE_PID_KD		(0.0f)	
#define  UP_ANGLE_PID_MAX_OUT	   (8000.0f)  //最大输出
#define  UP_ANGLE_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  UP_ANGLE_PID_DEAD_ZONE  (0.0f)      //死区
#define  UP_ANGLE_PID_I_SEPARATION  (0.0f)   //积分分离

//初始化抬升 速度环 PID参数
#define  UP_SPEED_PID_KP		10.0f//1.0f//(15.0f)	
#define  UP_SPEED_PID_KI		(0.0f)	
#define  UP_SPEED_PID_KD		(0.0f)	
#define  UP_SPEED_PID_MAX_OUT	   (16000.0f)  //最大输出
#define  UP_SPEED_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  UP_SPEED_PID_DEAD_ZONE  (0.0f)      //死区
#define  UP_SPEED_PID_I_SEPARATION  (0.0f)   //积分分离

/*************************************前伸**************************************/
//初始化前伸 角度环 PID参数
#define  STRETCH_ANGLE_PID_KP		3.0f//(1.0f)	//3
#define  STRETCH_ANGLE_PID_KI		(0.0f)	
#define  STRETCH_ANGLE_PID_KD		(0.0f)	
#define  STRETCH_ANGLE_PID_MAX_OUT	   (5500.0f)  //最大输出
#define  STRETCH_ANGLE_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  STRETCH_ANGLE_PID_DEAD_ZONE  (0.0f)      //死区
#define  STRETCH_ANGLE_PID_I_SEPARATION  (0.0f)   //积分分离

//初始化前伸 速度环 PID参数
#define  STRETCH_SPEED_PID_KP	10.0f//	(10.0f)	//12
#define  STRETCH_SPEED_PID_KI		(0.0f)	
#define  STRETCH_SPEED_PID_KD		(0.0f)	
#define  STRETCH_SPEED_PID_MAX_OUT	   (10000.0f)  //最大输出
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
#define  TOPMOVE_ANGLE_PID_KP		(0.0f)	//5
#define  TOPMOVE_ANGLE_PID_KI		(0.0f)	
#define  TOPMOVE_ANGLE_PID_KD		(0.0f)	
#define  TOPMOVE_ANGLE_PID_MAX_OUT	   (2500.0f)  //最大输出
#define  TOPMOVE_ANGLE_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  TOPMOVE_ANGLE_PID_DEAD_ZONE  (0.0f)      //死区
#define  TOPMOVE_ANGLE_PID_I_SEPARATION  (0.0f)   //积分分离

//初始化吸盘移动 速度环 PID参数
#define  TOPMOVE_SPEED_PID_KP		(0.0f)	//20
#define  TOPMOVE_SPEED_PID_KI		(0.0f)	
#define  TOPMOVE_SPEED_PID_KD		(0.0f)	
#define  TOPMOVE_SPEED_PID_MAX_OUT	   (8000.0f)  //最大输出
#define  TOPMOVE_SPEED_PID_MAX_IOUT   (1000.0f)   //最大积分输出
#define  TOPMOVE_SPEED_PID_DEAD_ZONE  (0.0f)      //死区
#define  TOPMOVE_SPEED_PID_I_SEPARATION  (0.0f)   //积分分离


/*=====================================电机减速比====================================*/
#define UP_REDUCTION_RATIO_3508	  (71.0f)
#define REDUCTION_RATIO_3508	    (19.0f)
#define REDUCTION_RATIO_2006	    (36.0f)

/*====================================各电机微调比率=================================*/
#define UP_FINETUNING_RATIO	          (0.1f)
#define STRETCH_FINETUNING_RATIO	    (0.1f)
#define TOP_PITCH_FINETUNING_RATIO	  (0.1f)
#define TOP_ROLL_FINETUNING_RATIO	    (0.1f)

/*=================================各电机运动目标点数目==============================*/
#define UP_COUNT	      5
#define STRETCH_COUNT	  5
#define TOPMOVE_COUNT	  2
#define TOP_PITCH_COUNT	4
#define TOP_ROLL_COUNT	1
#define RESCUE_COUNT	  2

/*===================================抬升电机转动角度===============================*/ //
#define L_UP_LONGS_HIGHTST		(3090.1f * REDUCTION_RATIO_3508)//  17332        11500
#define L_Up_LONGS_GOLD				(1556.1f * REDUCTION_RATIO_3508)//(455.0f * REDUCTION_RATIO_3508)//  8819   5353
#define	L_Up_LONGS_SILVER			(2185.5f * REDUCTION_RATIO_3508)//  16435   
#define L_Up_LONGS_Exchang	  (2262.0f * REDUCTION_RATIO_3508)//  14477        
#define L_Up_LONGS_LOWEST	 	  (0.0f * REDUCTION_RATIO_3508)  //  180.57   初   155.5


#define R_UP_LONGS_HIGHTST		(-3090.1f * REDUCTION_RATIO_3508)// -17045       -11438
#define R_Up_LONGS_GOLD				(-1556.1f * REDUCTION_RATIO_3508)//(-468.0f * REDUCTION_RATIO_3508)// -8766        -5048
#define	R_Up_LONGS_SILVER			(-2185.5f * REDUCTION_RATIO_3508)// -16148  
#define R_Up_LONGS_Exchang	  (-2262.0f * REDUCTION_RATIO_3508)// -14215  
#define R_Up_LONGS_LOWEST	 	  (0.0f * REDUCTION_RATIO_3508)   // 140.75    初   159

/*===================================前伸电机转动角度===============================*/
#define L_STRETCH_LONGS_LONGEST		(-1180.7f * REDUCTION_RATIO_2006)//  -41155   -1164.7
#define L_STRETCH_LONGS_GOLD		(-1016.2f * REDUCTION_RATIO_2006)//	(-961.7f * REDUCTION_RATIO_2006)//  -33835    -961.7
#define	L_STRETCH_LONGS_SILVER		(-536.2f * REDUCTION_RATIO_2006)//  -24387    -698.7
#define L_STRETCH_LONGS_Exchang	  (-376.7f * REDUCTION_RATIO_2006)//  -26062    -745.7
#define L_STRETCH_LONGS_SHORTEST	(-10.0f * REDUCTION_RATIO_2006)//  783     初            

#define R_STRETCH_LONGS_LONGEST		(1180.7f * REDUCTION_RATIO_2006)//   40965  1127
#define R_STRETCH_LONGS_GOLD		(1016.2f * REDUCTION_RATIO_2006)//	(943.0f * REDUCTION_RATIO_2006)//   34315   943
#define	R_STRETCH_LONGS_SILVER		(536.2f * REDUCTION_RATIO_2006)//   24905   682
#define R_STRETCH_LONGS_Exchang	  (376.7f * REDUCTION_RATIO_2006)//  26420   1001
#define R_STRETCH_LONGS_SHORTEST	(10.0f * REDUCTION_RATIO_2006)//    -374  初

/*===================================吸盘移动电机转动角度===============================*/
#define L_TOPMOVE_LONGS_FURTHEST	(10.0f)//  176    初
#define L_TOPMOVE_LONGS_CLOSEST		//(213.0f)//  389   213

#define R_TOPMOVE_LONGS_FURTHEST	(0.0f)//  158    初
#define R_TOPMOVE_LONGS_CLOSEST		(-222.0f)//  -64    -222

/*===================================吸盘Pitch轴电机转动角度===============================*/
#define TOP_PITCH_LONGS_UP		  (-190.4f * REDUCTION_RATIO_3508)//  -1873    -185.5
#define TOP_PITCH_LONGS_FRONT		(-97.3f * REDUCTION_RATIO_3508)//   20       -87
#define	TOP_PITCH_LONGS_DOWN		(-10.0f * REDUCTION_RATIO_3508)//  1650   初
#define TOP_PITCH_LONGS_BACK	  (-282.5f * REDUCTION_RATIO_3508)//  -3389    -265

/*===================================吸盘Roll轴电机转动角度===============================*/
#define TOP_ROLL_LONGS_RES		  (0.0f * REDUCTION_RATIO_2006)//  120   初

/*===================================救援电机转动角度=================================*/
#define L_RESCUE_LONGS_ON	    (0.0f * REDUCTION_RATIO_2006)//
#define L_RESCUE_LONGS_OFF		(0.0f * REDUCTION_RATIO_2006)//

#define R_RESCUE_LONGS_ON	    (0.0f * REDUCTION_RATIO_2006)//
#define R_RESCUE_LONGS_OFF		(0.0f * REDUCTION_RATIO_2006)//


#define ISNSerialCtrl  1   // 如果不使用串级

typedef enum
{
	HIGHEST = 0,
	UP_GOLD,
	UP_SILVER,
	UP_EXCHANGE,
	LOWEST,
	
} eUpPos;

typedef enum
{
	LONGEST = 0,
	STRETCH_GOLD,
	STRETCH_SILVER,
	STRETCH_EXCHANGE,
	SHORTEST,
	
} eStretchPos;

typedef enum
{
	FURTHEST = 0,
	CLOSEST,
	
} eTopmovePos;

typedef enum
{
	ON_POS = 0,
	OFF_POS,
	
} eRescuePos;

typedef enum
{
	UP = 0,
	FRONT,
	DOWN,
	BACK,
	
} eToppitchPos;

typedef enum
{
	RES = 0,
	
} eToprollPos;


typedef enum
{
	NONE = 0,
	GOLD,
	SILVER,
	CATCH,
	GROUND,
	EXCHANGE,
	RESCUE,
	
} eWorkState;

typedef enum
{
	DO_NOTHING = 0,
	DO_RESET,
	ORE_GOLD,
	ORE_SILVER,
	ORE_CATCH,
	ORE_GROUND,
	ORE_EXCHANGE,
	DO_RESCUE,
	
} eMakeMoney;

typedef enum
{
	CTRL_NULL = 0,
	CTRL_RC,
	CTRL_PC,
	CTRL_FINETUNING,
	
} eCtrlMode;


typedef struct
{
	const MotorMeasure_t* motor_measure;
	
	fp32 CenterOffset;
	fp32 AngleRef;
	fp32 AngleFdb;
	fp32 SpeedRef;
	fp32 SpeedFdb;
	fp32 Current;
	newPidTypeDef AnglePid;
	newPidTypeDef SpeedPid;
	bool ResFlag;

} MOTOR_t;

typedef struct 
{
	const RC_ctrl_t* Input_Point;
	const int* topMoveKeyPoint;
	
	eWorkState workStste;
	eWorkState LastworkStste;
	
	eMakeMoney makeMoney;
	eMakeMoney LastmakeMoney;
	
	eCtrlMode ctrlMode;
	eCtrlMode LastctrlMode;
	
	MOTOR_t UpMotor[2];
	MOTOR_t StretchMotor[2];
	MOTOR_t TopMoveMotor[2];
	MOTOR_t RescueMotor[2];
	MOTOR_t TopPitchMotor;
	MOTOR_t TopRollMotor;
	
	bool VacuumFlag;
	
}ControlTask_t;

void task_control_Create(void);
const ControlTask_t* get_Makemoney_Point(void);
#endif
