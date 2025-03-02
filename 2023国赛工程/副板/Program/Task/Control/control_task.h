#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "stm32f4xx.h"  
#include "gpio.h" 
#include "CAN_Receive.h"
#include "motor_control_task.h"
#include "new_pid.h"
#include <stdint.h>
#include <stdbool.h>

/*=====================================电机减速比====================================*/
#define REDUCTION_RATIO_3508	    (19.0f)
#define REDUCTION_RATIO_2006	    (36.0f)

/*====================================各电机微调比率=================================*/
#define PC_UP_RATIO	          (0.05f)
#define PC_SUC_PITCH_RATIO	  (0.05f)
#define PC_SUC_YAW_RATIO	    (0.05f)
#define PC_SUC_ROLL_RATIO	    (0.05f)

#define RC_UP_RATIO	          (0.01f)
#define RC_SUC_PITCH_RATIO	  (0.01f)
#define RC_SUC_YAW_RATIO	    (0.01f)
#define RC_SUC_ROLL_RATIO	    (0.01f)

/*=================================各电机运动目标点数===============================*/
#define UP_COUNT	            (2)
#define SUC_PITCH_COUNT	      (3)
#define SUC_ROLL_COUNT	      (2)
#define SUC_YAW_COUNT         (2)

/*===================================抬升电机转动角度===============================*/
#define L_UP_LONGS_HIGHTST		(-190.0f)//  
#define L_Up_LONGS_LOWEST	 	  (-13376.0f)//


#define R_UP_LONGS_HIGHTST		(190.0f)//
#define R_Up_LONGS_LOWEST	 	  (13376.0f)//

/*===================================吸盘Pitch轴电机转动角度===============================*/
#define SUC_PITCH_LONGS_FRONT		(3494.0f)//
#define	SUC_PITCH_LONGS_DOWN		(1746.0f)//
#define SUC_PITCH_LONGS_BACK	  (0.0f)//

/*===================================吸盘Roll轴电机转动角度===============================*/
#define SUC_ROLL_LONGS_RES      (0.0f)
#define SUC_ROLL_LONGS_READY		(6450.0f)//

/*===================================吸盘Yaw轴电机转动角度===============================*/
#define SUC_YAW_LONGS_YRES		  (0.0f)//
#define SUC_YAW_LONGS_CENTER		(0.0f)//

/*========================================遥控器触发======================================*/
#define CH4_TRIGGER_VAL         (550)

/*==========================================PC触发============================================*/
#define PC_TRIGGER_GOLD             (IF_KEY_PRESSED_E)  
#define PC_TRIGGER_SILVER           (IF_KEY_PRESSED_Q) 
#define PC_TRIGGER_EXCHANGE         (IF_KEY_PRESSED_R) 


typedef enum
{
	HIGHEST = 0,
	LOWEST,
	
} eUpPos;


typedef enum
{
	FRONT = 0,
	DOWN,
	BACK,
	
} eSucpitchPos;

typedef enum
{
	YRES = 0,
	CENTER,
} eSucyawPos;

typedef enum
{
	RES = 0,
	READY,
	
} eSucrollPos;


typedef enum
{
	RUN_RESET = 0,  //复位
	RUN_TURNING,   //中间转换过程
	RUN_NORMAL,     //正常运行
	
}eRunState;

typedef enum
{
	DO_NOTHING = 0,
	ORE_GOLD_FIR,
	ORE_GOLD_SEC,
	ORE_CATCH,
	ORE_SILVER_FIR,
	ORE_SILVER_SEC,
	ORE_EXCHANGE,
	
} eWorkState;

typedef enum
{
	CTRL_NULL = 0,
	CTRL_RC,
	CTRL_PC,
	CTRL_FINETUNING,
	
} eCtrlMode;


typedef struct 
{
	const RC_ctrl_t* Input_Point;
	MotorControl_t motorCtrl;  //要控制的电机结构体变量
	
	eRunState runState;//运行状态
	eRunState LastrunState;
	
	eWorkState workStste;//工作状态（取金矿、银矿、空接、兑换等）
	eWorkState LastworkStste;
	
	eCtrlMode ctrlMode;  //控制模式
	eCtrlMode LastctrlMode;
	

}ControlTask_t;

extern ControlTask_t ctrlTask;
void task_control_Create(void);
const ControlTask_t* get_Makemoney_Point(void);
#endif
