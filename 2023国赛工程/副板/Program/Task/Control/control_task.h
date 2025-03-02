#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "stm32f4xx.h"  
#include "gpio.h" 
#include "CAN_Receive.h"
#include "motor_control_task.h"
#include "new_pid.h"
#include <stdint.h>
#include <stdbool.h>

/*=====================================������ٱ�====================================*/
#define REDUCTION_RATIO_3508	    (19.0f)
#define REDUCTION_RATIO_2006	    (36.0f)

/*====================================�����΢������=================================*/
#define PC_UP_RATIO	          (0.05f)
#define PC_SUC_PITCH_RATIO	  (0.05f)
#define PC_SUC_YAW_RATIO	    (0.05f)
#define PC_SUC_ROLL_RATIO	    (0.05f)

#define RC_UP_RATIO	          (0.01f)
#define RC_SUC_PITCH_RATIO	  (0.01f)
#define RC_SUC_YAW_RATIO	    (0.01f)
#define RC_SUC_ROLL_RATIO	    (0.01f)

/*=================================������˶�Ŀ�����===============================*/
#define UP_COUNT	            (2)
#define SUC_PITCH_COUNT	      (3)
#define SUC_ROLL_COUNT	      (2)
#define SUC_YAW_COUNT         (2)

/*===================================̧�����ת���Ƕ�===============================*/
#define L_UP_LONGS_HIGHTST		(-190.0f)//  
#define L_Up_LONGS_LOWEST	 	  (-13376.0f)//


#define R_UP_LONGS_HIGHTST		(190.0f)//
#define R_Up_LONGS_LOWEST	 	  (13376.0f)//

/*===================================����Pitch����ת���Ƕ�===============================*/
#define SUC_PITCH_LONGS_FRONT		(3494.0f)//
#define	SUC_PITCH_LONGS_DOWN		(1746.0f)//
#define SUC_PITCH_LONGS_BACK	  (0.0f)//

/*===================================����Roll����ת���Ƕ�===============================*/
#define SUC_ROLL_LONGS_RES      (0.0f)
#define SUC_ROLL_LONGS_READY		(6450.0f)//

/*===================================����Yaw����ת���Ƕ�===============================*/
#define SUC_YAW_LONGS_YRES		  (0.0f)//
#define SUC_YAW_LONGS_CENTER		(0.0f)//

/*========================================ң��������======================================*/
#define CH4_TRIGGER_VAL         (550)

/*==========================================PC����============================================*/
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
	RUN_RESET = 0,  //��λ
	RUN_TURNING,   //�м�ת������
	RUN_NORMAL,     //��������
	
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
	MotorControl_t motorCtrl;  //Ҫ���Ƶĵ���ṹ�����
	
	eRunState runState;//����״̬
	eRunState LastrunState;
	
	eWorkState workStste;//����״̬��ȡ������󡢿սӡ��һ��ȣ�
	eWorkState LastworkStste;
	
	eCtrlMode ctrlMode;  //����ģʽ
	eCtrlMode LastctrlMode;
	

}ControlTask_t;

extern ControlTask_t ctrlTask;
void task_control_Create(void);
const ControlTask_t* get_Makemoney_Point(void);
#endif
