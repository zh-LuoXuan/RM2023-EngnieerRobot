#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"
#include "CAN_Receive.h"
#include "stm32f4xx.h"

//////////////////////////////////////////////////////////////////////////////////	 
			  
////////////////////////////////////////////////////////////////////////////////// 	

//引脚定义
/*******************************************************/

#define LED1_PIN                  GPIO_Pin_0

#define LED2_PIN                  GPIO_Pin_1

#define LED3_PIN                  GPIO_Pin_2

#define LED4_PIN                  GPIO_Pin_3

#define LED5_PIN                  GPIO_Pin_4

#define LED6_PIN                  GPIO_Pin_5

#define LED7_PIN                  GPIO_Pin_6

#define LED8_PIN                  GPIO_Pin_7


#define LED_GPIO_CLK             RCC_AHB1Periph_GPIOD
#define LED_GPIO_PORT            GPIOD

/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  0
#define OFF 1


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 {p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态


/* 带参宏，可以像内联函数一样使用 */

/*================== LED ====================*/

#define LED1(a)	  \
          if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED1_PIN)

#define LED2(a)	  \
					if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED2_PIN)

#define LED3(a)	  \
					if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED3_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED3_PIN)

#define LED4(a)	  \
					if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED4_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED4_PIN)
					
#define LED5(a)	  \
					if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED5_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED5_PIN)

#define LED6(a)	  \
					if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED6_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED6_PIN)

#define LED7(a)	  \
					if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED7_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED7_PIN)

#define LED8(a)	  \
					if (a)	\
					GPIO_SetBits(LED_GPIO_PORT,LED8_PIN);\
					else		\
					GPIO_ResetBits(LED_GPIO_PORT,LED8_PIN)
					
/* 定义控制IO的宏 */

#define LED1_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED1_PIN)
#define LED2_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED2_PIN)
#define LED3_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED3_PIN)
#define LED4_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED4_PIN)
#define LED5_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED5_PIN)
#define LED6_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED6_PIN)
#define LED7_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED7_PIN)
#define LED8_TOGGLE			GPIO_ToggleBits(LED_GPIO_PORT,LED8_PIN)


#define LED_ALL_OFF	\
					LED1(OFF);\
					LED2(OFF);\
					LED3(OFF);\
          LED4(OFF);

#define LED_ALL_TOGGLE	\
					LED1_TOGGLE;\
					LED2_TOGGLE;\
					LED3_TOGGLE;\
          LED4_TOGGLE;

#define LED_ALL_ON	\
					LED1(ON);\
					LED2(ON);\
					LED3(ON);\
          LED4(ON);


/*================== Laser ====================*/
					
#define LASER  GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)

/*================== Key ====================*/

#define KEY1 	GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)

#define PRESS  0
#define RELEASE 1

					
/*======================= Air ============ =============*/
					
#define PD4(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_4); \
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_4)
#define PD6(a)	  \
					if (a)	\
					GPIO_ResetBits(GPIOD,GPIO_Pin_6); \
					else		\
					GPIO_SetBits(GPIOD,GPIO_Pin_6)
#define PD7(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_7); \
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_7)					
					
#define PE11(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_11); \
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_11)

#define PE12(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_12); \
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_12)

#define PE13(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_13); \
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_13)

#define PE14(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_14); \
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_14)			

#define PE15(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_15); \
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_15)						
										
/*===============================================*/										
										
#define Air_Cylinder_RESET	\
					PD6 (OFF);\
					PD7 (ON);\
					PD4 (ON);\
					PE11(ON);\
					PE12(ON);\
					PE13(ON);\
					PE14(ON);\
					PE15(ON);
					
#define Air_Cylinder_ALL_DOWN	\
					PD6 (OFF);\
					PD7 (OFF);\
					PD4 (ON);\
					PE11(ON);\
					PE12(ON);\
					PE13(ON);\
					PE14(ON);\
					PE15(ON);

#define Air_Cylinder_ALL_UP	\
					PD6 (OFF);\
					PD7 (OFF);\
					PD4 (OFF);\
					PE11(OFF);\
					PE12(OFF);\
					PE13(OFF);\
					PE14(OFF);\
					PE15(OFF);

				
/*================== 方案1 ====================*/	
/*

PE8——PE9 --> 框架升降
PE10 --> 夹手伸缩
PE11 --> 夹子开合
PE12 --> 救援钩子
PE13 --> 救援卡
PE14 --> 推矿石
PE15 --> 障碍快
*/

//#define FRAME_TOGGLE 		   GPIO_ToggleBits(GPIOE,GPIO_Pin_8 | GPIO_Pin_9)	
//#define STRETCH_TOGGLE     GPIO_ToggleBits(GPIOE,GPIO_Pin_10)	    
//#define CLAMP_TOGGLE       GPIO_ToggleBits(GPIOE,GPIO_Pin_11)  
//#define HOOK_TOGGLE        GPIO_ToggleBits(GPIOE,GPIO_Pin_12)
//#define CARD_TOGGLE        GPIO_ToggleBits(GPIOE,GPIO_Pin_13)
//#define PUSH_TOGGLE        GPIO_ToggleBits(GPIOE,GPIO_Pin_14)
//#define OBSTACLE_TOGGLE    GPIO_ToggleBits(GPIOE,GPIO_Pin_15)

//#define Frame_Up 	\
//					PE8 (OFF);\
//					PE9 (OFF);
//					
//#define Frame_Down 	\
//					PE8 (ON);\
//					PE9 (ON);


//#define Frame_1_2_Up 	   PE8(OFF)
//#define Frame_1_2_Down	 PE8(ON)
//#define Frame_3_4_Up 		 PE9(OFF)
//#define Frame_3_4_Down	 PE9(ON)


//#define Stretch_Up    PE10(OFF)
//#define Stretch_Down  PE10(ON)


//#define Clamp_Up			PE11(ON)
//#define Clamp_Down		PE11(OFF)


//#define Hook_Up       PE12(OFF)
//#define Hook_Down     PE12(ON)


//#define Card_Up			PE13(OFF)
//#define Card_Down		PE13(ON)


//#define Push_Up			PE14(OFF)
//#define Push_Down		PE14(ON)

//#define Obstacle_Up			PE15(OFF)
//#define Obstacle_Down		PE15(ON)


/*================== 方案2 ====================*/	
/*

PD6 --> 夹手伸缩
PD7 --> 夹子开合
PD4 --> 推矿石
PE11 --> 障碍快
PE12 --> 救援钩子
PE13 --> 框架升降
PE14 --> 矿石仓
PE15 --> 救援卡
*/

#define STRETCH_TOGGLE     GPIO_ToggleBits(GPIOD,GPIO_Pin_6)	    
#define CLAMP_TOGGLE       GPIO_ToggleBits(GPIOD,GPIO_Pin_7) 
#define PUSH_TOGGLE        GPIO_ToggleBits(GPIOD,GPIO_Pin_4)
#define HOOK_TOGGLE        GPIO_ToggleBits(GPIOE,GPIO_Pin_11)
#define OBSTACLE_TOGGLE    GPIO_ToggleBits(GPIOE,GPIO_Pin_12)
#define FRAME_TOGGLE 		   GPIO_ToggleBits(GPIOE,GPIO_Pin_13)	
#define WAREHOUSE_TOGGLE   GPIO_ToggleBits(GPIOE,GPIO_Pin_14)
#define CARD_TOGGLE        GPIO_ToggleBits(GPIOE,GPIO_Pin_15)


#define Stretch_Up 	   PD6(ON)//D12
#define Stretch_Down	 PD6(OFF)


#define Clamp_Up    PD7(ON)//D11
#define Clamp_Down  PD7(OFF)


#define Push_Up			PD4(OFF)//D10
#define Push_Down		PD4(ON)


#define Hook_Up		  PE11(OFF)
#define Hook_Down		PE11(ON)


#define Obstacle_Up		  PE12(OFF)
#define Obstacle_Down		PE12(ON)


#define Frame_Up			PE13(OFF)
#define Frame_Down		PE13(ON)


#define WareHouse_Up			PE14(OFF)
#define WareHouse_Down		PE14(ON)

#define Card_Up			PE15(OFF)
#define Card_Down		PE15(ON)

/*==========================*/
typedef enum
{
	Reset_unfinished = 0,
	Reset_Finish = 1,
}System_Reset_State;

typedef enum
{
	Sensor_NotTrigger = 0,
	Sensor_Trigger = 1,
}Sensor_State;

void Key_Init(void);
void Led_Init(void);
void Air_Cylinder(void);
void Laser_Init(void);
//u8 Key0(void);
u8 Key1(void);
void GPIO_Init_Configuration(void);
#endif
