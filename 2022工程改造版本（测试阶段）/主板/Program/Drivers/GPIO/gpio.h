#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"

#include "stm32f4xx.h"

//////////////////////////////////////////////////////////////////////////////////	 
			  
////////////////////////////////////////////////////////////////////////////////// 	

//���Ŷ���
/*******************************************************/

#define LED1_PIN                  GPIO_Pin_10

#define LED2_PIN                  GPIO_Pin_11

#define LED3_PIN                  GPIO_Pin_12

#define LED4_PIN                  GPIO_Pin_2

/************************************************************/


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  0
#define OFF 1

#define KEY0 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//����Ϊ�ߵ�ƽ
#define digitalLo(p,i)			 {p->BSRRH=i;}		//����͵�ƽ
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//�����ת״̬


/* ���κ꣬��������������һ��ʹ�� */

/*================== LED ====================*/

#define LED1(a)	  \
          if (a)	\
					GPIO_SetBits(GPIOC , LED1_PIN);\
					else		\
					GPIO_ResetBits(GPIOC , LED1_PIN)

#define LED2(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOC , LED2_PIN);\
					else		\
					GPIO_ResetBits(GPIOC , LED2_PIN)

#define LED3(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOC , LED3_PIN);\
					else		\
					GPIO_ResetBits(GPIOC , LED3_PIN)

#define LED4(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOD , LED4_PIN);\
					else		\
					GPIO_ResetBits(GPIOD , LED4_PIN)

/* �������IO�ĺ� */

#define LED1_TOGGLE			GPIO_ToggleBits(GPIOC , LED1_PIN)
#define LED2_TOGGLE			GPIO_ToggleBits(GPIOC , LED2_PIN)
#define LED3_TOGGLE			GPIO_ToggleBits(GPIOC , LED3_PIN)
#define LED4_TOGGLE			GPIO_ToggleBits(GPIOD , LED4_PIN)


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
void GPIO_Init_Configuration(void);
#endif
