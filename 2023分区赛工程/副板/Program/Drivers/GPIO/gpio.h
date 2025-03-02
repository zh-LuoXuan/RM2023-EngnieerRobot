#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"
#include "stm32f4xx.h"


/*************************************************引脚定义************************************************************/

#define L_UP_KEY_PIN                GPIO_Pin_1  //D5
#define R_UP_KEY_PIN                GPIO_Pin_0  //D6
#define L_STRETCH_KEY_PIN           GPIO_Pin_3  //D7
#define R_STRETCH_KEY_PIN           GPIO_Pin_2  //D8
#define TOP_PITCH_KEY_PIN           GPIO_Pin_5  //D9

#define TOP_ROLL_KEY_PIN            GPIO_Pin_4  //D10
#define L_RESCUE_KEY_PIN            GPIO_Pin_7  //D11
#define R_RESCUE_KEY_PIN            GPIO_Pin_6  //D12

/*************************************************微动开关*************************************************************/

#define L_UP_KEY      	GPIO_ReadInputDataBit(GPIOD, L_UP_KEY_PIN)         //左抬升电机微动开关       丝印：D5
#define R_UP_KEY      	GPIO_ReadInputDataBit(GPIOD, R_UP_KEY_PIN)         //右抬升电机微动开关       丝印：D6
#define L_STRETCH_KEY 	GPIO_ReadInputDataBit(GPIOD, L_STRETCH_KEY_PIN)    //左前伸电机微动开关       丝印：D7
#define R_STRETCH_KEY 	GPIO_ReadInputDataBit(GPIOD, R_STRETCH_KEY_PIN)    //右前伸电机微动开关       丝印：D8
#define TOP_PITCH_KEY   GPIO_ReadInputDataBit(GPIOD, TOP_PITCH_KEY_PIN)    //吸盘pitch轴电机微动开关  丝印：D9

#define TOP_ROLL_KEY 	  GPIO_ReadInputDataBit(GPIOD, TOP_ROLL_KEY_PIN)     //吸盘roll轴电机微动开关   丝印：D10
#define L_RESCUE_KEY  	GPIO_ReadInputDataBit(GPIOD, L_RESCUE_KEY_PIN)     //左救援电机微动开关       丝印：D11
#define R_RESCUE_KEY  	GPIO_ReadInputDataBit(GPIOD, R_RESCUE_KEY_PIN)     //右救援电机微动开关       丝印：D12


#define ON  0
#define OFF 1


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 {p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态

void GPIO_Init_Configuration(void);
#endif
