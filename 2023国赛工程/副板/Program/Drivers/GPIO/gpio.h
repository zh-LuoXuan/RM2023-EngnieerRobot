#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"
#include "stm32f4xx.h"


/*************************************************引脚定义************************************************************/
//输入
 //GPIOD
#define L_UP_KEY_PIN                GPIO_Pin_1  //内框架左抬升电机微动开关    丝印：D5
#define R_UP_KEY_PIN                GPIO_Pin_0  //内框架右抬升电机微动开关    丝印：D6
#define SUCTION_PITCH_KEY_PIN       GPIO_Pin_3  //吸盘pitch轴电机微动开关     丝印：D7
#define SUCTION_ROLL_KEY_PIN        GPIO_Pin_2  //吸盘roll轴电机微动开关      丝印：D8
#define SUCTION_YAW_KEY_PIN         GPIO_Pin_5  //吸盘yaw轴电机微动开关       丝印：D9
#define INFRARAD_PIN                GPIO_Pin_7  //红外传感器                  丝印：D11
#define WAREHOUSE_PIN               GPIO_Pin_3  //矿石检测微动开关            丝印：D12
    
//输出	
 //GPIOD
#define IMPLICATION_PIN             GPIO_Pin_4  //涵道风扇继电器              丝印：D10
 //GPIOE
#define FRAME_PIN                   GPIO_Pin_8   //外框架升降气缸             丝印：PE8
#define CLAMP_STRETCH_PIN           GPIO_Pin_9   //夹爪前伸气缸               丝印：PE9
#define SUCTION_STRETCH_PIN         GPIO_Pin_10  //吸盘前伸气缸               丝印：PE10
#define CLAMP_PIN                   GPIO_Pin_11  //夹爪夹紧气缸               丝印：PE11
#define MAGNETIC_PIN                GPIO_Pin_12  //磁耦气缸                   丝印：PE12
#define GIMBAL_UP_PIN               GPIO_Pin_13  //云台升降气缸               丝印：PE13

/*************************************************读GPIO*************************************************************/

#define IF_UP_KEY_L         	      ((GPIO_ReadInputDataBit(GPIOD, L_UP_KEY_PIN))          ==  1 )    
#define IF_UP_KEY_R          	      ((GPIO_ReadInputDataBit(GPIOD, R_UP_KEY_PIN))          ==  1 ) 
#define IF_SUCTION_PITCH_KEY 	      ((GPIO_ReadInputDataBit(GPIOD, SUCTION_PITCH_KEY_PIN)) ==  1 )
#define IF_SUCTION_YAW_KEY          ((GPIO_ReadInputDataBit(GPIOD, SUCTION_YAW_KEY_PIN))   ==  1 )  
#define IF_SUCTION_ROLL_KEY         ((GPIO_ReadInputDataBit(GPIOD, SUCTION_ROLL_KEY_PIN))  ==  0 ) 
#define IF_INFRARAD          	      ((GPIO_ReadInputDataBit(GPIOD, INFRARAD_PIN))          ==  0 )     
#define IF_WAREHOUSE_KEY     	      ((GPIO_ReadInputDataBit(GPIOD, WAREHOUSE_PIN))         ==  1 )    


/**********************************************输出电平反转**************************************************************/

#define IMPLICATION_TOGGLE          GPIO_ToggleBits(GPIOD,IMPLICATION_PIN)	    
#define FRAME_TOGGLE                GPIO_ToggleBits(GPIOE,FRAME_PIN) 
#define CLAMP_STRETCH_TOGGLE        GPIO_ToggleBits(GPIOE,CLAMP_STRETCH_PIN ) 
#define MAGNETIC_TOGGLE             GPIO_ToggleBits(GPIOE,MAGNETIC_PIN)
#define CLAMP_TOGGLE                GPIO_ToggleBits(GPIOE,CLAMP_PIN)
#define SUCTION_STRETCH_TOGGLE 	    GPIO_ToggleBits(GPIOE,SUCTION_STRETCH_PIN)	
#define GIMBAL_UP_TOGGLE            GPIO_ToggleBits(GPIOE,GIMBAL_UP_PIN)

/********************************************************************************************************************/

#define ON  1
#define OFF 0

/**********************************************涵道风扇*************************************************************/

#define IMPLICATION(a) 	   \
					if (a)	\
					GPIO_ResetBits(GPIOD,IMPLICATION_PIN); \
					else		\
					GPIO_SetBits(GPIOD,IMPLICATION_PIN);

/**********************************************气路****************************************************************/

//外框架升降气缸			
#define FRAME(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,FRAME_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,FRAME_PIN);

//夹爪前伸气缸					
#define CLAMP_STRETCH(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,CLAMP_STRETCH_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,CLAMP_STRETCH_PIN);

//磁耦气缸    
#define MAGNETIC(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,MAGNETIC_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,MAGNETIC_PIN);

//夹爪夹紧气缸					
#define CLAMP(a) 	   \
					if (a)	\
					GPIO_ResetBits(GPIOE,CLAMP_PIN); \
					else		\
					GPIO_SetBits(GPIOE,CLAMP_PIN);					

//吸盘前伸气缸
#define SUCTION_STRETCH(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,SUCTION_STRETCH_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,SUCTION_STRETCH_PIN);

//云台升降气缸					
#define GIMBAL_UP(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,WAREHOUSE_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,WAREHOUSE_PIN);					

/********************************************************************************************************************/
					
/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 {p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态

void Air_Reset(void);
void GPIO_Init_Configuration(void);
#endif
