#include "gpio.h" 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//////////////////////////////////////////////////////////////////////////////////	 

void GPIO_Init_Configuration(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
	
/*******************************************使能时钟**********************************************/  
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
  
/******************************************输入**********************************************/ 
    GPIO_InitStruct.GPIO_Pin = L_UP_KEY_PIN | R_UP_KEY_PIN | WAREHOUSE_PIN | SUCTION_PITCH_KEY_PIN | \
	                             SUCTION_YAW_KEY_PIN | SUCTION_ROLL_KEY_PIN | INFRARAD_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_Init(GPIOD, &GPIO_InitStruct);

/******************************************输出**********************************************/ 
    GPIO_InitStruct.GPIO_Pin = IMPLICATION_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_Init(GPIOD, &GPIO_InitStruct);	

    GPIO_InitStruct.GPIO_Pin = FRAME_PIN | CLAMP_STRETCH_PIN | MAGNETIC_PIN | \
		                           CLAMP_PIN | SUCTION_STRETCH_PIN | GIMBAL_UP_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	  IMPLICATION(OFF);
		Air_Reset();
}

void Air_Reset(void)
{
		FRAME(OFF);
		CLAMP_STRETCH(OFF);
		MAGNETIC(OFF);
		CLAMP(ON);
		SUCTION_STRETCH(OFF);
		GIMBAL_UP(OFF);
}

