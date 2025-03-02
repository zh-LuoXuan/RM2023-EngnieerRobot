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
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
  
 /******************************************微动开关**********************************************/ 
       GPIO_InitStruct.GPIO_Pin = L_UP_KEY_PIN | R_UP_KEY_PIN | L_STRETCH_KEY_PIN | R_STRETCH_KEY_PIN |    \
	                               TOP_PITCH_KEY_PIN | TOP_ROLL_KEY_PIN | L_RESCUE_KEY_PIN | R_RESCUE_KEY_PIN;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
			GPIO_Init(GPIOD, &GPIO_InitStruct);

}

