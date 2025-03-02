#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"

#include "stm32f4xx.h"

/*************************************************���Ŷ���************************************************************/
#define TOPMOVE_KEY_PIN             GPIO_Pin_0   //PA0
#define VACUUM_KEY_PIN              GPIO_Pin_13  //PB13
#define PRESSURE_RELIEF_PIN         GPIO_Pin_11  //PB11
/*************************************************΢������*************************************************************/

#define TOPMOVE_KEY 	              GPIO_ReadInputDataBit(GPIOA, TOPMOVE_KEY_PIN) //�����ƶ����΢������     ˿ӡ��PA0

/*************************************************��ձ�*************************************************************/

#define VACUUM_ON  								  GPIO_ResetBits(GPIOB,VACUUM_KEY_PIN)   //PB11
#define VACUUM_OFF 								  GPIO_SetBits(GPIOB,VACUUM_KEY_PIN)

/*************************************************йѹ��*************************************************************/

#define PRESSURE_ON  								  GPIO_ResetBits(GPIOB,PRESSURE_RELIEF_PIN)   //PB13
#define PRESSURE_OFF 								  GPIO_SetBits(GPIOB,PRESSURE_RELIEF_PIN)


void GPIO_Init_Configuration(void);
#endif
