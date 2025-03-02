#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"

#include "stm32f4xx.h"

/*************************************************引脚定义************************************************************/
#define TOPMOVE_KEY_PIN             GPIO_Pin_0   //PA0
#define VACUUM_KEY_PIN              GPIO_Pin_13  //PB13
#define PRESSURE_RELIEF_PIN         GPIO_Pin_11  //PB11
/*************************************************微动开关*************************************************************/

#define TOPMOVE_KEY 	              GPIO_ReadInputDataBit(GPIOA, TOPMOVE_KEY_PIN) //吸盘移动电机微动开关     丝印：PA0

/*************************************************真空泵*************************************************************/

#define VACUUM_ON  								  GPIO_ResetBits(GPIOB,VACUUM_KEY_PIN)   //PB11
#define VACUUM_OFF 								  GPIO_SetBits(GPIOB,VACUUM_KEY_PIN)

/*************************************************泄压阀*************************************************************/

#define PRESSURE_ON  								  GPIO_ResetBits(GPIOB,PRESSURE_RELIEF_PIN)   //PB13
#define PRESSURE_OFF 								  GPIO_SetBits(GPIOB,PRESSURE_RELIEF_PIN)


void GPIO_Init_Configuration(void);
#endif
