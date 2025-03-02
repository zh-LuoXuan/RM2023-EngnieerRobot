#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"
#include "stm32f4xx.h"


/*************************************************���Ŷ���************************************************************/

#define L_UP_KEY_PIN                GPIO_Pin_1  //D5
#define R_UP_KEY_PIN                GPIO_Pin_0  //D6
#define L_STRETCH_KEY_PIN           GPIO_Pin_3  //D7
#define R_STRETCH_KEY_PIN           GPIO_Pin_2  //D8
#define TOP_PITCH_KEY_PIN           GPIO_Pin_5  //D9

#define TOP_ROLL_KEY_PIN            GPIO_Pin_4  //D10
#define L_RESCUE_KEY_PIN            GPIO_Pin_7  //D11
#define R_RESCUE_KEY_PIN            GPIO_Pin_6  //D12

/*************************************************΢������*************************************************************/

#define L_UP_KEY      	GPIO_ReadInputDataBit(GPIOD, L_UP_KEY_PIN)         //��̧�����΢������       ˿ӡ��D5
#define R_UP_KEY      	GPIO_ReadInputDataBit(GPIOD, R_UP_KEY_PIN)         //��̧�����΢������       ˿ӡ��D6
#define L_STRETCH_KEY 	GPIO_ReadInputDataBit(GPIOD, L_STRETCH_KEY_PIN)    //��ǰ����΢������       ˿ӡ��D7
#define R_STRETCH_KEY 	GPIO_ReadInputDataBit(GPIOD, R_STRETCH_KEY_PIN)    //��ǰ����΢������       ˿ӡ��D8
#define TOP_PITCH_KEY   GPIO_ReadInputDataBit(GPIOD, TOP_PITCH_KEY_PIN)    //����pitch����΢������  ˿ӡ��D9

#define TOP_ROLL_KEY 	  GPIO_ReadInputDataBit(GPIOD, TOP_ROLL_KEY_PIN)     //����roll����΢������   ˿ӡ��D10
#define L_RESCUE_KEY  	GPIO_ReadInputDataBit(GPIOD, L_RESCUE_KEY_PIN)     //���Ԯ���΢������       ˿ӡ��D11
#define R_RESCUE_KEY  	GPIO_ReadInputDataBit(GPIOD, R_RESCUE_KEY_PIN)     //�Ҿ�Ԯ���΢������       ˿ӡ��D12


#define ON  0
#define OFF 1


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//����Ϊ�ߵ�ƽ
#define digitalLo(p,i)			 {p->BSRRH=i;}		//����͵�ƽ
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//�����ת״̬

void GPIO_Init_Configuration(void);
#endif
