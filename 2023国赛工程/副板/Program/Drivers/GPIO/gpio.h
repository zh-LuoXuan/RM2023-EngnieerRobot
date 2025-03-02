#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"
#include "stm32f4xx.h"


/*************************************************���Ŷ���************************************************************/
//����
 //GPIOD
#define L_UP_KEY_PIN                GPIO_Pin_1  //�ڿ����̧�����΢������    ˿ӡ��D5
#define R_UP_KEY_PIN                GPIO_Pin_0  //�ڿ����̧�����΢������    ˿ӡ��D6
#define SUCTION_PITCH_KEY_PIN       GPIO_Pin_3  //����pitch����΢������     ˿ӡ��D7
#define SUCTION_ROLL_KEY_PIN        GPIO_Pin_2  //����roll����΢������      ˿ӡ��D8
#define SUCTION_YAW_KEY_PIN         GPIO_Pin_5  //����yaw����΢������       ˿ӡ��D9
#define INFRARAD_PIN                GPIO_Pin_7  //���⴫����                  ˿ӡ��D11
#define WAREHOUSE_PIN               GPIO_Pin_3  //��ʯ���΢������            ˿ӡ��D12
    
//���	
 //GPIOD
#define IMPLICATION_PIN             GPIO_Pin_4  //�������ȼ̵���              ˿ӡ��D10
 //GPIOE
#define FRAME_PIN                   GPIO_Pin_8   //������������             ˿ӡ��PE8
#define CLAMP_STRETCH_PIN           GPIO_Pin_9   //��צǰ������               ˿ӡ��PE9
#define SUCTION_STRETCH_PIN         GPIO_Pin_10  //����ǰ������               ˿ӡ��PE10
#define CLAMP_PIN                   GPIO_Pin_11  //��צ�н�����               ˿ӡ��PE11
#define MAGNETIC_PIN                GPIO_Pin_12  //��������                   ˿ӡ��PE12
#define GIMBAL_UP_PIN               GPIO_Pin_13  //��̨��������               ˿ӡ��PE13

/*************************************************��GPIO*************************************************************/

#define IF_UP_KEY_L         	      ((GPIO_ReadInputDataBit(GPIOD, L_UP_KEY_PIN))          ==  1 )    
#define IF_UP_KEY_R          	      ((GPIO_ReadInputDataBit(GPIOD, R_UP_KEY_PIN))          ==  1 ) 
#define IF_SUCTION_PITCH_KEY 	      ((GPIO_ReadInputDataBit(GPIOD, SUCTION_PITCH_KEY_PIN)) ==  1 )
#define IF_SUCTION_YAW_KEY          ((GPIO_ReadInputDataBit(GPIOD, SUCTION_YAW_KEY_PIN))   ==  1 )  
#define IF_SUCTION_ROLL_KEY         ((GPIO_ReadInputDataBit(GPIOD, SUCTION_ROLL_KEY_PIN))  ==  0 ) 
#define IF_INFRARAD          	      ((GPIO_ReadInputDataBit(GPIOD, INFRARAD_PIN))          ==  0 )     
#define IF_WAREHOUSE_KEY     	      ((GPIO_ReadInputDataBit(GPIOD, WAREHOUSE_PIN))         ==  1 )    


/**********************************************�����ƽ��ת**************************************************************/

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

/**********************************************��������*************************************************************/

#define IMPLICATION(a) 	   \
					if (a)	\
					GPIO_ResetBits(GPIOD,IMPLICATION_PIN); \
					else		\
					GPIO_SetBits(GPIOD,IMPLICATION_PIN);

/**********************************************��·****************************************************************/

//������������			
#define FRAME(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,FRAME_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,FRAME_PIN);

//��צǰ������					
#define CLAMP_STRETCH(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,CLAMP_STRETCH_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,CLAMP_STRETCH_PIN);

//��������    
#define MAGNETIC(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,MAGNETIC_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,MAGNETIC_PIN);

//��צ�н�����					
#define CLAMP(a) 	   \
					if (a)	\
					GPIO_ResetBits(GPIOE,CLAMP_PIN); \
					else		\
					GPIO_SetBits(GPIOE,CLAMP_PIN);					

//����ǰ������
#define SUCTION_STRETCH(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,SUCTION_STRETCH_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,SUCTION_STRETCH_PIN);

//��̨��������					
#define GIMBAL_UP(a) 	   \
					if (a)	\
					GPIO_SetBits(GPIOE,WAREHOUSE_PIN); \
					else		\
					GPIO_ResetBits(GPIOE,WAREHOUSE_PIN);					

/********************************************************************************************************************/
					
/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//����Ϊ�ߵ�ƽ
#define digitalLo(p,i)			 {p->BSRRH=i;}		//����͵�ƽ
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//�����ת״̬

void Air_Reset(void);
void GPIO_Init_Configuration(void);
#endif
