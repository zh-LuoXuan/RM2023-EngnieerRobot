#ifndef __USART6_H
#define __USART6_H

#include "sys.h"
#include "stm32f4xx.h"
#include "freeRTOS.h"
#include "queue.h"


#define DBUS_RX_BUF_NUM     30u
#define RC_FRAME_LENGTH     18u

//typedef __packed struct
//{
//        __packed struct
//        {
//                int16_t ch[5];
//                char s[2];
//        } rc;

//} RC_ctrl_t;

/* TX */
#define    GPIO_TX                   GPIOC
#define    GPIO_PIN_TX               GPIO_Pin_6
#define    GPIO_PINSOURCE_TX         GPIO_PinSource6
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOC

/* RX */
#define    GPIO_RX                   GPIOC
#define    GPIO_PIN_RX               GPIO_Pin_7
#define    GPIO_PINSOURCE_RX         GPIO_PinSource7
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOC

void USART6_Init(u32 bound);
//static void DBUS_TO_RC(volatile const uint8_t *dbus_buf, RC_ctrl_t *rc_ctrl);

void vCom2RxTxTest( void );
//volatile uint8_t *pucCom2ReadBuffer( void );

void USART1_DMA_Init( void );

void Judge_restart(void);


#endif

