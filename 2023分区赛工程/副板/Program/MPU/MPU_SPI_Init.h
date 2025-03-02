#ifndef __MPU_SPI_H_
#define __MPU_SPI_H_

#include "stm32f4xx.h"
//#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"

void SPI1_Init(void);			 //��ʼ��SPI1��
void SPI2_Init(void);      //��ʼ��SPI2��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);//����SPI2�ٶ�  
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1���߶�дһ���ֽ�
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2���߶�дһ���ֽ�
void SPI2_WriteByte(u8 TxData);  //SPI2дһ���ֽ�
u8 SPI2_ReadByte(void);          //SPI2��һ���ֽ�
void spidmainit(void);
enum {SPICOM1,SPICOM2,SPICOM3};
typedef struct 
{
  SPI_TypeDef        *SPIx;
	SemaphoreHandle_t  WxBinarySemaphore;
	QueueHandle_t      WxMessage_Queue;
	QueueHandle_t      RxMessage_Queue;	
}SPI_DataStruct;





extern SPI_DataStruct   xSPI_Handler[3];
extern xSemaphoreHandle MPUsemaphore;
#endif

