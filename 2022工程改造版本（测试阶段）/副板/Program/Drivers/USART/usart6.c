#include "usart6.h"
#include "judge_Task.h"
u8 Judge_Buffer[JUDGE_BUFFER_LEN];	
Judge_FLAG Judge_Flag;
QueueHandle_t TxCOM6;
QueueHandle_t RxCOM6;
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((USART6->SR & 0X40) == 0); //ѭ������,ֱ���������

    USART6->DR = (u8) ch;
    return ch;
}
#endif
///* TX */
//#define    GPIO_TX                   GPIOC
//#define    GPIO_PIN_TX               GPIO_Pin_6
//#define    GPIO_PINSOURCE_TX         GPIO_PinSource6
//#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOC

///* RX */
//#define    GPIO_RX                   GPIOC
//#define    GPIO_PIN_RX               GPIO_Pin_7
//#define    GPIO_PINSOURCE_RX         GPIO_PinSource7
//#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOC
///*
//*/

void USART6_Init( u32 bound )
{
    USART_InitTypeDef  xUsartInit;
    GPIO_InitTypeDef   xGpioInit;

    NVIC_InitTypeDef   xNvicInit;

    RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART6, ENABLE );

    GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_USART6 );
    GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_USART6 );

    xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
    xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
    xGpioInit.GPIO_OType = GPIO_OType_PP;
    xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    xGpioInit.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init( GPIO_TX, &xGpioInit );

    xGpioInit.GPIO_Pin = GPIO_PIN_RX;
    GPIO_Init( GPIO_RX, &xGpioInit );

    xUsartInit.USART_BaudRate            = bound;
    xUsartInit.USART_WordLength          = USART_WordLength_8b;
    xUsartInit.USART_StopBits            = USART_StopBits_1;
    xUsartInit.USART_Parity              = USART_Parity_No;
    xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx; //
    xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;


    USART_Init( USART6, &xUsartInit );
    USART_Cmd( USART6, ENABLE );

    USART_ITConfig( USART6, USART_IT_RXNE, ENABLE  );

    {
        /* ����SEND��Ϣ���� */
        TxCOM6 = xQueueCreate(50, sizeof(u8));

        if( TxCOM6 == 0 ) /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
        {
        }

        /* ����REVICE��Ϣ���� */
        RxCOM6 = xQueueCreate(50, sizeof(u8));

        if( RxCOM6 == 0 ) /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
        {
        }
    }

    USART_Cmd(USART6, ENABLE);

    xNvicInit.NVIC_IRQChannel                    = USART6_IRQn;
    xNvicInit.NVIC_IRQChannelPreemptionPriority  = 9;
    xNvicInit.NVIC_IRQChannelSubPriority         = 0;
    xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init( &xNvicInit );

}

//�������߿����жϣ��ڼ�⵽��������֮������������һ���ֽڵ�ʱ����û�н��յ����������ж�
//USART1 + DMA + IDLE��ʽ�����Խ���һ֡һ֡������ 
u8 RES;
void USART6_IRQHandler(void)
{
  
//    BaseType_t  TaskWoken;


    if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
			        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
        (void)USART6->SR;
        (void)USART6->DR;

        RES = USART_ReceiveData(USART6);	//��ȡ���յ�������

					if(Judge_Flag.Flag==0 && RES==FRAME_HEADER)	//֡ͷ�ĵ�һ���ֽ�
		{
			Judge_Flag.Flag=1;
			Judge_Buffer[0]=RES;
		}
		else if(Judge_Flag.Flag==1)				//����
		{
			Judge_Flag.Flag=2;
			Judge_Buffer[1]=RES;
		}
		else if(Judge_Flag.Flag==2)				//����
		{
			Judge_Flag.Flag=3;
			Judge_Buffer[2]=RES;
			Judge_Flag.data_len=(u16)(RES<<8)|Judge_Buffer[1]+ LEN_CMDID + LEN_TAIL;
			if(Judge_Flag.data_len>100)
			{
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==3)				//�����
		{
			Judge_Flag.Flag=4;
			Judge_Buffer[3]=RES;
		}
		else if(Judge_Flag.Flag==4)				//CRC8
		{
			Judge_Flag.Flag=5;
			Judge_Buffer[4]=RES;
			if (verify_crc8_check_sum( Judge_Buffer, HEADER_LEN ) != NULL)	//CRC8У��
			{
				Judge_Flag.data_cnt=0;
			}
			else				//У��û��ͨ�������¿�ʼ
			{
				Judge_Flag.data_cnt=0;
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==5 && Judge_Flag.data_len>0)	//��ʼ��������
		{
			Judge_Flag.data_len--;
//			Judge_Flag.data_cnt=Judge_Flag.data_cnt+1;
			Judge_Buffer[LEN_HEADER+Judge_Flag.data_cnt++]=RES;
			if(Judge_Flag.data_len==0)
			{
				Judge_Flag.Flag=6;
			}	
		}
		else 
		{
			Judge_Flag.Flag=0;
			Judge_Flag.data_len=0;
			Judge_Flag.data_cnt=0;
		}
		if(Judge_Flag.Flag==6)
		{
			Judge_DataVerify(Judge_Buffer);		//֡ͷ���ּ����ϣ���ȡ����ϵͳ����
			Judge_Flag.Flag=0;								//������������
			Judge_Flag.data_len=0;
			Judge_Flag.data_cnt=0;
		}
//        //clear the idle pending flag         
// 
//        USART_ClearITPendingBit(USART6, USART_IT_RXNE);

//	    
//        Res = USART6->DR;	//��ȡ���յ�������
////			DetectHook(JudgementWDG);
////       if(xQueueIsQueueFullFromISR(RxCOM6)==pdFALSE)   
//           xQueueSendFromISR(RxCOM6, &Res, &TaskWoken);
//		    	(void)USART6->SR;
//         (void)USART6->DR;
    }
		if(USART_GetITStatus(USART6,USART_IT_ORE_RX)!=RESET)
		{
        (void)USART6->SR;
        (void)USART6->DR;
		}

	
		   
}
void Judge_DataVerify(u8 *Buff)
{
	Dateframe_t	*frame;
  if(Buff!=NULL)
  {
	frame=(Dateframe_t *)Buff;
	//������֡ͷ����֡����CRCУ��  ֡ͷCRC8  ��֡CRC16
	if(verify_crc16_check_sum((uint8_t *)frame, HEADER_LEN + CMD_LEN + frame->FrameHeader.DataLength + CRC_LEN))
	{
		judgement_data_handler(Buff);  //ͨ��У��������ݽ���
	}
 }
}

