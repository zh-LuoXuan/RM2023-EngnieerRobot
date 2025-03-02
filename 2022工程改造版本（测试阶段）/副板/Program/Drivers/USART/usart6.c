#include "usart6.h"
#include "judge_Task.h"
u8 Judge_Buffer[JUDGE_BUFFER_LEN];	
Judge_FLAG Judge_Flag;
QueueHandle_t TxCOM6;
QueueHandle_t RxCOM6;
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART6->SR & 0X40) == 0); //循环发送,直到发送完毕

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
        /* 创建SEND消息队列 */
        TxCOM6 = xQueueCreate(50, sizeof(u8));

        if( TxCOM6 == 0 ) /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
        {
        }

        /* 创建REVICE消息队列 */
        RxCOM6 = xQueueCreate(50, sizeof(u8));

        if( RxCOM6 == 0 ) /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
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

//串口总线空闲中断：在检测到接收数据之后，数据总线上一个字节的时间内没有接收到数据则发生中断
//USART1 + DMA + IDLE方式，可以接收一帧一帧的数据 
u8 RES;
void USART6_IRQHandler(void)
{
  
//    BaseType_t  TaskWoken;


    if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
			        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
        (void)USART6->SR;
        (void)USART6->DR;

        RES = USART_ReceiveData(USART6);	//读取接收到的数据

					if(Judge_Flag.Flag==0 && RES==FRAME_HEADER)	//帧头的第一个字节
		{
			Judge_Flag.Flag=1;
			Judge_Buffer[0]=RES;
		}
		else if(Judge_Flag.Flag==1)				//长度
		{
			Judge_Flag.Flag=2;
			Judge_Buffer[1]=RES;
		}
		else if(Judge_Flag.Flag==2)				//长度
		{
			Judge_Flag.Flag=3;
			Judge_Buffer[2]=RES;
			Judge_Flag.data_len=(u16)(RES<<8)|Judge_Buffer[1]+ LEN_CMDID + LEN_TAIL;
			if(Judge_Flag.data_len>100)
			{
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==3)				//包序号
		{
			Judge_Flag.Flag=4;
			Judge_Buffer[3]=RES;
		}
		else if(Judge_Flag.Flag==4)				//CRC8
		{
			Judge_Flag.Flag=5;
			Judge_Buffer[4]=RES;
			if (verify_crc8_check_sum( Judge_Buffer, HEADER_LEN ) != NULL)	//CRC8校验
			{
				Judge_Flag.data_cnt=0;
			}
			else				//校验没有通过，重新开始
			{
				Judge_Flag.data_cnt=0;
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==5 && Judge_Flag.data_len>0)	//开始接受数据
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
			Judge_DataVerify(Judge_Buffer);		//帧头部分检测完毕，读取裁判系统数据
			Judge_Flag.Flag=0;								//其他数据清零
			Judge_Flag.data_len=0;
			Judge_Flag.data_cnt=0;
		}
//        //clear the idle pending flag         
// 
//        USART_ClearITPendingBit(USART6, USART_IT_RXNE);

//	    
//        Res = USART6->DR;	//读取接收到的数据
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
	//将进行帧头与整帧数据CRC校验  帧头CRC8  整帧CRC16
	if(verify_crc16_check_sum((uint8_t *)frame, HEADER_LEN + CMD_LEN + frame->FrameHeader.DataLength + CRC_LEN))
	{
		judgement_data_handler(Buff);  //通过校验进行数据解析
	}
 }
}

