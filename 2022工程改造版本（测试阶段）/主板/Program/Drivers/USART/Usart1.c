//����ϵͳͨѶʹ��USART1
#include "usart1.h"
#include "detect_task.h"
QueueHandle_t TxCOM1;
QueueHandle_t RxCOM1;
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
//#if 1
//#pragma import(__use_no_semihosting)
////��׼����Ҫ��֧�ֺ���
//struct __FILE
//{
//    int handle;
//};

//FILE __stdout;
////����_sys_exit()�Ա���ʹ�ð�����ģʽ
//void _sys_exit(int x)
//{
//    x = x;
//}
////�ض���fputc����
//int fputc(int ch, FILE *f)
//{
//    while((USART1->SR & 0X40) == 0); //ѭ������,ֱ���������

//    USART1->DR = (u8) ch;
//    return ch;
//}
//#endif
/* TX */
#define    GPIO_TX                   GPIOA
#define    GPIO_PIN_TX               GPIO_Pin_9
#define    GPIO_PINSOURCE_TX         GPIO_PinSource9
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOA

/* RX */
#define    GPIO_RX                   GPIOA
#define    GPIO_PIN_RX               GPIO_Pin_10
#define    GPIO_PINSOURCE_RX         GPIO_PinSource10
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOA
/*
*/

void Device_Usart1_ENABLE_Init( u32 bound )
{
    USART_InitTypeDef  xUsartInit;
    GPIO_InitTypeDef   xGpioInit;

    NVIC_InitTypeDef   xNvicInit;

    RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );

    GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_USART1 );
    GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_USART1 );

    xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
    xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
    xGpioInit.GPIO_OType = GPIO_OType_PP;
    xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init( GPIO_TX, &xGpioInit );

    xGpioInit.GPIO_Pin = GPIO_PIN_RX;
    GPIO_Init( GPIO_RX, &xGpioInit );

    xUsartInit.USART_BaudRate            = bound;
    xUsartInit.USART_WordLength          = USART_WordLength_8b;
    xUsartInit.USART_StopBits            = USART_StopBits_1;
    xUsartInit.USART_Parity              = USART_Parity_No;
    xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;


    USART_Init( USART1, &xUsartInit );
    USART_Cmd( USART1, ENABLE );

    USART_ITConfig( USART1, USART_IT_RXNE, ENABLE  );

    {
        /* ����SEND��Ϣ���� */
        TxCOM1 = xQueueCreate(45, sizeof(u8));

        if( TxCOM1 == 0 ) /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
        {
        }

        /* ����REVICE��Ϣ���� */
        RxCOM1 = xQueueCreate(45, sizeof(u8));

        if( RxCOM1 == 0 ) /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
        {
        }
    }

    USART_Cmd(USART1, ENABLE);

    xNvicInit.NVIC_IRQChannel                    = USART1_IRQn;
    xNvicInit.NVIC_IRQChannelPreemptionPriority  = 7;
    xNvicInit.NVIC_IRQChannelSubPriority         = 0;
    xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init( &xNvicInit );

}

//�������߿����жϣ��ڼ�⵽��������֮������������һ���ֽڵ�ʱ����û�н��յ����������ж�
//USART1 + DMA + IDLE��ʽ�����Խ���һ֡һ֡������
void USART1_IRQHandler(void)
{
    u8 Res;
    BaseType_t  TaskWoken;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        //clear the idle pending flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        (void)USART1->SR;
        (void)USART1->DR;

        Res = USART_ReceiveData(USART1);	//��ȡ���յ�������
        DetectHook(JudgementWDG);

        xQueueSendFromISR(RxCOM1, &Res, &TaskWoken);
    }
}






