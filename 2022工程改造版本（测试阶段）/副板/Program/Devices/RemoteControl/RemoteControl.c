/**
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "detect_task.h"

//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700



//ȡ������
static int16_t RC_abs(int16_t value);
//ң����������
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static void RC_MODE_CONTROL(RC_ctrl_t *rc_ctrl);
/*******************************************
ң�������Ʊ���
********************************************/



//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

//��ʼ��DMA������3
void remote_control_init(void)
{
    RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}
/*******************************************
����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
********************************************/

RC_ctrl_t rc_ctrl;// static
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

/*******************************************
����ң��������ģʽ��ͨ��ָ�봫�ݷ�ʽ������Ϣ
********************************************/
remote_mode_e INPUTMOD = RUN_STOP;   //����ģʽ�趨

const remote_mode_e *get_remote_mode_point(void)
{
    return &INPUTMOD;
}
/*******************************************
������Ʊ���
********************************************/
//������־λ
uint8_t Q_Flag = 0;
uint8_t E_Flag = 0;
uint8_t F_Flag = 0;
uint8_t G_Flag = 0;
uint8_t V_Flag = 0;
uint8_t R_Flag = 0;
uint8_t MOU_LEFT_F = 0;
uint8_t MOU_RIGH_F = 0;
uint8_t W_Flag = 0;
uint8_t A_Flag = 0;
uint8_t S_Flag = 0;
uint8_t D_Flag = 0;
uint8_t Shift_Flag = 2;
uint8_t Ctrl_Flag = 0;
//�ж�ң���������Ƿ����
uint8_t RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }

    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }

    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}
static void RC_data_clean(RC_ctrl_t *rc_ctrl)//������������
{
    rc_ctrl->rc.ch[0] = 0;
    rc_ctrl->rc.ch[1] = 0;
    rc_ctrl->rc.ch[2] = 0;
    rc_ctrl->rc.ch[3] = 0;
    rc_ctrl->rc.ch[4] = 0;
    rc_ctrl->rc.s[0] = RC_SW_DOWN;
    rc_ctrl->rc.s[1] = RC_SW_DOWN;
    rc_ctrl->mouse.x = 0;
    rc_ctrl->mouse.y = 0;
    rc_ctrl->mouse.z = 0;
    rc_ctrl->mouse.press_l = 0;
    rc_ctrl->mouse.press_r = 0;
    rc_ctrl->key.v = 0;
}
void slove_RC_lost(void)
{
    RC_data_clean(&rc_ctrl);//������������
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//�����ж�
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//���ָ����USART�ж��Ƿ���  �����ж�
    {
        USART_ReceiveData(USART2);//
	
    }
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//�����ж�
    {
//			LED2(ON);
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART2);

        if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
        {
            //��������DMA
            DMA_Cmd(DMA1_Stream5, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR |= DMA_SxCR_CT;
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
            DMA_Cmd(DMA1_Stream5, ENABLE);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
                RC_MODE_CONTROL( &rc_ctrl);
//                //��¼���ݽ���ʱ��
//                DetectHook(DBUSTOE);
            }
        }
        else
        {
//					LED2(OFF);
            //��������DMA
            DMA_Cmd(DMA1_Stream5, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
            DMA_Cmd(DMA1_Stream5, ENABLE);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
                RC_MODE_CONTROL( &rc_ctrl);
                //��¼���ݽ���ʱ��
                DetectHook(DBUSTOE);
            }
        }
				USART_ClearITPendingBit(USART2, USART_IT_IDLE);
    }
}

//ȡ������
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;        //!< Channel 2
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

static void RC_MODE_CONTROL(RC_ctrl_t *rc_ctrl)
{
//    if(rc_ctrl == NULL)
//    {
//        return;
//    }

//    if (rc_ctrl->rc.ch[0] == -660 && rc_ctrl->rc.ch[1] == -660 && rc_ctrl->rc.ch[2] == 660 && rc_ctrl->rc.ch[3] == -660) //�����ڰ����ǿ�Ƹ�λ
//    {
//        __set_FAULTMASK(1);//�ر������ж�
//        NVIC_SystemReset();//�����λ
//    }
//    
//    else if (switch_is_mid(rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(rc_ctrl->rc.s[ModeChannel_R]))//��������
//    {
//        INPUTMOD = KEYMOUSE_INPUT;//���̿���
//    }
//    else if (switch_is_mid(rc_ctrl->rc.s[ModeChannel_R]))//����
//    {
//        INPUTMOD = REMOTE_INPUT;//ң��������
//    }
//    else if (switch_is_down(rc_ctrl->rc.s[ModeChannel_L]) && switch_is_down(rc_ctrl->rc.s[ModeChannel_R]))//˫��
//    {
//        INPUTMOD = RUN_STOP;//ֹͣ
//    }
//		else if (switch_is_up(rc_ctrl->rc.s[ModeChannel_L]) && switch_is_up(rc_ctrl->rc.s[ModeChannel_R]))//˫��
//		{
//				INPUTMOD = CONTROL_P_Y_R;
//		}

}
/**
  * @brief  ��ȡң��������
  * @param  ң��������
  * @retval void
  */
void Get_RC_Data(RC_ctrl_t *RC_Ctl,CanRxMsg *msg)
{
	 /* Channel 0, 1, 2, 3 */
    RC_Ctl->rc.ch[0] = (msg->Data[0]<<8)| msg->Data[1];
    RC_Ctl->rc.ch[1] = (msg->Data[2]<<8)| msg->Data[3];
    RC_Ctl->rc.ch[2] = (msg->Data[4]<<8)| msg->Data[5];
    RC_Ctl->rc.ch[3] = (msg->Data[6]<<8)| msg->Data[7];	
	if (rc_ctrl.rc.ch[0] == -660 && rc_ctrl.rc.ch[1] == -660 && rc_ctrl.rc.ch[2] == 660 && rc_ctrl.rc.ch[3] == -660) //�����ڰ����ǿ�Ƹ�λ
    {
        __set_FAULTMASK(1);//�ر������ж�
        NVIC_SystemReset();//�����λ
    }
}


/**
  * @brief  ��ȡ�����������
  * @param  ��������
  * @retval void
  */
void Get_KEY_Data(RC_ctrl_t *RC_Ctl,CanRxMsg *msg)
{

	RC_Ctl->rc.ch[4] = (msg->Data[0]<<8)| msg->Data[1];
	RC_Ctl->key.v  = (msg->Data[2]<<8)| msg->Data[3];
	RC_Ctl->mouse.press_l = msg->Data[4];
	RC_Ctl->mouse.press_r = msg->Data[5];
	RC_Ctl->rc.s[0] = msg->Data[6];
	RC_Ctl->rc.s[1] = msg->Data[7];
	
	
}

