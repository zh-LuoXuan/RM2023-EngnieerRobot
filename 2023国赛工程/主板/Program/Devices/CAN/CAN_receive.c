#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"
#include "gpio.h" 

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
#include "RemoteControl.h"
#include "gimbal_task.h"

//���̵�����ݶ�ȡ
#define get_motor_measure(ptr, rx_message)  \
		{                                       \
				if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ; \
				else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ; \
				(ptr)->last_ecd = (ptr)->ecd; \
				(ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
				(ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 |(rx_message)->Data[3]); \
				(ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
				(ptr)->temperate = (rx_message)->Data[6];  \
				(ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd; 	\
				(ptr)->real_ecd=(ptr)->all_ecd/ECD_RATE;   \
		}
		

/**********************************************************************************
���ص�����������ݣ�ͨ��ָ�봫�ݷ�ʽ������Ϣ
***********************************************************************************/
 MotorMeasure_t  chassisMotor[4];

const MotorMeasure_t* get_Chassis_Measure_Point(uint8_t i)
{
    return &chassisMotor[i];
}

/**********************************************************************************
���ص�����������ݣ�ͨ��ָ�봫�ݷ�ʽ������Ϣ
***********************************************************************************/
EncoderProcess_t yawMotor, 
								 pitchMotor;

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const EncoderProcess_t* get_Yaw_EncoderProcess_Point(void)
{
    return &yawMotor;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const EncoderProcess_t* get_Pitch_EncoderProcess_Point(void)
{
    return &pitchMotor;
}


/*============================================Can�ص�����========================================*/
static void CAN1_Hook(CanRxMsg *RxMessage)
{
    switch (RxMessage->StdId)
    {
			case CHASSIS_M1_ID:
			case CHASSIS_M2_ID:
			case CHASSIS_M3_ID:
			case CHASSIS_M4_ID:
				{
					static uint8_t i = 0;
					//������ID��
					i = RxMessage->StdId - CHASSIS_M1_ID;
					//���������ݺ꺯��
					get_motor_measure(&chassisMotor[i], RxMessage);
					break;
				}
			case GIMBAL_YAW_ID:
				{
					EncoderProcess(&yawMotor, RxMessage);
					break;
				}
			case GIMBAL_PITCH_ID:
				{
					EncoderProcess(&pitchMotor, RxMessage);
					break;
				}
			default:
			{
				break;
			}
    }
}

static void CAN2_Hook(CanRxMsg *RxMessage)
{
	;
}

/*============================================Can�жϷ�����========================================*/
//can1
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_Hook(&rx1_message);
    }
}

//can2
void CAN2_RX1_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
        CAN2_Hook(&rx2_message);
    }
}


/*============================================CAN1 ����========================================*/

//CAN ���� 0x700��ID�����ݣ�������M3508�����������IDģʽ
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = CHASSIS_FAST_STDID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN1, &TxMessage);
}


/**
  * @brief  CAN1���͵��̿�������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
		CanTxMsg TxMessage;
		TxMessage.StdId = CHASSIS_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = data3>>8;;
		TxMessage.Data[5] = data3;
		TxMessage.Data[6] = data4>>8;;
		TxMessage.Data[7] = data4;

		CAN_Transmit(CAN1 , &TxMessage);
}

/**
  * @brief  CAN1������̨��������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_GIMBAL(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = GIMBAL_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}


/**
  * @brief  CAN1�򸱰巢��ң��������
  * @param  RC�ṹ��
  * @retval void
  */
void CAN1_CMD_RC(const RC_ctrl_t* RcData)
{
	
		CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_RCSTD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x08;							 

    TxMessage.Data[0]=(u8)((int16_t)RcData->rc.ch[0]>>8);
    TxMessage.Data[1]=(u8)((int16_t)RcData->rc.ch[0]);
    TxMessage.Data[2]=(u8)((int16_t)RcData->rc.ch[1]>>8);
    TxMessage.Data[3]=(u8)((int16_t)RcData->rc.ch[1]);
    TxMessage.Data[4]=(u8)((int16_t)RcData->rc.ch[2]>>8);
    TxMessage.Data[5]=(u8)((int16_t)RcData->rc.ch[2]);
    TxMessage.Data[6]=(u8)((int16_t)RcData->rc.ch[3]>>8);
    TxMessage.Data[7]=(u8)((int16_t)RcData->rc.ch[3]);

		CAN_Transmit(CAN1 , &TxMessage);
}


/**
  * @brief  CAN1�򸱰巢�ͼ�������
  * @param  RC�ṹ��
  * @retval void
  */
void CAN1_CMD_KEYMOUSE(const RC_ctrl_t* RcData)
{
		CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_KEYSTD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x08;							 

    TxMessage.Data[0]=(u8)((int16_t)RcData->rc.ch[4]>>8);
    TxMessage.Data[1]=(u8)((int16_t)RcData->rc.ch[4]);
    TxMessage.Data[2]=(u8)((int16_t)RcData->key.v>>8);
    TxMessage.Data[3]=(u8)((int16_t)RcData->key.v);
    TxMessage.Data[4]=(u8)(RcData->mouse.press_l);
    TxMessage.Data[5]=(u8)(RcData->mouse.press_r);
    TxMessage.Data[6]=(u8)RcData->rc.s[0];
    TxMessage.Data[7]=(u8)RcData->rc.s[1];

		CAN_Transmit(CAN1 , &TxMessage);
}


/**
  * @brief  CAN1�򸱰巢�����x��y��z����
  * @param  RC�ṹ��
  * @retval void
  */
void CAN1_CMD_MOUSEXYZ(const RC_ctrl_t* RcData)
{
		CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_MOUSE_XYZSTD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x08;							 

    TxMessage.Data[0]=(u8)((int16_t)RcData->mouse.x >>8);
    TxMessage.Data[1]=(u8)((int16_t)RcData->mouse.x );
    TxMessage.Data[2]=(u8)((int16_t)RcData->mouse.y >>8);
    TxMessage.Data[3]=(u8)((int16_t)RcData->mouse.y );
    TxMessage.Data[4]=(u8)((int16_t)RcData->mouse.z >>8);;
    TxMessage.Data[5]=(u8)((int16_t)RcData->mouse.z );
    TxMessage.Data[6]=0;
    TxMessage.Data[7]=0;

		CAN_Transmit(CAN1 , &TxMessage);
}

/*============================================CAN2 ����========================================*/
/**
  * @brief  CAN2���͵��̿�������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
		CanTxMsg TxMessage;
		TxMessage.StdId = CHASSIS_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = data3>>8;;
		TxMessage.Data[5] = data3;
		TxMessage.Data[6] = data4>>8;;
		TxMessage.Data[7] = data4;

		CAN_Transmit(CAN2 , &TxMessage);
}

/**
  * @brief  CAN2������̨��������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_GIMBAL(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = GIMBAL_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}

/**
  * @brief  CAN2�򸱰巢��ң��������
  * @param  RC�ṹ��
  * @retval void
  */
void CAN2_CMD_RC(const RC_ctrl_t* RcData)
{
	
		CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_RCSTD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x08;							 

    TxMessage.Data[0]=(u8)((int16_t)RcData->rc.ch[0]>>8);
    TxMessage.Data[1]=(u8)((int16_t)RcData->rc.ch[0]);
    TxMessage.Data[2]=(u8)((int16_t)RcData->rc.ch[1]>>8);
    TxMessage.Data[3]=(u8)((int16_t)RcData->rc.ch[1]);
    TxMessage.Data[4]=(u8)((int16_t)RcData->rc.ch[2]>>8);
    TxMessage.Data[5]=(u8)((int16_t)RcData->rc.ch[2]);
    TxMessage.Data[6]=(u8)((int16_t)RcData->rc.ch[3]>>8);
    TxMessage.Data[7]=(u8)((int16_t)RcData->rc.ch[3]);

		CAN_Transmit(CAN2 , &TxMessage);
}


/**
  * @brief  CAN2�򸱰巢�ͼ�������
  * @param  RC�ṹ��
  * @retval void
  */
void CAN2_CMD_KEYMOUSE(const RC_ctrl_t* RcData)
{
		CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_KEYSTD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x08;							 

    TxMessage.Data[0]=(u8)((int16_t)RcData->rc.ch[4]>>8);
    TxMessage.Data[1]=(u8)((int16_t)RcData->rc.ch[4]);
    TxMessage.Data[2]=(u8)((int16_t)RcData->key.v>>8);
    TxMessage.Data[3]=(u8)((int16_t)RcData->key.v);
    TxMessage.Data[4]=(u8)(RcData->mouse.press_l);
    TxMessage.Data[5]=(u8)(RcData->mouse.press_r);
    TxMessage.Data[6]=(u8)RcData->rc.s[0];
    TxMessage.Data[7]=(u8)RcData->rc.s[1];

		CAN_Transmit(CAN2 , &TxMessage);
}


/**
  * @brief  CAN2�򸱰巢�����x��y��z����
  * @param  RC�ṹ��
  * @retval void
  */
void CAN2_CMD_MOUSEXYZ(const RC_ctrl_t* RcData)
{
		CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_MOUSE_XYZSTD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x08;							 

    TxMessage.Data[0]=(u8)((int16_t)RcData->mouse.x >>8);
    TxMessage.Data[1]=(u8)((int16_t)RcData->mouse.x );
    TxMessage.Data[2]=(u8)((int16_t)RcData->mouse.y >>8);
    TxMessage.Data[3]=(u8)((int16_t)RcData->mouse.y );
    TxMessage.Data[4]=(u8)((int16_t)RcData->mouse.z >>8);;
    TxMessage.Data[5]=(u8)((int16_t)RcData->mouse.z );
    TxMessage.Data[6]=0;
    TxMessage.Data[7]=0;

		CAN_Transmit(CAN2 , &TxMessage);
}



