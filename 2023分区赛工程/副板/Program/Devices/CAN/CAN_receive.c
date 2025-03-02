#include "CAN_Receive.h"
#include "stm32f4xx.h"
#include "gpio.h" 

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
#include "RemoteControl.h"

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
����ң�����ݣ�ͨ��ָ�봫�ݷ�ʽ������Ϣ
***********************************************************************************/		
RC_ctrl_t RC_PC_Control;
RC_ctrl_t RC_PC_Control;const RC_ctrl_t* get_RC_PC_Point(void)
{
    return &RC_PC_Control;
}


/**********************************************************************************
���ص�����������ݣ�ͨ��ָ�봫�ݷ�ʽ������Ϣ
***********************************************************************************/
MotorMeasure_t upMotor[2], 
								 stretchMotor[2],//��ʯ��ת���
                 topMoveMotor[2],
                 rescueMotor[2],
                 topPitchMotor,
                 topRollMotor; 

//����roll���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const MotorMeasure_t* get_TopRoll_EncoderProcess_Point(void)
{
    return &topRollMotor;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const MotorMeasure_t* get_TopPitch_EncoderProcess_Point(void)
{
    return &topPitchMotor;
}

//����up���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const MotorMeasure_t* get_Up_EncoderProcess_Point(uint8_t i)
{
    return &upMotor[i];
}

//����stretch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const MotorMeasure_t* get_Stretch_EncoderProcess_Point(uint8_t i)
{
    return &stretchMotor[i];
}

//����topmove���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const MotorMeasure_t* get_TopMove_EncoderProcess_Point(uint8_t i)
{
    return &topMoveMotor[i];
}

//����rescue���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const MotorMeasure_t* get_Rescue_EncoderProcess_Point(uint8_t i)
{
    return &rescueMotor[i];
}

/**********************************************************************************
���������ƶ������λ�źţ�ͨ��ָ�봫�ݷ�ʽ������Ϣ
***********************************************************************************/
int topMove_key ;
const int* get_topMoveKey_Point(void)
{
    return &topMove_key;
}



//ͳһ����can���պ���
static void CAN_hook1(CanRxMsg *RxMessage);
//ͳһ����can���պ���
static void CAN_hook2(CanRxMsg *RxMessage);

//can1�ж�
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook1(&rx1_message);
    }
}

//can2�ж�
void CAN2_RX1_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
        CAN_hook2(&rx2_message);
    }
}

void CAN1_CMD_0x200(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = 0x200;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = data3>>8;
		TxMessage.Data[5] = data3;
		TxMessage.Data[6] = data4>>8;
		TxMessage.Data[7] = data4;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}

void CAN2_CMD_0x200(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = 0x200;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = data3>>8;
		TxMessage.Data[5] = data3;
		TxMessage.Data[6] = data4>>8;
		TxMessage.Data[7] = data4;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}

void CAN1_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = 0x200;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = data3>>8;
		TxMessage.Data[5] = data3;
		TxMessage.Data[6] = data4>>8;
		TxMessage.Data[7] = data4;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}

void CAN2_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = 0x200;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = data3>>8;
		TxMessage.Data[5] = data3;
		TxMessage.Data[6] = data4>>8;
		TxMessage.Data[7] = data4;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}

/**
  * @brief  CAN1���������ƶ�������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_TOPMOVE(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = TOPMOVE_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = data1>>8;
		TxMessage.Data[5] = data1;
		TxMessage.Data[6] = data2>>8;
		TxMessage.Data[7] = data2;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}

/**
  * @brief  CAN2���������ƶ�������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_TOPMOVE(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = TOPMOVE_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = data1>>8;
		TxMessage.Data[5] = data1;
		TxMessage.Data[6] = data2>>8;
		TxMessage.Data[7] = data2;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}


/**
  * @brief  CAN1����̧��������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_UP(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = UP_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}

/**
  * @brief  CAN2����̧��������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_UP(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = UP_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}

/**
  * @brief  CAN1����ǰ�������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_STRETCH(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = STRETCH_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}

/**
  * @brief  CAN2����ǰ�������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_STRETCH(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = STRETCH_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}


/**
  * @brief  CAN1���;�Ԯ������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_RESCUE(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = RESCUE_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = data1>>8;
		TxMessage.Data[5] = data1;
		TxMessage.Data[6] = data2>>8;
		TxMessage.Data[7] = data2;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}

/**
  * @brief  CAN2���;�Ԯ������������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_RESCUE(int16_t data1 , int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = RESCUE_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = data1>>8;
		TxMessage.Data[5] = data1;
		TxMessage.Data[6] = data2>>8;
		TxMessage.Data[7] = data2;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}



/**
  * @brief  CAN1��������Pitch���������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_TOP_PITCH(int16_t data1,int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = PITCH_ROLL_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}


/**
  * @brief  CAN2��������Pitch���������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_TOP_PITCH(int16_t data1,int16_t data2)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = PITCH_ROLL_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}


/**
  * @brief  CAN1��������Roll���������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_TOP_ROLL(int16_t data)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = PITCH_ROLL_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = data>>8;
		TxMessage.Data[3] = data;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN1 , &TxMessage);
	}

}

/**
  * @brief  CAN2��������Roll���������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_TOP_ROLL(int16_t data)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = PITCH_ROLL_STDID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = data>>8;
		TxMessage.Data[3] = data;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		 
		CAN_Transmit(CAN2 , &TxMessage);
	}

}


/**
  * @brief  CAN1�����巢����ձÿ�������
  * @param  ����ֵ
  * @retval void
  */
void CAN1_CMD_VACUUM(int data)
{
		CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_VACUUM_CMD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x01;							 

    TxMessage.Data[0]=(u8)data;

		CAN_Transmit(CAN1 , &TxMessage);
}

/**
  * @brief  CAN2�����巢����ձÿ�������
  * @param  ����ֵ
  * @retval void
  */
void CAN2_CMD_VACUUM(int data)
{
	CanTxMsg	TxMessage;

    TxMessage.StdId = CUSTOM_VACUUM_CMD_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x01;							 

    TxMessage.Data[0]=(u8)data;

		CAN_Transmit(CAN2 , &TxMessage);
}




//ͳһ����can�жϺ���
static void CAN_hook1(CanRxMsg *RxMessage)
{
    switch (RxMessage->StdId)
    {
			case UP_M1_ID:
			case UP_M2_ID:
				{
					static uint8_t i = 0;
		
					i = RxMessage->StdId - UP_M1_ID;
					get_motor_measure(&upMotor[i], RxMessage);
					break;
				}
			case TOPMOVE_M1_ID:
			case TOPMOVE_M2_ID:	
				{
					static uint8_t i = 0;
					
					i = RxMessage->StdId - TOPMOVE_M1_ID;
					get_motor_measure(&topMoveMotor[i], RxMessage);
					break;
				}
			case TOP_PITCH_ID:
				{
					get_motor_measure(&topPitchMotor, RxMessage);
					break;
				}
			case TOP_ROLL_ID:
				{
					get_motor_measure(&topRollMotor, RxMessage);
					break;
				}
			default:
			{
				break;
			}
    }
}

static void CAN_hook2(CanRxMsg *RxMessage)
{
    switch (RxMessage->StdId)
    {
			case STRETCH_M1_ID:
			case STRETCH_M2_ID:
				{
					static uint8_t i = 0;
					
					i = RxMessage->StdId - STRETCH_M1_ID;
					get_motor_measure(&stretchMotor[i], RxMessage);
					break;
				}
			
//			case CUSTOM_RCSTD_ID:	
//				{
//					Get_RC_Data(&RC_PC_Control, RxMessage);
//					break;
//				}
//			case CUSTOM_KEYSTD_ID:
//				{
//					Get_KEY_Data(&RC_PC_Control, RxMessage);
//					break;
//				}
//			case CUSTOM_MOUSEXYZ_ID:
//				{
//					Get_MOUSExyz_Data(&RC_PC_Control, RxMessage);
//					break;
//				}
			case RESCUE_M1_ID:
			case RESCUE_M2_ID:
				{
					static uint8_t i = 0;
					
					i = RxMessage->StdId - RESCUE_M1_ID;
					get_motor_measure(&rescueMotor[i], RxMessage);
					break;
				}
			case CUSTOM_TOPMOVE_KEY_ID:
				{
					topMove_key = RxMessage->Data[0];
					break;
				}
			default:
			{
				break;
			}
    }
}

