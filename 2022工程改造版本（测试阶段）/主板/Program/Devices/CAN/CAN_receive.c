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
Encoder_process_t Encoder_yaw, 
		              Encoder_pit,
		              Encoder_motor_power[4],
		              Encoder_motor_steer[4];

//����yaw������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void)
{
    return &Encoder_pit;
}
//����pit������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void)
{
    return &Encoder_yaw;
}
 motor_measure_t motor_yaw, 
                       motor_pit, 
                       motor_power[4],
                       motor_steer[4];
	


/**********************************************************************************
���ص��������ͨ��ָ�봫�ݷ�ʽ������Ϣ
***********************************************************************************/

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Yaw_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//����power���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Power_Motor_Measure_Point(uint8_t i)
{
    return &motor_power[i];
}
//����steer���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Steer_Motor_Measure_Point(uint8_t i)
{
    return &motor_steer[i];
}


//ͳһ����can���պ���
static void CAN_hook1(CanRxMsg *rx_message);
//ͳһ����can���պ���
static void CAN_hook2(CanRxMsg *rx_message);


#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif
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

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}

//CAN ���� 0x700��ID�����ݣ�������M3508�����������IDģʽ
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_M3508_SET_ID;
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
  * @brief  CAN1���͵����������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
		CanTxMsg TxMessage;
		TxMessage.StdId = CAN_POWER_ALL_ID;
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
  * @brief  CAN1���͵����������
  * @param  PID���ֵ
  * @retval void
  */
void CAN1_CMD_GIMBAL(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
	//��̨
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = CAN_GIMBAL_ALL_ID;
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
  * @brief  �򸱰巢��IO����
  * @param  IO
  * @retval void
  */
void CAN_CMD_IO(u8 data)
{
	
		CanTxMsg	TxMessage;

    TxMessage.StdId = CAN_AUX_IO_ID;	 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;		  
    TxMessage.DLC = 0x08;							 

    TxMessage.Data[0]= data;
    
		CAN_Transmit(CAN2 , &TxMessage);
}

/**
  * @brief  �򸱰巢��ң��������
  * @param  RC�ṹ��
  * @retval void
  */
void CAN_CMD_RC(RC_ctrl_t* RcData)
{
	
		CanTxMsg	TxMessage;

    TxMessage.StdId = CAN_AUX_RC_ID;	 
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
  * @brief  �򸱰巢�ͼ�������
  * @param  RC�ṹ��
  * @retval void
  */
void CAN_CMD_KEYMOUSE(RC_ctrl_t* RcData)
{
		CanTxMsg	TxMessage;

    TxMessage.StdId = CAN_AUX_KEY_ID;	 
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
  * @brief  CAN2���͵����������
  * @param  PID���ֵ
  * @retval void
  */
void CAN2_CMD_CONTROL(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = CAN_STEER_ALL_ID;
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
}

void CAN_GIMBAL(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
}
//ͳһ����can�жϺ��������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
static void CAN_hook1(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
			case CAN_POWER_M1_ID:
			case CAN_POWER_M2_ID:
			case CAN_POWER_M3_ID:
			case CAN_POWER_M4_ID:
				{
					static uint8_t i = 0;
					//������ID��
					i = rx_message->StdId - CAN_POWER_M1_ID;
					//���������ݺ꺯��
					get_motor_measure(&motor_power[i], rx_message);
					//��¼ʱ��
					DetectHook(YawChassisMotorTOE);
					break;
				}
			case CAN_YAW_MOTOR_ID:
				{
					get_motor_measure(&motor_yaw, rx_message);
					//��¼ʱ��
					DetectHook(YawChassisMotorTOE);
					break;
				}
			case CAN_PIT_MOTOR_ID:
				{
					get_motor_measure(&motor_pit, rx_message);
					//��¼ʱ��
					DetectHook(YawChassisMotorTOE);
					break;
				}
			default:
			{
				break;
			}
    }
}

static void CAN_hook2(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
		case CAN_STEER_M1_ID:
		case CAN_STEER_M2_ID:
		case CAN_STEER_M3_ID:
		case CAN_STEER_M4_ID:
		{
			static uint8_t i = 0;
			//������ID��
			i = rx_message->StdId - CAN_STEER_M1_ID;
			//���������ݺ꺯��
			get_motor_measure(&motor_steer[i], rx_message);
			//��¼ʱ��
			DetectHook(TriggerMotor1TOE + i);
			break;
		}
		default:
		{
			break;
		}
    }
}

