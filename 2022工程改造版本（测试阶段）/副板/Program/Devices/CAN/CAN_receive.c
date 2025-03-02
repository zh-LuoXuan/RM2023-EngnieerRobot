#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"
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
	
		
#define get_motor2006_measure(ptr, rx_message)  \
		{                                       \
				if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ; \
				else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ; \
				(ptr)->last_ecd = (ptr)->ecd; \
				(ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
				(ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 |(rx_message)->Data[3]); \
				(ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd; 	\
				(ptr)->real_ecd=(ptr)->all_ecd/ECD_RATE;   \
		}
static motor_measure_t up_motor[2],
	                     clamp_P_motor,
											 clamp_Y_motor,
                       clamp_R_motor;
	


/**********************************************************************************
���ص��������ͨ��ָ�봫�ݷ�ʽ������Ϣ
***********************************************************************************/

//����up���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Up_Motor_Measure_Point(uint8_t i)
{
    return &up_motor[i];
	
}
//����clamp���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Clamp_P_Motor_Measure_Point(void)
{
    return &clamp_P_motor;
}
const motor_measure_t *get_Clamp_Y_Motor_Measure_Point(void)
{
    return &clamp_Y_motor;
}
const motor_measure_t *get_Clamp_R_Motor_Measure_Point(void)
{
    return &clamp_R_motor;
}

u8 KEY_CAN;
u8* get_Key_Can_Point(void)
{
	return &KEY_CAN;
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
void CAN1_CMD_Control(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
		CanTxMsg TxMessage;
		TxMessage.StdId = CAN_MOTOR_ALL_ID;
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
void CAN1_YR_Control(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
		CanTxMsg TxMessage;
		TxMessage.StdId = CAN_CLAMP_YR_ID;
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

//ͳһ����can�жϺ��������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
static void CAN_hook1(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
			case CAN_UP_M1_ID:
			case CAN_UP_M2_ID:
				{
						static uint8_t i = 0;
						//������ID��
						i = rx_message->StdId - CAN_UP_M1_ID;
						//���������ݺ꺯��2
						get_motor_measure(&up_motor[i], rx_message);
						break;
				}
			case CAN_CLAMP_P_ID:
			  {
						//���������ݺ꺯��
						get_motor_measure(&clamp_P_motor, rx_message);
						break;
				}
		  case CAN_CLAMP_Y_ID:
				{
						//���������ݺ꺯��
						get_motor2006_measure(&clamp_Y_motor, rx_message);
						break;
				}
			case CAN_CLAMP_R_ID:
				{
						//���������ݺ꺯��
						get_motor2006_measure(&clamp_R_motor, rx_message);
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
//	KEY_CAN = 1;
    switch (rx_message->StdId)
    {
			case CAN_AUX_RC_ID:
				{
					Get_RC_Data(&rc_ctrl, rx_message);
					break;
				}
				
			case CAN_AUX_KEY_ID:
				{
					Get_KEY_Data(&rc_ctrl, rx_message);
					break;
				}
		  case CAN_AUX_IO_ID:
				{
					KEY_CAN = rx_message->Data[0];
				}
			default:
				{
						break;
				}
    }
}
