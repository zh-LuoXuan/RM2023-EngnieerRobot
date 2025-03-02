#include "CAN_Receive.h"
#include "stm32f4xx.h"
#include "gpio.h" 

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"

//底盘电机数据读取
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
返回电机编码器数据，通过指针传递方式传递信息
***********************************************************************************/
MotorMeasure_t upMotor[2],
               sucPitchMotor,
               sucRollMotor;
#if ISYAWSUC 
MotorMeasure_t upMotor[2],
               sucPitchMotor,
               sucRollMotor, 
               sucYawMotor;
#endif

//返回up电机变量地址，通过指针方式获取原始数据
const MotorMeasure_t* get_Up_EncoderProcess_Point(uint8_t i)
{
    return &upMotor[i];
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const MotorMeasure_t* get_SucPitch_EncoderProcess_Point(void)
{
    return &sucPitchMotor;
}
//返回roll电机变量地址，通过指针方式获取原始数据
const MotorMeasure_t* get_SucRoll_EncoderProcess_Point(void)
{
    return &sucRollMotor;
}
#if ISYAWSUC 
//返回yaw电机变量地址，通过指针方式获取原始数据
const MotorMeasure_t* get_SucYaw_EncoderProcess_Point(void)
{
    return &sucYawMotor;
}
#endif


/*************************************************CAN回调函数************************************************************/

//统一处理can中断函数
static void CAN1_Hook(CanRxMsg *RxMessage)
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
			case SUCKER_PITCH_ID:
			{
				get_motor_measure(&sucPitchMotor, RxMessage);
				break;
			}
			case SUCKER_ROLL_ID:
			{
				get_motor_measure(&sucRollMotor, RxMessage);
				break;
			}
#if ISYAWSUC 
			case SUCKER_YAW_ID:
			{
				get_motor_measure(&sucYawMotor, RxMessage);
				break;
			}
#endif
			default:
			{
				break;
			}
    }
}

static void CAN2_Hook(CanRxMsg *RxMessage)
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
			case SUCKER_PITCH_ID:
			{
				get_motor_measure(&sucPitchMotor, RxMessage);
				break;
			}
			case SUCKER_ROLL_ID:
			{
				get_motor_measure(&sucRollMotor, RxMessage);
				break;
			}
#if ISYAWSUC 
			case SUCKER_YAW_ID:
			{
				get_motor_measure(&sucYawMotor, RxMessage);
				break;
			}
#endif
			default:
			{
				break;
			}
    }
}

/*************************************************CAN接收************************************************************/

//can1中断
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

//can2中断
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


/*************************************************CAN发送************************************************************/

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

void CAN1_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = 0x1FF;
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



/*************************************************CAN2 发送************************************************************/

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

void CAN2_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4)
{
	{
		CanTxMsg TxMessage;
		TxMessage.StdId = 0x1FF;
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




