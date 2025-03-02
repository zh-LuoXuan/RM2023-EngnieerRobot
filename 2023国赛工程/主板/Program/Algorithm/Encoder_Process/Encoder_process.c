#include "Encoder_process.h"
#include "CAN_Receive.h"


/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
***********************************************************************************************
*/
/*
  采用向量法进行编码器总和的值的计算
*/
void EncoderProcess(volatile EncoderProcess_t *v, CanRxMsg * msg)
{
    	int i=0;
	int32_t temp_sum = 0; 

	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)| msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;

	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
		{
			v->round_cnt++;
			v->ecd_raw_rate = v->diff + 8192;
		}
	else if(v->diff > 4096)
		{
			v->round_cnt--;
			v->ecd_raw_rate = v->diff- 8192;
		}		
	else
		{
			v->ecd_raw_rate = v->diff;
		}

	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
//		if(v->ecd_value < 0)
//		{
//			v->ecd_value = ~v->ecd_value;
//		}
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}

	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
}
//	if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ; \
//				else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ; \
//				(ptr)->last_ecd = (ptr)->ecd; \
//				(ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
//				(ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 |(rx_message)->Data[3]); \
//				(ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
//				(ptr)->temperate = (rx_message)->Data[6];  \
//				(ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd; 	\
//				(ptr)->real_ecd=(ptr)->all_ecd/ECD_RATE;   \
}




