#ifndef _ENCODER_PROCESS_H_
#define _ENCODER_PROCESS_H_
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define  RATE_BUF_SIZE 6

typedef struct
{
    int32_t raw_value;   									//���������������ԭʼֵ
    int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
    int32_t ecd_value;
    int32_t diff;
    int32_t ecd_bias;											//��ʼ������ֵ
    int32_t ecd_raw_rate;
    int32_t round_cnt;
    int32_t rate_buf[RATE_BUF_SIZE]; 	//buf��for filter
    uint8_t buf_count;					//�˲�����buf��
    int32_t filter_rate;				//�ٶ�
    float ecd_angle;											//�Ƕ�

} EncoderProcess_t;

/*
  �������������б������ܺ͵�ֵ�ļ���
*/
extern void EncoderProcess(volatile EncoderProcess_t* v, CanRxMsg * msg);


#endif /*_DATA_PROCESS_TASK_H_*/


