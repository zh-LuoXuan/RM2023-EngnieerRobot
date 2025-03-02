#ifndef _ENCODER_PROCESS_H_
#define _ENCODER_PROCESS_H_
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define  RATE_BUF_SIZE 6

typedef struct
{
    int32_t raw_value;   									//编码器不经处理的原始值
    int32_t last_raw_value;								//上一次的编码器原始值
    int32_t ecd_value;
    int32_t diff;
    int32_t ecd_bias;											//初始编码器值
    int32_t ecd_raw_rate;
    int32_t round_cnt;
    int32_t rate_buf[RATE_BUF_SIZE]; 	//buf，for filter
    uint8_t buf_count;					//滤波更新buf用
    int32_t filter_rate;				//速度
    float ecd_angle;											//角度

} EncoderProcess_t;

/*
  采用向量法进行编码器总和的值的计算
*/
extern void EncoderProcess(volatile EncoderProcess_t* v, CanRxMsg * msg);


#endif /*_DATA_PROCESS_TASK_H_*/


