#ifndef __USER_LIB_H
#define __USER_LIB_H

#include "sys.h"
#include "stdint.h"


typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num;       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
//extern fp32 invSqrt1(fp32 num);


//��Χӳ��
fp32 fp32_map(fp32 a, fp32 amin, fp32 amax, fp32 bmin, fp32 bmax);
int int_map(int a, int amin, int amax, int bmin, int bmax);

//��������
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int16����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
int Constrain(int amt, int low, int high);
float Constrain_float(float amt, float low, float high);
int16_t Constrain_int16_t(int16_t amt, int16_t low, int16_t high);
int32_t Constrain_int32_t(int32_t amt, int32_t low, int32_t high);
int16_t Limit_int16_t(int16_t amt, int16_t limit);
//б�º���
float RAMP_float( float final, float now, float ramp );
float RampInc_float( float *buffer, float now, float ramp );
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );

//һ�׵�ͨ�˲���ʼ��
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period,  fp32 num);
//һ���˲�����
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//��������
void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
fp32 sign(fp32 value);
//�õ���ԽǶ�
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);

#endif

