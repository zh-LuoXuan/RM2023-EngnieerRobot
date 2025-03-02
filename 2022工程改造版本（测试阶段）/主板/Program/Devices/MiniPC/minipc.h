#ifndef __MINIPC_H
#define __MINIPC_H

#include "sys.h"
#include "usart.h"

//PC����ת���������ṹ����ȡ�ϳ���������
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
	uint16_t yaw_limit;
	uint16_t outpost_HP;
    uint32_t time;      //��λ����������ʱ���ǩ
    float distance;
    uint32_t CpuTime;   //��λ��CPUʱ���ǩ
    float PcPitch;      //��λ����������pitch������
    float PcYaw;       //��λ����������yaw������
	unsigned char shoot_flag;//���ָ��
	unsigned char frequency_adj;//��Ƶ����
} PC_Data;

//ԭ�ṹ��
typedef __packed struct 
{
	unsigned char head;
	float angle_pitch;
	float angle_yaw;
//	float distance;
	unsigned char shoot_flag;
	unsigned char frequency_adj;
	unsigned char end;
}PC_Ctrl_t;

//��λ������ת��������
typedef union 
{
	PC_Ctrl_t PcDate;
	unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
}PC_Ctrl_Union_t;


//ͨ��ָ�뷽ʽ��ȡԭʼ����
const PC_Data *get_PC_Data_Point(void);
void minipc_Get_Message(DataRevice *Buffer);

#endif

