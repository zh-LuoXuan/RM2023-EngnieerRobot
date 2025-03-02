#ifndef CANTASK_H
#define CANTASK_H
#include "sys.h"

#include "RemoteControl.h"
#include "Encoder_process.h"

#define  ECD_RATE  (8192.f/360.f)


/* CAN send and receive ID */
typedef enum
{
	//̧������
	UP_STDID = 0x200,
	UP_M1_ID = 0x201,
	UP_M2_ID = 0x202,
	
	//�����ƶ�����
	TOPMOVE_STDID = 0x200,
	TOPMOVE_M1_ID = 0x203,
	TOPMOVE_M2_ID = 0x204,
	
	//ǰ�����
	STRETCH_STDID = 0x200,
	STRETCH_M1_ID = 0x201,
	STRETCH_M2_ID = 0x202,
	
	//��Ԯ����
	RESCUE_STDID = 0x200,
	RESCUE_M1_ID = 0x203,
	RESCUE_M2_ID = 0x204,
	
	//ĩ��ִ�л���
	PITCH_ROLL_STDID = 0x1FF,
	TOP_PITCH_ID = 0x205,
	TOP_ROLL_ID = 0x206,
	
	//ң����
	CUSTOM_RCSTD_ID = 0x300,
	CUSTOM_KEYSTD_ID = 0x301,
	CUSTOM_MOUSEXYZ_ID = 0x302,
	
  //��ձ�
  CUSTOM_VACUUM_CMD_ID = 0x400,
	
	 //΢������
  CUSTOM_TOPMOVE_KEY_ID = 0x401,
	
} eCanMassageID;



typedef struct
{
    fp32 ecd;
    int16_t speed_rpm;
    int16_t given_current;
    fp32  all_ecd;   //��������ֵ(��ֵ)
    fp32  real_ecd;
		int32_t  count;
    uint8_t temperate;
    fp32 last_ecd;
	
} MotorMeasure_t;

void CAN1_CMD_TOPMOVE(int16_t data1 , int16_t data2);//CAN1���������ƶ�������������
void CAN2_CMD_TOPMOVE(int16_t data1 , int16_t data2);//CAN2���������ƶ�������������

void CAN1_CMD_UP(int16_t data1 , int16_t data2);//CAN1����̧��������������
void CAN2_CMD_UP(int16_t data1 , int16_t data2);//CAN2����̧��������������

void CAN1_CMD_STRETCH(int16_t data1 , int16_t data2);//CAN1����ǰ�������������
void CAN2_CMD_STRETCH(int16_t data1 , int16_t data2);//CAN2����ǰ�������������

void CAN1_CMD_RESCUE(int16_t data1 , int16_t data2);//CAN1���;�Ԯ������������
void CAN2_CMD_RESCUE(int16_t data1 , int16_t data2);//CAN2���;�Ԯ������������

void CAN1_CMD_TOP_PITCH(int16_t data1,int16_t data2);//CAN1��������Pitch���������
void CAN2_CMD_TOP_PITCH(int16_t data1,int16_t data2);//CAN2��������Pitch���������

void CAN1_CMD_TOP_ROLL(int16_t data);//CAN1��������Roll���������
void CAN2_CMD_TOP_ROLL(int16_t data);//CAN2��������Roll���������

void CAN1_CMD_VACUUM(int data);//CAN1�����巢����ձÿ�������
void CAN2_CMD_VACUUM(int data);//CAN2�����巢����ձÿ�������

void CAN1_CMD_0x200(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);
void CAN2_CMD_0x200(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);

void CAN1_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);
void CAN2_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);

//ͨ��ָ�뷽ʽ��ȡ����
const MotorMeasure_t* get_TopRoll_EncoderProcess_Point(void);
const MotorMeasure_t* get_TopPitch_EncoderProcess_Point(void);
const MotorMeasure_t* get_Up_EncoderProcess_Point(uint8_t i);
const MotorMeasure_t* get_Stretch_EncoderProcess_Point(uint8_t i);
const MotorMeasure_t* get_TopMove_EncoderProcess_Point(uint8_t i);
const MotorMeasure_t* get_Rescue_EncoderProcess_Point(uint8_t i);
const RC_ctrl_t* get_RC_PC_Point(void);
const int* get_topMoveKey_Point(void);

#endif
