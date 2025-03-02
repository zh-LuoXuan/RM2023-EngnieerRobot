#ifndef CANTASK_H
#define CANTASK_H
#include "sys.h"

#include "RemoteControl.h"
#include "Encoder_process.h"

#define  ECD_RATE  (8192.f/360.f)


/* CAN send and receive ID */
typedef enum
{
	CHASSIS_FAST_STDID = 0x700,
	
  //底盘
	CHASSIS_M1_ID = 0x201,
	CHASSIS_M2_ID = 0x202,
	CHASSIS_M3_ID = 0x203,
	CHASSIS_M4_ID = 0x204,
	
	//云台
	GIMBAL_YAW_ID = 0x205,
	GIMBAL_PITCH_ID = 0x206,
	
	//框架抬升
	UP_M1_ID = 0x201,
	UP_M2_ID = 0x202,

	//吸盘
	SUCKER_PITCH_ID = 0x203,
	SUCKER_ROLL_ID = 0x204,
	SUCKER_YAW_ID = 0x205,
	
	//遥控器
	CUSTOM_RCSTD_ID = 0x300,
	CUSTOM_KEYSTD_ID = 0x301,
	CUSTOM_MOUSE_XYZSTD_ID = 0x302,
	
} eCanMassageID;



typedef struct
{
    fp32 ecd;
    int16_t speed_rpm;
    int16_t given_current;
    fp32  all_ecd;   //编码器的值(总值)
    fp32  real_ecd;
		int32_t  count;
    uint8_t temperate;
    fp32 last_ecd;
	
} MotorMeasure_t;

void CAN1_CMD_0x200(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);
void CAN2_CMD_0x200(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);

void CAN1_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);
void CAN2_CMD_0x1FF(int16_t data1 , int16_t data2, int16_t data3 , int16_t data4);

//通过指针方式获取数据
const MotorMeasure_t* get_SucRoll_EncoderProcess_Point(void);
const MotorMeasure_t* get_SucYaw_EncoderProcess_Point(void);
const MotorMeasure_t* get_SucPitch_EncoderProcess_Point(void);
const MotorMeasure_t* get_Up_EncoderProcess_Point(uint8_t i);

#endif
