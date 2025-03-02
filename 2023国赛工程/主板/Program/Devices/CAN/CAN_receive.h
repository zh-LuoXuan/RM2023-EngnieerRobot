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
	CHASSIS_STDID = 0x200,
	CHASSIS_M1_ID = 0x201,
	CHASSIS_M2_ID = 0x202,
	CHASSIS_M3_ID = 0x203,
	CHASSIS_M4_ID = 0x204,
	
	//云台
	GIMBAL_STDID = 0x1FF,
	GIMBAL_YAW_ID = 0x205,
	GIMBAL_PITCH_ID = 0x206,
	
	UP_SUCKER_RP_STDID = 0x200,
	//框架抬升
	UP_M1_ID = 0x201,
	UP_M2_ID = 0x202,
	//吸盘
	SUCKER_ROLL_ID = 0x203,
	SUCKER_PITCH_ID = 0x204,
	SUCKER_Y_STDID = 0x1FF,
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



void CAN1_CMD_GIMBAL(int16_t data1 , int16_t data2);//CAN1发送云台控制命令
void CAN2_CMD_GIMBAL(int16_t data1 , int16_t data2);//CAN2发送云台控制命令

void CAN1_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);//CAN1发送底盘控制命令
void CAN2_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);//CAN2发送底盘控制命令

void CAN1_CMD_RC(const RC_ctrl_t* RcData);//CAN1向副板发送遥控器数据
void CAN2_CMD_RC(const RC_ctrl_t* RcData);//CAN2向副板发送遥控器数据

void CAN1_CMD_KEYMOUSE(const RC_ctrl_t* RcData);//CAN1向副板发送键鼠数据
void CAN2_CMD_KEYMOUSE(const RC_ctrl_t* RcData);//CAN2向副板发送键鼠数据

void CAN1_CMD_MOUSEXYZ(const RC_ctrl_t* RcData);
void CAN2_CMD_MOUSEXYZ(const RC_ctrl_t* RcData);
		
void CAN1_CMD_IO(int key_IO);//CAN1向副板发送IO数据
void CAN2_CMD_IO(int key_IO);//CAN2向副板发送IO数据

//通过指针方式获取数据
const EncoderProcess_t* get_Yaw_EncoderProcess_Point(void);
const EncoderProcess_t* get_Pitch_EncoderProcess_Point(void);
const MotorMeasure_t *get_Chassis_Measure_Point(uint8_t i);
const int* get_VacuumCMD_Point(void);

#endif
