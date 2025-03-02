#ifndef CANTASK_H
#define CANTASK_H
#include "sys.h"

#include "RemoteControl.h"


#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif


#define  RATE_BUF_SIZE 5
#define  ECD_RATE  (8192.f/360.f)

/* CAN send and receive ID */
typedef enum
{
	  CAN_M3508_SET_ID = 0x700,
	
    CAN_POWER_ALL_ID = 0x200,
    CAN_POWER_M1_ID = 0x201,
    CAN_POWER_M2_ID = 0x202,
    CAN_POWER_M3_ID = 0x203,
    CAN_POWER_M4_ID = 0x204,
	  
	  CAN_STEER_ALL_ID = 0x1FF,
	  CAN_STEER_M1_ID = 0x205,
	  CAN_STEER_M2_ID = 0x206,
	  CAN_STEER_M3_ID = 0x207,
	  CAN_STEER_M4_ID = 0x208,
	
	  CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
	
	  CAN_AUX_RC_ID = 0x301,
	  CAN_AUX_KEY_ID = 0x302,

    CAN_AUX_IO_ID = 0x407
	
} can_msg_id_e;

//rm电机统一数据结构体
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
} motor_measure_t;



typedef struct
{
    int32_t diff;
    int32_t round_cnt;
    int32_t ecd_raw_rate;
    int32_t rate_buf[RATE_BUF_SIZE]; 	//buf，for filter
    uint8_t buf_count;					//滤波更新buf用
    int32_t filter_rate;				//速度
} Encoder_process_t;

//CAN1发送电机控制命令
void CAN1_CMD_GIMBAL(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);
//CAN1发送电机控制命令
void CAN1_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);
//CAN1向副板发送遥控器数据
void CAN_CMD_RC(RC_ctrl_t* RcData);
//向副板发送IO数据
void CAN_CMD_IO(u8 data);
//CAN1向副板发送键鼠数据
void CAN_CMD_KEYMOUSE(RC_ctrl_t* RcData);
//CAN2发送电机控制命令
void CAN2_CMD_CONTROL(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);

//返回遥控器控制变量，通过指针传递方式传递信息
const RC_ctrl_t *get_can_remote_control_point(void);
//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Motor_Measure_Point(void);
//返回power电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Power_Motor_Measure_Point(uint8_t i);
//返回Steer电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Steer_Motor_Measure_Point(uint8_t i);



//返回yaw编码器变量地址，通过指针方式获取原始数据
const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void);
//返回pit编码器变量地址，通过指针方式获取原始数据
const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void);

const motor_measure_t *get_Power_Motor_Measure_Point(uint8_t i);

const motor_measure_t *get_Power_Motor_Measure_Point(uint8_t i);


#endif
