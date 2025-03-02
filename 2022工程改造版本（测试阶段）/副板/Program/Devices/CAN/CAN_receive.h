/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef CANTASK_H
#define CANTASK_H
#include "sys.h"

#include "RemoteControl.h"

#define  RATE_BUF_SIZE 5
#define  ECD_RATE  (8192.f/360.f)


/* CAN send and receive ID */
typedef enum
{
	  CAN_M3508_SET_ID = 0x700,
	  
    CAN_MOTOR_ALL_ID = 0x200,
	  CAN_CLAMP_YR_ID = 0x1FF,
	
    CAN_UP_M1_ID = 0x201,
    CAN_UP_M2_ID = 0x202,
	
    CAN_CLAMP_P_ID = 0x203,
    CAN_CLAMP_Y_ID = 0x205,
		CAN_CLAMP_R_ID = 0x206,
	
		CAN_AUX_RC_ID = 0x301,
	  CAN_AUX_KEY_ID = 0x302,

    CAN_AUX_IO_ID = 0x407

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    fp32  all_ecd;   //��������ֵ(��ֵ)
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
    int32_t rate_buf[RATE_BUF_SIZE]; 	//buf��for filter
    uint8_t buf_count;					//�˲�����buf��
    int32_t filter_rate;				//�ٶ�
} Encoder_process_t;

const RC_ctrl_t *get_can_remote_control_point(void);
//����up���������ַ
const motor_measure_t *get_Up_Motor_Measure_Point(uint8_t i);
//����clamp���������ַ
const motor_measure_t *get_Clamp_P_Motor_Measure_Point(void);
const motor_measure_t *get_Clamp_Y_Motor_Measure_Point(void);
const motor_measure_t *get_Clamp_R_Motor_Measure_Point(void);
//CAN1���͵����������
void CAN1_CMD_Control(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);
void CAN1_YR_Control(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);


const RC_ctrl_t *get_Rc_Key_Point(void);
u8* get_Key_Can_Point(void);
#endif
