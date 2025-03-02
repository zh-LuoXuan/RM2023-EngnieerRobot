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

//rm���ͳһ���ݽṹ��
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

//CAN1���͵����������
void CAN1_CMD_GIMBAL(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);
//CAN1���͵����������
void CAN1_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);
//CAN1�򸱰巢��ң��������
void CAN_CMD_RC(RC_ctrl_t* RcData);
//�򸱰巢��IO����
void CAN_CMD_IO(u8 data);
//CAN1�򸱰巢�ͼ�������
void CAN_CMD_KEYMOUSE(RC_ctrl_t* RcData);
//CAN2���͵����������
void CAN2_CMD_CONTROL(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);

//����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const RC_ctrl_t *get_can_remote_control_point(void);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Yaw_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Motor_Measure_Point(void);
//����power���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Power_Motor_Measure_Point(uint8_t i);
//����Steer���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Steer_Motor_Measure_Point(uint8_t i);



//����yaw������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Yaw_Gimbal_Encoder_Measure_Point(void);
//����pit������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const Encoder_process_t *get_Pitch_Gimbal_Encoder_Measure_Point(void);

const motor_measure_t *get_Power_Motor_Measure_Point(uint8_t i);

const motor_measure_t *get_Power_Motor_Measure_Point(uint8_t i);


#endif
