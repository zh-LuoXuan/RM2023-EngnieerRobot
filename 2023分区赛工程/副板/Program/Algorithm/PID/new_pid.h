#ifndef __NEW_CAN_H
#define __NEW_CAN_H
#include "sys.h"
#include "math.h"
#include "stm32f4xx.h"
typedef struct
{
    uint8_t mode;
    // PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];
    fp32 error[3];

    fp32 Dead_Zone; //死区
    fp32 gama;
    fp32 I_Separation;
    fp32 angle_max;
    fp32 angle_min;
		fp32 lastdout;
		fp32 lastout;
		
} newPidTypeDef;

enum PID_MODE_e
{
    NEW_PID_POSITION = 0,
    NEW_PID_DELTA
};

void New_PID_Init(newPidTypeDef* pid, u8 mode, const fp32 PID[3], float max_out, float max_iout, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation);
fp32 New_PID_Calc(newPidTypeDef* pid, const fp32 ref, const fp32 set);
void New_PID_Clear(newPidTypeDef* pid);
void New_PID_Reset(newPidTypeDef* pid, fp32 PID[3]);

#endif
