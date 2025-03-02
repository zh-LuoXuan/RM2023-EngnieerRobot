/*
*new_pid.c lzh
带死区，梯形积分，不完全微分算法，积分分离的pid算法

*/
#include "new_pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void New_PID_Init(newPidTypeDef *pid, u8 mode, const fp32 PID[3], float max_out, float max_iout, float Dead_Zone, float gama, float angle_max, float angle_min, float I_Separation)
{
    if (pid == 0 || PID == 0)
    {
        return;
    }
    if (fabs(pid->Dead_Zone) < 1e-5)
    {
        pid->Dead_Zone = 0;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->Dead_Zone = Dead_Zone;
    pid->I_Separation = I_Separation;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}
int de;
fp32 New_PID_Calc(newPidTypeDef *pid, const fp32 ref, const fp32 set)
{
    if (pid == 0)
    {
        return 0.0f;
    }
    pid->error[0] = set - ref;
    if (pid->angle_max != pid->angle_min) //使积分增长变为梯形，如果angle_min和angle_max相等则不启用
    {
        if (pid->error[0] > (pid->angle_max + pid->angle_min) / 2)
            pid->error[0] -= (pid->angle_max + pid->angle_min);
        else if (pid->error[0] < -(pid->angle_max + pid->angle_min) / 2)
            pid->error[0] += (pid->angle_max + pid->angle_min);
    }

    pid->set = set;
    pid->fdb = ref;

    if (fabs(pid->error[0]) < pid->Dead_Zone) //死区，可以使pid系统更快的稳定下来，但是不能给太大，会导致系统滞后，且存在较大静差
    {
        pid->error[0] = pid->error[1] = 0;
    }
    if (fabs(pid->error[0]) > pid->I_Separation) //误差过大，采用积分分离
    {
        de = 0;
    }
    else
    {
        de = 1;
    }
    //只写增量式pid
    pid->Pout = pid->Kp * pid->error[0];

    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);

    //计算微分项增量带不完全微分，可以减少高频干扰，gama应是一个0-1的值
    pid->Dout = pid->Kd * (1 - pid->gama) * (pid->Dbuf[0]) + pid->gama * pid->lastdout;
    LimitMax(pid->Iout, pid->max_iout);

    pid->out = pid->Pout + de * pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->lastdout = pid->Dout;
    pid->lastout = pid->out;
    return pid->out;
}

//对全部pid参数的清空
void New_PID_Clear(newPidTypeDef *pid)
{
    if (pid == 0)
    {
        return;
    }
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
//需要对pid参数进行改变，和以前的用法相同
void New_PID_Reset(newPidTypeDef *pid, fp32 PID[3])
{
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
}
