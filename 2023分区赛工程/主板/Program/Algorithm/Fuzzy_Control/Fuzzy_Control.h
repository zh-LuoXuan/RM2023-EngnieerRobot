#ifndef _FUZZY_CONTROL_H_
#define _FUZZY_CONTROL_H_

#include "stm32f4xx.h"                  // Device header
#include "Task_Chassis.h"

#define ID_dKp 1
#define ID_dKi 2
#define ID_dKd 3

typedef struct  Fuzzyoriginal
{

    u16 MaxMotorSpeed;
    float Ge;//误差
    float Gec;//误差变化率
    float Gkp;
    float Gki;
    float Gkd;

} Fuzzy_original;

static const float levelInterval = 5;//论域的分级间隔必须相同，此处设置为5
static const float NL = -3 * levelInterval;
static const float NM = -2 * levelInterval;
static const float NS = -1 * levelInterval;
static const float ZE = 0;
static const float PS = 1 * levelInterval;
static const float PM = 2 * levelInterval;
static const float PL = 3 * levelInterval;

static const float fuzzyRuleKp[7][7]= {

    PL,PL,PM,PM,PS,PS,ZE,
    PL,PL,PM,PM,PS,ZE,ZE,
    PM,PM,PM,PS,ZE,NS,NM,
    PM,PS,PS,ZE,NS,NM,NM,
    PS,PS,ZE,NS,NS,NM,NM,
    ZE,ZE,NS,NM,NM,NM,NL,
    ZE,NS,NS,NM,NM,NL,NL

};//dKp模糊规则表

static const float fuzzyRuleKi[7][7]= {

    NL,NL,NL,NM,NM,ZE,ZE,
    NL,NL,NM,NM,NS,ZE,ZE,
    NM,NM,NS,NS,ZE,PS,PS,
    NM,NS,NS,ZE,PS,PS,PM,
    NS,NS,ZE,PS,PS,PM,PM,
    ZE,ZE,PS,PM,PM,PL,PL,
    ZE,ZE,PS,PM,PL,PL,PL

};//dKi模糊规则表

static const float fuzzyRuleKd[7][7]= {

    PS,PS,ZE,ZE,ZE,PL,PL,
    NS,NS,NS,NS,ZE,NS,PM,
    NL,NL,NM,NS,ZE,PS,PM,
    NL,NM,NM,NS,ZE,PS,PM,
    NL,NM,NS,NS,ZE,PS,PS,
    NM,NS,NS,NS,ZE,PS,PS,
    PS,ZE,ZE,ZE,ZE,PL,PL

};//dKd模糊规则表

typedef struct {

    float Kp;
    float Ki;
    float Kd;

} FPID;

void PidControler(Fuzzy_original *AL,PidTypeDef* motor_type);
void  AlgoriCreate(Fuzzy_original *AL,u16 MaxMotorSpeed,u16 Gkp,u16 Gki,u16 Gkd);
void  AlgoriReset(Fuzzy_original *AL);

#endif
