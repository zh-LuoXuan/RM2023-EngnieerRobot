#include "Fuzzy_Control.h"
#include "math.h"

#define Inter_Max   2000
static float DeFuzzy(int eLevel,int ecLevel,u8 ID_item,Fuzzy_original* AL)  //���ķ���ģ��
{

    switch(ID_item)
    {

    case ID_dKp:
        return fuzzyRuleKp[ecLevel+3][eLevel+3] / AL->Gkp;

    case ID_dKi:
        return fuzzyRuleKi[ecLevel+3][eLevel+3] / AL->Gki;

    case ID_dKd:
        return fuzzyRuleKd[ecLevel+3][eLevel+3] / AL->Gkd;

    default:
        return 0;

    }

}
static FPID Fuzzifier(float e, float ec,Fuzzy_original* AL)//e����ec�����仯��
{
    int eLeftIndex,eRightIndex,ecLeftIndex,ecRightIndex;   //���ģ���ȼ���exxx��ƫ�ecxxx��ƫ��仯��
    float eLeftMs,eRightMs,ecLeftMs,ecRightMs;						 //������
    FPID fuzzyDetPID;																			 //PID�����ṹ��

    //*****�о�Ӧ����ƫ�������������������1.0�е�㲻�����******��������õ�ƫ�������0~1�ڲ��ñ�����
    e /= AL->Ge;
    ec /= AL->Gec;
    //����ģ������   ȷ��ģ���ȼ�(����)
    //�ɼ�ƫ��e���ģ���ȼ� �� ƫ��e�Ҳ�ģ���ȼ�
    eLeftIndex = (e/levelInterval)>3.0f?3:(e/levelInterval)<-3.0f?-4:(e/levelInterval)>0?(int)(e/levelInterval):(int)(e/levelInterval)-1;
    eRightIndex = eLeftIndex + 1;
    //����ƫ�������Ҳ�������
    eLeftMs = eLeftIndex<-3?0:eLeftIndex==3?1.0f:eRightIndex-e/levelInterval;
    eRightMs = eRightIndex>3?0:eRightIndex==-3?1.0f:e/levelInterval-eLeftIndex;
    //�ɼ�ƫ��仯��ec���ģ���ȼ� �� ƫ��仯��ec�Ҳ�ģ���ȼ�
    ecLeftIndex = (ec/levelInterval)>3.0f?3:(ec/levelInterval)<-3.0f?-4:(ec/levelInterval)>0?(int)(ec/levelInterval):(int)(ec/levelInterval)-1;
    ecRightIndex = ecLeftIndex + 1;
    //����ƫ��仯�������Ҳ�������
    ecLeftMs = ecLeftIndex<-3?0:ecLeftIndex==3?1.0f:ecRightIndex-ec/levelInterval;
    ecRightMs = ecRightIndex>3?0:ecRightIndex==-3?1.0f:ec/levelInterval-ecLeftIndex;
    //���ķ���ģ��  �ó�ֵ��Ϊģ�����������

    fuzzyDetPID.Kp = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKp,AL)
                      + eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKp,AL)
                      + eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKp,AL)
                      + eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKp,AL));

    fuzzyDetPID.Ki = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKi,AL)
                      + eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKi,AL)
                      + eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKi,AL)
                      + eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKi,AL));

    fuzzyDetPID.Kd = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKd,AL)
                      + eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKd,AL)
                      + eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKd,AL)
                      + eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKd,AL));

    return fuzzyDetPID;
}
void  AlgoriCreate(Fuzzy_original *AL,u16 MaxMotorSpeed,u16 Gkp,u16 Gki,u16 Gkd)
{
    AL->MaxMotorSpeed=MaxMotorSpeed;
    AL->Ge=1.0;
    AL->Gec=1.0;
    AL->Gkp=Gkp;
    AL->Gki=Gki;
    AL->Gkd=Gkd;
}
void  AlgoriReset(Fuzzy_original *AL)
{
    AL->MaxMotorSpeed=0;
    AL->Ge=1.0;
    AL->Gec=1.0;
    AL->Gkp=1.0;
    AL->Gki=1.0;
    AL->Gkd=1.0;
}
void  PidControler(Fuzzy_original *AL,PidTypeDef* motor_type)
{

    FPID dPID = {
        0, 0, 0
    };				//��̬PID���ڡ�������ģ��PID�����ÿ�ν��뺯�����㶯̬PID
    //���ת���޷�
    motor_type->set = motor_type->set>AL->MaxMotorSpeed?AL->MaxMotorSpeed:motor_type->set<-AL->MaxMotorSpeed?-AL->MaxMotorSpeed:motor_type->set;

    motor_type->error[1]=motor_type->error[0];
    motor_type->error[0] = motor_type->set - motor_type->fdb;
    motor_type->error_rate=motor_type->error[0]- motor_type->error[1];
    motor_type->error_inter += motor_type->error[0];

    // limit intergration of pid
    if(motor_type->error_inter>Inter_Max)//Inter_Max=2000,error_inter(������)
        motor_type->error_inter = Inter_Max;
    if(motor_type->error_inter<-Inter_Max)
        motor_type->error_inter = -Inter_Max;

    if(fabs(motor_type->error[0]/motor_type->set)<0.05)//���С��5%ʹ��ģ������PID
        dPID = Fuzzifier(motor_type->error[0], motor_type->error_rate,AL);

    motor_type->out = (motor_type->Kp+dPID.Kp) * motor_type->error[0]+	(motor_type->Kd+dPID.Kd) * motor_type->error_rate +(motor_type->Ki+dPID.Ki) * motor_type->error_inter ;

}
