#ifndef ADRC_CORE_H
#define ADRC_CORE_H
#include "ADRC_user.h"

typedef struct
{
	//�м������������Ҫ�û������Լ���ֵ
/****************TD*******************/
float 	x1,		//��������
		x2;		//���������΢��
/****************ESO******************/
float	e,		//���
		z1,		//���ٷ���ֵ
		z2,		//���ٷ���ֵ�Ķ�΢��
		z3,		//����ϵͳ���Ŷ������Ŷ���
		z3_limit;
/**************NLSEF******************/
float	u,		//���ֵ
		u0,		//��������ϵ����
		e1,		//z1��x1�����
		e2;		//z2��x2�����
}state_param;

//			v: x1���趨ֵ		y����ǰx1
float ADRC(ADRC_t *ADRC,state_param *trans_num, float v,float y);

#endif
