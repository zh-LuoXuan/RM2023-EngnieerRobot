#include "user_lib.h"
#include "Filter_Control.h"
#include "arm_math.h"

/**
  * @brief			��Χӳ��
  * @author			RW
  * @param[in]		ԭʼ��ֵ
  * @param[in]		ԭʼ��Сֵ
  * @param[in]		ԭʼ���ֵ
  * @param[in]		ӳ������Сֵ
  * @param[in]		ӳ�������ֵ
  * @retval			ӳ������ֵ
  */
fp32 fp32_map(fp32 a, fp32 amin, fp32 amax, fp32 bmin, fp32 bmax)
{
	fp32 adel = amax - amin, bdel = bmax - bmin;
	if(a > amax)a = amax;
	if(a < amin)a = amin;
	return (fp32)(bdel * ((fp32)(a-amin) / adel))+bmin;
}
int int_map(int a, int amin, int amax, int bmin, int bmax)
{
	int adel = amax - amin, bdel = bmax - bmin;
	if(a > amax)a = amax;
	if(a < amin)a = amin;
	return (bdel * ((float)(a-amin) / adel))+bmin;
}

//��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }

    return Value;
}

//int16����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }

    return Value;
}

/**
  * @brief  �޷�����
  * @param  ��ǰ���룬���ޣ�����
  * @retval ��ǰ���
  * @attention  
  */
int Constrain(int amt, int low, int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

float Constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t Constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int32_t Constrain_int32_t(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t Limit_int16_t(int16_t amt, int16_t limit)
{
	if (amt > limit)
			return limit;
	else if(amt < -limit)
		  return (-limit);
	else
	    return amt;
}
/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
	if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

		if (*buffer > 0)
		{
				if (*buffer > ramp)
				{  
						 now    += ramp;
					  *buffer -= ramp;
				}   
				else
				{
						 now    += *buffer;
					  *buffer  = 0;
				}
		}
		else
		{
				if (*buffer < -ramp)
				{
						now     += -ramp;
					  *buffer -= -ramp;
				}
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		
		return now;
}


/**
  * @brief  ����б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  *             
*/
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
	float  factor = 0;
	factor = 0.05 * sqrt( 0.15 * (*time) );  //�����ٶ�б��,time�ۼӵ�296.3б�¾����
	if (status == 1)//����������
	{
		if (factor < 1)//��ֹtime̫��
		{
			*time += inc;
		}
	}
	else		//�����ɿ�
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}
	factor = constrain( factor, 0, 1 );//ע��һ����float�����޷�
	return factor;  //ע�ⷽ��
}

/**
  * @brief       һ�׵�ͨ�˲���ʼ��
  * @author      RM
  * @param[in]   һ�׵�ͨ�˲��ṹ��
  * @param[in]   �����ʱ�䣬��λ s
  * @param[in]   �˲�����
  * @retval      ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period,  fp32 num)
{
    first_order_filter_type->frame_period = frame_period;   //�˲���ʱ����
    first_order_filter_type->num = num;    //  �˲�����
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief       һ�׵�ͨ�˲�����
  * @author      RM
  * @param[in]   һ�׵�ͨ�˲��ṹ��
  * @param[in]   �����ʱ�䣬��λ s
  * @retval      ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period)
	* first_order_filter_type->out
	+ first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period)
	* first_order_filter_type->input;
}
// chu=  num/(timer + num )*out  +  timer/(num+timer) * int
//��������
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//�жϷ���λ
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

/**
  * @brief  �õ���ԽǶ�
  * @param  
  * @retval 
  * @attention  
  *             
*/
fp32 get_relative_pos(fp32 raw_ecd, fp32 center_offset)
{
	fp32 tmp = 0;
	if (center_offset >= 4096)
	{
		if (raw_ecd > center_offset - 4096)
			tmp = raw_ecd - center_offset;
		else
		  tmp = raw_ecd + 8192 - center_offset;
	}
	else
	{
		if (raw_ecd > center_offset + 4096)
		  tmp = raw_ecd - 8192 - center_offset;
		else
		  tmp = raw_ecd - center_offset;
	}
	return tmp;
}
