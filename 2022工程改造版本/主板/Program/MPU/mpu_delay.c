#include "mpu_delay.h"

void mpu_delay_ms(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=42000; //at 168MHz 42000 is ok
		while(a--);
	}
}

void mpu_delay_us(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=40;  //at 168MHz 40 is ok,the higher the number the more timing precise
		while(a--);
	}
}
