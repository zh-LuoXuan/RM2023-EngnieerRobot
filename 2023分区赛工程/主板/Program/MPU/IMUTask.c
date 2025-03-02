#include "IMUTask.h"
#include "Filter_Control.h"
#include "cmsis_os.h"
#include "Filter_Control.h"
#include "MPU_TIME_Init.h"
#include "misc.h"
#include "accelerometer_calibration.h"
#include "mpu_delay.h"
MPU9250_DATA_t   MPU9250_Real_Data;
static MPU9250_DATA   MPU9250_Raw_Data;
static MPU9250_DATA   MPU9250_Offest_Data;
MPU9250_DATA_t MPU6500_Offest_data;
mpu6500_real_data_t mpu6500_real_data; //MPUԭʼ����

Angular_Handle  Angular_Handler;   
unsigned char spi_rtx[sizeof(mpu6500_real_data_t)]={0x3a|0x80};
float last_yaw = 0;
float timeText ;
static float exInt, eyInt, ezInt;  // ������
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float last_yaw_temp ,yaw_temp;
static int yaw_count;
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0f   // integral gain governs rate of convergence of gyroscope biases
float K[3]={1.0,1.0,1.0};//Ĭ�ϱ��(����)���
float B[3]={0,0,0};//Ĭ����λ���
static AK8963 Mag_AK8963;	
float Accex_x;
#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t IMUTaskStack;
#endif

/*=================================================*/
#define IMU_TASK_PRIO 31
#define IMU_TASK_SIZE 512
static TaskHandle_t MPUTask_Handler;

void IMU_task(void);
/*=================================================*/
void task_IMU_Create(void)
{
	xTaskCreate((TaskFunction_t )IMU_task,
                (const char*    )"IMU_task",
                (uint16_t       )IMU_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )IMU_TASK_PRIO,
                (TaskHandle_t*  )&MPUTask_Handler);
}
/*=================================================*/
void IMU_Init(void)
{
	while(MPU6500_Init());
	IMU_Calibration();
	accel_mat_init();
	ACCEL_Calibration();
}

void IMU_task(void)
{
	static portTickType TickCount; 
	TickCount = xTaskGetTickCount();
	
  while(1)
	{
		 ACCEL_Filter();
		 GYRO_Filter();
		 cali_fenction();
		 MPU9250_Get_Temperature(&Angular_Handler.Temp);
//		 Mag_Filter();   //ȥ��������
		 AHRSUpdate_GraDes_Delay_Corretion();
		 Angular_Handler.V_X=MPU9250_Real_Data.Gyro_X*GYRO_CALIBRATION_COFF;	
		 Angular_Handler.V_Y=MPU9250_Real_Data.Gyro_Y*GYRO_CALIBRATION_COFF;	
		 Angular_Handler.V_Z=MPU9250_Real_Data.Gyro_Z*GYRO_CALIBRATION_COFF;	
//		 vTaskDelay(1);
		 vTaskDelayUntil(&TickCount, 1); 
		
#if INCLUDE_uxTaskGetStackHighWaterMark
	IMUTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
		
	 }

}


//���������Ǳ�����ַ
const Angular_Handle* get_Gyro_Angle_Point(void)
{
	return &Angular_Handler;
}
//IMU�¶ȿ��Ƴ�ʼ��
void IMU_TempInit(void)
{
	while(1)
	{
		MPU9250_Get_Temperature(&Angular_Handler.Temp);
		if(Angular_Handler.Temp >=30)
		{
			break;
		}
	}

}
//IMU�¶ȿ��ƺ���
void IMU_TempContrl(void)
{

}
//��������ƫУ׼
void IMU_Calibration(void)
{
	u8 i;
	static s32 g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
	for (i = 0; i < 200; i++)			//��������100��
	{
		MPU9250_Get_Gyroscope(&MPU9250_Raw_Data.Gyro_X,&MPU9250_Raw_Data.Gyro_Y,&MPU9250_Raw_Data.Gyro_Z);
		g_Gyro_xoffset +=MPU9250_Raw_Data.Gyro_X;
		g_Gyro_yoffset +=MPU9250_Raw_Data.Gyro_Y;
		g_Gyro_zoffset +=MPU9250_Raw_Data.Gyro_Z;
		mpu_delay_ms(3);
	}
	MPU9250_Offest_Data.Gyro_X =g_Gyro_xoffset/200;//�õ��궨ƫ��
	MPU9250_Offest_Data.Gyro_Y =g_Gyro_yoffset/200;
	MPU9250_Offest_Data.Gyro_Z =g_Gyro_zoffset/200;
	
}
//��ʼ����Ԫ��
void ACCEL_Calibration(void)
{
	u8 i;
	atti_d_t atti;
	matrix_3X3_d Cbn,Cnb;
	quater_d_t Quat;
	float accX,accY,accZ,norm_ACC;
	float  sum_accX = 0,sum_accY = 0,sum_accZ = 0; 
	for (i = 0; i < 100; i++)			//��������100��
	{
		 MPU9250_Get_Accelerometer(&MPU9250_Raw_Data.Accel_X,&MPU9250_Raw_Data.Accel_Y,&MPU9250_Raw_Data.Accel_Z);
		 accX=MPU9250_Raw_Data.Accel_X/32768.0f*2*GRAVITY_MSS;
		 accY=MPU9250_Raw_Data.Accel_Y/32768.0f*2*GRAVITY_MSS;
		 accZ=MPU9250_Raw_Data.Accel_Z/32768.0f*2*GRAVITY_MSS;
		 caculate_mat(&accX,&accY,&accZ);
		 sum_accX+=accX;
		 sum_accY+=accY;
		 sum_accZ+=accZ;
		 mpu_delay_ms(3);
	}
	accX = sum_accX / 100;
	accY = sum_accY / 100;
	accZ = sum_accZ / 100;		
	
  norm_ACC = my_sqrt(accX*accX + accY*accY + accZ*accZ);	
	
	atti.pitch = (double)(asin(accX / norm_ACC) * RAD2DEG);
	atti.roll  = (double)atan2(-accY, -accZ) * RAD2DEG-180.0f;
	atti.yaw=0;
	
  Cbn = Atti2Mat_d(atti);							//������ϵ����������ϵ��ת������
	Cnb = Mat_Trans_d(Cbn);							//��������ϵ��������ϵ��ת������
	Quat =Mat2Quat(Cnb);								//��������ϵ������ϵ֮��ĵ�Ч��Ԫ��

//��ʼ����Ԫ�� 	
	q0=Quat.q0;
	q1=Quat.q1;
	q2=Quat.q2;
	q3=Quat.q3;	
	
//
	
}

//��ǰ���ٶȼ�У׼���ݲɼ�
void Accel_calibration_get(void)
{
	u8 i;
	float  sum_accX = 0,sum_accY = 0,sum_accZ = 0; 
		for (i = 0; i < 200; i++)			//��������100��
	{
		 MPU9250_Get_Accelerometer(&MPU9250_Raw_Data.Accel_X,&MPU9250_Raw_Data.Accel_Y,&MPU9250_Raw_Data.Accel_Z);
		 sum_accX+=MPU9250_Raw_Data.Accel_X/32768.0f*2*GRAVITY_MSS;
		 sum_accY+=MPU9250_Raw_Data.Accel_Y/32768.0f*2*GRAVITY_MSS;
		 sum_accZ+=MPU9250_Raw_Data.Accel_Z/32768.0f*2*GRAVITY_MSS;
		 mpu_delay_ms(3);
	}
		MPU6500_Offest_data.Accel_X = (float)sum_accX / 100;
	  MPU6500_Offest_data.Accel_Y = (float)sum_accY / 100;	
	  MPU6500_Offest_data.Accel_Z = (float)sum_accZ / 100;	
}

void ACCEL_Filter(void)
{
	float X_Origion,Y_Origion,Z_Origion;
	  MPU9250_Get_Accelerometer(&MPU9250_Raw_Data.Accel_X,&MPU9250_Raw_Data.Accel_Y,&MPU9250_Raw_Data.Accel_Z);
    
		X_Origion=MPU9250_Raw_Data.Accel_X/32768.0f*2*GRAVITY_MSS;//��������У�����������ٶ���
		Y_Origion=MPU9250_Raw_Data.Accel_Y/32768.0f*2*GRAVITY_MSS;
		Z_Origion=MPU9250_Raw_Data.Accel_Z/32768.0f*2*GRAVITY_MSS;
		caculate_mat(&X_Origion,&Y_Origion,&Z_Origion);
		Acce_Correct_Filter(X_Origion,Y_Origion,Z_Origion);   /* Butterworth�˲� */ 		
}

short issu;
void GYRO_Filter(void)//���ٶȵ�ͨ�˲���������̬����
{
	 float  X_GYRO,Y_GYRO,Z_GYRO;

   MPU9250_Get_Gyroscope(&MPU9250_Raw_Data.Gyro_X,&MPU9250_Raw_Data.Gyro_Y,&MPU9250_Raw_Data.Gyro_Z);
	 X_GYRO=(float)MPU9250_Raw_Data.Gyro_X-(float)MPU9250_Offest_Data.Gyro_X;
	 Y_GYRO=(float)MPU9250_Raw_Data.Gyro_Y-(float)MPU9250_Offest_Data.Gyro_Y;
	 Z_GYRO=(float)MPU9250_Raw_Data.Gyro_Z-(float)MPU9250_Offest_Data.Gyro_Z;
	 Gyro_Correct_Filter(X_GYRO,Y_GYRO,Z_GYRO);
}

void Mag_Filter(void)  //�����ƻ��������˲�
{
	 static  float Data_X_MAG[10]={0};
   static  float Data_Y_MAG[10]={0};
   static  float Data_Z_MAG[10]={0};
	 if(MPU9250_Get_Mag(&MPU9250_Raw_Data.Mag_X,&MPU9250_Raw_Data.Mag_Y,&MPU9250_Raw_Data.Mag_Z)==MPU9250_OK) 
	 {
		//����ӳ���������������
		 Mag_AK8963.x = MPU9250_Raw_Data.Mag_X;
		 Mag_AK8963.y = MPU9250_Raw_Data.Mag_Y;
		 Mag_AK8963.z = MPU9250_Raw_Data.Mag_Z;
 	 }	 
	 MPU9250_Real_Data.Mag_X=GildeAverageValueFilter_MAG(Mag_AK8963.x-MPU9250_Offest_Data.Mag_X,Data_X_MAG);//���������˲�������У׼
   MPU9250_Real_Data.Mag_Y=GildeAverageValueFilter_MAG(Mag_AK8963.y-MPU9250_Offest_Data.Mag_Y,Data_Y_MAG);
   MPU9250_Real_Data.Mag_Z=GildeAverageValueFilter_MAG(Mag_AK8963.z-MPU9250_Offest_Data.Mag_Z,Data_Z_MAG); 
	 DeclineG();
}


void AHRSUpdate_GraDes_Delay_Corretion( void)
{
	  float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
	  double halfT;
    float tempq0,tempq1,tempq2,tempq3;	 
	  float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
	
	
	  float gx=MPU9250_Real_Data.Gyro_X;
    float	gy=MPU9250_Real_Data.Gyro_Y;
		float gz=MPU9250_Real_Data.Gyro_Z;
	  float ax=MPU9250_Real_Data.Accel_X; 
		float ay=MPU9250_Real_Data.Accel_Y;
		float az=MPU9250_Real_Data.Accel_Z;
		float mx=MPU9250_Real_Data.Mag_X;
		float my=MPU9250_Real_Data.Mag_Y;
		float mz=MPU9250_Real_Data.Mag_Z; 

		static volatile double last_update, now_update;
    now_update = Get_Sys_Ticks(); //ms
    halfT =  ((float)(now_update - last_update) / 2000.0f);
    last_update = now_update;
		
		norm = invSqrt(ax*ax + ay*ay + az*az);     
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
		norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
	  /**************���ٶ�������ת���ɽǶ��ƣ���λ:��/��(deg/s)*************/
		gx*=GYRO_CALIBRATION_COFF;
		gy*=GYRO_CALIBRATION_COFF;
		gz*=GYRO_CALIBRATION_COFF;
		/* ת��Ϊ�����ƣ�������̬����*/
		gx*=DEG2RAD;
		gy*=DEG2RAD;
		gz*=DEG2RAD;	
		if(fabs(gx)<0.003)
		{
		  gx=0;
		}
		if(fabs(gy)<0.003)
		{
		  gy=0;
		}
		if(fabs(gz)<0.005)
		{
		  gz=0;
		}
	
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 	
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	  norm = sqrt(gx*gx+gy*gy+gz*gz);
    if (norm<0.314f)
		{
			exInt = exInt + ex * Ki * halfT;
			eyInt = eyInt + ey * Ki * halfT;	
			ezInt = ezInt + ez * Ki * halfT;		
			gx = gx + Kp*ex + exInt;
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;
		}
		else 
		{
			gx = gx + 0.5f*Kp*ex ;
			gy = gy + 0.5f*Kp*ey ;
			gz = gz + 0.5f*Kp*ez ;
		}
		// ��Ԫ��΢�ַ���
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;		
		
	last_yaw_temp = yaw_temp;
	yaw_temp = -atan2(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2*q2 - 2.0f * q3 * q3 + 1.0f)* RAD2DEG; // yaw        -180----180
    Angular_Handler.Pitch = -asin(-2.0f * q1 * q3 + 2.0f * q0 * q2)* RAD2DEG; // pitch    -pi/2    --- pi/2 
    Angular_Handler.ROLL = atan2(2.0f * q2 * q3 + 2 * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f)* RAD2DEG; // roll       -pi----- 	

		if(yaw_temp-last_yaw_temp>=330)  //yaw��ǶȾ����������������
		{
			yaw_count--;
		}
		else if (yaw_temp-last_yaw_temp<=-330)
		{
			yaw_count++;
		}		
		Angular_Handler.YAW  = yaw_temp + yaw_count*360;  //yaw��Ƕ�
		
}

//�Ľ������������
void RungleKuta(void )
{
//	static float lasttime;
	static int num = 4;
	if(num>=4)
	{
//	  lasttime= Get_Sys_Ticks();
		
	}
}

//�Ľ���ŷ���㷨
void Eula(float input)
{
	
}

//���ٶ���������ȥ��
void DeclineG(void )
{
//	Accex_x = (Angular_Handler.A_X + arm_sin_f32(Angular_Handler.Pitch * DEG2RAD)*GRAVITY_MSS)/fabs(arm_sin_f32(PI/2-Angular_Handler.Pitch * DEG2RAD));
}
