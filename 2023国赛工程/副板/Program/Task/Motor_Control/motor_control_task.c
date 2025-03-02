#include "motor_control_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "control_task.h"
#include "CAN_Receive.h"
#include "new_pid.h"
#include "Time7.h"
#include "user_lib.h"

/*==============================================================*/
#define MOTOR_CONTROL_TASK_PRIO 18
#define MOTOR_CONTROL_STK_SIZE 256
TaskHandle_t Motor_Control_Task_Handler;
void Motor_Control_task(void);

/*==============================================================*/
void task_Motor_Control_Create(void)
{
	xTaskCreate((TaskFunction_t)Motor_Control_task,
                (const char *)"Motor_Control_task",
                (uint16_t)MOTOR_CONTROL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)MOTOR_CONTROL_TASK_PRIO,
                (TaskHandle_t *)&Motor_Control_Task_Handler);
}
/*==============================================================*/

fp32 sucPitchAgePid[3] = {SUC_PITCH_ANGLE_PID_KP, SUC_PITCH_ANGLE_PID_KI, SUC_PITCH_ANGLE_PID_KD};
fp32 sucPitchSpdPid[3] = {SUC_PITCH_SPEED_PID_KP, SUC_PITCH_SPEED_PID_KI, SUC_PITCH_SPEED_PID_KD};
fp32 sucRollAgePid[3] = {SUC_ROLL_ANGLE_PID_KP, SUC_ROLL_ANGLE_PID_KI, SUC_ROLL_ANGLE_PID_KD};
fp32 sucRollSpdPid[3] = {SUC_ROLL_SPEED_PID_KP, SUC_ROLL_SPEED_PID_KI, SUC_ROLL_SPEED_PID_KD};
fp32 upAgePid[3] = {UP_ANGLE_PID_KP, UP_ANGLE_PID_KI, UP_ANGLE_PID_KD};
fp32 upSpdPid[3] = {UP_SPEED_PID_KP, UP_SPEED_PID_KI, UP_SPEED_PID_KD};


void Motor_Control_task(void)
{
	static portTickType TickCount; 
	TickCount = xTaskGetTickCount();
	
	while(1)
	{
		PID_Output(&ctrlTask.motorCtrl);
		vTaskDelay(1);
	}
}



/*********************************************************************************************************************/
void motorInit(MotorControl_t* Mctrl)
{
	if(Mctrl == NULL)
	{
		return;
	}
	uint8_t i = 0;
/*****************************************��ȡ����ָ��************************************/	
	for(i = 0; i < 2; i++)
	{
		Mctrl->UpMotor[i].motor_measure = get_Up_EncoderProcess_Point(i);	
	}
	Mctrl->SucPitchMotor.motor_measure = get_SucPitch_EncoderProcess_Point();
	Mctrl->SucRollMotor.motor_measure = get_SucRoll_EncoderProcess_Point();
	
/***************************��ʼ����������************************************/	
	for(i = 0; i < 2; i++)
	{
		Mctrl->UpMotor[i].AngleFdb = Mctrl->UpMotor[i].motor_measure->real_ecd;		
		Mctrl->UpMotor[i].SpeedFdb = Mctrl->UpMotor[i].motor_measure->speed_rpm;	
	}	
	Mctrl->SucPitchMotor.AngleFdb = Mctrl->SucPitchMotor.motor_measure->real_ecd;
	Mctrl->SucPitchMotor.SpeedFdb = Mctrl->SucPitchMotor.motor_measure->speed_rpm;
	Mctrl->SucRollMotor.AngleFdb = Mctrl->SucRollMotor.motor_measure->real_ecd;
	Mctrl->SucRollMotor.SpeedFdb = Mctrl->SucRollMotor.motor_measure->speed_rpm;
	
	/***************************************��ʼ����������************************************/		
	for(i = 0; i < 2; i++)
	{
		Mctrl->UpMotor[i].CenterOffset = Mctrl->UpMotor[i].motor_measure->real_ecd;
	}
	Mctrl->SucPitchMotor.CenterOffset = Mctrl->SucPitchMotor.motor_measure->real_ecd;
	Mctrl->SucRollMotor.CenterOffset = Mctrl->SucRollMotor.motor_measure->real_ecd;
	
	/****************************************��ʼ����������************************************/
	for(i = 0; i < 2; i++)
	{
		Mctrl->UpMotor[i].AngleRef = Mctrl->UpMotor[i].CenterOffset;
	}		
	Mctrl->SucPitchMotor.AngleRef = Mctrl->SucPitchMotor.CenterOffset;
	Mctrl->SucRollMotor.AngleRef = Mctrl->SucRollMotor.CenterOffset;
	
/******************************************̧��PID��ʼ��************************************/
  for(i = 0; i < 2; i++)
		{	
			New_PID_Init(&Mctrl->UpMotor[i].AnglePid, 
									 NEW_PID_POSITION, 
									 upAgePid, 
									 UP_ANGLE_PID_MAX_OUT, 
									 UP_ANGLE_PID_MAX_IOUT, 
									 UP_ANGLE_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 UP_ANGLE_PID_I_SEPARATION);
			
			
			New_PID_Init(&Mctrl->UpMotor[i].SpeedPid, 
									 NEW_PID_POSITION, 
									 upSpdPid, 
									 UP_SPEED_PID_MAX_OUT, 
									 UP_SPEED_PID_MAX_IOUT, 
									 UP_SPEED_PID_DEAD_ZONE,
									 0.0f,
									 0.0f,
									 0.0f,
									 UP_SPEED_PID_I_SEPARATION);

	 }				
		
/*************************����pitch��PID��ʼ��************************************/
	New_PID_Init(&Mctrl->SucPitchMotor.AnglePid, 
	             NEW_PID_POSITION, 
	             sucPitchAgePid, 
	             SUC_PITCH_ANGLE_PID_MAX_OUT, 
	             SUC_PITCH_ANGLE_PID_MAX_IOUT, 
               SUC_PITCH_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             SUC_PITCH_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&Mctrl->SucPitchMotor.SpeedPid, 
	             NEW_PID_POSITION, 
	             sucPitchSpdPid, 
	             SUC_PITCH_SPEED_PID_MAX_OUT, 
	             SUC_PITCH_SPEED_PID_MAX_IOUT, 
               SUC_PITCH_SPEED_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             SUC_PITCH_SPEED_PID_I_SEPARATION);

/*************************����roll��PID��ʼ��************************************/
	New_PID_Init(&Mctrl->SucRollMotor.AnglePid, 
	             NEW_PID_POSITION, 
	             sucRollAgePid, 
	             SUC_ROLL_ANGLE_PID_MAX_OUT, 
	             SUC_ROLL_ANGLE_PID_MAX_IOUT, 
               SUC_ROLL_ANGLE_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             SUC_ROLL_ANGLE_PID_I_SEPARATION);
	
	New_PID_Init(&Mctrl->SucRollMotor.SpeedPid, 
	             NEW_PID_POSITION, 
	             sucRollSpdPid, 
	             SUC_ROLL_SPEED_PID_MAX_OUT, 
	             SUC_ROLL_SPEED_PID_MAX_IOUT, 
               SUC_ROLL_SPEED_PID_DEAD_ZONE,
	             0.0f,
	             0.0f,
	             0.0f,
	             SUC_ROLL_SPEED_PID_I_SEPARATION);
							 
/***************************�޷���־��ʼ��************************************/
  for(i = 0; i < 2; i++)
	{
		Mctrl->UpMotor[i].Amplitude_Flag = false;
	}
	Mctrl->SucPitchMotor.Amplitude_Flag = false;
	Mctrl->SucRollMotor.Amplitude_Flag = false;
	
/***************************��λ��־��ʼ��************************************/
	for(i = 0; i < 2; i++)
	{
		Mctrl->UpMotor[i].ResFlag = false;
	}
	Mctrl->SucPitchMotor.ResFlag = false;
	Mctrl->SucRollMotor.ResFlag = false;
	
	/***************************PID��־��ʼ��************************************/
	Mctrl->ifSpid = false;
}

/**
  * @brief  PID���
  * @param  ����ṹ��
  * @retval void
  * @attention 
  */
void PID_Output(MotorControl_t *Output_Calc)
{
	if(Output_Calc == NULL)
	{
		return;
	}

		if(Output_Calc->UpMotor[LEFT_MOTOR].Amplitude_Flag && Output_Calc->UpMotor[RIGHT_MOTOR].Amplitude_Flag)
	{
		Output_Calc->UpMotor[LEFT_MOTOR].AngleRef = Constrain_float(Output_Calc->UpMotor[LEFT_MOTOR].AngleRef, \
																																				(Output_Calc->UpMotor[LEFT_MOTOR].CenterOffset + L_Up_LONGS_LOWEST), \
																																				(Output_Calc->UpMotor[LEFT_MOTOR].CenterOffset + L_UP_LONGS_HIGHTST));
		Output_Calc->UpMotor[RIGHT_MOTOR].AngleRef = Constrain_float(Output_Calc->UpMotor[RIGHT_MOTOR].AngleRef,  \
																																				 (Output_Calc->UpMotor[RIGHT_MOTOR].CenterOffset + R_UP_LONGS_HIGHTST), \
																																				 (Output_Calc->UpMotor[RIGHT_MOTOR].CenterOffset + R_Up_LONGS_LOWEST));
	}
	if(Output_Calc->SucPitchMotor.Amplitude_Flag)
	{
		Output_Calc->SucPitchMotor.AngleRef = Constrain_float(Output_Calc->SucPitchMotor.AngleRef,  \
																																	(Output_Calc->SucPitchMotor.CenterOffset), \
																																	(Output_Calc->SucPitchMotor.CenterOffset + (210 *REDUCTION_RATIO_3508)));
	}
	if(Output_Calc->SucRollMotor.Amplitude_Flag)
	{
		Output_Calc->SucRollMotor.AngleRef = Constrain_float(Output_Calc->SucRollMotor.AngleRef,  \
																																	(Output_Calc->SucRollMotor.CenterOffset), \
																																	(Output_Calc->SucRollMotor.CenterOffset + ((180 + 60) * REDUCTION_RATIO_2006)));
	}
	
	if(Output_Calc->ifSpid)
	{
		for(uint8_t i = 0; i < 2; i++)
			{
				New_PID_Calc(&Output_Calc->UpMotor[i].AnglePid , Output_Calc->UpMotor[i].motor_measure->real_ecd , Output_Calc->UpMotor[i].AngleRef);
				New_PID_Calc(&Output_Calc->UpMotor[i].SpeedPid , Output_Calc->UpMotor[i].motor_measure->speed_rpm , Output_Calc->UpMotor[i].AnglePid.out);
			}
	}	
	else
	{
    for(uint8_t i = 0; i < 2; i++)
			{
				New_PID_Calc(&Output_Calc->UpMotor[i].SpeedPid , Output_Calc->UpMotor[i].motor_measure->speed_rpm , Output_Calc->UpMotor[i].SpeedRef);
			}
	}		
		New_PID_Calc(&Output_Calc->SucPitchMotor.AnglePid , Output_Calc->SucPitchMotor.motor_measure->real_ecd , Output_Calc->SucPitchMotor.AngleRef);
		New_PID_Calc(&Output_Calc->SucPitchMotor.SpeedPid , Output_Calc->SucPitchMotor.motor_measure->speed_rpm , Output_Calc->SucPitchMotor.AnglePid.out);
		
	  New_PID_Calc(&Output_Calc->SucRollMotor.AnglePid , Output_Calc->SucRollMotor.motor_measure->real_ecd , Output_Calc->SucRollMotor.AngleRef);
		New_PID_Calc(&Output_Calc->SucRollMotor.SpeedPid , Output_Calc->SucRollMotor.motor_measure->speed_rpm , Output_Calc->SucRollMotor.AnglePid.out);
		
    CAN1_CMD_0x200(Output_Calc->UpMotor[0].SpeedPid.out , \
			             Output_Calc->UpMotor[1].SpeedPid.out , \
			             Output_Calc->SucPitchMotor.SpeedPid.out , \
	                 Output_Calc->SucRollMotor.SpeedPid.out);
}

/**
  * @brief  ����Pitchת����ָ��λ��
  * @param  ����ṹ��
  * @retval void
  * @attention 
  */
void SucPitch_Turn(MotorControl_t* sucPitchTurn, fp32 Location)
{
	TIM_Cmd(TIM7,ENABLE); //ʹ�ܶ�ʱ��7
	while(time7_count < TIME_STAMP_1500MS)
	{
//		sucPitchTurn->SucPitchMotor.AngleRef = RAMP_float( Location, sucPitchTurn->SucPitchMotor.motor_measure->real_ecd, 150);
		sucPitchTurn->SucPitchMotor.AngleRef = Location;
		if(fabs(Location - sucPitchTurn->SucPitchMotor.motor_measure->real_ecd) < 0.1f)
		{
			TIM_Cmd(TIM7,DISABLE);
	    time7_count = 0;
			break;
		}
		
		PID_Output(sucPitchTurn);
		vTaskDelay(TIME_STAMP_1MS);//ϵͳ��ʱ
	}
	TIM_Cmd(TIM7,DISABLE);
	time7_count = 0;
	
}


/**
  * @brief  ����Rollת����ָ��λ��
  * @param  ����ṹ��
  * @retval void
  * @attention 
  */
void SucRoll_Turn(MotorControl_t* sucRollTurn, fp32 Location)
{
	TIM_Cmd(TIM7,ENABLE); //ʹ�ܶ�ʱ��7
	while(time7_count < TIME_STAMP_1500MS)
	{
//		sucRollTurn->SucRollMotor.AngleRef = RAMP_float( Location, sucRollTurn->SucRollMotor.motor_measure->real_ecd, 150);
		sucRollTurn->SucRollMotor.AngleRef = Location;
		if(fabs(Location - sucRollTurn->SucRollMotor.motor_measure->real_ecd) < 0.1f)
		{
			TIM_Cmd(TIM7,DISABLE);
	    time7_count = 0;
			break;
		}
		
		PID_Output(sucRollTurn);
		vTaskDelay(TIME_STAMP_1MS);//ϵͳ��ʱ
	}
	
	TIM_Cmd(TIM7,DISABLE);
	time7_count = 0;
	
}


/**
  * @brief  �ڿ���˶���ָ��λ��
  * @param  ����ṹ��, ����λ�ã� �ҵ��λ��
  * @retval void
  * @attention 
  */
void Up_Down_To_Point(MotorControl_t* Motor_Up_Down, fp32 L_Location, fp32 R_Location)
{
	TIM_Cmd(TIM7,ENABLE); //ʹ�ܶ�ʱ��7
	while(time7_count < TIME_STAMP_500MS)
	{
  	Motor_Up_Down->UpMotor[LEFT_MOTOR].AngleRef = L_Location;
		Motor_Up_Down->UpMotor[RIGHT_MOTOR].AngleRef = R_Location;
		
		if((fabs(L_Location-Motor_Up_Down->UpMotor[LEFT_MOTOR].motor_measure->real_ecd)<0.1f) \
			&&(fabs(R_Location-Motor_Up_Down->UpMotor[RIGHT_MOTOR].motor_measure->real_ecd)<0.1f))
		  {
		  	TIM_Cmd(TIM7,DISABLE);
	      time7_count = 0;
		  	break;
		  }
		PID_Output(Motor_Up_Down);
		vTaskDelay(TIME_STAMP_1MS);//ϵͳ��ʱ
	}
	TIM_Cmd(TIM7,DISABLE);
	time7_count = 0;
}

