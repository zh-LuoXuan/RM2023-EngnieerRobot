#include "fine_tuning_task.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "gpio.h" 
//#include "control_task.h"
/*==============================================================*/
#define FINE_TUNING_TASK_PRIO 26
#define FINE_TUNING_STK_SIZE 256
TaskHandle_t FineTuning_Handler;
extern int rest_flag;
void fine_tuning_task(void);

/*==============================================================*/

void task_fine_tuning_Crate(void)
{
	xTaskCreate((TaskFunction_t)fine_tuning_task,
                (const char *)"fine_tuning_task",
                (uint16_t)FINE_TUNING_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)FINE_TUNING_TASK_PRIO,
                (TaskHandle_t *)&FineTuning_Handler);
}
extern int flage1;
int s_pidflag=0;
static uint16_t QQ_Flag = 1,EE_Flag=1;
int UP_LEFT_Motor_limit=0;
int UP_RIGHT_Motor_limit=0;
int AHEAD_LEFT_Motor_limit=0;
int AHEAD_RIGHT_Motor_limit=0;
void fine_tuning_task(void)
{
	static portTickType TickCount; 
	
	TickCount = xTaskGetTickCount();
	
	while(1)
	{
		
		if(makeMoney.ctrlMode == CTRL_FINETUNING)
		{	
			
			/**********************************上下控制限幅**********************************************/
			if(makeMoney.UpMotor[LEFT_MOTOR].AngleFdb-makeMoney.UpMotor[LEFT_MOTOR].CenterOffset>58749)
					UP_LEFT_Motor_limit=1;
			else
				UP_LEFT_Motor_limit=0;
			if(makeMoney.UpMotor[RIGHT_MOTOR].AngleFdb-makeMoney.UpMotor[RIGHT_MOTOR].CenterOffset<-58749)
       	UP_RIGHT_Motor_limit=1;
			else
					UP_RIGHT_Motor_limit=0;
		if(UP_LEFT_Motor_limit==0&&UP_RIGHT_Motor_limit==0 )
			{
      if(makeMoney.Input_Point->rc.ch[3] > 10)
			{
				makeMoney.UpMotor[LEFT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[3] * 5;
				makeMoney.UpMotor[RIGHT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[3] * 5;				
			}
			else if(makeMoney.Input_Point->rc.ch[3] < -10)
			{
				makeMoney.UpMotor[LEFT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[3] * 5;
				makeMoney.UpMotor[RIGHT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[3] * 5;
			}	
      else
			{
        makeMoney.UpMotor[LEFT_MOTOR].SpeedRef = 0;
				makeMoney.UpMotor[RIGHT_MOTOR].SpeedRef = 0;
			}	
		}
   else if(UP_LEFT_Motor_limit==1||UP_RIGHT_Motor_limit==1)//只能进行向下移动
	  	{
		 if(makeMoney.Input_Point->rc.ch[3] < -10)
			{
				makeMoney.UpMotor[LEFT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[3] * 5;
				makeMoney.UpMotor[RIGHT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[3] * 5;
			}	
      else
			{
        makeMoney.UpMotor[LEFT_MOTOR].SpeedRef = 0;
				makeMoney.UpMotor[RIGHT_MOTOR].SpeedRef = 0;
			}	
		}
		//向下限位
else if(L_UP_KEY == 1||R_UP_KEY == 1)//微动开关按下//只能进行向上移动
			{
			 if(makeMoney.Input_Point->rc.ch[3] > 10)
				{
					makeMoney.UpMotor[LEFT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[3] * 5;
					makeMoney.UpMotor[RIGHT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[3] * 5;
				}	
				else
				{
					makeMoney.UpMotor[LEFT_MOTOR].SpeedRef = 0;
					makeMoney.UpMotor[RIGHT_MOTOR].SpeedRef = 0;
				}	
			}			


			/***********************************PITCH轴**********************************************/
			
			if(makeMoney.Input_Point->rc.ch[1] > 10)
			{
				makeMoney.TopPitchMotor.AngleRef -= (makeMoney.Input_Point->rc.ch[1]-10)*0.0010f;
			}
			else if(makeMoney.Input_Point->rc.ch[1] < -10)
			{
				makeMoney.TopPitchMotor.AngleRef -= (makeMoney.Input_Point->rc.ch[1]+10)*0.0010f;
			}			
			else
			{			
				makeMoney.TopPitchMotor.AngleRef +=0;
			}				
			
			/**********************************前后控制限幅**********************************************/
			if(makeMoney.StretchMotor[LEFT_MOTOR].AngleFdb-makeMoney.StretchMotor[LEFT_MOTOR].CenterOffset<-43000)
					AHEAD_LEFT_Motor_limit=1;
			else
				AHEAD_LEFT_Motor_limit=0;
			if(makeMoney.StretchMotor[RIGHT_MOTOR].AngleFdb-makeMoney.StretchMotor[RIGHT_MOTOR].CenterOffset>43000)
					AHEAD_RIGHT_Motor_limit=1;
			else
					AHEAD_RIGHT_Motor_limit=0;
			
			if(AHEAD_LEFT_Motor_limit==0&&AHEAD_RIGHT_Motor_limit==0 )
			{
			if(makeMoney.Input_Point->rc.ch[2] > 10)
			{
				
				makeMoney.StretchMotor[LEFT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[2] * 5;
				makeMoney.StretchMotor[RIGHT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[2] * 5;
				s_pidflag=0;
			}
			else if(makeMoney.Input_Point->rc.ch[2] < -10)
			{
				makeMoney.StretchMotor[LEFT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[2] * 5;
				makeMoney.StretchMotor[RIGHT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[2] * 5;
				s_pidflag=0;
			}			
			else
			{
        makeMoney.StretchMotor[LEFT_MOTOR].SpeedRef = 0;
				makeMoney.StretchMotor[RIGHT_MOTOR].SpeedRef = 0;
			}			
			
		}
			else if(AHEAD_LEFT_Motor_limit==1||AHEAD_RIGHT_Motor_limit==1)//只能进行向后移动
		{
		  if(makeMoney.Input_Point->rc.ch[2] < -10)
			{
				makeMoney.StretchMotor[LEFT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[2] * 5;
				makeMoney.StretchMotor[RIGHT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[2] * 5;
				s_pidflag=0;
			}			
			else
			{
        makeMoney.StretchMotor[LEFT_MOTOR].SpeedRef = 0;
				makeMoney.StretchMotor[RIGHT_MOTOR].SpeedRef = 0;

			}			
		}
			else if(L_STRETCH_KEY == 1||L_STRETCH_KEY == 1)//微动开关按下
		{
			if(makeMoney.Input_Point->rc.ch[2] > 10)
			{
				
				makeMoney.StretchMotor[LEFT_MOTOR].SpeedRef = -makeMoney.Input_Point->rc.ch[2] * 5;
				makeMoney.StretchMotor[RIGHT_MOTOR].SpeedRef = makeMoney.Input_Point->rc.ch[2] * 5;
				s_pidflag=0;
			}
			
			else
			{
        makeMoney.StretchMotor[LEFT_MOTOR].SpeedRef = 0;
				makeMoney.StretchMotor[RIGHT_MOTOR].SpeedRef = 0;
//				s_pidflag=0;
			}		
		}		
		

//			if(makeMoney.Input_Point->rc.ch[2] > 10)
//			{
//				makeMoney.StretchMotor[LEFT_MOTOR].AngleRef -= (makeMoney.Input_Point->rc.ch[2]-10)*0.010f;
//				makeMoney.StretchMotor[RIGHT_MOTOR].AngleRef += (makeMoney.Input_Point->rc.ch[2]-10)*0.010f;
//				s_pidflag=0;
//			}
//			else if(makeMoney.Input_Point->rc.ch[2] < -10)
//			{
//				makeMoney.StretchMotor[LEFT_MOTOR].AngleRef -= (makeMoney.Input_Point->rc.ch[2]+10)*0.010f;
//				makeMoney.StretchMotor[RIGHT_MOTOR].AngleRef += (makeMoney.Input_Point->rc.ch[2]+10)*0.010f;
//			}			
//			else
//			{
//        makeMoney.StretchMotor[LEFT_MOTOR].AngleRef += 0;
//				makeMoney.StretchMotor[RIGHT_MOTOR].AngleRef -= 0;
//				s_pidflag=0;
//			}		
			
					/***********************************ROLL轴**********************************************/
			
			if(makeMoney.Input_Point->rc.ch[0] > 10)
			{
				makeMoney.TopRollMotor.AngleRef += (makeMoney.Input_Point->rc.ch[0]-10)*0.0010f;
			}
			else if(makeMoney.Input_Point->rc.ch[0] < -10)
			{
				makeMoney.TopRollMotor.AngleRef += (makeMoney.Input_Point->rc.ch[0]+10)*0.0010f;
			}		
			else
			{
       makeMoney.TopRollMotor.AngleRef += 0;
			}	
			
			
		  if(IF_KEY_PRESSED_Q)//吸盘位置变换
				{
					QQ_Flag = 0;
				}
			if((!IF_KEY_PRESSED_Q && !QQ_Flag))
				{
					fanzhuan_Task(&makeMoney);
					QQ_Flag = 1;
				}	
				  if(IF_KEY_PRESSED_E)
				{
					EE_Flag = 0;
				}
			if((!IF_KEY_PRESSED_E && !EE_Flag))
				{
					qianshen_Task(&makeMoney);
					EE_Flag = 1;
				}	
					
	  }
		if(makeMoney.ctrlMode == CTRL_NULL&&(rest_flag==0))
		{
			CAN1_CMD_TOP_PITCH(0,0);
		}
		else
		{
			CAN1_CMD_TOP_PITCH(makeMoney.TopPitchMotor.Current,makeMoney.TopRollMotor.Current);
		}
		
		CAN2_CMD_VACUUM((int)&makeMoney.VacuumFlag);
		
		vTaskDelay(1);
//		vTaskDelayUntil(&TickCount, 1);
	}
}
static void fanzhuan_Task(ControlTask_t* fanzhuan)
{
	if(fanzhuan == NULL)
	{
		return ;
	}
	static u8 Flag = 0;
	
	if(Flag == 0)
	{
		fanzhuan->TopPitchMotor.AngleRef=fanzhuan->TopPitchMotor.CenterOffset-3500;
	}
	else if(Flag == 1)
	{
	fanzhuan->TopMoveMotor[0].AngleRef=fanzhuan->TopMoveMotor[0].CenterOffset+176;
	fanzhuan->TopMoveMotor[1].AngleRef=fanzhuan->TopMoveMotor[1].CenterOffset-183;//+-daidin
	}
	else if(Flag == 2)
	{
	s_pidflag=1;
	fanzhuan->StretchMotor[0].AngleRef=fanzhuan->StretchMotor[0].CenterOffset;	
	fanzhuan->StretchMotor[1].AngleRef=fanzhuan->StretchMotor[1].CenterOffset;
	}
	else if(Flag == 3)
	{
  fanzhuan->TopPitchMotor.AngleRef=fanzhuan->TopPitchMotor.CenterOffset-5100;
	}
	
	else if(Flag == 4)
	{
//	CAN2_CMD_VACUUM(0);
//	vTaskDelay(10);
   fanzhuan->TopPitchMotor.AngleRef=fanzhuan->TopPitchMotor.CenterOffset;
	}
//	else if(Flag == 5)
//	{
//	CAN2_CMD_VACUUM(1);
////	vTaskDelay(10);
//	fanzhuan->TopMoveMotor[0].AngleRef=fanzhuan->TopMoveMotor[0].CenterOffset+180;
//	fanzhuan->TopMoveMotor[1].AngleRef=fanzhuan->TopMoveMotor[1].CenterOffset-187;//+-daidin	
//	}
//	else if(Flag == 6)
//	{
//   fanzhuan->TopPitchMotor.AngleRef=fanzhuan->TopPitchMotor.CenterOffset-3500;
//	 vTaskDelay(500);	
//   fanzhuan->TopMoveMotor[0].AngleRef=fanzhuan->TopMoveMotor[0].CenterOffset;
//	 fanzhuan->TopMoveMotor[1].AngleRef=fanzhuan->TopMoveMotor[1].CenterOffset;//+-daidin	;
//	}
////	else if(Flag == 7)
////	{
////   fanzhuan->TopMoveMotor[0].AngleRef=fanzhuan->TopMoveMotor[0].CenterOffset;q
////	fanzhuan->TopMoveMotor[1].AngleRef=fanzhuan->TopMoveMotor[1].CenterOffset;//+-daidin	;
////	}
//	else if(Flag == 7)
//	{
//   fanzhuan->TopPitchMotor.AngleRef=fanzhuan->TopPitchMotor.CenterOffset;
//	}
	
	Flag ++;
		
	if(Flag==5)
	{
		Flag=0;
//		s_pidflag=0;
		
	}


}
	
//******************前伸任务**************/

static void qianshen_Task(ControlTask_t* qianshen)
{
		if(qianshen == NULL)
	{
		return ;
	}
	static u8 Flag1 = 0;
		 if(Flag1 == 0)
	{
	qianshen->TopMoveMotor[0].AngleRef=qianshen->TopMoveMotor[0].CenterOffset;
	qianshen->TopMoveMotor[1].AngleRef=qianshen->TopMoveMotor[1].CenterOffset;//+-daidin
	}
	else if(Flag1 == 1)
	{
	qianshen->TopMoveMotor[0].AngleRef=qianshen->TopMoveMotor[0].CenterOffset+190;	
	qianshen->TopMoveMotor[1].AngleRef=qianshen->TopMoveMotor[1].CenterOffset-197;
	}
Flag1++;
if(Flag1==2)
{
	Flag1=0;

}

}












