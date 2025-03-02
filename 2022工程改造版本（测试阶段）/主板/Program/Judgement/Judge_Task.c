#include "judge_Task.h"
#include "detect_task.h"
#define JUDGE_BUFFER_LEN 		100
#define JUDGE_INTER_TIME 		2
u8 Judge_Buffer[JUDGE_BUFFER_LEN];		

Judge_FLAG Judge_Flag;
uint32_t judgeisTaskStack;
//��ȡ����ϵͳ
void Judge_task(void *pvParameters)
{
	u8 	RES;
	while(1)
	{
		xQueueReceive(RxCOM1,&RES,portMAX_DELAY);		
		if(Judge_Flag.Flag==0 && RES==FRAME_HEADER)	//֡ͷ�ĵ�һ���ֽ�
		{
			Judge_Flag.Flag=1;
			Judge_Buffer[0]=RES;
		}
		else if(Judge_Flag.Flag==1)				//����
		{
			Judge_Flag.Flag=2;
			Judge_Buffer[1]=RES;
		}
		else if(Judge_Flag.Flag==2)				//����
		{
			Judge_Flag.Flag=3;
			Judge_Buffer[2]=RES;
			Judge_Flag.data_len=(u16)(RES<<8)|Judge_Buffer[1]+ LEN_CMDID + LEN_TAIL;
			if(Judge_Flag.data_len>100)
			{
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==3)				//�����?
		{
			Judge_Flag.Flag=4;
			Judge_Buffer[3]=RES;
		}
		else if(Judge_Flag.Flag==4)				//CRC8
		{
			Judge_Flag.Flag=5;
			Judge_Buffer[4]=RES;
			if (verify_crc8_check_sum( Judge_Buffer, HEADER_LEN ) != NULL)	//CRC8У��
			{
				Judge_Flag.data_cnt=0;
			}
			else				//У��û��ͨ�������¿�ʼ
			{
				Judge_Flag.data_cnt=0;
				Judge_Flag.Flag=0;
			}
		}
		else if(Judge_Flag.Flag==5 && Judge_Flag.data_len>0)	//��ʼ��������
		{
			Judge_Flag.data_len--;
//			Judge_Flag.data_cnt=Judge_Flag.data_cnt+1;
			Judge_Buffer[LEN_HEADER+Judge_Flag.data_cnt++]=RES;
			if(Judge_Flag.data_len==0)
			{
				Judge_Flag.Flag=6;
			}	
		}
		else 
		{
			Judge_Flag.Flag=0;
			Judge_Flag.data_len=0;
			Judge_Flag.data_cnt=0;
		}
		if(Judge_Flag.Flag==6)
		{
			Judge_DataVerify(Judge_Buffer);		//֡ͷ���ּ����ϣ���ȡ����ϵͳ����
			Judge_Flag.Flag=0;								//������������
			Judge_Flag.data_len=0;
			Judge_Flag.data_cnt=0;
		}judgeisTaskStack = uxTaskGetStackHighWaterMark(NULL);
	}
 
}





//����ϵͳ���ݽ���
void Judge_DataVerify(u8 *Buff)
{
	Dateframe_t	*frame;
  if(Buff!=NULL)
  {
	frame=(Dateframe_t *)Buff;
	//������֡ͷ����֡����CRCУ��  ֡ͷCRC8  ��֡CRC16
	if(verify_crc16_check_sum((uint8_t *)frame, HEADER_LEN + CMD_LEN + frame->FrameHeader.DataLength + CRC_LEN))
	{
		judgement_data_handler(Buff);  //ͨ��У��������ݽ���?
	}
 }
}




