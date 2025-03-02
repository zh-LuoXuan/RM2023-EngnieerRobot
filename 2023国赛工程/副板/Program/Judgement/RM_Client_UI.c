/*************************************************************
  ****************************(C) COPYRIGHT 2021 NCIST****************************
  * @file       UI_task.c/h
  * @brief      上位机通信任务，视觉数据处理
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     2021           zhuxunfu              finished
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */

#include "RM_Client_UI.h"
#include "protocol.h"
#include "control_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <string.h>
#include "RemoteControl.h"
#include "judgement_info.h"
#define Get_CRC16_Check_Sum_UI get_crc16_check_sum
#define Get_CRC8_Check_Sum_UI get_crc8_check_sum 



#define UI_TASK_PRIO 4
#define UI_TASK_SIZE 1024
TaskHandle_t UI_TASK_Handler;
void UI_task(void);
	
/*==============================================================*/
void task_UI_Create(void)
{
	 xTaskCreate((TaskFunction_t )UI_task,
                (const char*    )"ui_task",
                (uint16_t       )UI_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )UI_TASK_PRIO,
                (TaskHandle_t*  )&UI_TASK_Handler);
}
/*==============================================================*/

extern uint8_t Judge_Self_ID;
extern uint16_t Judge_SelfClient_ID;

portTickType xvTaskLasttime = 0;


extern int UI_Flag;
int ff;

extern ext_game_robot_state_t			  	GameRobotStat;				//0x0201
extern char mouse_r; //自瞄标志位
ext_robot_hurt_t hurt_data;
extern u32 Robot_ID,Cilent_ID;

String_Data H1[2],H2[2];
char Gold[4] = "Gold";
char Catch[5] = "Catch";
char onesilver[12] = "Silver_First";
char twosilver[13] = "Silver_Second";
char exchange[8] = "Exchange";
char do_nothing[10] = "Do_Nothing";
Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11,G12,G13,G14,G15,G16,G17,G18,G19,G20,G21;//,G22,G23,G24,G25,G26,G27,G28;
void UI_task(void)
{

		Line_Draw(&G2,"106",UI_Graph_ADD,9,UI_Color_Orange,4,830,480,830,1000);
		Line_Draw(&G3,"107",UI_Graph_ADD,9,UI_Color_Orange,4,1050,480,1050,1000);
		Line_Draw(&G5,"095",UI_Graph_ADD,9,UI_Color_White,4,600,450,1300,450);	
		Line_Draw(&G13,"103",UI_Graph_ADD,9,UI_Color_White,4,620,0,800,290);	
		Line_Draw(&G14,"104",UI_Graph_ADD,9,UI_Color_White,4,1150,290,1370,0);
		Line_Draw(&G11,"101",UI_Graph_ADD,9,UI_Color_White,4,500,0,760,290);	
		Line_Draw(&G12,"102",UI_Graph_ADD,9,UI_Color_White,4,1190,290,1480,0);
		Char_Draw(&H1[0],"001",UI_Graph_ADD,1,UI_Color_Pink,18,9,2,180,540,do_nothing);
	
	 Char_ReFresh(H1[0]);
//	UI_ReFresh(1,G17);
//		Line_Draw(&G17,"120",UI_Graph_ADD,9,UI_Color_Cyan,3,860,500,940,500); 
		
	//		Line_Draw(&G11,"101",UI_Graph_ADD,9,UI_Color_Cyan,4,730,623,730,1000);//..//
	//		Line_Draw(&G12,"102",UI_Graph_ADD,9,UI_Color_Cyan,4,1070,623,1070,1000);//..//	
	//		Arc_Draw(&G10,"100", UI_Graph_ADD,9,UI_Color_Pink,90,270,3,900,623,170,170);//..//
	//		Line_Draw(&G9,"099",UI_Graph_ADD,9,UI_Color_Pink,4,750,520,200,250);//..//  
	//  	Char_Draw(&NCIST,"111",UI_Graph_ADD,2,UI_Color_Pink,18,12,2,180,780,&ncist[0]);//..	
	//  	Line_Draw(&G6,"096",UI_Graph_ADD,9,UI_Color_White,4,770,430,800,480);	//
	//		Line_Draw(&G7,"097",UI_Graph_ADD,9,UI_Color_White,4,1010,430,970,480);//	
	//		Line_Draw(&G8,"098",UI_Graph_ADD,9,UI_Color_Pink,4,1050,500,1700,105);//
	//		Line_Draw(&G15,"108",UI_Graph_ADD,9,UI_Color_Cyan,3,850,580,810,600);//..
	//		Line_Draw(&G16,"109",UI_Graph_ADD,9,UI_Color_Cyan,3,950,580,990,600);//..
	//		Line_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Cyan,4,680,623,1115,623);//
for(;;)
{
			
	if(UI_Flag == 0)
	{
		memset(&H1[1],0,sizeof(H1[1]));
		Char_Draw(&H1[1],"001",UI_Graph_Change,1,UI_Color_Cyan,24,10,4,1500,800,&do_nothing[0]);
		ff = 0;
	}	
	if(UI_Flag == 1)
	{
		memset(&H1[1],0,sizeof(H1[1]));
		Char_Draw(&H1[1],"001",UI_Graph_Change,1,UI_Color_Cyan,24,4,4,1500,800,&Gold[0]);
		ff = 1;
	}
	if(UI_Flag == 5)
	{
		memset(&H1[1],0,sizeof(H1[1]));
		Char_Draw(&H1[1],"001",UI_Graph_Change,1,UI_Color_Cyan,24,5,4,1500,800,&Catch[0]);
		ff = 5;
	}
	if(UI_Flag == 3)
	{
		memset(&H1[1],0,sizeof(H1[1]));
		Char_Draw(&H1[1],"001",UI_Graph_Change,1,UI_Color_Cyan,24,12,4,1500,800,&onesilver[0]);
		ff = 3;
	}
	if(UI_Flag == 6)
	{
		memset(&H1[1],0,sizeof(H1[1]));
		Char_Draw(&H1[1],"001",UI_Graph_Change,1,UI_Color_Cyan,24,8,4,1500,800,&exchange[0]);
		ff = 6;

	}
	if(UI_Flag == 4)
	{
		memset(&H1[1],0,sizeof(H1[1]));
		Char_Draw(&H1[1],"001",UI_Graph_Change,1,UI_Color_Cyan,24,13,4,1500,800,&twosilver[0]);
		ff = 4;
	}
	
			
	
	 
	if(GameRobotStat.robot_id == 2)
	{
		Robot_ID = 2;
		Cilent_ID = 0x0102;
	}
	if(GameRobotStat.robot_id == 102)
	{
		Robot_ID = 102;
		Cilent_ID = 0x0166;
	}
	UI_ReFresh(7,G2,G3,G5,G13,G14,G11,G12);

	Char_ReFresh(H1[0]);
	Char_ReFresh(H1[1]);
	
	vTaskDelay(1);

//		PID_Output(&ctrlTask.motorCtrl);
}


}



/****************************************************************************************/
unsigned char UI_Seq;                      //包序号
u32 Robot_ID,Cilent_ID;
/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch)
{
   USART_SendData(USART6,ch);
   while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);	
}



/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i,n;
   Graph_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
	
   is_red_or_blue();
	 determine_ID();
//	 ID_Read(); 
	
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
   datahead.Sender_ID = Judge_Self_ID;
   datahead.Receiver_ID = Judge_SelfClient_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验
      
      for(n=0;n<sizeof(imageData);n++)
      {
         UI_SendByte(*framepoint);
         framepoint++;             
      }                                               //发送图片帧
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   va_end(ap);
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}



/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate,u8 Del_Layer)
{

   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   int loop_control;                       //For函数循环控制
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   datahead.Data_ID=UI_Data_ID_Del;
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //控制信息
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16校验值计算
   
   framepoint=(unsigned char *)&framehead;
   for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&del;
   for(loop_control=0;loop_control<sizeof(del);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                                 //发送所有帧
   framepoint=(unsigned char *)&frametail;
   for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   UI_Seq++;                                                         //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
        
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
   image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate  图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
        
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = Graph_Float;
}



/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->Graph_Control.graphic_name[i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}


/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_DrawChar;

   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //发送图片帧
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}
