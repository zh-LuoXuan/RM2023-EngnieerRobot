#include "ui_task.h"
#include "RM_Client_UI.h"
#include "string.h"

/*==============================================================*/
#define USER_UI_TASK_PRIO 20
#define USER_UI_TASK_STK_SIZE 512
TaskHandle_t User_UI_Task_Handler;
void user_UI_Task(void *pvParameters);

/*==============================================================*/
void task_user_UI_Create(void)
{
	xTaskCreate((TaskFunction_t)user_UI_Task,
                (const char *)"user_UI_Task",
                (uint16_t)USER_UI_TASK_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)USER_UI_TASK_PRIO,
                (TaskHandle_t *)&User_UI_Task_Handler);
}
/*==============================================================*/



Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11,G12,G13,G14,G15,G16,G17,G18,G19,G20,G21,G22,G23,G24,G25,G26,G27,G28;
void user_UI_Task(void *pvParameters)
{
		memset(&G1,0,sizeof(G1));//中心垂线
		memset(&G2,0,sizeof(G2));//上击打线
		memset(&G3,0,sizeof(G3));//中心水平线
		memset(&G4,0,sizeof(G4));//枪管轴心线
		memset(&G5,0,sizeof(G5));//下击打线
		memset(&G6,0,sizeof(G6));//远距离击打线
		memset(&G7,0,sizeof(G7));//摩擦轮状态
//		memset(&CH_SHOOT,0,sizeof(CH_SHOOT));//摩擦轮标识
	    memset(&G8,0,sizeof(G8));
	    memset(&G9,0,sizeof(G9));
//	  memset(&G10,0,sizeof(G10));//右装甲板状态
//	  memset(&G11,0,sizeof(G11));//后装甲板状态
//	  memset(&CH_FLRB,0,sizeof(CH_FLRB));//装甲板标识
	
	


	
//抓取矿石UI
	  //金矿
//		Line_Draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Cyan,3,860,700,1068,700);
		//银矿
//		Line_Draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Cyan,3,860,422,1068,422);
//		Line_Draw(&G4,"094",UI_Graph_ADD,9,UI_Color_Cyan,3,860,508,1068,508);
		
		//地面矿石
		Line_Draw(&G5,"095",UI_Graph_ADD,9,UI_Color_White,5,860,247,1068,247);

//救援抓钩UI		
	  Line_Draw(&G6,"096",UI_Graph_ADD,9,UI_Color_Cyan,3,524,0,561,277);	
		Line_Draw(&G7,"097",UI_Graph_ADD,9,UI_Color_Cyan,3,559,0,593,277);
		Line_Draw(&G8,"098",UI_Graph_ADD,9,UI_Color_Cyan,3,561,277,593,277);

		Line_Draw(&G9,"099",UI_Graph_ADD,9,UI_Color_Cyan,3,1418,0,1393,277);
		Line_Draw(&G10,"100",UI_Graph_ADD,9,UI_Color_Cyan,3,1386,0,1368,277);
		Line_Draw(&G11,"101",UI_Graph_ADD,9,UI_Color_Cyan,3,1393,277,1368,277);

//障碍块UI
//		Line_Draw(&G12,"102",UI_Graph_ADD,9,UI_Color_Cyan,3,353,158,472,418);		
//		Line_Draw(&G13,"103",UI_Graph_ADD,9,UI_Color_Cyan,3,399,154,472,418);
//		Line_Draw(&G14,"104",UI_Graph_ADD,9,UI_Color_Cyan,3,1586,158,1457,418);
//		Line_Draw(&G15,"105",UI_Graph_ADD,9,UI_Color_Cyan,3,1549,154,1457,418);

//兑换矿石UI

		
		
//车宽透视UI
		Line_Draw(&G16,"106",UI_Graph_ADD,9,UI_Color_Orange,4,680,352,680,684);
		Line_Draw(&G17,"107",UI_Graph_ADD,9,UI_Color_Orange,4,1257,352,1257,684);   
		Line_Draw(&G18,"108",UI_Graph_ADD,9,UI_Color_Orange,4,680,684,1257,684);
		Line_Draw(&G19,"109",UI_Graph_ADD,9,UI_Color_Orange,4,442,0,680,352);
		Line_Draw(&G20,"110",UI_Graph_ADD,9,UI_Color_Orange,4,1529,0,1257,352);
		
		Line_Draw(&G21,"111",UI_Graph_ADD,9,UI_Color_Cyan,4,632,0,790,352);
		Line_Draw(&G22,"112",UI_Graph_ADD,9,UI_Color_Cyan,4,1375,0,1147,352);
		
		Line_Draw(&G23,"113",UI_Graph_ADD,9,UI_Color_Cyan,4,0,1056,790,623);
		Line_Draw(&G24,"114",UI_Graph_ADD,9,UI_Color_Cyan,4,1920,1056,1147,623);
		
		
    Line_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Cyan,4,790,623,1147,623);
		
		Line_Draw(&G25,"115",UI_Graph_ADD,9,UI_Color_Cyan,4,790,352,790,623);
		Line_Draw(&G26,"116",UI_Graph_ADD,9,UI_Color_Cyan,4,1147,352,1147,623);
		
		
//空接定位UI
    Line_Draw(&G27,"117",UI_Graph_ADD,9,UI_Color_Purplish_red,6,864,273,894,986);
		Line_Draw(&G28,"118",UI_Graph_ADD,9,UI_Color_Purplish_red,6,1064,273,1028,986);

for(;;)
{
		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);                          //绘制图形	
		UI_ReFresh(7,G8,G9,G10,G11,G12,G13,G14); 
		UI_ReFresh(7,G15,G16,G17,G18,G19,G20,G21); 
		UI_ReFresh(7,G22,G23,G24,G25,G26,G27,G28);
		vTaskDelay(100);
}
//        Char_ReFresh(CH_SHOOT);
//		static double angle =0,heat=0,heat_door=0;//这里angle直接赋成差值
//		static int flag_cnn1=0,flag_cnn2=0,heat_flag1=0,heat_flag2=0;;//计时1//计时2//受击打标志位//受击打备用标志位;
//		Circle_Draw(&G8,"081",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
//		Circle_Draw(&G9,"082",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
//		Circle_Draw(&G10,"083",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
//		Circle_Draw(&G11,"084",UI_Graph_ADD,8,UI_Color_Green,3,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
//		UI_ReFresh(5,G7,G8,G9,G10,G11);//G7缺省   
//		Char_Draw(&CH_FLRB,"077",UI_Graph_ADD,7 ,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),&flrb_arr[0]);
//		Char_ReFresh(CH_FLRB);
//		Char_Draw(&CH_FLRB,"076",UI_Graph_ADD,7,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),&flrb_arr[1]);
//		Char_ReFresh(CH_FLRB);
//		Char_Draw(&CH_FLRB,"075",UI_Graph_ADD,7 ,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),&flrb_arr[2]);
//		Char_ReFresh(CH_FLRB);
//		Char_Draw(&CH_FLRB,"074",UI_Graph_ADD,7 ,UI_Color_Yellow,24,1,4,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),&flrb_arr[3]);
//		Char_ReFresh(CH_FLRB);
//  while(1)
//	{
//		//模拟装甲板受击判定||将装甲板ID对应angle，这段4个angle用旋转来模拟的装甲板移动与被击打！！！！！
//		if(angle==50) heat=1;
//		if(angle==80) heat=2;
//		if(angle==85) heat=2;
//		if(angle==90) heat=3;
//		if(heat) heat_door=heat;
//		if(heat_door)
//		{		
//			if(!heat_flag1){heat_flag1=heat;flag_cnn1=0;}//首次被击打
//			else if(heat_flag1==heat) {flag_cnn1=0;}//二次被击打在同一装甲板
//			else if(heat_flag1!=heat) 
//				{ 
//					if(!heat_flag2) {heat_flag2=heat;flag_cnn2=0;}
//					if(heat_flag2==heat) {flag_cnn2=0;}
//				  else if(heat){ if(flag_cnn1>=flag_cnn2){heat_flag1=heat;flag_cnn1=0;}else {heat_flag2=heat;flag_cnn2=0;}}
//				}//二次击打位置不在同一装甲板
//			if(flag_cnn1>=20) 
//				{
//					heat_flag1=0;flag_cnn1=0;
//				}
//			else if(heat_flag1) flag_cnn1+=1;//时间标志位1累加
//			if(flag_cnn2>=20) 
//				{
//					heat_flag2=0;flag_cnn2=0;
//				}
//			else if(heat_flag2) flag_cnn2+=1;//时间标志位2累加
//			heat=0;
//		}
//			//左右摩擦轮发射UI,初始化黄色，出错紫红色，正常绿色
//		if(Shoot_Right_Date.V&&Shoot_Right_Date.V)
//			Circle_Draw(&G7,"007",UI_Graph_Change,9,UI_Color_Green,15,230,770,15);
//		else Circle_Draw(&G7,"007",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,770,15);
//		//装甲板旋转移动，正常绿色，受击紫红色闪烁，两s反应时间
//		if(heat_flag1==1||heat_flag2==1)
//			Circle_Draw(&G8,"081",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
//		else Circle_Draw(&G8,"081",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
//		if(heat_flag1==2||heat_flag2==2) 
//			Circle_Draw(&G9,"082",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
//		else Circle_Draw(&G9,"082",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
//		if(heat_flag1==3||heat_flag2==3) 	
//			Circle_Draw(&G10,"083",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
//		else Circle_Draw(&G10,"083",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
//		if(heat_flag1==4||heat_flag2==4) 	
//		  Circle_Draw(&G11,"084",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
//		else Circle_Draw(&G11,"084",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
//		UI_ReFresh(5, G7, G8, G9, G10, G11);
//		Char_Draw(&CH_FLRB, "077", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle) * 2 * PI / 360.0), 540 + (int)340 * cos((angle) * 2 * PI / 360.0), &flrb_arr[0]);
//		Char_ReFresh(CH_FLRB);
//		Char_Draw(&CH_FLRB, "076", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 90) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 90) * 2 * PI / 360.0), &flrb_arr[1]);
//		Char_ReFresh(CH_FLRB);
//		Char_Draw(&CH_FLRB, "075", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 180) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 180) * 2 * PI / 360.0), &flrb_arr[2]);
//		Char_ReFresh(CH_FLRB);
//		Char_Draw(&CH_FLRB, "074", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 270) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 270) * 2 * PI / 360.0), &flrb_arr[3]);
//		Char_ReFresh(CH_FLRB);
		//频率控制10hz
//		

//	}		

}

/*
 *
 */
//Graph_Data G5, G8, G10, G11, G12, G21, G22, G23, G24, G25, G26, G40, G41, G42, G43, G44;
//Float_Data G6;
//fp32 yaw_angle, pitch_angle, x1_ui, y1_ui, x0_ui, y0_ui, sin_ui, cos_ui, speed, superpower_x, super_ratio;
//ui_t ui;
//uint32_t UiTaskStack;
//static void ui_data_update(ui_t *ui_init);
//extern u32 Robot_ID, Cilent_ID;
//extern u8 RFlag_state;
//extern chassis_move_t chassis_move;
//extern Gimbal_Control_t gimbal_control;//static
//static void ID_change(ui_t *ui_init);
//String_Data CH_FRICTION, CH_GIMBAL, CH_BUFF, CH_CHASSIC, CH_BULLET, SUPER;
//char		 	Friction[9] = "FRICTION:", Spin[5] = "SPIN:", Auto_aim[9] = "AUTO_AIM:", Shoot_buff[5] = "BUFF:", Magazine[9] = "MAGAZINE:";
//char 	 Super_cap[5] = {0};
//uint8_t VFlag_state;

//void ui_task(void  *pvParameters)
//{
//    //超级电容
//    memset(&G5, 0, sizeof(G5));
//    //底盘位置
//    memset(&G8, 0, sizeof(G8));
//    //底盘位置
//    memset(&G10, 0, sizeof(G10));
//    memset(&G11, 0, sizeof(G11));
//    memset(&G12, 0, sizeof(G12));
//    //准星
//    memset(&G21, 0, sizeof(G21));
//    memset(&G22, 0, sizeof(G22));
//    memset(&G23, 0, sizeof(G23));
//    memset(&G24, 0, sizeof(G24));
//    memset(&G25, 0, sizeof(G25));
//		memset(&G26, 0, sizeof(G26));
//    //各类姿态
//    memset(&CH_CHASSIC, 0, sizeof(CH_CHASSIC));
//    memset(&CH_GIMBAL, 0, sizeof(CH_GIMBAL));
//		memset(&CH_BUFF, 0, sizeof(CH_BUFF));
//    memset(&CH_FRICTION, 0, sizeof(CH_FRICTION));
//		memset(&CH_BULLET, 0, sizeof(CH_BULLET));
//    memset(&SUPER, 0, sizeof(SUPER));


//		Line_Draw(&G21, "075", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 900, 405, 1020, 405); //准星---->2m,3m
//    Line_Draw(&G22, "076", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 910, 390, 1010, 390); //准星----->1m
//		Line_Draw(&G23, "077", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 920, 375, 1000, 375); //准星---->4m
//		Line_Draw(&G24, "078", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 930, 345, 990, 345);//准星----->5m
//    Line_Draw(&G25, "079", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 940, 300, 980, 300); //准星---->6m
//		Line_Draw(&G26, "080", UI_Graph_ADD, 0, UI_Color_Yellow, 1, 960, 405, 960, 300); //准星竖线

//    Line_Draw(&G10, "081", UI_Graph_ADD, 0, UI_Color_White, 3, 620, 0, 770, 300); //底盘可过位置左标志位
//    Line_Draw(&G11, "082", UI_Graph_ADD, 0, UI_Color_White, 3, 1220, 0, 1060, 300); //底盘可过位置右标志位
//    Line_Draw(&G12, "083", UI_Graph_ADD, 0, UI_Color_White, 3, 800, 177, 1120, 177); //能量机关击打标志位

//		Line_Draw(&G40, "084", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 800, 460, 800);//BUFF
//		Line_Draw(&G41, "085", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 750, 460, 750);//底盘
//		Line_Draw(&G42, "086", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 700, 460, 700);//摩擦轮
//		Line_Draw(&G43, "087", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 650, 460, 650);//自瞄
//		Line_Draw(&G44, "088", UI_Graph_ADD, 0, UI_Color_White, 30, 430, 600, 460, 600);//弹舱

//    Line_Draw(&G8, "101", UI_Graph_ADD, 1, UI_Color_Green, 2, 960, 620, 960, 660); //底盘位置
//    Char_Draw(&CH_FRICTION, "102", UI_Graph_ADD, 1, UI_Color_Main, 20, 9, 1, 250, 710, &Friction[0]); //摩擦轮状态
//    Char_Draw(&CH_CHASSIC, "103", UI_Graph_ADD, 1, UI_Color_Main, 20, 5, 1, 330, 760, &Spin[0]); //底盘状态
//    Char_Draw(&CH_GIMBAL, "104", UI_Graph_ADD, 1, UI_Color_Main, 20, 9, 1, 250, 660, &Auto_aim[0]); //自瞄
//    Char_Draw(&CH_BULLET, "105", UI_Graph_ADD, 1, UI_Color_Main, 20, 9, 1, 250, 610, &Magazine[0]); //弹舱状态
//    Char_Draw(&SUPER, "106", UI_Graph_ADD, 1, UI_Color_Orange, 40, 5, 1, 1500, 830, &Super_cap[0]); //超级电容——百分比
//    Line_Draw(&G5, "107", UI_Graph_ADD, 1, UI_Color_Orange, 15, 700, 100, (Super_power.volt - 17000) / 10 + 700, 100); //超级电容——进度条
//		Char_Draw(&CH_BUFF, "109", UI_Graph_ADD, 1, UI_Color_Main, 20, 5, 1, 330, 810, &Shoot_buff[0]); //打符
//		
//    UI_ReFresh(5, G5, G8, G10, G11, G12);
//    UI_ReFresh(7, G21, G22, G23, G24, G25);
//		UI_ReFresh(1, G26);
//		UI_ReFresh(5, G40, G41, G42, G43, G44);
//		
//    Char_ReFresh(CH_GIMBAL);
//    Char_ReFresh(CH_FRICTION);
//    Char_ReFresh(CH_CHASSIC);
//		Char_ReFresh(CH_BUFF);
//    Char_ReFresh(CH_BULLET);
//    Char_ReFresh(SUPER);
//		
//    while(1)
//    {
//        ui_data_update(&ui);

//        //V键按下打开UI
//        if((KEY_PRESSED_OFFSET_V & ui.ui_remote_ctrl->key.v) != 0 )
//        {
//            V_Flag = 1;
//        }

//        if(KEY_PRESSED_OFFSET_V & ui.ui_remote_ctrl->key.v && V_Flag == 1)
//        {
//            V_Flag = 0;
//            VFlag_state ++;
//            VFlag_state %= 2;
//        }

//        if(VFlag_state)
//        {
//            ID_change(&ui);
//            /*-------------------------------------------------------UI推送---------------------------------------------------------------*/
//            UI_ReFresh(5, G5, G8, G10, G11, G12);
//						UI_ReFresh(7, G21, G22, G23, G24, G25);
//						UI_ReFresh(1, G26);
//						UI_ReFresh(5, G40, G41, G42, G43, G44);

//            Char_ReFresh(CH_GIMBAL);
//            Char_ReFresh(CH_FRICTION);
//            Char_ReFresh(CH_CHASSIC);
//						Char_ReFresh(CH_BUFF);
//            Char_ReFresh(CH_BULLET);
//            Char_ReFresh(SUPER);

//            /*-------------------------------------------------------底盘位置---------------------------------------------------------------*/
//            x0_ui = 960.0f + arm_sin_f32(yaw_angle) * 80.0f;
//            y0_ui = arm_cos_f32(yaw_angle) * 80.0f + 540.0f;
//            x1_ui = 960.0f + arm_sin_f32(yaw_angle) * 120.0f;
//            y1_ui = arm_cos_f32(yaw_angle) * 120.0f + 540.0f;
//            Line_Draw(&G8, "101", UI_Graph_Change, 1, UI_Color_Green, 3, x0_ui, y0_ui, x1_ui, y1_ui); //底盘位置
//            /*-------------------------------------------------------超级电容---------------------------------------------------------------*/
//            superpower_x = Super_power.volt;

//            if(superpower_x >= 22000.0f)
//            {
//                superpower_x = 22000.0f;
//            }
//            else if(superpower_x <= 17000.0f)
//            {
//                superpower_x = 17000.0f;
//            }

//            super_ratio = (superpower_x - 17000.0f) / 5000.0f * 100.0f;
//            sprintf(Super_cap, "%0.3f", super_ratio);
//            Char_Draw(&SUPER, "106", UI_Graph_Change, 1, UI_Color_Orange, 40, 5, 5, 1500, 830, &Super_cap[0]);
//            Line_Draw(&G5, "107", UI_Graph_Change, 1, UI_Color_Orange, 15, 700, 100, (superpower_x - 17000) / 10 + 700, 100);

//            /*-------------------------------------------------------底盘模式---------------------------------------------------------------*/
//            if(chassis_move.chassis_mode == CHASSIS_FOLLOW_GIMBAL)
//            {
//                //底盘跟随模式
//                memset(&CH_CHASSIC, 0, sizeof(CH_CHASSIC));
//                Line_Draw(&G41, "085", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 750, 460, 750);
//            }
//            if(chassis_move.chassis_mode == CHASSIS_DODGE_MODE)
//            {
//                //底盘闪避模式
//                memset(&CH_CHASSIC, 0, sizeof(CH_CHASSIC));
//                Line_Draw(&G41, "085", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 750, 460, 750);
//            }

//            /*-------------------------------------------------------云台模式---------------------------------------------------------------*/
//            if(gimbal_control.gimbal_behaviour == GIMBAL_SHOOT_BUFF)
//            {
//                //云台击打能量机关模式
//                memset(&CH_BUFF, 0, sizeof(CH_BUFF));
//                Line_Draw(&G40, "084", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 800, 460, 800);
//            }
//            else if(gimbal_control.gimbal_behaviour == GIMBAL_TRACK_ARMOR)
//            {
//                //云台追踪装甲板模式
//                memset(&CH_GIMBAL, 0, sizeof(CH_GIMBAL));
//                Line_Draw(&G43, "087", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 650, 460, 650);
//            }
//            else if(gimbal_control.gimbal_behaviour == GIMBAL_MANUAL_MODE)
//            {
//                //云台手控模式
//								memset(&CH_BUFF, 0, sizeof(CH_BUFF));
//                memset(&CH_GIMBAL, 0, sizeof(CH_GIMBAL));
//								Line_Draw(&G40, "084", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 800, 460, 800);
//								Line_Draw(&G43, "087", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 650, 460, 650);
//            }

//            /*-------------------------------------------------------摩擦轮---------------------------------------------------------------*/
//            if(speed > 2000)
//            {
//                //打开摩擦轮
//                memset(&CH_FRICTION, 0, sizeof(CH_FRICTION));
//                Line_Draw(&G42, "086", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 700, 460, 700);
//            }
//            else
//            {
//                //关闭摩擦轮
//                memset(&CH_FRICTION, 0, sizeof(CH_FRICTION));
//								Line_Draw(&G42, "086", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 700, 460, 700);
//            }

//            /*-------------------------------------------------------弹舱---------------------------------------------------------------*/
//            if(RFlag_state == 0)
//            {
//                //弹舱关
//                memset(&CH_BULLET, 0, sizeof(CH_BULLET));
//                Line_Draw(&G44, "088", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 430, 600, 460, 600);
//            }
//            else if(RFlag_state == 1)
//            {
//                //弹舱开
//                memset(&CH_BULLET, 0, sizeof(CH_BULLET));
//                Line_Draw(&G44, "088", UI_Graph_Change, 0, UI_Color_Green, 30, 430, 600, 460, 600);
//            }
//        }

//        vTaskDelay(100);
//        UiTaskStack = uxTaskGetStackHighWaterMark(NULL);
//    }
//}

//static void ui_data_update(ui_t *ui_init)
//{
// 
//    ui_init->ui_remote_ctrl = get_remote_control_point();   //获取遥控器指针
//    ui_init->shoot_hurt_point = get_robot_hurt_t(); //获取裁判系统指针
//    ui_init->ui_yaw_motor = get_yaw_motor_point();//获取yaw轴数据
//    yaw_angle = ui_init->ui_yaw_motor->relative_angle * UI_ANGLE_TO_RAD;

//    for(uint8_t i = 0; i < 2; i++)
//    {
//        ui_init->friction_motor_measure[i] = get_Friction_Motor_Measure_Point(i);
//    }
//    speed = ui_init->friction_motor_measure[0]->speed_rpm;
//    
//    ui_init->ui_pitch_motor = get_pitch_motor_point();//获取pitch轴数据
//    pitch_angle = ui_init->ui_pitch_motor->relative_angle;
//}

//static void ID_change(ui_t *ui_init)
//{
//    ui_init->chassis_status_measure = get_game_robot_state_t();

//    if(ui_init->chassis_status_measure->robot_id == 3)
//    {
//        Robot_ID = 3;
//        Cilent_ID = 0x0103;
//    }

//    if(ui_init->chassis_status_measure->robot_id == 4)
//    {
//        Robot_ID = 4;
//        Cilent_ID = 0x0104;
//    }

//    if(ui_init->chassis_status_measure->robot_id == 5)
//    {
//        Robot_ID = 5;
//        Cilent_ID = 0x0105;
//    }

//    if(ui_init->chassis_status_measure->robot_id == 103)
//    {
//        Robot_ID = 103;
//        Cilent_ID = 0x0167;
//    }

//    if(ui_init->chassis_status_measure->robot_id == 104)
//    {
//        Robot_ID = 104;
//        Cilent_ID = 0x0168;
//    }

//    if(ui_init->chassis_status_measure->robot_id == 105)
//    {
//        Robot_ID = 105;
//        Cilent_ID = 0x0169;
//    }
//}






