#ifndef __FUNCTION_TASH_H
#define __FUNCTION_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "motor_control_task.h"

/*======================================================================*/
#define    TIME_STAMP_1MS        1
#define    TIME_STAMP_2MS        2
#define    TIME_STAMP_4MS        4
#define    TIME_STAMP_10MS      10
#define    TIME_STAMP_20MS      20
#define    TIME_STAMP_30MS      30
#define    TIME_STAMP_40MS      40
#define    TIME_STAMP_50MS      50
#define    TIME_STAMP_60MS      60
#define    TIME_STAMP_80MS      80
#define    TIME_STAMP_100MS    100
#define    TIME_STAMP_150MS    150
#define    TIME_STAMP_200MS    200
#define    TIME_STAMP_250MS    250
#define    TIME_STAMP_300MS    300
#define    TIME_STAMP_400MS    400
#define    TIME_STAMP_500MS    500
#define    TIME_STAMP_750MS    750
#define    TIME_STAMP_1000MS  1000
#define    TIME_STAMP_1500MS  1500
#define    TIME_STAMP_2000MS  2000
#define    TIME_STAMP_2500MS	2500
#define    TIME_STAMP_3000MS	3000
#define    TIME_STAMP_10S     10000
/*======================================================================*/
#define UP_MOVE_COUNT	6
#define	CLAMP_TURN_COUNT	4
/*======================================================================*/
#define Level_Turn_Longs      (180.0f * REDUCTION_RATIO_3508)
#define floor_Turn_Longs      (90.0f * REDUCTION_RATIO_3508)
#define EXCHANG_GET_Longs			(30.0f * REDUCTION_RATIO_3508)
#define Back_Longs_Min				(0.0f)
/*======================================================================*/
#define L_UP_Longs_HIGHTST		(0.0f)
#define L_Up_Longs_Exchang	  (644.0f * REDUCTION_RATIO_3508)
#define	L_Up_Longs_SILVER			(563.0f * REDUCTION_RATIO_3508)
#define L_Up_Longs_PICK_FLR		(450.0f * REDUCTION_RATIO_3508)
#define L_Up_Longs_PUT				(420.0f * REDUCTION_RATIO_3508)
#define L_Up_Longs_LOWWST	 	  (528.0f * REDUCTION_RATIO_3508)
/*======================================================================*/
#define R_UP_Longs_HIGHTST		(0.0f)
#define R_Up_Longs_Exchang		(644.0f * REDUCTION_RATIO_3508)
#define	R_Up_Longs_SILVER			(563.0f * REDUCTION_RATIO_3508)
#define R_Up_Longs_PICK_FLR	  (450.0f * REDUCTION_RATIO_3508)
#define R_Up_Longs_PUT				(420.0f * REDUCTION_RATIO_3508)
#define R_Up_Longs_LOWWST	 	  (528.0f * REDUCTION_RATIO_3508)
/*======================================================================*/
#define Level_Turn_R_Longs    (180.0f * REDUCTION_RATIO_2006)
#define floor_Turn_R_Longs    (0.0f)
#define EXCHANG_R_GET_Longs		(180.0f * REDUCTION_RATIO_2006)
#define Back_R_Longs_Min			(180.0f * REDUCTION_RATIO_2006)

typedef enum
{
	LEVEL = 0,
	FLOOR,
	EXGET,
	BACK	
} eclamp_turn_appoint;
typedef enum
{
  Air_Connection=1,
	Gold_Mine,
	Silver_Ore,
  UpExchange,
	DOwnEXchange,
  FloorMIne
}MODEFLAG;
typedef enum
{
	HIGHTST = 0,
	EXCHANG,
	SILVER,
	PICK_FLR,
	PUT,
	LOWWST	
} emotor_appoint;

void task_function_Create(void);
//void function_mode_set(Motor_Control_t *ModeSet);
void Reomte_Mode_Set(void);
#endif

