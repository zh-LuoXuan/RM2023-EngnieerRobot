#ifndef UI_TASK_H
#define UI_TASK_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "start_task.h"

#include "cmsis_os.h"
#include "stdlib.h"
#include "judgement_info.h"
#include "protocol.h"
#include "CAN_Receive.h"
//#include "BSP_USART_Init.h"

#define UI_ANGLE_TO_RAD   0.01745329251994329576923690768489f

typedef struct
{
    const ext_robot_hurt_t		  *shoot_hurt_point;
    const RC_ctrl_t *ui_remote_ctrl;

} ui_t;

void task_user_UI_Create(void);

#endif

