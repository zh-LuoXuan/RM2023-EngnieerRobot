#ifndef __CLAMP_Y_P_R_TASH_H
#define __CLAMP_Y_P_R_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"
#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"
#include "motor_control_task.h"

#define CHANNEL_PRESS  (0.005f)
#define CHANNEL_PRESS_Up  (0.025f)

#define CLAMP_YAW_OFFSET_MIN  (45.0f * REDUCTION_RATIO_2006)
#define CLAMP_YAW_OFFSET_MAX  (45.0f * REDUCTION_RATIO_2006)

#define CLAMP_PITCH_OFFSET_MIN  (0.0f * REDUCTION_RATIO_3508)
#define CLAMP_PITCH_OFFSET_MAX  (200.0f * REDUCTION_RATIO_3508)

#define CLAMP_ROLL_OFFSET_MIN  (60.0f * REDUCTION_RATIO_2006)
#define CLAMP_ROLL_OFFSET_MAX  (60.0f * REDUCTION_RATIO_2006)

#define UP_LEFT_OFFSET_MIN  (10129.2255f * REDUCTION_RATIO_3508)
#define UP_LEFT_OFFSET_MAX  (0.0f * REDUCTION_RATIO_3508)

#define UP_RIGHT_OFFSET_MIN  (10027.0839f * REDUCTION_RATIO_3508)
#define UP_RIGHT_OFFSET_MAX  (0.0f * REDUCTION_RATIO_3508)

void task_Clamp_Y_P_R_Crate(void);

#endif

