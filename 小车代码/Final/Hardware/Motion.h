#ifndef __MOTION_H
#define __MOTION_H

#include "sys.h"

typedef enum
{
    MOTION_STOP = 0,        /* 停 */
    MOTION_STRAIGHT,        /* 直行 */
    MOTION_BACK,            /* 后退 */
    MOTION_TURN,            /* 原地转：目标角度 0 = 一直转；90 = 转 90° 后自动停 */
    MOTION_VISUAL_TRACK     /* 视觉追踪：tx/ty 外环 → 左右轮差速 */
} Motion_Mode_t;

extern float left, right, leftOut, rightOut;
extern float Turn_Angle;    /* 累计转角（度），左转为正 */

void Motion_Init(void);
void Motion_SetMode(Motion_Mode_t mode);
Motion_Mode_t Motion_GetMode(void);

void Motion_SetStraightSpeed(float speed);      /* counts/20ms */
void  Motion_SetFastGear(uint8_t on);   /* 1 = 高速直行档, 0 = 常规 */
float Motion_GetSpeedNow(void);         /* 当前直行基础速度 counts/20ms */
void Motion_SetTurnSpeed(float speed, int8_t dir);
void Motion_SetBackSpeed(float speed);
void Motion_SetTrackTarget(uint16_t tx, uint16_t ty);   /* 视觉追踪目标（像素） */
void Motion_SetTurnAngle(float deg);                    /* 转圈目标角：0=一直转，>0 转到自动停 */

void Motion_Control(void);      /* 20ms 周期，在 TIM5 中断里调 */

#endif


