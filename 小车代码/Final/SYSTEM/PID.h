#ifndef __PID_H
#define __PID_H

#include "sys.h"

typedef struct
{
    float Kp, Ki, Kd;      // 三个增益
    float Target;          // 目标值
    float LastError;       // 上一次误差
    float Integral;        // 积分累加
    float Int_Limit;   // 积分限幅（防饱和）
    float OutMin, OutMax;  // 输出限幅
} PID_TypeDef;

void  PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
               float outMin, float outMax, float integralLimit);
void  PID_SetTarget(PID_TypeDef *pid, float target);
float PID_Calc(PID_TypeDef *pid, float measure);   // 位置式 PID
static float AngleDiff(float target, float current);
float AnglePID_Calc(PID_TypeDef *pid, float target, float measure);

#endif




