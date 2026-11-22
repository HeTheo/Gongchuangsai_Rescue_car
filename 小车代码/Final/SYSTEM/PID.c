#include "pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float outMin, float outMax, float Int_Limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Target = 0;
    pid->LastError = 0;
    pid->Integral = 0;
    pid->Int_Limit = Int_Limit;
    pid->OutMin = outMin;
    pid->OutMax = outMax;
}

void PID_SetTarget(PID_TypeDef *pid, float Target)
{
    pid->Target = Target;
}

// 内环速度环PID
// 位置式：out = Kp*e + Ki*∫e + Kd*de/dt
float PID_Calc(PID_TypeDef *pid, float measure)
{
    float error = pid->Target - measure;
	
	// 积分项（积分限幅）
    pid->Integral += error;
    if (pid->Integral >  pid->Int_Limit) pid->Integral =  pid->Int_Limit;
    if (pid->Integral < -pid->Int_Limit) pid->Integral = -pid->Int_Limit;

	// 微分项
    float derivative = error - pid->LastError;
    pid->LastError = error;

    float out = pid->Kp * error
              + pid->Ki * pid->Integral
              + pid->Kd * derivative;
	
	// 输出限幅 
    if (out >  pid->OutMax) out =  pid->OutMax;
    if (out <  pid->OutMin) out =  pid->OutMin;
    return out;
}

// 角度差归一化到 -180 ~ 180
static float AngleDiff(float target, float current)
{
    float diff = target - current;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// 角度环 PID（带 ±180° 归一化，仿照 PID_Calc）
float AnglePID_Calc(PID_TypeDef *pid, float target, float measure)
{
    float error = AngleDiff(target, measure);

    pid->Integral += error;
    if (pid->Integral >  pid->Int_Limit) pid->Integral =  pid->Int_Limit;
    if (pid->Integral < -pid->Int_Limit) pid->Integral = -pid->Int_Limit;

    float derivative = error - pid->LastError;
    pid->LastError = error;

    float out = pid->Kp * error + pid->Ki * pid->Integral + pid->Kd * derivative;
    if (out >  pid->OutMax) out =  pid->OutMax;
    if (out <  pid->OutMin) out =  pid->OutMin;
    return out;
}



