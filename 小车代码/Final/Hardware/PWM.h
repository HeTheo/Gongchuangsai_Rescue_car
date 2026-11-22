#ifndef _PWM_H
#define _PWM_H
#include "sys.h"

void TIM1_PWM_Init(void);
void TIM1_SetPWM1(float percent);
void TIM1_SetPWM4(float percent);

#endif
