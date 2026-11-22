#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"

extern volatile uint32_t g_MsTick;

void TIM5_Init(void);

/* 全局毫秒时基，由 TIM5 的 1ms 中断累加 */
uint32_t Tick_Get(void);                    /* 取当前毫秒计数 */
uint32_t Tick_Elapsed(uint32_t from);       /* 从 from 到现在过了多少毫秒 */

#endif
