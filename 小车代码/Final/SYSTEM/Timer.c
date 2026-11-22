#include "timer.h"
#include "led.h"

volatile uint32_t g_MsTick = 0;	//时基，1ms加一次，判断时间用

// 定时器4初始化，1ms 触发一次中断
void TIM5_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE); //使能TIM5时钟, APB1主线上，84MHz主频
	
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1; 	//自动重装载值 ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=84 - 1;  	//定时器分频 PSC
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);	//初始化TIM5
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器5更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器5
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/* =====================================================================
 * 全局毫秒时基
 * ---------------------------------------------------------------------
 * TIM5 每 1ms 中断一次，在中断里把 g_MsTick 加 1（见 main.c 的
 * TIM5_IRQHandler）。这样任何地方都能拿到一个单调递增的毫秒数，
 * 用来做超时判断、动作计时、ack 重发计时。
 *
 * 为什么不用 SysTick：SysTick 被 delay.c 的延时函数占着，
 * TIM5 本来就 1ms 跑着，顺手加一下最省事。
 * ===================================================================== */


uint32_t Tick_Get(void)
{
    return g_MsTick;
}

/* 用无符号减法算时间差，天然处理 32 位回绕
 * （g_MsTick 要 49.7 天才绕一圈，实际跑不到那时候，
 *   但这个写法比 if (now > from) ... 更可靠也更简单） */
uint32_t Tick_Elapsed(uint32_t from)
{
    return (uint32_t)(g_MsTick - from);
}

//定时器5中断服务函数
//void TIM5_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
//	{
//		
//	}
//	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
//}
