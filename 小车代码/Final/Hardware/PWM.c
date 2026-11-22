#include "PWM.h"

#define PWM1_ARR  (1000 - 1)  // 自动重装载值ARR
#define PWM1_PSC  (10 - 1)   	// 定时器分频PSC

// PWM初始化，ARR 1000-1，PSC 10-1， PWM频率16.8KHz
void TIM1_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef  TIM_BDTRInitStructure;      // 高级定时器专用：刹车/死区寄存器

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);   // TIM1 在 APB2 总线，168MHz主频
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // GPIOA 在 AHB1 总线

    // PA8 → TIM1_CH1 (AF1)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA11 → TIM1_CH4 (AF1)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);   // 其余参数沿用上面的 PP+上拉

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseStructure.TIM_Period = PWM1_ARR;                
	TIM_TimeBaseStructure.TIM_Prescaler = PWM1_PSC;               
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//一分频
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;           	// 高级定时器特有的重复计数器
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // PWM 模式配置（通道1 和 通道4 共用同一组参数）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  // 高电平有效，占空比=CCR/ARR
    TIM_OCInitStructure.TIM_Pulse = 0;                         // 初始占空比为 0，防止上电电机就转

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 高级定时器必须配置 BDTR（刹车与死区寄存器）
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = 0;                    // 不用互补输出，死区设 0
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;       // 不使用刹车输入
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);   // 高级定时器必须使能主输出 MOE，否则无 PWM
}

// 输入 0~100 百分比，内部自动换算成 CCR
void TIM1_SetPWM1(float percent)
{
    if (percent > 100) percent = 100;
    TIM_SetCompare1(TIM1, (u32)(percent * PWM1_ARR / 100));
	// 4.78 * 999 / 100 = 47.75 → (u32)47
}

// 输入 0~100 百分比，内部自动换算成 CCR
void TIM1_SetPWM4(float percent)
{
    if (percent > 100) percent = 100;
    TIM_SetCompare4(TIM1, (u32)(percent * PWM1_ARR / 100));
}


//// 设置通道1（PA8）占空比，duty 范围 0 ~ ARR
//void TIM1_SetPWM1(u16 duty)
//{
//    TIM_SetCompare1(TIM1, duty);
//}

//// 设置通道4（PA11）占空比，duty 范围 0 ~ ARR
//void TIM1_SetPWM4(u16 duty)
//{
//    TIM_SetCompare4(TIM1, duty);
//}

