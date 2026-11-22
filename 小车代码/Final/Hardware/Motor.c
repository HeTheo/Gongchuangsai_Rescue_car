#include "Motor.h"
#include "PWM.h"

// 方向引脚初始化（PC0~PC3，推挽输出）
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   // GPIOC 在 AHB1 总线

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           // 普通推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        // 输出引脚，上下拉无所谓
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 初始全部输出低电平，电机停止
    GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
	
	TIM1_PWM_Init();     // 初始化 PWM（PA8/PA11）
}

// 左电机设置PWM百分比速度
void Motor_Left_SetPWM(float PWM)
{	
	if (PWM >= 0)						//如果设置正转的PWM
	{
		AIN1(0);						//PC0置低电平
		AIN2(1);						//PC1置高电平
		TIM1_SetPWM1(PWM);				//设置PWM占空比
	}
	else								//否则，即设置反转的速度值
	{
		AIN1(1);						//PC0置高电平
		AIN2(0);						//PC1置低电平
		TIM1_SetPWM1(-PWM);				//设置PWM占空比
	}
}

// 右电机设置PWM百分比速度
void Motor_Right_SetPWM(float PWM)
{
	if (PWM >= 0)						//如果设置正转的PWM
	{
		BIN1(0);						//PC2置低电平
		BIN2(1);						//PC3置高电平
		TIM1_SetPWM4(PWM);				//设置PWM占空比
	}
	else								//否则，即设置反转的速度值
	{
		BIN1(1);						//PC2置高电平
		BIN2(0);						//PC3置低电平
		TIM1_SetPWM4(-PWM);				//设置PWM占空比
	}
}
