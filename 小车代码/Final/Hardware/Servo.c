#include "Servo.h"

/* 50Hz PWM��20ms ���ڣ�1us �ֱ���
 * TIM9 �� APB2��ʱ�� 168MHz / 168 = 1MHz��ARR = 20000 -> 20ms */
#define SERVO_PSC         (168 - 1)
#define SERVO_ARR         (20000 - 1)

#define SERVO_PULSE_MIN   500    /* 0.5ms = 0�� */
#define SERVO_PULSE_MAX   2500   /* 2.5ms = 180�� */

static void TIM9_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);   /* TIM9 �� APB2 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /* PE5 = TIM9_CH1, PE6 = TIM9_CH2 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);   /* PE5 -> CH1 */
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);   /* PE6 -> CH2 */

    TIM_TimeBaseStructure.TIM_Period            = SERVO_ARR;
    TIM_TimeBaseStructure.TIM_Prescaler         = SERVO_PSC;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = 1500;    /* ��ʼ 90�� ��λ */

    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
    /* ͨ�ö�ʱ��������Ҫ BDTR������Ҫ TIM_CtrlPWMOutputs */
}

void Servo_Init(void)
{
    TIM9_PWM_Init();
    //Servo_SetAngle(SERVO_CH1, 81);
	Servo_SetAngle(SERVO_CH1, 170);
    Servo_SetAngle(SERVO_CH2, 150);
	//Servo_SetAngle(SERVO_CH2, 35);
}

void Servo_SetPulse(uint8_t ch, uint16_t us)
{
    if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
    if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;

    if      (ch == SERVO_CH1) TIM_SetCompare1(TIM9, us);
    else if (ch == SERVO_CH2) TIM_SetCompare2(TIM9, us);
}

void Servo_SetAngle(uint8_t ch, float angle)
{
    float us;

    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    us = 500.0f + (angle / 180.0f) * 2000.0f;
    Servo_SetPulse(ch, (uint16_t)us);
}

/* ---------------- Ӧ�ò��װ ---------------- */
void Servo_Grab(void)
{
    Servo_SetAngle(SERVO_GRIPPER, GRIPPER_CLOSE_DEG);
}

void Servo_Open(void)
{
    Servo_SetAngle(SERVO_GRIPPER, GRIPPER_OPEN_DEG);
}

void Servo_Drop(void)
{
    Servo_Open();
}

void Servo_SetCam(uint8_t mode)     /* 0=̧ͷ 1=��ͷ 2=����(55��) */
{
    switch (mode)
    {
        case 0:  Servo_SetAngle(SERVO_CAMERA, CAM_UP_DEG);   break;
        case 1:  Servo_SetAngle(SERVO_CAMERA, CAM_DOWN_DEG); break;
        case 2:  Servo_SetAngle(SERVO_CAMERA, CAM_NEAR_DEG); break;
        default: Servo_SetAngle(SERVO_CAMERA, CAM_UP_DEG);   break;  /* �Ƿ�ֵ �� ̧ͷ����ȫ�� */
    }
}
