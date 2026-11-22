#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

/* 方向引脚：PC0~PC3 */
#define AIN1_PORT   GPIOC
#define AIN1_PIN    GPIO_Pin_0

#define AIN2_PORT   GPIOC
#define AIN2_PIN    GPIO_Pin_1

#define BIN1_PORT   GPIOC
#define BIN1_PIN    GPIO_Pin_2

#define BIN2_PORT   GPIOC
#define BIN2_PIN    GPIO_Pin_3

/* 方向引脚电平控制：1=高，0=低 */
#define AIN1(x)  GPIO_WriteBit(AIN1_PORT, AIN1_PIN, (BitAction)(x))
#define AIN2(x)  GPIO_WriteBit(AIN2_PORT, AIN2_PIN, (BitAction)(x))

#define BIN1(x)  GPIO_WriteBit(BIN1_PORT, BIN1_PIN, (BitAction)(x))
#define BIN2(x)  GPIO_WriteBit(BIN2_PORT, BIN2_PIN, (BitAction)(x))

void Motor_Init(void);
void Motor_Left_SetPWM(float PWM);
void Motor_Right_SetPWM(float PWM);


#endif

