#ifndef __GYRO_H
#define __GYRO_H

#include "sys.h"

#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口2接收

struct SAngle
{
	float Yaw;
};
extern struct SAngle stcAngle;

struct SGyro
{
    short rawWz;
    float wz;
};
extern struct SGyro stcGyro;


void Gyro_Init(void);
void CopeSerial2Data(unsigned char ucData);
void USART2_SendByte(uint8_t byte); 

void sendCaliYawCommand(void) ;
void performCaliBias(void) ;
float Yaw(void);
float GyroZ(void);

#endif
