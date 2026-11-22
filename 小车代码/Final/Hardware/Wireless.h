#ifndef __WIRELESS_H
#define __WIRELESS_H

#include <stdio.h>
#include "stm32f4xx_conf.h"
#include "sys.h"

void Wireless_Init(void);
void Wireless_SendByte(uint8_t Byte);
void Wireless_SendArray(uint8_t *Array, uint16_t Length);
void Wireless_SendString(char *String);
void Wireless_SendNumber(uint32_t Number, uint8_t Length);
void Wireless_Printf(char *format, ...);

uint8_t Wireless_GetRxFlag(void);
uint8_t Wireless_GetRxData(void);

#endif

