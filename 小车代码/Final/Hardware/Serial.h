#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define SERIAL_RX_BUF_SIZE   256

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);
/* ===== 新增：环形缓冲接口 ===== */
void     Serial_ClearRxBuffer(void);
uint16_t Serial_Available(void);            /* 缓冲区中可读字节数 */
uint8_t  Serial_ReadByte(uint8_t *out);     /* 读 1 字节，返回 1=成功 0=空 */

/* 缓冲区溢出计数（调试用，正常应恒为 0） */
extern volatile uint32_t g_SerialRxOverflow;

#endif

