#include "Wireless.h"
#include "sys.h"
#include <stdio.h>
#include <stdarg.h>

uint8_t Wireless_RxData;
uint8_t Wireless_RxFlag;

void Wireless_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    /* USART3_TX: PB10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

    /* USART3_RX: PB11 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    USART_Init(USART3, &USART_InitStructure);

    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_Cmd(USART3, ENABLE);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

void Wireless_SendByte(uint8_t Byte)
{
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);

    USART_SendData(USART3, Byte);
}

void Wireless_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        Wireless_SendByte(Array[i]);
    }
}

void Wireless_SendString(char *String)
{
    uint8_t i;

    for (i = 0; String[i] != '\0'; i++)
    {
        Wireless_SendByte(String[i]);
    }
}

static uint32_t Wireless_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;

    while (Y--)
    {
        Result *= X;
    }

    return Result;
}

void Wireless_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;

    for (i = 0; i < Length; i++)
    {
        Wireless_SendByte(
            Number / Wireless_Pow(10, Length - i - 1) % 10 + '0'
        );
    }
}

void Wireless_Printf(char *format, ...)
{
    char String[100];
    va_list arg;

    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);

    Wireless_SendString(String);
}

uint8_t Wireless_GetRxFlag(void)
{
    if (Wireless_RxFlag == 1)
    {
        Wireless_RxFlag = 0;
        return 1;
    }

    return 0;
}

uint8_t Wireless_GetRxData(void)
{
    return Wireless_RxData;
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        Wireless_RxData = USART_ReceiveData(USART3);
        Wireless_RxFlag = 1;

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

