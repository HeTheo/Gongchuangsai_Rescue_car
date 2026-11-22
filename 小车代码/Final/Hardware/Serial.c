#include "sys.h"
#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

static volatile uint8_t  s_RxBuf[SERIAL_RX_BUF_SIZE];
static volatile uint16_t s_RxHead = 0;      /* 只由中断写 */
static volatile uint16_t s_RxTail = 0;      /* 只由主循环写 */
volatile uint32_t g_SerialRxOverflow = 0;

void Serial_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);    // F4: GPIOA 挂在 AHB1 总线
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);   // USART1 挂在 APB2 总线

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             // F4: 复用功能模式(不是 F1 的 GPIO_Mode_AF_PP)
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           // 推挽复用输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             // 上拉

    // 串口1 TX 口为 PA9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // PA9 复用为 USART1_TX

    // 串口1 RX 口为 PA10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // PA10 复用为 USART1_RX

    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // 发送/接收模式
    USART_InitStructure.USART_Parity = USART_Parity_No;				//无硬件数据流
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

	USART_ClearFlag(USART1, USART_FLAG_TC);						//清残留发送标志位
    USART_Cmd(USART1, ENABLE);									//使能串口

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);            	// 开启串口1接收中断

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//抢占优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//子优先级为1
    NVIC_Init(&NVIC_InitStructure);

}

void Serial_SendByte(uint8_t Byte)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, Byte);
}

/****************************************************************************** 
 * 向串口1发送字节包 
******************************************************************************/ 
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial_SendByte(Array[i]);
    }
}
/****************************************************************************** 
 * 向串口1发送字符串 
******************************************************************************/ 
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)
    {
        Serial_SendByte(String[i]);
    }
}

/****************************************************************************** 
 * 将字符串的内容转成数字
******************************************************************************/ 
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}

/****************************************************************************** 
 * 向串口1发送数字
******************************************************************************/ 
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

/****************************************************************************** 
 * 串口1打印内容，调试用（不是发送！！！）
******************************************************************************/ 
void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

uint16_t Serial_Available(void)
{
    uint16_t head = s_RxHead;               /* 先取一次，防中断中途改变 */
    uint16_t tail = s_RxTail;
    return (uint16_t)((head + SERIAL_RX_BUF_SIZE - tail) % SERIAL_RX_BUF_SIZE);
}

uint8_t Serial_ReadByte(uint8_t *out)
{
    if (s_RxTail == s_RxHead) return 0;
    *out     = s_RxBuf[s_RxTail];
    s_RxTail = (uint16_t)((s_RxTail + 1) % SERIAL_RX_BUF_SIZE);
    return 1;
}

void Serial_ClearRxBuffer(void)
{
    s_RxTail = s_RxHead;
}

/* ---------- 中断：只做一件事，把字节塞进环形缓冲 ---------- */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t  b   = (uint8_t)USART_ReceiveData(USART1);
        uint16_t next = (uint16_t)((s_RxHead + 1) % SERIAL_RX_BUF_SIZE);

        if (next == s_RxTail)
        {
            /* 缓冲区满：丢弃最老的字节（保住最新数据），并计数 */
            s_RxTail = (uint16_t)((s_RxTail + 1) % SERIAL_RX_BUF_SIZE);
            g_SerialRxOverflow++;
        }
        s_RxBuf[s_RxHead] = b;
        s_RxHead = next;

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

