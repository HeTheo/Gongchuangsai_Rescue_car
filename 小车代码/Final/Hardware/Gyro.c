#include "sys.h"
#include "Gyro.h"  
#include <string.h>
#include "delay.h"

struct SAngle stcAngle;
struct SGyro stcGyro;

void Gyro_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   // F4: GPIOA 时钟在 AHB1 总线
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // USART2 时钟仍在 APB1 总线（不变）
    
    // PA2 复用为 USART2_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // F4: 复用模式(不是 F1 的 AF_PP)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          // 推挽
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);   // AF7 → USART2_TX
    
    // PA3 复用为 USART2_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // F4: RX 也要用复用模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // OType/PuPd 沿用上面的 PP+上拉
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);   // AF7 → USART2_RX
    
    USART_InitStructure.USART_BaudRate = 115200;             // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    
    USART_Cmd(USART2, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC);
    
#if EN_USART2_RX	
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);          // 开启接收中断

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}


/****************************************************************************** 
 * 向串口2发送一个字节 
******************************************************************************/ 
void USART2_SendByte(uint8_t byte) 
{ 
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, byte); 
}


/****************************************************************************** 
 * 向串口2发送字节包 
******************************************************************************/ 
void SART2_SendBytes(uint8_t* data, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        USART2_SendByte(data[i]);   
    }
}


void CopeSerial2Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[11];
    static unsigned char ucRxCnt = 0;

    ucRxBuffer[ucRxCnt++] = ucData;

    if (ucRxBuffer[0] != 0x5A)
    {
        ucRxCnt = 0;
        return;
    }

    if (ucRxCnt < 5) return;

    unsigned char sum = 0;
    if (ucRxBuffer[1] == 0xAA)
    {
        // 角速度帧校验和：0x5A + 0xAA + AzL + AzH 
        sum = ucRxBuffer[0] + ucRxBuffer[1] +
              ucRxBuffer[2] + ucRxBuffer[3] ;

        if (sum != ucRxBuffer[4])
        {
            ucRxCnt = 0;
            return;
        }

        short wz    = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);

        stcGyro.wz    = (float)wz    / 32768.0f * 2000.0f;
    }
    else if (ucRxBuffer[1] == 0xBB)
    {
        // 角度帧校验和：0x5A + 0xBB + YawH + YawL 
        sum = ucRxBuffer[0] + ucRxBuffer[1] +
              ucRxBuffer[2] + ucRxBuffer[3];

        if (sum != ucRxBuffer[4])
        {
            ucRxCnt = 0;
            return;
        }

        short rawYaw = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
        stcAngle.Yaw = (float)rawYaw / 32768.0f * 180.0f;
    }
    ucRxCnt = 0;
}

/******************************************************************************
 * 返回Z轴角速度（单位：°/s）
******************************************************************************/
float GyroZ(void)
{
    return stcGyro.wz;
}

/******************************************************************************
 * 返回当前Yaw角（Z轴角度），单位°，范围 -180 ~ 180
******************************************************************************/
float Yaw(void)
{
   return stcAngle.Yaw;
}


//解锁指令
uint8_t Key[5] = {0x55, 0xAA, 0x13, 0x8E, 0x5F};
//Z轴角度归零指令
uint8_t Yaw_Zero[5] = {0x55, 0xAA, 0x15, 0x00, 0x00};
//保存指令
uint8_t Save[5] = {0x55, 0xAA, 0x00, 0x00, 0x00};
//获取零偏指令
uint8_t BIAS_CAL[5] = {0x55, 0xAA, 0x0A, 0x01, 0x00};


/****************************************************************************** 
 * 发送 Z轴角度归零命令
******************************************************************************/ 
void sendCaliYawCommand(void) 
{ 
   SART2_SendBytes(Key, 5);
	 delay_ms(100);
	 SART2_SendBytes(Yaw_Zero, 5);
	 delay_ms(100);
	 SART2_SendBytes(Save, 5);
}

/****************************************************************************** 
* 发送校准指令(校准过程中请勿移动，否则会校准失败或者校准效果不好)
******************************************************************************/ 
void performCaliBias(void) 
{ 
   SART2_SendBytes(Key, 5);
	 delay_ms(100);
	 SART2_SendBytes(BIAS_CAL, 5);
	 delay_ms(21000);
	 SART2_SendBytes(Save, 5);
}

/******************************************************************************
 * 串口2中断服务函数
******************************************************************************/
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        CopeSerial2Data((unsigned char)USART2->DR);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART2, USART_IT_ORE) != RESET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_ORE);
    }
}

