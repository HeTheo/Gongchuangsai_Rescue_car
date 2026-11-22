#include "Comm.h"
#include "Serial.h"
#include "Timer.h"

/* =====================================================================
 * 诊断计数
 * ===================================================================== */
volatile uint32_t g_CommOkCnt     = 0;
volatile uint32_t g_CommBadCnt    = 0;
volatile uint32_t g_CommDropCnt   = 0;
volatile uint32_t g_CommResyncCnt = 0;

/* =====================================================================
 * 内部状态
 * ---------------------------------------------------------------------
 * 帧解析本质是一条"收字节"的流水线，只有两个状态：
 *
 *   s_Idx == 0     还没看到帧头，正在等 0xAA
 *   s_Idx == 1~11  已经收了多少个字节
 *   s_Idx == 12    收满了，立刻校验一次，然后清 0 重来
 *
 * 这个写法不需要额外的状态变量，用"已收字节数"当状态就够了。
 * ===================================================================== */
static uint8_t       s_Buf[COMM_FRAME_LEN];   /* 攒帧用的临时缓冲 */
static uint8_t       s_Idx      = 0;          /* 当前已收字节数 */

static Comm_Frame_t  s_Frame;                 /* 最近一次解析成功的帧 */
static uint8_t       s_NewFrame = 0;          /* 1 = 有帧还没被 Comm_GetFrame 取走 */
static uint32_t      s_LastTick = 0;          /* 最近一次成功收帧的时刻 */
static uint8_t       s_Inited   = 0;          /* 0 = 还一帧都没收到过 */

/* =====================================================================
 * 内部小工具
 * ===================================================================== */

/* 判断命令码是否是协议里定义过的。
 * 目的：校验和只有 1 字节，弱得很，万一数据里恰巧凑出一个 0xAA 开头
 * 且校验也碰巧对的"假帧"，靠白名单再挡一道。
 * 另外也能挡住"帧头错位但校验碰巧过"的诡异情况。 */
static uint8_t Comm_CmdIsValid(uint8_t cmd)
{
    switch (cmd)
    {
        case COMM_CMD_ROT_LEFT:
        case COMM_CMD_FORWARD:
        case COMM_CMD_GRAB:
        case COMM_CMD_ROT_FIND:
        case COMM_CMD_GO_SAFE:
        case COMM_CMD_DROP:
        case COMM_CMD_BACK:
        case COMM_CMD_CAM:
        case COMM_CMD_OPEN:
        case COMM_CMD_STRAIGHT:
        case COMM_CMD_STOP:
        case COMM_CMD_DONE:
		case COMM_CMD_TURN_L90:
		case COMM_CMD_TURN_R90:    
		case COMM_CMD_TURN_L:
		case COMM_CMD_TURN_R:	
		case COMM_CMD_SPIN:
        case COMM_CMD_STRAIGHT_FAST:	
		case COMM_CMD_GO_SAFE_FAST:		return 1;
    
        default:
            return 0;
    }
}

/* 校验：把 buf[0..10] 全部加起来取低 8 位，和 buf[11] 比。
 * 这和 K230 侧 send_frame() 里的 frame[11] = sum(frame[:11]) & 0xFF 是一回事 */
static uint8_t Comm_CheckSum(uint8_t *b)
{
    uint16_t sum = 0;
    uint8_t  i;

    for (i = 0; i < COMM_FRAME_LEN - 1; i++)
    {
        sum += b[i];
    }
    return ((sum & 0xFF) == b[COMM_FRAME_LEN - 1]) ? 1 : 0;
}

/* 解析成功 → 填结构体、记录时刻、置"有新帧"标志 */
static void Comm_HandleFrame(uint8_t *b)
{
    s_Frame.cmd  = b[1];
    s_Frame.cls  = b[2];
    s_Frame.tx   = ((uint16_t)b[3] << 8) | b[4];      /* 大端：高字节在前 */
    s_Frame.ty   = ((uint16_t)b[5] << 8) | b[6];
    s_Frame.sx   = ((uint16_t)b[7] << 8) | b[8];
    s_Frame.sy   = ((uint16_t)b[9] << 8) | b[10];
    s_Frame.tick = Tick_Get();

    s_LastTick = s_Frame.tick;
    s_NewFrame = 1;

    s_Inited = 1;              /* ★ 关键：收到第一帧后链路才算"活"了。
                                *   不加这行，Comm_Valid() 会永远返回 0 */
    g_CommOkCnt++;
}

/* =====================================================================
 * 对外接口
 * ===================================================================== */

void Comm_Init(void)
{
    s_Idx      = 0;
    s_NewFrame = 0;
    s_LastTick = 0;
    s_Inited   = 0;

    Serial_ClearRxBuffer();    /* 扔掉开机期间堆下的垃圾数据 */

    g_CommOkCnt     = 0;
    g_CommBadCnt    = 0;
    g_CommDropCnt   = 0;
    g_CommResyncCnt = 0;
}

void Comm_Poll(void)
{
    uint8_t b;

    /* 把环形缓冲里现有的字节一次全部取干净。
     * Serial_ReadByte 不带延时，缓冲区空了立刻返回 0 跳出循环，
     * 所以这里不会阻塞主循环 */
    while (Serial_ReadByte(&b))
    {
        if (s_Idx == 0)
        {
            /* ---- 状态 0：等帧头 ---- */
            if (b == COMM_HEAD)
            {
                s_Buf[s_Idx++] = b;        /* 收到帧头，开始攒帧 */
            }
            else
            {
                g_CommDropCnt++;           /* 不是帧头，丢 */
            }
        }
        else
        {
            /* ---- 状态 1：继续收，直到凑满 12 字节 ---- */
            s_Buf[s_Idx++] = b;

            if (s_Idx >= COMM_FRAME_LEN)
            {
                s_Idx = 0;                 /* 无论成功失败，都从"等帧头"重新开始 */

                if (!Comm_CheckSum(s_Buf))
                {
                    /* 校验不过：可能是假帧头，尝试重新同步 */
                    g_CommBadCnt++;
                    s_Idx = 0;          // 直接从头等帧头
                }
                else if (!Comm_CmdIsValid(s_Buf[1]))
                {
                    /* 校验过了但命令码不认识：当噪声丢掉，不重新同步
                     * （帧头位置是对的，只是内容不对） */
                    g_CommBadCnt++;
                }
                else
                {
                    Comm_HandleFrame(s_Buf);   /* 成功！ */
                }
            }
        }
    }
}

uint8_t Comm_GetFrame(Comm_Frame_t *out)
{
    if (!s_NewFrame) return 0;

    s_NewFrame = 0;                 /* 消费掉这个标志 */
    if (out) *out = s_Frame;        /* 结构体整体赋值，很快 */
    return 1;
}

uint8_t Comm_Valid(void)
{
    if (!s_Inited) return 0;        /* 一帧都没收过 → 不算"活" */
    return (Comm_IdleMs() < COMM_TIMEOUT_MS) ? 1 : 0;
}

uint32_t Comm_IdleMs(void)
{
    if (!s_Inited) return 0xFFFFFFFF;
    return Tick_Elapsed(s_LastTick);
}

void Comm_SendAck(uint8_t cmd)
{
    uint8_t t[2];

    t[0] = COMM_ACK_HEAD;           /* 0xBB */
    t[1] = cmd;                     /* 回显命令码 */

    Serial_SendArray(t, 2);
}

void Comm_SendAckN(uint8_t cmd, uint8_t times)
{
    uint8_t i;

    for (i = 0; i < times; i++)
    {
        Comm_SendAck(cmd);
    }
}

