#ifndef __COMM_H
#define __COMM_H

#include "sys.h"

/* =====================================================================
 * K230 <-> STM32F407 通信协议
 * ---------------------------------------------------------------------
 * 帧格式（固定 12 字节，多字节大端）：
 *   [0]      0xAA          帧头
 *   [1]      cmd           命令码
 *   [2]      cls           类别 ID；当 cmd = CMD_CAM 时表示摄像头角度档位
 *   [3][4]   txH txL       目标中心 X（display 坐标 0~639）
 *   [5][6]   tyH tyL       目标中心 Y（display 坐标 0~479）
 *   [7][8]   sxH sxL       安全区中心 X
 *   [9][10]  syH syL       安全区中心 Y
 *   [11]     sum           校验 = (byte0 + byte1 + ... + byte10) & 0xFF
 *
 * 数据方向：
 *   K230  --(12 字节 0xAA 帧)-->  STM32   每帧都重发，约 15~30 次/秒
 *   STM32 --(0xBB + cmd)-->      K230    只在离散动作完成时回，K230 靠它推进状态机
 * ===================================================================== */

/* ---------------- 帧结构常量 ---------------- */
#define COMM_HEAD            0xAA
#define COMM_ACK_HEAD        0xBB
#define COMM_FRAME_LEN       12		// 帧字节长度

/* ---------------- 链路超时（单位 ms） ----------------
 * 距上一帧有效帧超过这个时间，就认为 K230 挂了 / 串口线掉了，
 * 必须立刻停车，否则车会带着上一个命令一直跑下去。
 * 取值要大于 K230 单帧推理耗时（一般 30~80ms），500ms 有足够余量。
 * 若上板发现 K230 帧率很低导致误触发，加大到 800。 */
#define COMM_TIMEOUT_MS      1500

/* ---------------- 命令码（与 Track_wounded3 严格一致） ---------------- */
#define COMM_CMD_ROT_LEFT    0x01   /* 原地转圈搜索救援目标（约定：每次 = 转 90° + 停一下） */
#define COMM_CMD_FORWARD     0x03   /* 视觉追踪前进，tx/ty 是目标中心 */
#define COMM_CMD_GRAB        0x04   /* 停车 + 合爪，完成后要回 ack */
#define COMM_CMD_ROT_FIND    0x05   /* 原地转圈找安全区 */
#define COMM_CMD_GO_SAFE     0x06   /* 视觉追踪安全区中心，tx/ty 就是安全区中心 */
#define COMM_CMD_DROP        0x07   /* 松爪 + 后退，完成后要回 ack */
#define COMM_CMD_BACK        0x08   /* 后退（K230 自己计时，STM32 只管退） */
#define COMM_CMD_CAM         0x09   /* 摄像头舵机角度，cls = 0 抬头 / 1 低头 */
#define COMM_CMD_OPEN        0x0A   /* 只松爪，完成后要回 ack */
#define COMM_CMD_STRAIGHT    0x0B   /* 定距直行（不依赖视觉） */
#define COMM_CMD_STOP        0x0C   /* 立即停车 */
#define COMM_CMD_DONE        0x0D   /* 转口 / 完成信息 */
#define COMM_CMD_TURN_L90    0x0E   /* 左转 90°，转到位自动停（开局用） */
#define COMM_CMD_TURN_R90    0x0F   /* 右转 90°，转到位自动停（开局用） */
#define COMM_CMD_TURN_L      0x10   /* 左转指定角度，角度(0~180° )放在 cls 字段 */
#define COMM_CMD_TURN_R      0x11   /* 右转指定角度，角度(0~180° )放在 cls 字段 */
#define COMM_CMD_SPIN        0x12   /* 原地转一整圈 360°（角度闭环） */
#define COMM_CMD_STRAIGHT_FAST 0x13   /* 高速直行 */
#define COMM_CMD_GO_SAFE_FAST  0x14   /* ★ 高速追踪安全区（approach 用） */

/* ---------------- 类别 ID ---------------- */
#define COMM_CLS_BLUE        0
#define COMM_CLS_GREEN       1
#define COMM_CLS_BLACK       2
#define COMM_CLS_ORANGE      3
#define COMM_CLS_SAFETY      4

/* ---------------- CMD_CAM 的角度档位 ---------------- */
#define COMM_CAM_UP          0
#define COMM_CAM_DOWN        1

/* =====================================================================
 * 解析出来的一帧
 * ===================================================================== */
typedef struct
{
    uint8_t  cmd;        /* 命令码 */
    uint8_t  cls;        /* 类别 ID / 摄像头角度档位 */
    uint16_t tx;         /* 目标中心 X */
    uint16_t ty;         /* 目标中心 Y */
    uint16_t sx;         /* 安全区中心 X */
    uint16_t sy;         /* 安全区中心 Y */
    uint32_t tick;       /* 收到这一帧的时刻（ms） */
    uint32_t seq;        /* 帧序号，每次成功收帧自增，可用来判"是不是新帧" */
} Comm_Frame_t;


/* 初始化：清空解析状态和串口缓冲区。放在 Serial_Init() 之后调用 */
void Comm_Init(void);

/* ★ 核心函数：主循环里高频调用（无阻塞、无延时）。
 *   它把串口环形缓冲里的字节全部喂进帧解析状态机。
 *   必须尽可能频繁地调，否则数据堆在缓冲区里，控制会严重滞后。
 *   注意：它读的是"缓冲区"，跟串口中断是两个独立的东西，不会互相阻塞 */
void Comm_Poll(void);

/* 取最新帧。返回 1 = 这一轮有新帧到了，已拷贝到 *out；
 *                返回 0 = 没有新帧。
 * 是"消费型"的：同一帧只会返回一次，不会重复返回 */
uint8_t Comm_GetFrame(Comm_Frame_t *out);

/* 链路是否健康：距上一帧有效帧 < COMM_TIMEOUT_MS。
 * 注意：第一次收到有效帧之前一直返回 0（这样开机时不会误判成"掉线"） */
uint8_t Comm_Valid(void);

/* 距上一帧有效帧过了多少毫秒。从未收到过任何帧时返回 0xFFFFFFFF */
uint32_t Comm_IdleMs(void);

/* 回 ack：向 K230 发送 0xBB + cmd，告诉它"这个动作做完了"。
 * K230 只在 GRAB / DROP / OPEN 这几个动作上等 ack */
void Comm_SendAck(uint8_t cmd);

/* 连发 N 次 ack（防止被 K230 进入状态时的那次清缓冲吃掉） */
void Comm_SendAckN(uint8_t cmd, uint8_t times);

/* ---------------- 诊断计数 ----------------
 * 上板调试时挂到串口打印。正常运行时 g_CommBadCnt 和
 * g_SerialRxOverflow 都应该恒为 0 */
extern volatile uint32_t g_CommOkCnt;       /* 累计解析成功的帧数 */
extern volatile uint32_t g_CommBadCnt;      /* 累计校验失败 / cmd 非法的帧数 */
extern volatile uint32_t g_CommDropCnt;     /* 累计丢弃的非帧头字节数 */
extern volatile uint32_t g_CommResyncCnt;   /* 累计"重新同步"次数 */

#endif


