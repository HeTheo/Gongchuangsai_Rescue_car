#include "app.h"
#include "Comm.h"
#include "Motion.h"
#include "Servo.h"

static uint8_t s_PendingAckCmd = 0;   // 0 = 没有待回的 ACK


void Cmd_Dispatch(Comm_Frame_t *f)
{
    static uint8_t last_cmd = 0xFF;

    /* 同一命令连发：只更新坐标，不重复切模式 */
    if (f->cmd == last_cmd)
    {
        if (f->cmd == COMM_CMD_FORWARD)      Motion_SetTrackTarget(f->tx, f->ty);
        if (f->cmd == COMM_CMD_GO_SAFE)      Motion_SetTrackTarget(f->tx, f->ty);
		if (f->cmd == COMM_CMD_GO_SAFE_FAST)  Motion_SetTrackTarget(f->tx, f->ty);  /* ★ 必须加 */
		if (f->cmd == COMM_CMD_CAM)          Servo_SetCam(f->cls);   // ★ 加这行
        return;
    }
    last_cmd = f->cmd;

    switch (f->cmd)
    {
        case COMM_CMD_ROT_LEFT:             /* 搜索，一直转 */
        case COMM_CMD_ROT_FIND:             /* 找安全区，一直转 */
            Motion_SetTurnSpeed(30, 1);
            Motion_SetTurnAngle(0);         /* 0 = 一直转，视觉喊停 */
            Motion_SetMode(MOTION_TURN);
            break;

        case COMM_CMD_FORWARD:              /* 追踪目标 */
			Motion_SetFastGear(0);
            Motion_SetTrackTarget(f->tx, f->ty);
            Motion_SetMode(MOTION_VISUAL_TRACK);
            break;

        case COMM_CMD_GO_SAFE:              /* 追踪安全区中心 */
			Motion_SetFastGear(0);
            Motion_SetTrackTarget(f->tx, f->ty);   // ★ 改：追 K230 偏移好的 tx
            Motion_SetMode(MOTION_VISUAL_TRACK);
            break;
		
		case COMM_CMD_GO_SAFE_FAST:         /* ★ 高速追踪（只给 approach 用） */
            Motion_SetFastGear(1);
            Motion_SetTrackTarget(f->tx, f->ty);
            Motion_SetMode(MOTION_VISUAL_TRACK);
            break;

        case COMM_CMD_BACK:                 /* 后退 */
            Motion_SetMode(MOTION_BACK);
            break;

        case COMM_CMD_STRAIGHT:             /* 定距直行 */
			Motion_SetFastGear(0);          /* ★ 显式回落常规档 */
            Motion_SetMode(MOTION_STRAIGHT);
            break;
		
		case COMM_CMD_STRAIGHT_FAST:        /* ★ 高速直行 */
            Motion_SetFastGear(1);
            Motion_SetMode(MOTION_STRAIGHT);
            break;

        case COMM_CMD_STOP:                 /* 停车 */
            Motion_SetMode(MOTION_STOP);
            break;

        case COMM_CMD_CAM:                  /* 摄像头舵机：cls=0抬头/1低头 */
            Servo_SetCam(f->cls);           /* 舵机模块还没写，先留接口 */
            break;

        case COMM_CMD_GRAB:                 /* 抓取，完成后回 ack */
			Motion_SetMode(MOTION_STOP);    // ★ 先停车
            Servo_Grab();
            Comm_SendAck(COMM_CMD_GRAB);
            break;

        case COMM_CMD_DROP:                 /* 松爪 */
			Motion_SetMode(MOTION_STOP);    // ★ 先停车
            Servo_Drop();
            Comm_SendAck(COMM_CMD_DROP);
            break;

        case COMM_CMD_OPEN:                 /* 只松爪 */
			Motion_SetMode(MOTION_STOP);    // ★ 先停车
            Servo_Open();
            Comm_SendAck(COMM_CMD_OPEN);
            break;
		
		case COMM_CMD_TURN_L90:
			Motion_SetTurnSpeed(30, 1);
			Motion_SetTurnAngle(90);        // 90° 自动停
			Motion_SetMode(MOTION_TURN);
			s_PendingAckCmd = COMM_CMD_TURN_L90;
			break;
		
		case COMM_CMD_TURN_R90:
			Motion_SetTurnSpeed(30, -1);
			Motion_SetTurnAngle(90);
			Motion_SetMode(MOTION_TURN);
			s_PendingAckCmd = COMM_CMD_TURN_R90;
			break;
		
		case COMM_CMD_TURN_L:
		case COMM_CMD_TURN_R:
		{
			uint8_t deg = f->cls;                       /* 角度放在 cls */
			if (deg == 0 || deg > 180) deg = 90;        /* 兜底：非法值按 90° 处理 */
		
			Motion_SetTurnSpeed(30, (f->cmd == COMM_CMD_TURN_L) ? 1 : -1);
			Motion_SetTurnAngle((float)deg);
			Motion_SetMode(MOTION_TURN);
			s_PendingAckCmd = f->cmd;                   /* 转到位自动停 → 回 ack */
			break;
		}
		
		case COMM_CMD_SPIN:
			Motion_SetTurnSpeed(20, 1);        /* 1 = 左转 */
			Motion_SetTurnAngle(360.0f);       /* ← 目标 360° */
			Motion_SetMode(MOTION_TURN);
			s_PendingAckCmd = COMM_CMD_SPIN;   /* 转完自动停 → 回 ack */
			break;

        case COMM_CMD_DONE:                 /* 只记录，不动 */
        default:
            break;
    }
}

// 新增：主循环里周期性调用，检测转完就回 ACK
void App_Poll(void)
{
    if (s_PendingAckCmd && Motion_GetMode() == MOTION_STOP)   // 转到位自动停
    {
        Comm_SendAck(s_PendingAckCmd);   // 发 0xBB + cmd
        s_PendingAckCmd = 0;
    }
}

