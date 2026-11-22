#include "Motion.h"
#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "Gyro.h"
#include "Serial.h"
#include "delay.h"


/* ================= 转向角度来源 =================
 * 1 = 陀螺仪（推荐），0 = 编码器（旧方案，可随时回退）
 */
#define TURN_USE_GYRO     1

/* 陀螺仪符号：
 *   ***************** 必须实测 *****************
 *   原地往【左】转，看 Yaw() 变大还是变小
 *   变大 → +1.0f
 *   变小 → -1.0f
 */
#define YAW_SIGN          (-1.0f)

static PID_TypeDef pidLeftSpeed, pidRightSpeed;  	// 速度内环
static PID_TypeDef pidYaw;                       	// 角度外环
static PID_TypeDef pidLeftTurn, pidRightTurn;    	// 转向速度内环
static PID_TypeDef pidLeftBack, pidRightBack;    	// 后退速度内环

static Motion_Mode_t Mode = MOTION_STOP;

//35，慢速可用
static float s_BaseSpeed = 49;    	// 直线基础速度（counts/20ms）
static float s_SpeedFast = 75;      // ★ 高速直行（counts/20ms ≈ PWM%）
static uint8_t s_FastGear = 0;       // 0 = 常规, 1 = 高速（只影响 MOTION_STRAIGHT）
static float s_TurnSpeed = 49;		// 转向速度 counts/20ms（比直行小，安全）
static int8_t s_TurnDir  = 1;       // +1 左转，-1 右转
static float s_TurnTarget = 0.0f;  	// 转圈目标角度：0=一直转 
static float s_YawLast  = 0.0f;     // 上一次的陀螺仪读数 */
static float s_YawAccum = 0.0f;     // 累计转角（可以超过 180°）
static float s_BackSpeed = 49;      // 后退速度，和直行一致
static float s_TrackX = 320.0f;    	// 视觉追踪目标 X 
static float s_TrackY = 240.0f;    	// 视觉追踪目标 Y（暂未用，留给速度调制） 
static float s_YawStart = 0.0f;     // 本次转弯开始时记下的陀螺仪角度 
static float s_StraightYaw = 0.0f;    // ★ 直行目标航向（进 STRAIGHT 时快照）

#define TRACK_CENTER_X   320.0f
#define TRACK_DEADBAND   15.0f     	// 像素死区 
#define TRACK_KP         0.10f     	// 像素 -> 差速(counts/20ms)，可调 
#define TRACK_OUT_MAX    15.0f     	// 差速限幅 
#define TURN_STOP_MARGIN_DEG   7.0f // 提前断电的角度，抵消惯性 
#define SPEED_PID_OMAX     50.0f       // 常规档 PID 输出上限（原值，别动）
#define SPEED_PID_OMAX_PAD 10.0f       // ★ 高速档额外余量（电池掉压时靠它维持）

static void Motion_ApplySpeedGear(Motion_Mode_t mode);
float left = 0;
float right = 0;
float leftOut = 0;
float rightOut = 0;	
float Turn_Angle = 0;
#define DEG_PER_COUNT 0.028f   // 先占位，标定后改

void Motion_Init(void)
{
    //                       Kp     Ki    Kd   Out_min Out_max Int_limit
    PID_Init(&pidLeftSpeed,  0.5f, 0.2f, 0.0f, 0, 100, 500);
    PID_Init(&pidRightSpeed, 0.3f, 0.2f, 0.0f, 0, 100, 500);
    PID_Init(&pidYaw,        0.3f, 0.0f, 0.0f, -8, 8, 10);
	
	// 转向环：对称输出，可正可负
    PID_Init(&pidLeftTurn,   0.5f, 0.2f, 0.0f, -100, 100, 500);
    PID_Init(&pidRightTurn,  0.3f, 0.2f, 0.0f, -100, 100, 500);
	
	// 后退环：对称输出，可正可负
	PID_Init(&pidLeftBack,  0.5f, 0.2f, 0.0f, -100, 100, 500);
	PID_Init(&pidRightBack, 0.3f, 0.2f, 0.0f, -100, 100, 500);
}

// 内部模式设置函数
void Motion_SetMode(Motion_Mode_t mode)
{
    if (mode == Mode) return;

    Mode = MOTION_STOP;                 // ★ 先停，避免下面阻塞期间 PID 乱纠偏
    Motor_Left_SetPWM(0);
    Motor_Right_SetPWM(0);

    if (mode == MOTION_STRAIGHT)
    {
        sendCaliYawCommand();           // 阻塞 ~200ms，此时车是停着的
        delay_ms(60);                   // ★ 等归零后的第一帧角度
        s_StraightYaw = Yaw();          // ★ 用当前朝向当直行目标
    }

    // ── 下面保持你原来的 PID_Init 那几行不变 ──
    Motion_ApplySpeedGear(mode);        // ★ 按目标模式 + 当前档位设限幅
    PID_Init(&pidYaw,        0.3f, 0.0f, 0.0f, -8, 8, 10);
    PID_Init(&pidLeftTurn,   0.5f, 0.2f, 0.0f, -50, 50, 500);
    PID_Init(&pidRightTurn,  0.3f, 0.2f, 0.0f, -50, 50, 500);
    PID_Init(&pidLeftBack,   0.5f, 0.2f, 0.0f, -50, 50, 500);
    PID_Init(&pidRightBack,  0.3f, 0.2f, 0.0f, -50, 50, 500);

    Mode = mode;                        // ★ 最后才真正切过去
}

// 外部读取模式函数
Motion_Mode_t Motion_GetMode(void)
{
    return Mode;
}

// 设置直行速度
void Motion_SetStraightSpeed(float speed)
{
    s_BaseSpeed = speed;
}

/* 按当前档位重设速度环输出限幅。
   换档时必须重新 PID_Init：① OutMax 要跟着变 ② 顺手清掉积分，防止换档瞬间冲 */
static void Motion_ApplySpeedGear(Motion_Mode_t mode)
{
    /* ★ 直行 和 视觉追踪 都吃速度档 */
    uint8_t spd_mode = (mode == MOTION_STRAIGHT || mode == MOTION_VISUAL_TRACK);
    float omax = (spd_mode && s_FastGear)
                 ? (s_SpeedFast + SPEED_PID_OMAX_PAD)
                 : SPEED_PID_OMAX;

    PID_Init(&pidLeftSpeed,  0.5f, 0.2f, 0.0f, 0, omax, 500);
    PID_Init(&pidRightSpeed, 0.3f, 0.2f, 0.0f, 0, omax, 500);
}

void Motion_SetFastGear(uint8_t on)
{
    uint8_t g = on ? 1 : 0;
    if (g == s_FastGear) return;        // 没变就不折腾
    s_FastGear = g;
    Motion_ApplySpeedGear(Mode);        // ★ 已在直行中换档，这里要立刻生效
}

float Motion_GetSpeedNow(void)
{
    return s_FastGear ? s_SpeedFast : s_BaseSpeed;
}

// 设置转圈速度，方向
void Motion_SetTurnSpeed(float speed, int8_t dir)
{
    s_TurnSpeed = speed;
    s_TurnDir   = dir;
}

//设置转圈角度
void Motion_SetTurnAngle(float deg)
{
    s_TurnTarget = deg;
    Turn_Angle   = 0;

#if TURN_USE_GYRO
    sendCaliYawCommand();      /* 清零，阻塞约 200ms */
    delay_ms(30);              /* 等一帧新角度进来 */
    s_YawLast  = Yaw();        /* 记录起点（此时应接近 0） */
    s_YawAccum = 0.0f;         /* ★ 累加器清零 */
#endif
}

// 设置后退速度
void Motion_SetBackSpeed(float speed)
{
    s_BackSpeed = speed;
}

// 设置追踪目标坐标
void Motion_SetTrackTarget(uint16_t tx, uint16_t ty)
{
    s_TrackX = (float)tx;
    s_TrackY = (float)ty;
}

void Motion_Control(void)
{
	/* 编码器必须每 20ms 读一次，否则 16 位计数器会溢出 */
    left  = Encoder_GetLeft();
    right = Encoder_GetRight();
    switch (Mode)
    {
        case MOTION_STRAIGHT:
        {
            float TurnOut = 0;
            float yaw = Yaw();
            float err = yaw - s_StraightYaw;          // ★ 相对目标，不是绝对 0

            // ★ 跨 ±180 边界归一化
            if (err >  180.0f) err -= 360.0f;
            if (err < -180.0f) err += 360.0f;

            if (err >  1.5f) TurnOut =  AnglePID_Calc(&pidYaw, 0.0f, err);
            if (err < -1.5f) TurnOut = -AnglePID_Calc(&pidYaw, 0.0f, err);

            // ★★★ 下面这 8 行是刚才丢掉的速度环，必须有 ★★★
//            float lTgt = s_BaseSpeed + TurnOut;
//            float rTgt = s_BaseSpeed - TurnOut;
			float base = Motion_GetSpeedNow();   // ★ 按档位取速度
            float lTgt = base + TurnOut;
            float rTgt = base - TurnOut;
            if (lTgt < 0) lTgt = 0;
            if (rTgt < 0) rTgt = 0;

            PID_SetTarget(&pidLeftSpeed,  lTgt);
            PID_SetTarget(&pidRightSpeed, rTgt);

            leftOut  = PID_Calc(&pidLeftSpeed,  left);
            rightOut = PID_Calc(&pidRightSpeed, right);

            Motor_Left_SetPWM(leftOut);
            Motor_Right_SetPWM(rightOut);
            break;
        }

        case MOTION_TURN:
		{
			float lTgt, rTgt;
			if (s_TurnDir >= 0) { lTgt = -s_TurnSpeed; rTgt =  s_TurnSpeed; }
			else                { lTgt =  s_TurnSpeed; rTgt = -s_TurnSpeed; }
		
			PID_SetTarget(&pidLeftTurn,  lTgt);
			PID_SetTarget(&pidRightTurn, rTgt);
		
			leftOut  = PID_Calc(&pidLeftTurn,  left);
			rightOut = PID_Calc(&pidRightTurn, right);
		
			#if TURN_USE_GYRO
				{
					float y = Yaw();
					float d = y - s_YawLast;              /* 与上一周期相比转了多少 */
			
					/* 处理 ±180 跳变：单次 20ms 内转不到 180°，所以这两句一定成立 */
					if (d >  180.0f) d -= 360.0f;
					if (d < -180.0f) d += 360.0f;
			
					s_YawAccum += d;                      /* ★ 累加 */
					s_YawLast   = y;                      /* ★ 更新上一周期读数 */
			
					Turn_Angle = YAW_SIGN * s_YawAccum;   /* 这个可以超过 180° */
				}
			#else
				Turn_Angle += (right - left) * DEG_PER_COUNT;
			#endif
		
			/* 指定了目标角度 → 转到自动停 */
			if (s_TurnTarget > 0.0f)
			{
				float a = Turn_Angle;
				if (a < 0) a = -a;
			
				if (a >= s_TurnTarget - TURN_STOP_MARGIN_DEG)   /* ★ 提前 5° 断电 */
				{
					Mode = MOTION_STOP;
					leftOut = 0;
					rightOut = 0;
				}
			}
		
			Motor_Left_SetPWM(leftOut);
			Motor_Right_SetPWM(rightOut);
			break;
		}
		
		case MOTION_BACK:
		{
			// 两轮等速反向 → 后退
			PID_SetTarget(&pidLeftBack,  -s_BackSpeed);
			PID_SetTarget(&pidRightBack, -s_BackSpeed);
		
			leftOut  = PID_Calc(&pidLeftBack,  left);
			rightOut = PID_Calc(&pidRightBack, right);
		
			Motor_Right_SetPWM(rightOut);
			Motor_Left_SetPWM(leftOut);     // 负 PWM，Motor 内部自动换方向
			
			break;
		}
		
		case MOTION_VISUAL_TRACK:
		{
			float err = s_TrackX - TRACK_CENTER_X;   /* 目标在右为正 */
		
			/* 死区：±15 像素内不修正 */
			if (err >  TRACK_DEADBAND)      err -= TRACK_DEADBAND;
			else if (err < -TRACK_DEADBAND) err += TRACK_DEADBAND;
			else                            err = 0;
		
			/* 外环：像素误差 -> 差速（先纯 P，好调） */
			float TurnOut = TRACK_KP * err;
            float omx = s_FastGear ? 20.0f : TRACK_OUT_MAX;   /* ★ 高速档 15→20 */
            if (TurnOut >  omx) TurnOut =  omx;
            if (TurnOut < -omx) TurnOut = -omx;
		
			float base = Motion_GetSpeedNow();   /* ★ 按档位取速度 */
            float lTgt = base + TurnOut;
            float rTgt = base - TurnOut;
			if (lTgt < 0) lTgt = 0;
			if (rTgt < 0) rTgt = 0;
		
			PID_SetTarget(&pidLeftSpeed,  lTgt);
			PID_SetTarget(&pidRightSpeed, rTgt);
		
			leftOut  = PID_Calc(&pidLeftSpeed,  left);
			rightOut = PID_Calc(&pidRightSpeed, right);
		
			Motor_Left_SetPWM(leftOut);
			Motor_Right_SetPWM(rightOut);
			break;
		}

        case MOTION_STOP:
        default:
            Motor_Left_SetPWM(0);
            Motor_Right_SetPWM(0);
            break;
    }
}




