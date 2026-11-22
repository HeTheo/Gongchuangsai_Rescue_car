'''
New_v8 —— 智能救援小车视觉决策主程序（比赛用）

★ 比赛前要改的配置都在最上面「配置」区（文件约第 110 行起）：
    SAFE_USE = SAFE_BLUE_CLS / SAFE_RED_CLS   抽签定自己安全区颜色
    CONF_NORMAL / CONF_FIND / CONF_SAFE       检测置信度三档
    TEST_MODE = False                         正式比赛必须 False

SHIFT 需要实标的物理量（SHIFT 参数块，用 ★ 标了）：
    SH_CAM_H   相机镜头离地高度（卷尺，米）0.18m
    SH_SAFE_W  安全区短边宽度（卷尺，米）0.36m
    SH_FH      垂直 f·h（两点标定）111.8
    SH_V       直行速度（2 秒量距离/2，米/秒）0.3m/s
'''

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import aidemo
from machine import UART, FPIOA
import sys, gc, time, math

DEBUG = False
def log(*args):
    if DEBUG:
        print(*args)
        
# ================= 串口 =================
fpioa = FPIOA()
fpioa.set_function(9, fpioa.UART1_TXD, ie=0, oe=1)
fpioa.set_function(10, fpioa.UART1_RXD, ie=1, oe=0)
uart1 = UART(UART.UART1, baudrate=115200)

# 帧: [AA][cmd][cls/参数][txH txL][tyH tyL][sxH sxL][syH syL][sum]
CMD_ROT_LEFT = 0x01   # 原地转圈搜索救援目标
CMD_FORWARD  = 0x03   # 前进，tx/ty 交给 STM32 的 PID
CMD_GRAB     = 0x04   # 停车 + 抓取
CMD_ROT_FIND = 0x05   # 原地转圈找安全区
CMD_GO_SAFE  = 0x06   # 追踪安全区中心
CMD_BACK     = 0x08   # 后退
CMD_CAM      = 0x09   # 摄像头舵机角度，角度放 frame[2]
CMD_OPEN     = 0x0A   # 只释放夹爪
CMD_STRAIGHT = 0x0B   # 定距直行（不依赖视觉）
CMD_STOP     = 0x0C   # 立即停车
CMD_TURN_L   = 0x10   # 左转指定角度，角度(°)放在 cls 字段
CMD_TURN_R   = 0x11   # 右转指定角度，角度(°)放在 cls 字段
CMD_SPIN     = 0x12   # 原地连续转一整圈 360°（角度闭环）
CMD_STRAIGHT_FAST = 0x13   # ★ 高速直行（只在开局空地/撞散时用）
CMD_GO_SAFE_FAST  = 0x14   # ★ 高速追踪（只在 APPROACH_SAFE 用）

CAM_UP, CAM_DOWN, CAM_NEAR = 0, 1, 2   # frame[2] 的档位

def send_frame(cmd, cls, tx, ty, sx, sy):
    """cls 普通命令=类别；CMD_CAM 时=角度档位。"""
    tx = 0 if int(tx) < 0 else int(tx)
    ty = 0 if int(ty) < 0 else int(ty)
    sx = 0 if int(sx) < 0 else int(sx)
    sy = 0 if int(sy) < 0 else int(sy)
    frame = bytearray(12)
    frame[0] = 0xAA
    frame[1] = cmd
    frame[2] = cls
    frame[3] = (tx >> 8) & 0xFF; frame[4] = tx & 0xFF
    frame[5] = (ty >> 8) & 0xFF; frame[6] = ty & 0xFF
    frame[7] = (sx >> 8) & 0xFF; frame[8] = sx & 0xFF
    frame[9] = (sy >> 8) & 0xFF; frame[10] = sy & 0xFF
    frame[11] = sum(frame[:11]) & 0xFF
    uart1.write(bytes(frame))

_ack_buf = b''

def read_ack():
    """带留存缓冲：0xBB 和 cmd 被拆到两批里也不会丢"""
    global _ack_buf

    n = uart1.any()
    if n > 0:
        _ack_buf += uart1.read(n)

    if len(_ack_buf) > 128:              # 防无限增长
        _ack_buf = _ack_buf[-128:]

    idx = _ack_buf.find(b'\xbb')
    if idx < 0:
        return None

    if idx + 1 >= len(_ack_buf):
        return None                      # 只收到 0xBB，等下一帧补齐

    cmd = _ack_buf[idx + 1]
    _ack_buf = _ack_buf[idx + 2:]        # ★ 消费掉，避免同一个 ack 被重复返回
    return cmd


# ================= 测试模式 =================
TEST_MODE    = True       # True = 跳过开局，直接从 START_STATE 开始（正式跑必须 False）
START_STATE  = "SCATTER_INTRO"    # 要测的状态名
START_CLS    = 2           # 预置"已锁定类别"（GRAB/CHECK/RETREAT/GO_SAFE/DROP 需要）
CALIB_BOX = False           # 抓取框标定，标定完改成 False，比赛时关掉

# ================= 配置 =================
labels = {0: 'Black', 1: 'Blue', 2: 'Green', 3: 'Orange', 4: 'Safe_blue', 5: 'Safe_red'}
BLACK_CLS = 0
BLUE_CLS  = 1        # 危险物（蓝色）
GREEN_CLS = 2
ORANGE_CLS = 3
SAFE_BLUE_CLS = 4
SAFE_RED_CLS  = 5

DANGER_CLS   = BLUE_CLS                       # 危险物
SAFE_CLASSES = (SAFE_BLUE_CLS, SAFE_RED_CLS)  # 安全区标记有两种
# SAFE_USE     = SAFE_RED_CLS    # 填 SAFE_BLUE_CLS 或 SAFE_RED_CLS；None=两种都当安全区
SAFE_USE     = SAFE_BLUE_CLS

# ★ 置信度：分「找/接近」和「精判/送」两档（比赛前可调）
# 远距离只看得到紫框、看不清内部颜色 → 蓝/红分不清、置信度也低。
# 所以「找/接近」用低阈值且蓝红通吃；凑近后内部颜色可见，才用高阈值精判。
CONF_NORMAL = 0.63     # 常规：抓取 / 撞散
CONF_FIND   = 0.46     # ★ 远距离找/接近：只求"那儿有安全区"，蓝/红先不分
CONF_SAFE   = 0.55     # ★ 近距离精判（ALIGN/SHIFT/GO_SAFE）：降到 0.55 防安全区闪烁
FIND_STATES  = ("FIND_SAFE", "APPROACH_SAFE")              # 低阈值档
SAFE_STATES  = ("ALIGN_X", "SHIFT", "TO_OWN_SAFE", "GO_SAFE")  # 高阈值精判档

# 优先级：橙 > 黑 > 绿（值越大越优先）
RESCUE_PRIO = {ORANGE_CLS: 3, BLACK_CLS: 2, GREEN_CLS: 1}

kmodel_path = "/sdcard/gongchuangsai_yolo11n_det_320_v3.kmodel"
model_input_size = [320, 320]
rgb888p_size = [320, 320]
display_mode = "lcd"
display_size = [640, 480]

# ================= 转向参数（按角度，不按时间）=================
SPIN_WAIT_MS = 7000   # 等一整圈的上限（ack 通了会提前返回）

# ================= 抓取参数=================
GRAB_TURN_DEG   = 20     # 抓完后左转的角度（从 15 开始试，别太大）
GRAB_TURN_WAIT  = 500   # 等左转完成的上限（STM32 转向内部要先标定陀螺仪，约 220ms）
TARGET_CONFIRM_FRAMES = 5    # 连续 N 帧都能选出同一个候选才锁定
GRAB_CAM_SETTLE_MS = 350    # 低头后等舵机转到位 + 画面稳定
GRAB_OPEN_MS       = 400    # 抬爪等待上限
GRAB_ADVANCE_MAX   = 1500   # 前进保底超时（一直没到位就一直开）
GRAB_CLOSE_MS      = 500    # 夹紧等待上限
GRAB_X_TOL         = 240    # ★ 到位判定的 X 容差（±px），比 BOX_W/2=270 略小
GRAB_ALIGN_TOL     = 100    # ★ TRACK 进 GRAB 前，目标 X 离中心(320)多少 px 内才算"摆正"
GRAB_LOST_FRAMES   = 15     # ★ 连续看不到目标 N 帧 → 回 SEARCH（约 0.75s @20fps）
GRAB_STUCK_FRAMES  = 30     # ★ 目标可见但一直不到位 N 帧 → 回 SEARCH（约 2.0s @20fps）

# ================= 追视野边缘目标：原地转圈摆正再追（车体会带目标走） =================
EDGE_TOL        = 100    # ★ 目标 X 离中心超过此值 = 在车旁边（视野边缘）
EDGE_GAIN       = 0.15    # ★ 原地转向对准目标的增益（度/像素）
EDGE_TURN_WAIT  = 1000    # ★ 原地转向后等多久再重新判断 (ms)
TRACK_ALIGN_Y   = 250     # ★ 目标 cy 大于此值 = 够近，先原地转对准再前进追
TRACK_ALIGN_TOL = 60      # ★ X 偏差 |cx-320| 小于此值 = 已对准，停止原地转

# ================= SMASH 撞散参数（55° 视角）=================
SMASH_ENABLE        = True    # False = 退回旧逻辑（簇也直接抓）
SMASH_STICK_TH      = 0.5     # 55° 视角下的粘连判据（和抬头的 0.5 不同，见下方说明）
SMASH_CAM_SETTLE_MS = 400     # 切 55° 后等舵机 + 画面稳定
SMASH_PUSH_MS       = 1500    # 单次撞进去的时长
SMASH_BACK_MS       = 500     # 每次撞完后退，重新对准再撞
SMASH_EXIT_BACK_MS  = 400     # 散开后后退脱离
SMASH_SKIP_SPIN     = True    # 撞完回 SEARCH 时跳过转圈，直接直行判定
SMASH_ALIGN_TOL     = 60    # 簇质心离画面中心(320)多少 px 内才算"对准"
SMASH_FORCE_PUSH_DY = 30    # cy 超过 Y_NEAR_TH 这么多还没对准 → 也强制撞
SMASH_TOTAL_TIMEOUT = 10000   # ★ 撞散总时长上限，超时还没散就放弃（防死循环）
SMASH_MAX_TRIES     = 2      # ★ 撞散计次：撞满这个次数还没散 → 抛弃这俩，去找别的目标
ABANDON_SKIP_MS     = 20000  # ★ 抛弃后多长时间内不框"抛弃位置附近"的目标
ABANDON_SKIP_PX     = 100    # ★ "抛弃位置附近"的判定半径（像素）

# ================= 一次多搬（green_ever 之后才启用）=================
MULTI_GRAB_MAX   = 2                          # 一次最多带几个（必须 ≤ check_vote 上限 2）
MULTI_GRAB_CLS   = (GREEN_CLS, BLACK_CLS)     # ★ 能配对搬的只有绿/黑
                                              #   蓝=危险物、橙=必须独占 → 一律去撞散
MULTI_CONFIRM    = 3                          # 连续 N 帧满足才生效（防 YOLO 抖动）

# ================= 开局动作（左转45° → 直行3s）=================
INTRO_TURN_DEG    = 45      # 开局左转角（°）
INTRO_TURN_WAIT   = 1200    # 等左转完成的上限（45°≈1.15s + 陀螺标定 0.22s）
INTRO_STRAIGHT_MS = 3500    # 开局直行时长
INTRO_GRAB_MS     = 600     # ★ 直行结束后"把框打下来"的等待上限

# ================= 开局撞散参数 =================
SCATTER_ENABLE          = True
SCATTER_NEAR_Y          = 400     # 目标框高 > 此值 = 够近（上板标定）
SCATTER_HIT_MS          = 1500    # 第 2、3 次撞：直行时长
SCATTER_BACK_MS         = 1500    # 每次撞完后退时长
SCATTER_FINAL_BACK_MS   = 1000    # 最后一次后退
SCATTER_TURN_DEG        = 15      # 偏转角（第2次左20°，第3次右40°）
TURN_DEG_WAIT_MS        = 700    # 等指定角度转完的上限（ack 通了会提前返回）
TURN_ENTRY_GAP_MS       = 50      # 两次动作之间留的间隔
SCATTER_FIND_TIMEOUT    = 15000   # 找不到目标的兜底超时

# ---- 撞散动作序列（左20°撞 → 退回 → 右40°撞 → 退回）----
SCATTER_STEPS = [
    ("TURN_L", SCATTER_TURN_DEG),       # ① 左转 20°
    ("HIT",    SCATTER_HIT_MS),         # ② 左前方撞
    ("BACK",   SCATTER_BACK_MS),        # ③ ★ 退回来（新增）
    ("TURN_R", SCATTER_TURN_DEG),   # ④ 右转 40° → 变成右偏 20°
    ("HIT",    SCATTER_HIT_MS),         # ⑤ 右前方撞
    ("BACK",   SCATTER_FINAL_BACK_MS),  # ⑥ 后退脱离
]

# ================= CHECK 分离动作参数 =================
APART_OPEN_MS   = 400     # 抬爪（张开，释放目标）
APART_BACK_MS   = 500     # 后退
APART_CLOSE_MS  = 400     # 放爪（闭合，当推板用）
APART_PUSH_MS   = 1300    # 往前推，把目标撞散
APART_BACK2_MS  = 800    # 再后退

# ---- 分离动作序列 ----
APART_STEPS = [
    ("OPEN",  APART_OPEN_MS),    # ① 抬爪（释放）
    ("BACK",  APART_BACK_MS),    # ② 后退
    ("CLOSE", APART_CLOSE_MS),   # ③ 放爪（闭合）
    ("PUSH",  APART_PUSH_MS),    # ④ 往前推撞散
    ("BACK",  APART_BACK2_MS),   # ⑤ 再后退
]

# 安全区参数
SAFE_Y_NEAR       = 265     # 保持：安全区中心 y 到这 = 已到安全区正前方
SAFE_DELIVER_DEG  = 30      # 送入时转的角度
SAFE_PUSH_MS      = 1700    # 转完后前进多久（把目标横向推进去）
GO_SAFE_TIMEOUT   = 2000    # ★ GO_SAFE 对准阶段的时间保底：超时直接转向送（防止一直往安全区里冲）
SAFE_BACK_MS      = 2000    # 松爪后退多久
SAFE_RETRY_MAX    = 2       # ★ 目标卡在安全区外时，最多补推几次
SAFE_AREA_MAX     = 160000   # ★ FIND_SAFE 刚找到安全区时，框面积超过此值 = 离太近，先退一次
SAFE_BACK_AREA_MS = 1500     # ★ 离太近时后退多久 (ms)
TURN_ACK_TIMEOUT  = 1000    # 等 STM32 转向 ack 的超时
LEAVE_TURN_DEG    = 120     # 送完后转离安全区的角度
LEAVE_TURN_WAIT   = 2500    # ★ 离场转向的等待上限（90° 约需 1.5~2s）

# ================= 安全区长边角度（只测量，不控制）=================
EDGE_ONLY  = False
EDGE_DRAW   = True     # ★ 任何状态都测长边并画在屏幕上
SHOW_THETA  = True      # ★ 实时在屏幕显示 theta（测 roll 用，测完改 False）
SHIFT_ENABLE = True     # False = 跳过 SHIFT（只调试时用）
COL_STEP   = 4          # 上限（自适应会往下调）
ROW_STEP   = 1          # ★ 1 → 不再跨过细线
MAX_ROWS   = 200        # 单次最多扫多少行（替代原 MAX_UP）
STALL_MAX  = 80         # ★ 连续这么多行没有新增命中就放弃
FIT_TOL    = 4.0
SPLIT_GAIN = 0.5        # ★ 面对角：分段拟合残差 < 整体拟合残差×此值 → 判定有"V"字拐点，切两段取长边
MED_N      = 5
PUR_DG     = 20         # ★ 30 → 20
PUR_DR     = 0          # ★ 5 → 0
PUR_BMIN   = 60         # ★ 80 → 60

# ================= FIND_SAFE：先找自己的，超时用对方的补救 =================
# 两个安全区面对面、不能同时看到：
#   phase 0：转圈只找自己的颜色（SAFE_USE），找到 → 正常 approach/shift/go_safe；
#   phase 1：超时找不到 → 转圈找对方区（非 SAFE_USE），找到直接 ALIGN，
#            SHIFT 末尾颜色投票会自动判成"对方区"→ 反向转 90°（掉头）面朝自己区。
FS_SPIN_MAX_MS  = 6000    # phase 0 转圈找自己区的超时 (ms)
FS_MIN_SPIN_MS  = 5000    # ★ 找到"远的自己区"前，先转这么久看有没有更近的（防把远处的蓝误认成红）
TO_OWN_TIMEOUT  = 4000    # 掉头后直行找自己区的超时 (ms)，超时回 FIND_SAFE 转圈

# ================= 全局卡住检测（画面长时间没变化 = 车被卡住） =================
STUCK_TIMEOUT_MS = 4000   # 画面连续这么久没变化 → 判定卡住
STUCK_SIG_STEP   = 16     # 画面签名下采样步长（像素）
STUCK_SIG_TH     = 500   # 相邻帧签名差 < 此值 = 视为"没变化"（需现场标）
STUCK_RECOVER_MS = 800    # 卡住后：抬框 + 后退 多久 (ms)

# ================= APPROACH：安全区太远时先追过去 =================
AP_Y_NEAR     = 160     # safe 中心 y ≥ 此值 = 够近，停车进 ALIGN_X
                        # ★ 必须明显小于 SAFE_Y_NEAR(300)，否则 SHIFT 还没做就送货
AP_Y_NEAR_FAR = 30     # ★ 中距离下限：sy ≥ 此值(且<AP_Y_NEAR) = 中距离，识别到直接去；低于此 = 特别远，先转圈
AP_Y_NEAR_LOOSE = 130   # ★ 对方区补救时的 approach 终点（比 AP_Y_NEAR 小，不用那么近）
AP_OWN_CONFIRM  = 3     # ★ 补救途中连续这么多帧判成"自己区" → 转正常 approach
AP_LOST_MAX   = 20      # 连续看不到 safe 这么多帧 → 回 FIND_SAFE
AP_LOST_KEEP  = 8       # 丢帧期间仍用"最后已知位置"继续追的帧数
AP_TIMEOUT    = 8000   # 追击总超时，防止一直追不到

# ================= ALIGN_X：原地转，让安全区 x 归中 =================
AX_TOL_PX    = 20      # 容差（640 宽显示坐标，20px ≈ 2°）
AX_GAIN      = 0.09    # 度 / 像素（经验增益，先这么用）
AX_MAX_DEG   = 90      # 单次最大转角
AX_MIN_DEG   = 2       # 单次最小转角（防抖）
AX_MAX_ITER  = 2       # 最多转几次
AX_SETTLE_MS = 150     # 转完等画面稳定
AX_MS_PER_DEG= 25      # ★ 每度多少毫秒（= STM32 实际转向速度）
AX_MARGIN    = 250     # ★ 转向等待的固定余量（替代固定 AX_WAIT_MS）
AX_WAIT_MS   = 2000    # 等转向 ack 的超时
AX_LOST_MAX  = 25      # 连续看不到安全区 N 帧 → 放弃（调大防安全区闪烁时来回切）

# ================= SHIFT：转(90-θ) → 直行 → 转回 90° =================
# 目标：把车身挪到「安全区中线」（车轴穿过安全区中心、且垂直于长边），
#       这样黑/绿才能确定地推进左半区、橙推进右半区。
#
# ★★★ 需要你实标的物理量（其余都是屏幕能直接读出的）：
#   SH_CAM_H  相机镜头到地面的高度（卷尺量，米）
#   SH_SAFE_W 安全区「短边」宽度（卷尺量规则场地，米）
#   SH_FH     垂直方向 f·h，由 README §4.7.5 阶段1 两点标定（现在 112）
#   SH_V      车直行速度 = 直行2秒量到的距离/2（米/秒）
#
# 算法见 shift_geom() / shift_S()，几何推导见 README §4.7.8。
SH_FH         = 111.8    # 垂直 f·h（display px·m），阶段1标定
SH_YH         = -37.2    # 地平线 y（display 坐标；抬头时为负，正常）
SH_CAM_H      = 0.18     # ★ 标定：相机镜头离地高度 (m)
SH_SAFE_W     = 0.36     # ★ 标定：安全区短边宽度 (m)
SH_ASPECT     = 0.75     # 320×320 → 640×480 的斜率换算 1.5/2，一般不动
SH_V          = 0.3    # ★ 标定：直行速度 (m/s)；以后改编码器定距就删掉
SH_SY_MIN     = 40       # Zq 反解下限（display px），防除爆
SH_S_MAX      = 0.7     # 横移距离保险丝 (m)，防止拟合异常冲过头
SH_DRIVE_MAX_MS = 2500   # 横移直行时长保险丝 (ms)
SH_MIN_DEG    = 3        # |θ| ≤ 此值 → 已正对，跳过 SHIFT
SH_MAX_TH     = 70       # |θ| > 此值 → 拟合大概率抓到短边/角，跳过
SH_BIG_TH     = 30       # ★ |θ| 超过此值 → 面对角，θ/S 估不准，横移加增量补偿
SH_BIG_GAIN   = 1.0      # ★ 角度过大时 S 的放大系数（>1 = 多横移一点；实测大角度要 2 倍才到位）
SH_S_GAIN     = 1.3      # ★ 横移距离整体补偿系数（>1 = 多横移；"回不到中线"就调到 1.1~1.2）
SH_MEAS_MS    = 700      # θ 采样窗口
SH_SAMPLE_SETTLE_MS = 400  # ★ 采样前停稳等待：车从 ALIGN/APPROACH 转过来有惯性，先等它停稳再采样
SH_STABLE_TH  = 2.0      # ★ 角度稳定阈值：最近 N 帧 θ 极差小于此值 → 提前锁定结束采样（度）
SH_STABLE_N   = 3        # ★ 连续稳定帧数：最近 N 帧 θ 都稳才锁定
SH_MEAS_MAX_MS = 2000    # ★ 保底：采样窗口后仍 0 样本，最多再等这么久就跳过
SH_MS_PER_DEG = 25       # 转向等待 = 角度×此值 + 余量（毫秒/度）
SH_TURN_MARGIN = 300     # 转向等待固定余量 (ms)
SH_MAX_MS     = 3000     # 单次转向等待上限 (ms)
SH_TRIM_DEG   = 0        # 第一段转向余量；θ 算准后应为 0（固件少转7°在净转角里抵消）
SH_SETTLE_MS  = 0        # 转向后停稳等待
SH_DIR_SIGN   = 1        # 方向开关：1=用 θ 原符号；-1=整体反向
TH_ZERO       = -6.0      # 正对时的残余偏置（一般 0）
SH_EDGE_DBG   = False    # True = 打印长边拟合命中情况

# ---- 送入后验证（确认绿色真的进了安全区）----
VERIFY_FRAMES     = 10     # 验证持续帧数
VERIFY_SHRINK_X   = 0.75   # ★ bbox 宽方向的收缩系数（斜边+透视 → 只算内部）
VERIFY_SHRINK_Y   = 0.65   # ★ bbox 高方向的收缩系数（竖直方向被透视压得更狠）
VERIFY_MIN_R      = 8      # 半径下限，防止框太小时判据退化成 0
VERIFY_LEFTOVER_FRAC = 0.25  # ★ 卡门口判据线：cy > 框中心 + 框高×此值 = 目标卡在下面（下 1/4 线）

# ---- 绿色保底：带非绿去送前，确认安全区左半已经有绿 ----
GREEN_CHECK_CONFIRM = 5     # ★ 连续这么多帧"左半没绿+带非绿" → 放弃重找绿
REGRAB_BACK1_MS    = 500    # ★ 放弃：第一次后退
REGRAB_OPEN_MS     = 400    # ★ 抬框（释放）
REGRAB_BACK2_MS    = 600    # ★ 第二次后退
REGRAB_CLOSE_MS    = 400    # ★ 落框
REGRAB_AFTER_SAFE_BACK_MS = 500   # ★ 补抓抓完后后退距离（再回 GO_SAFE 对准送，避免斜着送）

Y_NEAR_TH  = 250      # 目标 cy 大于此值 = 足够近，开始抓
MIN_TARGETS = 1       # 至少同时看到 1 个救援目标才锁定

# 抓取框（低头 CHECK 时视野会变，BOX 需按低头后的画面重新标定）
BOX_CX, BOX_CY = 340, 300
BOX_W, BOX_H   = 540, 320
GRAB_ARC_UP    = True      # True = 圆弧在上、直径在底边（矩形上面两角会被切掉）
                           # False = 圆弧在下、直径在顶边
                            
# ================= 目标粘连（簇）判定 =================
STICK_TH           = 0.4    # 归一化间隙 < 此值 = 粘连（0.5~1.2 之间调）
STICK_Y_W          = 3.0    # ★ Y 轴（前后）间隙权重：前后挨着的目标推不开，放大间隙不当簇
STICK_X_OVL        = 0.5    # ★ X 重叠量相对较小框宽度的阈值：超过=前后关系撞不开，不当簇（直接框）
LOCK_DRIFT_PX      = 160    # 相邻帧"同一目标"允许的最大位移（曼哈顿）
PICK_ISOLATED_ONLY = True   # True = 只锁周围没东西挨着的目标
STICK_PENALTY      = 1.8    # 粘连目标的距离惩罚系数（1.0=不惩罚，1.6=中等，3.0=强烈偏爱孤立）

# ================= 车体位置（距离度量）=================
CAR_FOOT_X = 320.0    # 画面底边中点 X（640×480 坐标系，不是 320×320）
CAR_FOOT_Y = 480.0    # 画面底边 Y
DIST_W_X   = 0.35      # ★ 横向偏差权重（调小 → 更看重"近"，不太排斥旁边的目标）
                      #   0.0 = 只看前后远近（边缘目标也照样选）
                      #   0.35 = 轻微偏爱正前方（推荐起始值）
                      #   0.8 = 强烈偏爱正前方（会放弃近的侧边目标）

# ================= AVOID 卡住退避参数 =================
AVOID_BACK_MS   = 600     # 后退时长
AVOID_TURN_DEG  = 30      # 左转避开的角度
AVOID_TURN_WAIT = 2000    # 转向等待上限（60° 约需 0.8~1.2s）

# ================= 全局：离对方安全区太近 → 后退 + 左转100° =================
OPP_TOO_CLOSE_Y   = 330     # 对方区中心 y 超过此值(≈0.30m) = 太近 → 躲开
OPP_AVOID_BACK_MS = 800     # 后退时长
OPP_AVOID_TURN_DEG= 100     # 左转角度
OPP_AVOID_TURN_MS = 3000    # 左转100°等待上限（45°≈1.15s，100°≈2.6s + 陀螺标定余量）

# 时序参数（上板标定）
SEARCH_STRAIGHT_MS  = 2000    # 直行时长
BACK_MS             = 500   # 抓完后后退时长
CAM_SETTLE_MS       = 300     # 停稳后等画面稳定（已低了一路，不用太久）
CHECK_FRAMES        = 10      # 投票帧数（多数表决）
CAM_UP_MS           = 600    # 判定完抬头
RETREAT_BACK_MS     = 500
SEARCH_CAM_MS       = 250     # ★ 进门第 1 步：抬头
SEARCH_GRAB_MS      = 400     # ★ 进门第 2 步：收爪

# ================= 检测类（官方模板，不变）=================
class ObjectDetectionApp(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                 confidence_threshold, nms_threshold=0.45,
                 rgb888p_size=[320,320], display_size=[640,480], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.color_four = get_colors(len(self.labels))
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [128,128,128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                            [1,3,self.model_input_size[1],self.model_input_size[0]])

    def preprocess(self, input_np):
        with ScopedTiming("preprocess", self.debug_mode > 0):
            return [nn.from_numpy(input_np)]

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            new_result = results[0][0].transpose()
            det_res = aidemo.yolov8_det_postprocess(
                new_result.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num)
            return det_res

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for i in range(len(dets[0])):
                    x, y, w, h = map(lambda x: int(round(x, 0)), dets[0][i])
                    pl.osd_img.draw_rectangle(x, y, w, h, color=self.color_four[dets[1][i]], thickness=4)
                    pl.osd_img.draw_string_advanced(x, y-50, 32,
                        " " + self.labels[dets[1][i]] + " " + str(round(dets[2][i], 2)),
                        color=self.color_four[dets[1][i]])
            else:
                pl.osd_img.clear()

# ================= 工具函数 =================
def flush_rx():
    """清掉串口和 ack 缓冲里的所有残留"""
    global _ack_buf
    _ack_buf = b''
    n = uart1.any()
    if n > 0:
        uart1.read(n)

def parse_detections(res):
    dets, dangers, safes = [], [], []
    if not res or len(res) < 3:
        return dets, dangers, safes
    for i in range(len(res[0])):
        x, y, w, h = res[0][i]
        cls_id = res[1][i]
        score = res[2][i]
        d = {"cls": cls_id, "score": score,
             "cx": x + w/2, "cy": y + h/2, "w": w, "h": h}
        if cls_id in SAFE_CLASSES:     
            safes.append(d)
        elif cls_id == DANGER_CLS:
            dangers.append(d)
        elif cls_id in RESCUE_PRIO:
            dets.append(d)
    return dets, dangers, safes

def in_safe_zone(cx, cy, safe):
    if safe is None:
        return False
    sx, sy, sw, sh = safe
    return (sx - sw/2) < cx < (sx + sw/2) and (sy - sh/2) < cy < (sy + sh/2)

def has_green_in_safe(dets, safe):
    """绿色是否已进安全区正确半区（绿应进左半区，cx < 框中心）"""
    if safe is None:
        return False
    sx, sy, sw, sh = safe
    for d in dets:
        if d["cls"] == GREEN_CLS:
            if d["cx"] < sx and abs(d["cy"] - sy) < sh / 2.0:
                return True
    return False

def car_dist(d):
    """离车越近 → 返回值越小。
       主判据 = 前向距离代理 (CAR_FOOT_Y - cy)：透视下 cy 越大越近，单调对应真实远近。
       横向偏差 (cx - 320) 只做小权重惩罚，相当于"开过去还要多转一点"的成本。"""
    dfwd = CAR_FOOT_Y - d["cy"]           # 前方距离代理（像素），越小越近
    dlat = d["cx"] - CAR_FOOT_X
    if dlat < 0:
        dlat = -dlat
    return dfwd + DIST_W_X * dlat

# ================= 簇（粘连）判定 =================
def box_gap(a, b):
    """两框之间的间隙，按较小框宽度归一化。< STICK_TH 视为粘连"""
    ax0, ax1 = a["cx"] - a["w"] / 2, a["cx"] + a["w"] / 2
    ay0, ay1 = a["cy"] - a["h"] / 2, a["cy"] + a["h"] / 2
    bx0, bx1 = b["cx"] - b["w"] / 2, b["cx"] + b["w"] / 2
    by0, by1 = b["cy"] - b["h"] / 2, b["cy"] + b["h"] / 2
    gx = max(ax0, bx0) - min(ax1, bx1)      # >0 分离，<0 重叠
    gy = max(ay0, by0) - min(ay1, by1)
    m  = min(a["w"], b["w"])
    if m <= 0:
        return 999.0
    # ★ X 高度重叠（前后关系）→ 撞不开，直接不当簇（返回大间隙）
    if -gx > STICK_X_OVL * m:
        return 999.0
    g  = max(gx, gy * STICK_Y_W)            # ★ Y 轴加权：前后挨着放大间隙，不当簇
    return g / float(m)

def split_clusters(dets, thr=STICK_TH):
    """单链接聚类：互相粘连的目标并成一簇"""
    clusters = []
    for d in dets:
        hit = []
        for c in clusters:
            for o in c:
                if box_gap(d, o) < thr:
                    hit.append(c)
                    break
        if not hit:
            clusters.append([d])
        else:
            base = hit[0]
            base.append(d)
            for c in hit[1:]:
                base.extend(c)
                clusters.remove(c)
    return clusters

def is_isolated(d, others, thr=STICK_TH):
    """d 周围有没有别的目标挨着（others 里必须包含 d 本身）"""
    for o in others:
        if o is d:
            continue
        if box_gap(d, o) < thr:
            return False
    return True

def cluster_of(t, others, thr=STICK_TH):
    """返回【包含 t 的那个簇】（单链接扩散）。
       others 里必须包含 t 本身。"""
    cl  = [t]
    ids = set([id(t)])
    changed = True
    while changed:
        changed = False
        for o in others:
            if id(o) in ids:
                continue
            for c in cl:
                if box_gap(o, c) < thr:
                    cl.append(o)
                    ids.add(id(o))
                    changed = True
                    break
    return cl

def cluster_center(cl):
    """簇的质心（各目标 bbox 中心的平均）= 撞散要瞄的点"""
    n = len(cl)
    if n <= 0:
        return None
    return (sum(d["cx"] for d in cl) / float(n),
            sum(d["cy"] for d in cl) / float(n))
    
def can_cograb(cl):
    """一次多搬判据：簇里 2~MULTI_GRAB_MAX 个，且成员全是绿/黑。
       不需要含绿色（黑黑也行）；蓝/橙一律不给合搬。"""
    n = len(cl)
    if n < 2 or n > MULTI_GRAB_MAX:
        return False
    for d in cl:
        if d["cls"] not in MULTI_GRAB_CLS:
            return False            # 蓝(危险) / 橙(必须独占) → 去撞散
    return True

def target_side(cls):
    return "right" if cls == ORANGE_CLS else "left"

def in_grab_box(d):
    """目标中心是否落在半圆形判定区内"""
    r   = BOX_W / 2.0                 # 半径 = 半圆直径的一半
    cx0 = BOX_CX

    if GRAB_ARC_UP:
        cy0 = BOX_CY + BOX_H / 2.0    # 直径在判定框底边
        dy  = cy0 - d["cy"]           # 向上为正
    else:
        cy0 = BOX_CY - BOX_H / 2.0    # 直径在判定框顶边
        dy  = d["cy"] - cy0           # 向下为正

    if dy < 0:                        # 落在直径线的另一侧 → 肯定不在半圆内
        return False

    dx = d["cx"] - cx0
    return dx * dx + dy * dy <= r * r

def check_vote(dets, dangers):
    """返回 'bad' / 'empty' / 'ok'"""
    box_danger = [d for d in dangers if in_grab_box(d)]
    box_rescue = [d for d in dets    if in_grab_box(d)]
    box_green  = [d for d in box_rescue if d["cls"] == GREEN_CLS]
    box_orange = [d for d in box_rescue if d["cls"] == ORANGE_CLS]

    # ① 框内出现危险物 → 一律不合格
    if box_danger:
        return "bad"

    # ② ★ 橙色一次只能框一个，且不能和别的混装
    if len(box_orange) > 1:
        return "bad"
    if box_orange and len(box_rescue) > 1:
        return "bad"

    if not green_ever:
        # 第一次送入：框内只能有 1 个绿色
        if not box_green:
            return "empty"
        if len(box_rescue) > 1:
            return "bad"
        return "ok"
    else:
        # 后续：不多于 2 个（黑黑 / 绿黑 / 绿绿 都可以）
        if len(box_rescue) > 2:
            return "bad"
        if not box_rescue:
            return "empty"
        return "ok"

    
def draw_grab_box(pl):
    r   = int(BOX_W / 2)
    cx0 = int(BOX_CX)

    if GRAB_ARC_UP:
        cy0 = int(BOX_CY + BOX_H / 2)      # 直径在底边
        sign = -1                           # 圆弧向上
    else:
        cy0 = int(BOX_CY - BOX_H / 2)
        sign = +1                           # 圆弧向下

    # ① 直径线
    pl.osd_img.draw_line(cx0 - r, cy0, cx0 + r, cy0,
                         color=(255, 255, 0), thickness=2)

    # ② 圆弧（用多段折线拼，共 24 段）
    steps = 24
    prev = None
    for i in range(steps + 1):
        a  = math.pi * i / steps            # 0 ~ π
        px = int(cx0 + r * math.cos(a))
        py = int(cy0 + sign * r * math.sin(a))
        if prev is not None:
            pl.osd_img.draw_line(prev[0], prev[1], px, py,
                                 color=(255, 255, 0), thickness=2)
        prev = (px, py)

    # ③ 到位线：目标中心 cy 跨过这条线 → 停车夹紧
    pl.osd_img.draw_line(0, int(BOX_CY), 639, int(BOX_CY),
                         color=(255, 0, 255), thickness=2)
    # ④ 中心点
    pl.osd_img.draw_rectangle(BOX_CX - 4, BOX_CY - 4, 8, 8,
                              color=(255, 0, 0), thickness=2)

    # ⑤ 参数文字
    tx = int(BOX_CX - BOX_W / 2)
    ty = int(min(BOX_CY - BOX_H / 2, cy0 + sign * r))
    pl.osd_img.draw_string_advanced(tx, max(0, ty - 28), 24,
        "BOX %d,%d %dx%d  半圆%s" % (BOX_CX, BOX_CY, BOX_W, BOX_H,
                                     "上" if GRAB_ARC_UP else "下"),
        color=(255, 255, 0))
    
def draw_verify_ellipse(pl, safe):
    """画 GO_SAFE phase5 的判定椭圆"""
    if safe is None:
        return

    cx, cy, sw, sh = safe
    rx = sw / 2.0 * VERIFY_SHRINK_X
    ry = sh / 2.0 * VERIFY_SHRINK_Y
    if rx < VERIFY_MIN_R: rx = VERIFY_MIN_R
    if ry < VERIFY_MIN_R: ry = VERIFY_MIN_R

    cx_i, cy_i = int(cx), int(cy)
    rx_i, ry_i = int(rx), int(ry)

    steps = 24
    prev = None
    for i in range(steps + 1):
        a  = 2.0 * math.pi * i / steps
        px = cx_i + int(rx_i * math.cos(a))
        py = cy_i + int(ry_i * math.sin(a))
        if prev is not None:
            pl.osd_img.draw_line(prev[0], prev[1], px, py,
                                 color=(255, 0, 255), thickness=3)
        prev = (px, py)

def draw_verify_debug(pl, bbox):
    """GO_SAFE 验证时的调试显示：椭圆 + 卡门口判据线"""
    if bbox is None:
        return
    cx, cy, sw, sh = bbox

    # 判定椭圆（洋红）
    draw_verify_ellipse(pl, bbox)

    # 卡门口判据线（下 1/4 线，青色）
    yl = int(cy + sh * VERIFY_LEFTOVER_FRAC)
    pl.osd_img.draw_line(0, yl, 639, yl, color=(0, 255, 255), thickness=2)

    # 下边沿参考线（红色，原来 vsh/2 的严格线）
    yb = int(cy + sh * 0.5)
    pl.osd_img.draw_line(0, yb, 639, yb, color=(255, 0, 0), thickness=1)

def occluded_bad(d, others):
    """d 是否和别的目标前后重叠（X 高度重合）且这对里含橙/蓝。
       这种撞不开、框不住，直接不作为目标。"""
    ax0, ax1 = d["cx"] - d["w"] / 2, d["cx"] + d["w"] / 2
    for o in others:
        if o is d:
            continue
        bx0, bx1 = o["cx"] - o["w"] / 2, o["cx"] + o["w"] / 2
        gx = max(ax0, bx0) - min(ax1, bx1)
        m = min(d["w"], o["w"])
        if m > 0 and -gx > STICK_X_OVL * m:
            # X 高度重叠 = 前后关系；含橙（必须独占）或蓝（危险物）就撞不开
            if d["cls"] in (ORANGE_CLS, BLUE_CLS) or o["cls"] in (ORANGE_CLS, BLUE_CLS):
                return True
    return False

def pick_target(dets, dangers, green_done, safe):
    cands = [d for d in dets if not in_safe_zone(d["cx"], d["cy"], safe)]
    if not green_done:
        cands = [d for d in cands if d["cls"] == GREEN_CLS]
    if not cands:
        return None

    # ★ 前后重叠且含橙/蓝 → 撞不开、框不住，直接不作为目标
    _all = dets + dangers
    cands = [d for d in cands if not occluded_bad(d, _all)]
    if not cands:
        return None

    # ★ 撞散抛弃名单：优先框"非抛弃位置"的目标，只有没别的目标才框抛弃的
    if abandon_x >= 0 and time.ticks_diff(time.ticks_ms(), abandon_t0) < ABANDON_SKIP_MS:
        fresh = [d for d in cands
                 if (d["cx"] - abandon_x) ** 2 + (d["cy"] - abandon_y) ** 2 >= ABANDON_SKIP_PX ** 2]
        if fresh:
            cands = fresh

    pool = cands + dangers          # 危险物也算"挨着"

    def score(d):
        """综合得分：距离为主，粘连做惩罚。越小越优先"""
        s = car_dist(d)
        if PICK_ISOLATED_ONLY and not is_isolated(d, pool):
            s *= STICK_PENALTY      # 粘连 → 距离放大，排到后面
        return s

    cands.sort(key=lambda d: (
        score(d),                       # ① 综合分（距离 + 粘连惩罚）
        -RESCUE_PRIO[d["cls"]],         # ② 颜色优先级
        -d["score"],                    # ③ 置信度
    ))
    return cands[0]

MATCH_MAX_DIST = 150   # 锁定目标匹配的最大距离（像素），超过视为丢失
def match_locked_target(dets, last_cls, last_tx, last_ty):
    """找和上次锁定目标最近的同类别目标，避免在两个目标间横跳"""
    same = [d for d in dets if d["cls"] == last_cls]
    if not same:
        return None
    same.sort(key=lambda d: (d["cx"] - last_tx) ** 2 + (d["cy"] - last_ty) ** 2)
    best = same[0]
    dist = ((best["cx"] - last_tx) ** 2 + (best["cy"] - last_ty) ** 2) ** 0.5
    if dist > MATCH_MAX_DIST:
        return None
    return best

# ================= 安全区长边角度测量 =================
def _rgb_planes(img):
    sh = img.shape
    if len(sh) == 3 and sh[0] == 3:
        return img[0], img[1], img[2], sh[2], sh[1]
    if len(sh) == 3 and sh[2] >= 3:
        return img[:, :, 0], img[:, :, 1], img[:, :, 2], sh[1], sh[0]
    return None


def frame_sig(img):
    """画面签名：下采样后所有像素 R+G+B 求和。
       用于判断画面是否长时间没变化（卡住）。不需要视野内有检测目标。"""
    pf = _rgb_planes(img)
    if pf is None:
        return None
    R, G, B, W, H = pf
    step = STUCK_SIG_STEP
    s = 0
    for yy in range(0, H, step):
        Rr = R[yy]; Gr = G[yy]; Br = B[yy]
        for xx in range(0, W, step):
            s += Rr[xx] + Gr[xx] + Br[xx]
    return s


def _lsq(xs, ys):
    """最小二乘 y = a*x + b，返回 (a, b)"""
    n = len(xs)
    if n < 2:
        return (None, None)
    sx = 0.0; sy = 0.0; sxx = 0.0; sxy = 0.0
    for i in range(n):
        x = xs[i]; y = ys[i]
        sx += x; sy += y; sxx += x * x; sxy += x * y
    d = n * sxx - sx * sx
    if abs(d) < 1e-6:
        return (None, None)
    a = (n * sxy - sx * sy) / d
    return (a, (sy - a * sx) / float(n))

# ★ 最近一次长边拟合的拐点调试信息（draw_edge_overlay 画两条边用）
_EDGE_DEBUG = {}

def safe_long_edge(img, safe, dbg=False):
    global _EDGE_DEBUG
    _EDGE_DEBUG = {}
    if safe is None:
        return None
    pf = _rgb_planes(img)
    if pf is None:
        return None

    R, G, B, W, H = pf
    sx, sy, sw, sh = safe
    fx = (sx - sw / 2.0) / 2.0
    fy = (sy - sh / 2.0) * 2.0 / 3.0
    fw = sw / 2.0
    fh = sh * 2.0 / 3.0

    padx = 12                      # ★ 8 → 12，多捞几列
    pady = 8
    padb = int(fh * 0.25) + 10     # ★ 0.45 → 0.25，别往下白扫

    x0 = max(0,   int(fx - padx))
    y0 = max(0,   int(fy - pady))
    x1 = min(320, int(fx + fw + padx))
    y1 = min(320, int(fy + fh + padb))
    if x1 - x0 < 12 or y1 - y0 < 8:
        return None

    # ---- ★ 自适应列步长：不管框多小，都保证 ~24 列 ----
    roi_w = x1 - x0
    cs = roi_w // 24
    if cs < 2: cs = 2
    if cs > COL_STEP: cs = COL_STEP
    ncol = (roi_w + cs - 1) // cs
    if ncol < 5:
        return None
    min_cols = ncol // 3
    if min_cols < 5: min_cols = 5

    ybot = [-1] * ncol
    remain = ncol

    # ---- ★ 扫满整个 ROI（不再用固定 MAX_UP），带 stall 早退 ----
    yy   = y1 - 1
    rows = 0
    stall = 0
    while yy >= y0 and rows < MAX_ROWS:
        Rr = R[yy]; Gr = G[yy]; Br = B[yy]
        new = 0
        ci = 0
        for xx in range(x0, x1, cs):
            if ybot[ci] < 0:
                if (Br[xx] - Gr[xx] > PUR_DG) and \
                   (Rr[xx] - Gr[xx] > PUR_DR) and (Br[xx] > PUR_BMIN):
                    ybot[ci] = yy
                    remain -= 1
                    new += 1
            ci += 1
        if remain <= 0:
            break
        if new:
            stall = 0
        else:
            stall += 1
            if stall > STALL_MAX:
                break
        yy -= ROW_STEP
        rows += 1

    xs = []; ys = []
    ci = 0
    for xx in range(x0, x1, cs):
        if ybot[ci] >= 0:
            xs.append(xx); ys.append(ybot[ci])
        ci += 1

    # ★ 记录扫描点（供 draw_edge_overlay 画散点诊断有没有扫到 V 字）
    _EDGE_DEBUG["xs"] = xs
    _EDGE_DEBUG["ys"] = ys

    if dbg:
        print("  fit: roi=%dx%d cs=%d ncol=%d hits=%d rows=%d"
              % (roi_w, y1 - y0, cs, ncol, len(xs), rows))

    if len(xs) < min_cols:
        return None

    # ---- 先整体拟合，再用分段拟合找拐点（面对角"V"字）----
    a, b = _lsq(xs, ys)
    if a is None:
        return None
    n = len(xs)

    # 整体拟合总残差（平方和）
    tot0 = 0.0
    for i in range(n):
        e = ys[i] - (a * xs[i] + b)
        tot0 += e * e

    # 分段拟合：对每个切分点 k（每段至少 2 点），左右各拟合，找总残差最小的 k
    best_k = -1
    best_tot = tot0
    best_aL = best_bL = best_aR = best_bR = 0.0
    for k in range(2, n - 1):
        aL, bL = _lsq(xs[:k], ys[:k])
        aR, bR = _lsq(xs[k:], ys[k:])
        if aL is None or aR is None:
            continue
        tot = 0.0
        for i in range(k):
            e = ys[i] - (aL * xs[i] + bL)
            tot += e * e
        for i in range(k, n):
            e = ys[i] - (aR * xs[i] + bR)
            tot += e * e
        if tot < best_tot:
            best_tot = tot
            best_k = k
            best_aL, best_bL, best_aR, best_bR = aL, bL, aR, bR

    # ---- 面对角：分段拟合显著优于整体拟合 → 切两段，取 |斜率| 小段 = 长边 ----
    seg_xs = xs
    seg_ys = ys
    ratio = (best_tot / tot0) if tot0 > 0 else 1.0
    if best_k >= 0 and ratio < SPLIT_GAIN:
        lx, ly = xs[:best_k], ys[:best_k]
        rx, ry = xs[best_k:], ys[best_k:]
        aL, bL = best_aL, best_bL
        aR, bR = best_aR, best_bR
        chosen = "L" if abs(aL) <= abs(aR) else "R"
        if chosen == "L":
            a, b = aL, bL
            seg_xs, seg_ys = lx, ly
        else:
            a, b = aR, bR
            seg_xs, seg_ys = rx, ry
        # ★ 记录拐点调试信息（frame 坐标），供 draw_edge_overlay 画两条边
        _EDGE_DEBUG = {
            "split": True,
            "k": best_k, "ratio": ratio,
            "kp": (xs[best_k], ys[best_k]),
            "segL": ((lx[0], aL*lx[0]+bL), (lx[-1], aL*lx[-1]+bL)),
            "segR": ((rx[0], aR*rx[0]+bR), (rx[-1], aR*rx[-1]+bR)),
            "chosen": chosen,
        }
        if dbg:
            print("  split: k=%d ratio=%.2f aL=%.3f aR=%.3f -> %s"
                  % (best_k, ratio, aL, aR, "左段" if chosen == "L" else "右段"))
    elif dbg:
        print("  no split: tot0=%.0f best_tot=%.0f ratio=%.2f k=%d"
              % (tot0, best_tot, ratio, best_k))

    # ---- 对选中的那段做剔离群点（剔两轮，比原来更稳）----
    xs2 = seg_xs
    ys2 = seg_ys
    for _ in range(2):
        xs2n = []; ys2n = []
        for i in range(len(xs2)):
            if abs(ys2[i] - (a * xs2[i] + b)) < FIT_TOL:
                xs2n.append(xs2[i])
                ys2n.append(ys2[i])
        if len(xs2n) < min_cols:
            break
        a2, b2 = _lsq(xs2n, ys2n)
        if a2 is None:
            break
        a, b = a2, b2
        xs2, ys2 = xs2n, ys2n

    th = math.degrees(math.atan(a))
    while th >   90: th -= 180
    while th <= -90: th += 180

    p1 = (x0, a * x0 + b)
    p2 = (x1, a * x1 + b)
    return (th, p1, p2, len(xs2))

# ================= SHIFT：图像直线 → 地面几何 =================
def shift_geom(a, b):
    """由拟合直线 y = a*x + b（320×320 缓冲坐标）反解：
         · θ  = 安全区长边相对车轴的真实偏角（度，带符号）
         · Zq = 车到「近长边」在中心列上的距离（米）
       返回 (θ, Zq) 或 None（拟合不可靠）。

       推导（小孔成像 + 地面平面）：
         ① 中心列 x=160 ↔ 车轴正前方，反解距离  Zq = FH / (1.5*y_c - YH)。
         ② 4:3 压扁 → 方形：display 斜率 = (1.5/2)·|a|，即 tan(TH_disp)=0.75·|a|。
         ③ 透视还原：tanθ = (Zq / H) · tan(TH_disp)。
    """
    y_c = (a * 160.0 + b) * 1.5          # 中心列上的 y（display 坐标）
    dz  = y_c - SH_YH
    if dz < SH_SY_MIN:                    # 线跑到地平线附近 → 不可用
        return None

    Zq = SH_FH / dz                       # 车 → 近长边（中心列）距离，米

    t_disp  = SH_ASPECT * abs(a)          # 去各向异性：tan(TH_display)
    t_theta = t_disp * Zq / SH_CAM_H      # tanθ
    # ★ 原 1.2(=tan50°) 太严：车在安全区侧面 θ 常 50~70° 是正常要横移的角度，
    #   被这里拦成 None → 0 样本 → SHIFT 直接跳过进 GO_SAFE（没横移就送）。
    #   放宽到与 SH_MAX_TH 对齐，让 θ∈(50,70] 能正常采样，只有 θ>SH_MAX_TH 才拦。
    if t_theta > math.tan(math.radians(SH_MAX_TH)):
        return None

    th = math.degrees(math.atan(t_theta))
    if a < 0:
        th = -th
    return th, Zq


def shift_S(th, Zq):
    """横移量 S（米）= 车轴到「安全区中线」的垂直距离。
       S = Zq·sinθ + (W/2)·tanθ；θ→0 时退化为 S ≈ (Zq + W/2)·θ。"""
    r = math.radians(abs(th))
    return Zq * math.sin(r) + 0.5 * SH_SAFE_W * math.tan(r)

def draw_edge_overlay(pl, safe, g):
    """画：安全区框(青) + 测量ROI(黄) + 拟合长边(绿) + 数值"""
    if safe is None:
        pl.osd_img.draw_string_advanced(8, 440, 24, "NO SAFE", color=(255, 0, 0))
        return

    sx, sy, sw, sh = safe

    # ① 安全区检测框（青）
    pl.osd_img.draw_rectangle(int(sx - sw / 2), int(sy - sh / 2),
                              int(sw), int(sh),
                              color=(0, 255, 255), thickness=2)

    # ② 测量 ROI（黄）—— 必须和 safe_long_edge 里的算法一模一样
    fx = (sx - sw / 2.0) / 2.0
    fy = (sy - sh / 2.0) * 2.0 / 3.0
    fw = sw / 2.0
    fh = sh * 2.0 / 3.0
    padx = 12
    pady = 8
    padb = int(fh * 0.25) + 10
    x0 = max(0,   int(fx - padx))
    y0 = max(0,   int(fy - pady))
    x1 = min(320, int(fx + fw + padx))
    y1 = min(320, int(fy + fh + padb))

    # frame -> display:  x*2, y*1.5
    pl.osd_img.draw_rectangle(int(x0 * 2), int(y0 * 1.5),
                              int((x1 - x0) * 2), int((y1 - y0) * 1.5),
                              color=(255, 255, 0), thickness=2)

    # ③ 拟合直线（绿）
    if g is None:
        pl.osd_img.draw_string_advanced(8, 440, 24,
            "TH=None  ROI=%dx%d" % (x1 - x0, y1 - y0), color=(255, 0, 0))
        return

    th, p1, p2, ncol = g
    ax = int(p1[0] * 2); ay = int(p1[1] * 1.5)
    bx = int(p2[0] * 2); by = int(p2[1] * 1.5)
    pl.osd_img.draw_line(ax, ay, bx, by, color=(0, 255, 0), thickness=3)

    # ★ 画扫描点散点（白点，诊断有没有扫到 V 字）
    xs_pts = _EDGE_DEBUG.get("xs")
    ys_pts = _EDGE_DEBUG.get("ys")
    if xs_pts:
        for i in range(len(xs_pts)):
            pl.osd_img.draw_rectangle(int(xs_pts[i] * 2) - 1, int(ys_pts[i] * 1.5) - 1, 3, 3,
                                      color=(255, 255, 255), thickness=1)

    # ★ 面对角"V"字时：画另一条边(橙) + 拐点(红方点)
    if _EDGE_DEBUG and _EDGE_DEBUG.get("split"):
        other = _EDGE_DEBUG["segR"] if _EDGE_DEBUG["chosen"] == "L" else _EDGE_DEBUG["segL"]
        ox1 = int(other[0][0] * 2); oy1 = int(other[0][1] * 1.5)
        ox2 = int(other[1][0] * 2); oy2 = int(other[1][1] * 1.5)
        pl.osd_img.draw_line(ox1, oy1, ox2, oy2, color=(255, 165, 0), thickness=2)
        kp = _EDGE_DEBUG["kp"]
        pl.osd_img.draw_rectangle(int(kp[0] * 2) - 4, int(kp[1] * 1.5) - 4, 8, 8,
                                  color=(255, 0, 0), thickness=2)

    pl.osd_img.draw_string_advanced(8, 440, 24,
        "TH=%.1f  cols=%d  ROI=%dx%d" % (th, ncol, x1 - x0, y1 - y0),
        color=(0, 255, 0))

# ================= 主程序 =================
if __name__ == "__main__":
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    ob_det = ObjectDetectionApp(
        kmodel_path,
        labels=labels,
        model_input_size=model_input_size,
        max_boxes_num=50,
        confidence_threshold=CONF_NORMAL,
        nms_threshold=0.45,
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=0)
    ob_det.config_preprocess()
    
    # 状态机变量
    safe = None
    last_tx = last_ty = 0
    last_sx = last_sy = 0
    last_sw = last_sh = 0       # ★ 最后已知安全区的宽高（闪烁时验证用）
    nav_sx  = nav_sy  = 0
    lost_count = 0
    check_frames = check_bad = check_empty = 0
    check_ok     = 0        # ★ 累计 ok 帧数
    scatter_idx  = 0       # ★ 撞散动作表执行到第几步
    apart_idx    = 0       # ★ 分离动作执行到第几步
    verify_frames = 0     # ★ 新增
    verify_in     = 0     # ★ 本次搬的目标(last_cls)在内切椭圆内的帧数
    verify_seen   = 0     # ★ 新增
    leftover_frames = 0   # ★ 目标卡在安全区外（没推进去）的帧数
    safe_retry    = 0     # ★ 补推次数
    regrab_after_safe = False  # ★ 补抓：GO_SAFE 卡门口→跳 GRAB 重抓，抓完回 GO_SAFE 再送
    safe_back_cnt = 0     # ★ 离太近时后退次数
    no_green_cnt  = 0     # ★ 绿色保底：连续"左半没绿+带非绿"的帧数
    target_hits = 0       # ★ 连续候选帧数
    grab_lost   = 0       # ★ GRAB：连续看不到目标的帧数
    grab_stuck  = 0       # ★ GRAB：目标可见但未到位的帧数
    track_aligning = False # ★ TRACK：追边缘目标时正在原地转圈摆正
    track_align_t0 = 0
    track_turn_cmd = 0     # ★ 原地转向命令
    track_turn_deg = 0     # ★ 原地转向角度
    cand_tx = cand_ty = 0 # ★ 上一帧候选位置（同一目标校验用）
    smash_idx = 0        # SMASH 已撞次数
    smash_total_t0 = 0   # ★ SMASH 本次撞散的总时长起点（超时放弃用）
    abandon_x = -1       # ★ 撞散抛弃的目标位置（-1 = 无抛弃）
    abandon_y = -1
    abandon_t0 = 0       # ★ 抛弃时间（配合 ABANDON_SKIP_MS 用）
    ax_iter = 0          # ALIGN_X：已转次数
    ax_lost = 0          # ALIGN_X：连续丢失帧数
    ap_lost = 0          # APPROACH：连续丢帧数
    ap_fallback = False  # ★ APPROACH 是否是"对方区补救"（终点用 AP_Y_NEAR_LOOSE）
    ap_own_cnt = 0       # ★ 补救途中连续判成"自己区"的帧数
    safe_cls = -1        # 当前锁定安全区的类别（蓝/红），-1 = 没看到
    ax_cmd  = 0          # ALIGN_X：本次转向命令
    sh_c1 = 0            # SHIFT：第一次转向命令
    sh_c2 = 0            # SHIFT：转回 90° 的命令
    _SH   = []           # SHIFT：(θ, Zq, 颜色) 采样缓冲
    sh_d1 = 0            # SHIFT：第 1 次转角
    sh_w1 = 0            # SHIFT：第 1 次要等多久（ms）
    sh_w2 = 0            # SHIFT：第 2 次要等多久（ms）
    sh_dms = 0           # SHIFT：横移直行时长（ms，由 S 换算）
    sh_theta = 0.0       # SHIFT：真实偏角 θ（显示用）
    sh_S     = 0.0       # SHIFT：横移距离 S（米，显示用）
    sh_flip  = False     # SHIFT：对齐的是对方区 → 最后反向转 90°（掉头）

    if TEST_MODE:
        state      = START_STATE
        last_cls   = START_CLS
        green_done = False            # 测试时追任意颜色，不受"只运绿"限制
        green_ever = False    # ★ 是否曾经成功送入过绿色（只置 True，永不回退）
        multi_hits = 0        # ★ 一次多搬：连续满足帧数
        multi_mode = False    # ★ 本次 GRAB 是否为多目标合搬
        phase = 0
        t0 = time.ticks_ms()
    else:
        last_cls   = 0
        green_done = False
        green_ever = False    # ★ 是否曾经成功送入过绿色（只置 True，永不回退）
        multi_hits = 0        # ★ 一次多搬：连续满足帧数
        multi_mode = False    # ★ 本次 GRAB 是否为多目标合搬
        phase      = 0
        t0         = time.ticks_ms()
        state      = "SCATTER_INTRO" if SCATTER_ENABLE else "SEARCH"

    _EH  = []
    _prev_sig       = None      # 上一帧画面签名（卡住检测）
    _stuck_t0       = time.ticks_ms()
    stuck_recovering = False    # 正在抬框后退恢复中
    stuck_rec_t0    = 0
    opp_avoiding    = False     # 正在躲开对方区（后退 + 左转100°）
    opp_avoid_t0    = 0

    while True:
        try:
            img = pl.get_frame()
            if state in FIND_STATES:
                ob_det.confidence_threshold = CONF_FIND
            elif state in SAFE_STATES:
                ob_det.confidence_threshold = CONF_SAFE
            else:
                ob_det.confidence_threshold = CONF_NORMAL
            res = ob_det.run(img)
            dets, dangers, safes = parse_detections(res)
            raw_safes = safes        # 未过滤的完整安全区列表（供"离对方区太近"全局检查用）

            # ================= 安全区选优 =================
            safe = None
            safe_cls = -1
            if safes:
                # 按颜色过滤：GO_SAFE 只要自己区；FIND_SAFE phase0 只要自己区、
                # phase1 只要对方区；其余（approach/align/shift）蓝红通吃
                if SAFE_USE is not None:
                    if state == "GO_SAFE":
                        safes = [s for s in safes if s["cls"] == SAFE_USE]
                    elif state == "FIND_SAFE" and phase == 0:
                        safes = [s for s in safes if s["cls"] == SAFE_USE]
                    elif state == "FIND_SAFE" and phase == 1:
                        safes = [s for s in safes if s["cls"] != SAFE_USE]
                if safes:
                    safes.sort(key=lambda s: -s["score"])
                    safe = (int(safes[0]["cx"]), int(safes[0]["cy"]),
                            int(safes[0]["w"]),  int(safes[0]["h"]))
                    safe_cls = safes[0]["cls"]

            now = time.ticks_ms()
            sx = safe[0] if safe else 0
            sy = safe[1] if safe else 0
            if safe is not None:
                last_sx, last_sy = sx, sy      # 只更新"最后已知位置"
                last_sw, last_sh = safe[2], safe[3]
            ack = read_ack()

            # ★ 全局长边测量：任何状态都测，供画图 / SHIFT 复用
            _g = None
            _theta_show = None
            if (EDGE_DRAW or SHOW_THETA) and safe is not None:
                _g = safe_long_edge(img, safe, SH_EDGE_DBG)
                if _g is not None:
                    a = (_g[2][1] - _g[1][1]) / float(_g[2][0] - _g[1][0])
                    b = _g[1][1] - a * _g[1][0]
                    geom = shift_geom(a, b)
                    if geom is not None:
                        _theta_show = geom[0] - TH_ZERO   # 显示减掉 TH_ZERO 后的有效 θ

            if DEBUG:
                log("state=%s dets=%s dangers=%s safe=%s" % (
                    state,
                    [(d["cls"], int(d["cx"]), int(d["cy"])) for d in dets],
                    [(d["cls"], int(d["cx"]), int(d["cy"])) for d in dangers],
                    safe))

            # ============ 只测几何，不控制（SHIFT 标定台） ============
            if EDGE_ONLY:
                send_frame(CMD_STOP, 0, 0, 0, 0, 0)
                ob_det.draw_result(pl, res)

                g = safe_long_edge(img, safe, SH_EDGE_DBG)
                th_img = th_med = None
                Zq = S = None
                if g is not None:
                    th_img = g[0]
                    a = (g[2][1] - g[1][1]) / float(g[2][0] - g[1][0])
                    b = g[1][1] - a * g[1][0]
                    geom = shift_geom(a, b)
                    if geom is not None:
                        th, Zq = geom
                        S = shift_S(th - TH_ZERO, Zq)   # 用修正后的 θ 算 S
                        _EH.append(th)
                        while len(_EH) > MED_N:
                            _EH.pop(0)
                        th_med = sorted(_EH)[len(_EH)//2]
                    else:
                        _EH = []
                else:
                    _EH = []

                if safe is not None:
                    draw_edge_overlay(pl, safe, g)   # ★ ROI黄框 + 拟合长边(绿) + 拐点两条边(橙)

                t1 = ("%.1f" % th_img) if th_img is not None else " -- "
                t2 = ("%.1f" % th_med) if th_med is not None else " -- "
                t2c = ("%.1f" % (th_med - TH_ZERO)) if th_med is not None else " -- "
                z1 = ("%.2f" % Zq) if Zq is not None else " -- "
                s1 = ("%.3f" % S) if S is not None else " -- "
                sy_s = safe[1] if safe is not None else 0
                area_s = safe[2] * safe[3] if safe is not None else 0

                pl.osd_img.draw_string_advanced(8, 8, 26,
                    "THimg=%s  theta=%s -> %s" % (t1, t2, t2c), color=(0, 255, 0))
                pl.osd_img.draw_string_advanced(8, 44, 26,
                    "Zq=%s m  S=%s m" % (z1, s1), color=(255, 255, 0))
                pl.osd_img.draw_string_advanced(8, 80, 26,
                    "sy=%d  area=%d" % (sy_s, area_s), color=(255, 255, 0))
                pl.osd_img.draw_string_advanced(8, 116, 26,
                    "YH=%.0f  FH=%.0f  H=%.2f  W=%.2f"
                    % (SH_YH, SH_FH, SH_CAM_H, SH_SAFE_W), color=(255, 255, 0))

                pl.show_image()
                gc.collect()
                continue

            # ================= 全局卡住检测（任何状态都跑） =================
            # 故意停住的对齐/测量/验证阶段不参与计时，否则会被误判成卡住
            _waiting = (state == "ALIGN_X" or state == "CHECK" or
                        (state == "SHIFT" and phase == 0) or
                        (state == "GO_SAFE" and phase == 5))
            if _waiting:
                _stuck_t0 = now
                _prev_sig = None
            else:
                sig = frame_sig(img)
                if sig is not None:
                    if _prev_sig is None or abs(sig - _prev_sig) > STUCK_SIG_TH:
                        _stuck_t0 = now                 # 画面有变化 → 重置计时
                    _prev_sig = sig
                if not stuck_recovering and not opp_avoiding and \
                   time.ticks_diff(now, _stuck_t0) > STUCK_TIMEOUT_MS:
                    stuck_recovering = True
                    stuck_rec_t0 = now
                    _stuck_t0 = now
                    log(">>> 检测到卡住（画面 %dms 没变化）→ 抬框 + 后退" % STUCK_TIMEOUT_MS)

            # ================= 全局：离对方安全区太近 → 后退 + 左转100° =================
            # ★ 找/送安全区阶段豁免：这些状态本来就在接近安全区，对方区近是正常场地布局，躲了反而送不进
            _approaching_safe = (state in ("FIND_SAFE", "APPROACH_SAFE", "ALIGN_X",
                                           "SHIFT", "TO_OWN_SAFE", "GO_SAFE"))
            if (not opp_avoiding and not stuck_recovering and not _approaching_safe and
                    SAFE_USE is not None and raw_safes):
                opp_safes = [s for s in raw_safes if s["cls"] != SAFE_USE]
                if opp_safes:
                    opp_safes.sort(key=lambda s: -s["cy"])   # 最近的对方区
                    opp_sy = int(opp_safes[0]["cy"])
                    if opp_sy > OPP_TOO_CLOSE_Y:
                        opp_avoiding = True
                        opp_avoid_t0 = now
                        flush_rx()
                        log(">>> 离对方区太近(sy=%d>%d) → 后退 + 左转%d°"
                            % (opp_sy, OPP_TOO_CLOSE_Y, OPP_AVOID_TURN_DEG))

            # ================= 开局动作 =================

            # ---------- 躲开对方区：后退 → 左转100° ----------
            if opp_avoiding:
                dt = time.ticks_diff(now, opp_avoid_t0)
                if dt < OPP_AVOID_BACK_MS:
                    send_frame(CMD_BACK, 0, 0, 0, 0, 0)
                elif dt < OPP_AVOID_BACK_MS + OPP_AVOID_TURN_MS:
                    send_frame(CMD_TURN_L, OPP_AVOID_TURN_DEG, 0, 0, sx, sy)
                else:
                    opp_avoiding = False
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    flush_rx()
                    state = "SEARCH"; phase = 0; t0 = now
                    log(">>> 躲开对方区完成 → SEARCH")

            # ---------- 卡住恢复：抬框 + 后退 ----------
            elif stuck_recovering:
                send_frame(CMD_OPEN, 0, 0, 0, 0, 0)
                send_frame(CMD_BACK, 0, 0, 0, 0, 0)
                if time.ticks_diff(now, stuck_rec_t0) > STUCK_RECOVER_MS:
                    stuck_recovering = False
                    send_frame(CMD_STOP, 0, 0, 0, 0, 0)
                    flush_rx()
                    state = "SEARCH"; phase = 0; t0 = now
                    log(">>> 卡住恢复完成 → SEARCH")

            # ---------- ⓿ 开局：左转 45° → 直行 3s → 把框打下来 ----------
            elif state == "SCATTER_INTRO":
                dt = time.ticks_diff(now, t0)

                # ① 左转 45°（每帧重发，喂 STM32 看门狗）
                if phase == 0:
                    send_frame(CMD_TURN_L, INTRO_TURN_DEG, 0, 0, sx, sy)
                    if dt > INTRO_TURN_WAIT:
                        if ack != CMD_TURN_L:
                            log(">>> 开局转向 ack 超时，按 %dms 强制继续"
                                % INTRO_TURN_WAIT)
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)   # 清 last_cmd
                        flush_rx()
                        phase = 1; t0 = now
                        log(">>> 开局：左转 %d° 完成 (耗时 %dms) → 直行 %dms"
                            % (INTRO_TURN_DEG, dt, INTRO_STRAIGHT_MS))

                # ② 直行（此时框还是抬起的，靠这个离开出发点）
                elif phase == 1:
                    send_frame(CMD_STRAIGHT_FAST, 0, 0, 0, sx, sy)   # ★ 高速
                    if dt > INTRO_STRAIGHT_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        phase = 2; t0 = now
                        log(">>> 开局：高速直行 %dms 完成 → 把框打下来"
                            % INTRO_STRAIGHT_MS)

                # ③ ★ 把框打下来（夹爪闭合，恢复推板姿态）
                elif phase == 2:
                    send_frame(CMD_CAM, CAM_UP, 0, 0, sx, sy)   # ← 新增这行
                    send_frame(CMD_GRAB, 0, 0, 0, sx, sy)
                    if dt > INTRO_GRAB_MS:
                        if ack != CMD_GRAB:
                            log(">>> 收爪 ack 超时，按 %dms 强制继续"
                                % INTRO_GRAB_MS)
                        flush_rx()
                        state = "SCATTER_TRACK"; phase = 0; t0 = now
                        log(">>> 开局动作完成（框已打下）→ 开始追踪最近目标")

            # ================= 开局撞散 =================

           # ---------- ① 追踪最近的目标 ----------
            elif state == "SCATTER_TRACK":
                # 离车最近的目标（偏爱正前方）
                t = min(dets, key=car_dist) if dets else None
                
                if t is None:
                    # 看不到目标：原地转，边转边找
                    send_frame(CMD_ROT_LEFT, 0, 0, 0, sx, sy)
                    if time.ticks_diff(now, t0) > SCATTER_FIND_TIMEOUT:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        log(">>> 撞散：找不到目标，直接进 SEARCH")
                        state = "SEARCH"; phase = 0; t0 = now
                else:
                    last_cls = t["cls"]
                    tx_i, ty_i = int(t["cx"]), int(t["cy"])

                    # 边走边对准（STM32 的视觉追踪）
                    send_frame(CMD_FORWARD, t["cls"], tx_i, ty_i, sx, sy)

                    # 够近 → 进下一步
                    if t["cy"] > SCATTER_NEAR_Y:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        state = "SCATTER_RUN"
                        scatter_idx = 0; phase = 0; t0 = now
                        log(">>> 撞散：开始三段撞击")

            # ---------- 撞散动作序列 ----------
            elif state == "SCATTER_RUN":
                dt = time.ticks_diff(now, t0)

                # 全部做完 → 进 SEARCH
                if scatter_idx >= len(SCATTER_STEPS):
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    state = "SEARCH"; phase = 0; t0 = now
                    scatter_idx = 0
                    log(">>> 撞散完成，进入 SEARCH")

                else:
                    name, param = SCATTER_STEPS[scatter_idx]

                    # ---------- 直行撞击 ----------
                    if name == "PUSH" or name == "HIT":
                        # ★ HIT 用高速；PUSH 保持常规（PUSH 在 SCATTER_STEPS 里目前没有，留着防以后加）
                        if name == "HIT":
                            send_frame(CMD_STRAIGHT_FAST, 0, 0, 0, sx, sy)
                        else:
                            send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)
                        if dt > param:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            log(">>> 撞散：第 %d 步(%s)完成" % (scatter_idx + 1, name))
                            scatter_idx += 1; phase = 0; t0 = now
                    # ---------- 后退 ----------
                    elif name == "BACK":
                        send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                        if dt > param:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            scatter_idx += 1; phase = 0; t0 = now

                    # ---------- 指定角度转向（必须插 STOP 清 last_cmd） ----------
                    elif name == "TURN_L" or name == "TURN_R":
                        cmd = CMD_TURN_L if name == "TURN_L" else CMD_TURN_R

                        if phase == 0:
                            send_frame(cmd, int(param), 0, 0, sx, sy)
                            if dt > TURN_DEG_WAIT_MS:
                                send_frame(CMD_STOP, 0, 0, 0, sx, sy)  # ★ 清 last_cmd
                                phase = 1; t0 = now
                        else:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            if dt > TURN_ENTRY_GAP_MS:
                                log(">>> 撞散：转 %d° 完成" % int(param))
                                scatter_idx += 1; phase = 0; t0 = now

            # ---------- 状态机 ----------
            elif state == "SEARCH":
                dt = time.ticks_diff(now, t0)

                # ---------- ★ ① 进门A：抬头（单独占一个 phase，绝不和收爪同帧） ----------
                if phase == 0:
                    send_frame(CMD_CAM, CAM_UP, 0, 0, sx, sy)
                    if dt > SEARCH_CAM_MS:
                        flush_rx()
                        phase = 1; t0 = now

                # ---------- ★ ② 进门B：收爪 ----------
                elif phase == 1:
                    send_frame(CMD_GRAB, last_cls, 0, 0, sx, sy)
                    if dt > SEARCH_GRAB_MS:
                        flush_rx()
                        phase = 2; t0 = now

                # ---------- ③ 连续转一整圈 ----------
                elif phase == 2:
                    send_frame(CMD_SPIN, 0, 0, 0, sx, sy)
                    if dt > SPIN_WAIT_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        phase = 3; t0 = now

                # ---------- ④ 转完没找到 → 直行一段，再扫 ----------
                else:
                    send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)
                    if dt > SEARCH_STRAIGHT_MS:
                        phase = 2; t0 = now
                    
                # ---------- ⑤ 锁定判定（转圈 / 直行 都判） ----------
                if phase >= 2:
                    if len(dets) < MIN_TARGETS:
                        target_hits = 0
                    else:
                        t = pick_target(dets, dangers, green_done, safe)
                        if t is None:
                            target_hits = 0
                        else:
                            tx_i, ty_i = int(t["cx"]), int(t["cy"])
                            # 必须是"同一个目标"才算连续（转圈时靠这个拦掉串位）
                            if target_hits > 0 and \
                               abs(tx_i - cand_tx) + abs(ty_i - cand_ty) < LOCK_DRIFT_PX:
                                target_hits += 1
                            else:
                                target_hits = 1
                            cand_tx, cand_ty = tx_i, ty_i

                            if target_hits >= TARGET_CONFIRM_FRAMES:
                                last_cls = t["cls"]
                                last_tx, last_ty = tx_i, ty_i
                                target_hits = 0
                                state = "TRACK"; phase = 0; t0 = now
                                log(">>> 锁定 cls=%d h=%d d=%.0f"
                                    % (last_cls, int(t["h"]), car_dist(t)))
                    
            elif state == "TRACK":
                t = match_locked_target(dets, last_cls, last_tx, last_ty)

                # ★ 目标跑进安全区 → 立刻放弃，不再追
                if t is not None and in_safe_zone(t["cx"], t["cy"], safe):
                    log(">>> 锁定目标已进入安全区(cx=%d, cy=%d)，放弃"
                        % (int(t["cx"]), int(t["cy"])))
                    state = "SEARCH"; phase = 0; t0 = now
                    lost_count = 0
                    target_hits = 0
                    multi_hits  = 0            # ★ 新增
                    regrab_after_safe = False

                elif t is None:
                    lost_count += 1
                    if lost_count > 20:
                        state = "SEARCH"
                        phase = 0; t0 = now
                        lost_count = 0
                        multi_hits  = 0            # ★ 新增
                        regrab_after_safe = False
                    else:
                        send_frame(CMD_FORWARD, last_cls, last_tx, last_ty, sx, sy)
                else:
                    lost_count = 0
                    last_tx, last_ty = int(t["cx"]), int(t["cy"])

                    # ★ 原地转摆正：① 视野边缘(很偏，不管远近) ② 够近但没对准
                    if abs(t["cx"] - 320) > EDGE_TOL or \
                       (t["cy"] > TRACK_ALIGN_Y and abs(t["cx"] - 320) > TRACK_ALIGN_TOL):
                        if not track_aligning:
                            track_aligning = True
                            track_align_t0 = now
                            track_turn_cmd = CMD_TURN_L if t["cx"] < 320 else CMD_TURN_R
                            track_turn_deg = int(abs(t["cx"] - 320) * EDGE_GAIN)
                            if track_turn_deg < 10: track_turn_deg = 10
                            if track_turn_deg > 45: track_turn_deg = 45
                            send_frame(track_turn_cmd, track_turn_deg, 0, 0, sx, sy)
                            log(">>> 目标偏(cx=%d) → 原地转 %s %d° 摆正"
                                % (int(t["cx"]), "L" if track_turn_cmd == CMD_TURN_L else "R", track_turn_deg))
                        else:
                            # ★ 已对准（|cx-320| <= TRACK_ALIGN_TOL）→ 停，下一帧走正常前进追
                            if abs(t["cx"] - 320) <= TRACK_ALIGN_TOL:
                                track_aligning = False
                                send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                                flush_rx()
                                log(">>> 已对准 cx=%d → 继续前进追" % int(t["cx"]))
                            elif time.ticks_diff(now, track_align_t0) > EDGE_TURN_WAIT:
                                # ★ 超时还没对准 → 也停，防死转
                                track_aligning = False
                                send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                                flush_rx()
                                log(">>> 原地对准超时 cx=%d → 继续前进追" % int(t["cx"]))
                            else:
                                send_frame(track_turn_cmd, track_turn_deg, 0, 0, sx, sy)  # 重发喂看门狗
                    else:
                        track_aligning = False
                        # ★ 目标是不是"挤在一堆里"
                        in_cluster = SMASH_ENABLE and \
                                     not is_isolated(t, dets + dangers, SMASH_STICK_TH)

                        # ★ 在簇里 → 追踪点换成"簇质心"
                        aim_x, aim_y = last_tx, last_ty
                        cl = None
                        if in_cluster:
                            cl = cluster_of(t, dets + dangers, SMASH_STICK_TH)
                            # ★ 开局（绿还没送成）只瞄绿色本身，不瞄整个簇质心：
                            #   否则绿簇和旁边无绿的簇被粘连成一簇时，质心会漂到无绿那侧，
                            #   车就去撞没有绿色的簇了。
                            if green_done:
                                cc = cluster_center(cl)
                                if cc is not None:
                                    aim_x, aim_y = int(cc[0]), int(cc[1])

                        # ★ 一次多搬判据：连续 MULTI_CONFIRM 帧一致才生效
                        if green_ever and in_cluster and can_cograb(cl):
                            multi_hits += 1
                        else:
                            multi_hits = 0
                        cograb = (multi_hits >= MULTI_CONFIRM)

                        send_frame(CMD_FORWARD, t["cls"], aim_x, aim_y, sx, sy)

                        if t["cy"] > Y_NEAR_TH:
                            flush_rx()

                            if in_cluster and not cograb:
                                # ---------- 原 SMASH 逻辑 ----------
                                # 必须对准簇质心才允许撞，否则继续用视觉修正
                                if abs(aim_x - 320) <= SMASH_ALIGN_TOL:
                                    state = "SMASH"; phase = 0; t0 = now
                                    smash_idx = 0
                                    regrab_after_safe = False
                                    log(">>> 簇(%d 个)已对准 cx=%d → SMASH"
                                        % (len(cl), aim_x))
                                # 太近了还没对准 → 也强制撞（防止顶着目标推）
                                elif t["cy"] > Y_NEAR_TH + SMASH_FORCE_PUSH_DY:
                                    state = "SMASH"; phase = 0; t0 = now
                                    smash_idx = 0
                                    regrab_after_safe = False
                                    log(">>> 簇没对准但已很近 cx=%d cy=%d → 强制 SMASH"
                                        % (aim_x, int(t["cy"])))
                            else:
                                # ---------- 孤立目标 或 合搬：X 摆正了才抓，否则继续追着对准 ----------
                                if abs(aim_x - 320) <= GRAB_ALIGN_TOL:
                                    state = "GRAB"; phase = 0; t0 = now
                                    grab_lost = grab_stuck = 0
                                    multi_mode = cograb
                                    if cograb:
                                        log(">>> 簇(%d 个，全绿/黑) → 不撞散，一起抓"
                                            % len(cl))
                                    else:
                                        log(">>> 孤立目标（cls=%d h=%d）→ 抓取"
                                            % (t["cls"], int(t["h"])))
                                # else: 还没摆正，本帧已发 CMD_FORWARD 继续追踪对准，下一帧再判
                        
            # ---------- 簇目标撞散（55° 视角） ----------
            elif state == "SMASH":
                dt = time.ticks_diff(now, t0)

                # ① 摄像头切到 55°
                if phase == 0:
                    send_frame(CMD_CAM, CAM_NEAR, 0, 0, sx, sy)
                    if dt > SMASH_CAM_SETTLE_MS:
                        flush_rx()
                        smash_idx = 0
                        smash_total_t0 = now
                        phase = 1; t0 = now
                        log(">>> SMASH：55° 就位，开始第 1 次撞击")

                # ② 前进撞击，每帧检查簇是否散开
                elif phase == 1:
                    send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)

                    pool = dets + dangers
                    cls_list = split_clusters(pool, SMASH_STICK_TH)
                    max_sz = max([len(c) for c in cls_list]) if cls_list else 0

                    if max_sz <= 1:
                        # ★ 已散开（没有任何粘连对）
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        phase = 3; t0 = now
                        log(">>> SMASH：簇已散开（最大簇=%d，检测到 %d 个目标）"
                            % (max_sz, len(pool)))

                    elif dt > SMASH_PUSH_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        smash_idx += 1
                        # ★ 撞散计次：撞满 SMASH_MAX_TRIES 或总时长超时 → 抛弃这俩，
                        #   记下位置，一段时间内不框它们（先去找别的目标）
                        if smash_idx >= SMASH_MAX_TRIES or \
                           time.ticks_diff(now, smash_total_t0) > SMASH_TOTAL_TIMEOUT:
                            abandon_x, abandon_y = last_tx, last_ty
                            abandon_t0 = now
                            phase = 3; t0 = now
                            log(">>> SMASH：撞了 %d 次仍未散开，抛弃(%.0f,%.0f)附近目标，去找别的"
                                % (smash_idx, abandon_x, abandon_y))
                        else:
                            phase = 2; t0 = now
                            log(">>> SMASH：第 %d 次撞完（最大簇=%d），后退再撞"
                                % (smash_idx, max_sz))

                # ③ 后退（不换角度），重新对准再撞
                elif phase == 2:
                    # ★ 只后退、不换角度：后退完回 phase1 重新对准再撞
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > SMASH_BACK_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)   # 清 last_cmd
                        flush_rx()
                        phase = 1; t0 = now
                        log(">>> SMASH：后退完成，重新对准再撞（第 %d 次）"
                            % (smash_idx + 1))

                # ④ 后退脱离（此时摄像头仍在 55°）
                elif phase == 3:
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > SMASH_EXIT_BACK_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        phase = 4; t0 = now            # ★ 关键：转到 phase 4
                        log(">>> SMASH：后退完成，抬头归位")
                        
                # ⑤ 抬头归位 → 回 SEARCH
                elif phase == 4:
                    send_frame(CMD_CAM, CAM_UP, 0, 0, sx, sy)
                    if dt > SMASH_CAM_SETTLE_MS:
                        flush_rx()
                        phase = 5; t0 = now
                        log(">>> SMASH：已抬头")

                # ⑥ 补收爪 → 回 SEARCH
                elif phase == 5:
                    send_frame(CMD_GRAB, last_cls, 0, 0, sx, sy)
                    if dt > 400:
                        flush_rx()
                        state = "SEARCH"
                        phase = 3 if SMASH_SKIP_SPIN else 0    # ★ 1 → 3
                        t0 = now
                        log(">>> SMASH 完成 → 回 SEARCH (phase=%d)" % phase)
                        
            elif state == "GRAB":
                dt = time.ticks_diff(now, t0)

                # ---------- ① 低头（STM32 收到 CMD_CAM 会先停车） ----------
                if phase == 0:
                    send_frame(CMD_CAM, CAM_DOWN, 0, 0, sx, sy)
                    if dt > GRAB_CAM_SETTLE_MS:
                        flush_rx()
                        phase = 1; t0 = now
                        log(">>> 低头完成，抬爪")

                # ---------- ② 抬爪（张开） ----------
                elif phase == 1:
                    send_frame(CMD_OPEN, 0, 0, 0, sx, sy)
                    if dt > GRAB_OPEN_MS:
                        flush_rx()
                        phase = 2; t0 = now
                        log(">>> 抬爪完成，前进等目标下探")

                # ---------- ③ 前进：X 对准 + Y 到位 才算成功 ----------
                elif phase == 2:
                    t = match_locked_target(dets, last_cls, last_tx, last_ty)
                    if t is None:
                        # 低头后画面突变，位置可能跳 > MATCH_MAX_DIST
                        same = [d for d in dets if d["cls"] == last_cls]
                        t = min(same, key=car_dist) if same else None

                    # ★ 合搬 → 瞄簇质心；单目标 → 瞄自己
                    if t is not None and multi_mode:
                        cl2 = cluster_of(t, dets + dangers, SMASH_STICK_TH)
                        if len(cl2) >= 2:
                            cc = cluster_center(cl2)
                            tgt_x, tgt_y = int(cc[0]), int(cc[1])
                        else:
                            tgt_x, tgt_y = int(t["cx"]), int(t["cy"])
                            multi_mode = False       # 已经分开了 → 退回单目标
                    elif t is not None:
                        tgt_x, tgt_y = int(t["cx"]), int(t["cy"])
                    else:
                        tgt_x, tgt_y = last_tx, last_ty

                    # ---- 帧计数 ----
                    if t is not None:
                        last_tx, last_ty = tgt_x, tgt_y
                        grab_lost = 0
                        grab_stuck += 1          # 可见但还没到位
                    else:
                        grab_lost += 1           # 完全看不到

                    y_ok = (t is not None) and (tgt_y >= BOX_CY)
                    x_ok = (t is not None) and (abs(tgt_x - BOX_CX) <= GRAB_X_TOL)

                    if y_ok and x_ok:
                        # ★ 到位 → 夹紧
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        grab_lost = grab_stuck = 0
                        phase = 3; t0 = now
                        flush_rx()
                        log(">>> 到位 cy=%d dx=%d → 停车夹紧"
                            % (tgt_y, tgt_x - BOX_CX))

                    elif grab_lost > GRAB_LOST_FRAMES:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        grab_lost = grab_stuck = 0
                        regrab_after_safe = False
                        state = "SEARCH"; phase = 0; t0 = now
                        flush_rx()
                        log(">>> GRAB 目标丢失 %d 帧 → 回 SEARCH"
                            % GRAB_LOST_FRAMES)

                    elif grab_stuck > GRAB_STUCK_FRAMES:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        grab_lost = grab_stuck = 0
                        regrab_after_safe = False
                        state = "AVOID"; phase = 0; t0 = now
                        flush_rx()
                        log(">>> GRAB 卡住 %d 帧 (cy=%d dx=%d) → 回 SEARCH"
                            % (GRAB_STUCK_FRAMES, tgt_y, tgt_x - BOX_CX))

                    elif dt > GRAB_ADVANCE_MAX:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        grab_lost = grab_stuck = 0
                        regrab_after_safe = False
                        state = "AVOID"; phase = 0; t0 = now
                        flush_rx()
                        log(">>> GRAB 前进超时(%dms) → 回 SEARCH" % GRAB_ADVANCE_MAX)

                    elif t is not None and not x_ok:
                        # X 偏了 → 用视觉追踪边走边对准
                        send_frame(CMD_FORWARD, last_cls, tgt_x, tgt_y, sx, sy)
                    else:
                        send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)

                # ---------- ④ 夹紧 ----------
                elif phase == 3:
                    send_frame(CMD_GRAB, last_cls, 0, 0, sx, sy)
                    if dt > GRAB_CLOSE_MS:
                        phase = 4; t0 = now
                        flush_rx()
                        log(">>> 夹紧完成，左转")

                # ---------- ⑤ 左转把外露部分甩进车体 ----------
                elif phase == 4:
                    send_frame(CMD_TURN_L, GRAB_TURN_DEG, 0, 0, sx, sy)
                    if dt > GRAB_TURN_WAIT:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)   # 清 last_cmd
                        if regrab_after_safe:
                            regrab_after_safe = False
                            # ★ 补抓抓完：抬头 → 后退一点 → 回 ALIGN_X 重新对准+SHIFT 再送
                            t_cam = time.ticks_ms()
                            while time.ticks_diff(time.ticks_ms(), t_cam) < CAM_UP_MS:
                                send_frame(CMD_CAM, CAM_UP, 0, 0, sx, sy)
                                time.sleep_ms(50)
                            # ★ 后退一点（抓完车离门口太近，退一点重新对准，避免斜着送）
                            t_b = time.ticks_ms()
                            while time.ticks_diff(time.ticks_ms(), t_b) < REGRAB_AFTER_SAFE_BACK_MS:
                                send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                                time.sleep_ms(50)
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            state = "ALIGN_X"; phase = 0; t0 = now
                            ax_iter = 0; ax_lost = 0
                            flush_rx()
                            log(">>> 补抓完成 → 抬头 → 后退 → 回 ALIGN_X 重新对准+SHIFT 再送")
                        else:
                            state = "BACK_OFF"; phase = 0; t0 = now
                            flush_rx()
                            log(">>> 左转完成，开始后退")

            elif state == "BACK_OFF":
                dt = time.ticks_diff(now, t0)

                # GRAB 阶段已经低头，这里不用再发 CMD_CAM，直接后退
                if dt <= BACK_MS:
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                else:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    flush_rx()
                    state = "CHECK"; phase = 0; t0 = now
                    check_frames = check_bad = check_empty = 0
                    check_ok = 0
                    log(">>> 后退完成（摄像头保持低头），进入 CHECK")
                    
            elif state == "CHECK":
                dt = time.ticks_diff(now, t0)

                # ---------- ① 停稳后短暂确认画面 ----------
                if phase == 0:
                    send_frame(CMD_CAM, CAM_DOWN, 0, 0, sx, sy)
                    if dt > CAM_SETTLE_MS:
                        flush_rx()          # 丢掉后退过程中的残留
                        check_frames = check_bad = check_empty = 0
                        check_ok = 0
                        phase = 1; t0 = now
                        log(">>> 停稳，开始投票")

                # ---------- ② 静止投票 ----------
                elif phase == 1:
                    send_frame(CMD_CAM, CAM_DOWN, 0, 0, sx, sy)
                    check_frames += 1

                    r = check_vote(dets, dangers)
                    if r == "bad":
                        check_bad += 1
                    elif r == "empty":
                        check_empty += 1
                    else:
                        check_ok += 1

                    if check_frames >= CHECK_FRAMES:
                        log(">>> CHECK 投票: ok=%d bad=%d empty=%d / %d"
                            % (check_ok, check_bad, check_empty, check_frames))

                        # ---------- ③ 抬头（判定后再抬） ----------
                        t_cam = time.ticks_ms()
                        while time.ticks_diff(time.ticks_ms(), t_cam) < CAM_UP_MS:
                            send_frame(CMD_CAM, CAM_UP, 0, 0, sx, sy)
                            time.sleep_ms(50)
                        flush_rx()

                        # ---------- ④ 多数表决 ----------
                        if check_bad * 2 >= check_frames:
                            log(">>> 不合格（危险/混装）→ APART")
                            state = "APART"; apart_idx = 0; t0 = time.ticks_ms()
                        elif check_empty * 2 >= check_frames:
                            log(">>> 框内空 → RETREAT")
                            state = "RETREAT"; phase = 0; t0 = time.ticks_ms()
                        else:
                            log(">>> 合格 → FIND_SAFE")
                            state = "FIND_SAFE"; phase = 0; t0 = time.ticks_ms()
                        flush_rx()

            # ---------- 抬爪 → 后退 → 放爪 → 前推撞散 → 后退 ----------
            elif state == "APART":
                dt = time.ticks_diff(now, t0)

                if apart_idx >= len(APART_STEPS):
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    state = "SEARCH"; phase = 0; t0 = now
                    apart_idx = 0
                    log(">>> 分离完成，进入 SEARCH")

                else:
                    name, param = APART_STEPS[apart_idx]

                    # ① 抬爪（张开）
                    if name == "OPEN":
                        send_frame(CMD_OPEN, 0, 0, 0, sx, sy)
                        if dt > param:
                            apart_idx += 1; t0 = now
                            log(">>> 分离：抬爪完成")

                    # ② 放爪（闭合，当推板）
                    elif name == "CLOSE":
                        send_frame(CMD_GRAB, 0, 0, 0, sx, sy)
                        if dt > param:
                            apart_idx += 1; t0 = now
                            log(">>> 分离：放爪完成")

                    # ③ 后退
                    elif name == "BACK":
                        send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                        if dt > param:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            apart_idx += 1; t0 = now

                    # ④ 往前推撞散
                    elif name == "PUSH":
                        send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)
                        if dt > param:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            apart_idx += 1; t0 = now
                            log(">>> 分离：撞散完成")

            # ---------- GRAB 卡住后的退避：后退 + 左转避开 ----------
            elif state == "AVOID":
                dt = time.ticks_diff(now, t0)

                if phase == 0:                       # ① 后退脱离
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > AVOID_BACK_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        phase = 1; t0 = now
                        log(">>> AVOID：后退完成，准备转向")

                elif phase == 1:                     # ② 左转避开目标
                    send_frame(CMD_TURN_L, AVOID_TURN_DEG, 0, 0, sx, sy)
                    if dt > AVOID_TURN_WAIT:
                        if ack != CMD_TURN_L:
                            log(">>> AVOID 转向 ack 超时，按 %dms 强制继续"
                                % AVOID_TURN_WAIT)
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)   # 清 last_cmd
                        flush_rx()
                        state = "SEARCH"; phase = 0; t0 = now
                        log(">>> AVOID 完成（后退 %dms + 左转 %d°），回 SEARCH"
                            % (AVOID_BACK_MS, AVOID_TURN_DEG))

            elif state == "RETREAT":
                # 框内空 → 只后退，不抬爪/落抓（省时间），回 SEARCH 重新框
                dt = time.ticks_diff(now, t0)
                send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                if dt > RETREAT_BACK_MS:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    state = "SEARCH"; phase = 0; t0 = now
                    flush_rx()
                    log(">>> RETREAT 完成（空框只后退），回 SEARCH")

            elif state == "REGRAB_GREEN":
                # ★ 绿色保底放弃：后退 → 抬框(释放) → 后退 → 落框 → 重找绿
                dt = time.ticks_diff(now, t0)
                if phase == 0:
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > REGRAB_BACK1_MS:
                        phase = 1; t0 = now
                elif phase == 1:
                    send_frame(CMD_OPEN, 0, 0, 0, sx, sy)
                    if dt > REGRAB_OPEN_MS:
                        phase = 2; t0 = now
                elif phase == 2:
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > REGRAB_BACK2_MS:
                        phase = 3; t0 = now
                elif phase == 3:
                    send_frame(CMD_GRAB, 0, 0, 0, sx, sy)
                    if dt > REGRAB_CLOSE_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        green_done = False
                        state = "SEARCH"; phase = 0; t0 = now
                        flush_rx()
                        log(">>> 放弃本次(非绿且左半没绿) → 重新去找绿")
                        
            elif state == "FIND_SAFE":
                dt = time.ticks_diff(now, t0)

                # ---------- phase 0：转圈只找自己的区（SAFE_USE） ----------
                if phase == 0:
                    send_frame(CMD_ROT_FIND, 0, 0, 0, sx, sy)
                    if safe is not None:
                        # ★ 刚找到：太近就先退一次（只退一次），退完重新转圈，不影响后面的送
                        if safe[2] * safe[3] > SAFE_AREA_MAX and safe_back_cnt == 0:
                            safe_back_cnt = 1
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            flush_rx()
                            phase = 2; t0 = now
                            log(">>> 刚找到但太近(面积%d) → 先退一次" % (safe[2] * safe[3]))
                        elif sy >= AP_Y_NEAR:
                            # 近：先进 ALIGN_X 对准中心（拍全 + X 居中），再测角横移
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            state = "ALIGN_X"; phase = 0; t0 = now
                            ax_iter = 0; ax_lost = 0
                            log(">>> 找到自己区，够近(sy=%d) → 先 ALIGN_X" % sy)
                        elif sy >= AP_Y_NEAR_FAR:
                            # 中距离：识别到了就直接去，不转圈等
                            state = "APPROACH_SAFE"; phase = 0; t0 = now
                            ap_lost = 0; ap_fallback = False
                            log(">>> 找到自己区，中距离(sy=%d) → 直接接近" % sy)
                        elif dt > FS_MIN_SPIN_MS:
                            # 特别远：先转满一圈看有没有更近的（防把远处的蓝误认成红）
                            state = "APPROACH_SAFE"; phase = 0; t0 = now
                            ap_lost = 0; ap_fallback = False
                            log(">>> 找到自己区，太远(sy=%d) → 先追踪接近" % sy)
                        # else: 特别远且还没转满一圈，继续转
                    elif dt > FS_SPIN_MAX_MS:
                        # 超时没找到自己区 → 改用对方区补救
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        phase = 1; t0 = now
                        log(">>> 找不到自己区 → 用对方区补救")

                # ---------- phase 1：转圈找对方区（非 SAFE_USE），找到就 approach（用松阈值） ----------
                elif phase == 1:
                    send_frame(CMD_ROT_FIND, 0, 0, 0, sx, sy)
                    if safe is not None:
                        state = "APPROACH_SAFE"; phase = 0; t0 = now
                        ap_lost = 0; ap_fallback = True; ap_own_cnt = 0
                        log(">>> 用对方区补救 → approach（凑近再判颜色）")

                # ---------- phase 2：刚找到但太近 → 先退一次，退完重新转圈 ----------
                elif phase == 2:
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > SAFE_BACK_AREA_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        phase = 0; t0 = now
                        log(">>> 太近后退完成 → 重新找安全区")

            # ---------- 追踪接近安全区（太远时先追，不站着测）----------
            elif state == "APPROACH_SAFE":
                dt = time.ticks_diff(now, t0)

                # ★ 绿色保底：带的是非绿、且安全区左半没看到绿 → 连续确认后放弃重找绿
                if last_cls != GREEN_CLS and safe is not None and \
                   not has_green_in_safe(dets, safe):
                    no_green_cnt += 1
                    if no_green_cnt >= GREEN_CHECK_CONFIRM:
                        no_green_cnt = 0
                        state = "REGRAB_GREEN"; phase = 0; t0 = now
                        flush_rx()
                        log(">>> 安全区左半没绿、带的是非绿 → 放弃重找绿")
                else:
                    no_green_cnt = 0

                # ---- 目标点：本帧实测优先，丢了用最后已知位置继续 ----
                if safe is not None:
                    ap_lost = 0
                    nav_sx, nav_sy = sx, sy
                else:
                    ap_lost += 1
                    if ap_lost <= AP_LOST_KEEP and (last_sx or last_sy):
                        nav_sx, nav_sy = last_sx, last_sy
                    else:
                        nav_sx = nav_sy = 0

                if nav_sx == 0 and nav_sy == 0:
                    send_frame(CMD_STOP, 0, 0, 0, 0, 0)      # 从没见过，别乱动
                else:
                    # ★ 0x14 高速追踪（STM32 已实现：App.c GO_SAFE_FAST → SetFastGear(1) + VISUAL_TRACK）
                    send_frame(CMD_GO_SAFE_FAST , last_cls,
                               nav_sx, nav_sy, nav_sx, nav_sy)

                # ---- 补救途中持续判颜色：连续确认是自己区 → 转正常 approach ----
                if ap_fallback and safe is not None and safe_cls == SAFE_USE:
                    ap_own_cnt += 1
                    if ap_own_cnt >= AP_OWN_CONFIRM:
                        ap_fallback = False
                        log(">>> 补救途中识别出是自己区 → 转正常 approach")
                else:
                    ap_own_cnt = 0

                # ---- 够近 → 停车，交给 ALIGN_X（对方区补救用更松的阈值） ----
                near_th = AP_Y_NEAR_LOOSE if ap_fallback else AP_Y_NEAR
                if safe is not None and sy >= near_th:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    flush_rx()
                    state = "ALIGN_X"; phase = 0; t0 = now
                    ax_iter = 0; ax_lost = 0
                    log(">>> 已接近安全区(sy=%d%s) → ALIGN_X"
                        % (sy, ",补救" if ap_fallback else ""))

                # ---- 丢帧太久 → 回 FIND_SAFE 重新找 ----
                elif ap_lost > AP_LOST_MAX:
                    send_frame(CMD_STOP, 0, 0, 0, 0, 0)
                    flush_rx()
                    ap_lost = 0
                    state = "FIND_SAFE"; phase = 0; t0 = now
                    log(">>> APPROACH 连续丢帧 %d → 回 FIND_SAFE" % AP_LOST_MAX)

                # ---- 总超时 → 强行交给 ALIGN_X ----
                elif dt > AP_TIMEOUT:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    flush_rx()
                    ap_lost = 0
                    state = "ALIGN_X"; phase = 0; t0 = now
                    ax_iter = 0; ax_lost = 0
                    log(">>> APPROACH 超时 %dms → 强行进 ALIGN_X" % AP_TIMEOUT)

            # ---------- 原地转，让安全区中心 x 归到画面中心 ----------
            elif state == "ALIGN_X":
                dt = time.ticks_diff(now, t0)

                # ---- 0：先停一下，等画面稳 ----
                if phase == 0:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    # ★ 绿色保底：align 刚进来再自检一次（带非绿、左半没绿 → 放弃）
                    if last_cls != GREEN_CLS and safe is not None and \
                       not has_green_in_safe(dets, safe):
                        no_green_cnt += 1
                        if no_green_cnt >= GREEN_CHECK_CONFIRM:
                            no_green_cnt = 0
                            state = "REGRAB_GREEN"; phase = 0; t0 = now
                            flush_rx()
                            log(">>> ALIGN 自检：安全区左半没绿 → 放弃重找绿")
                    else:
                        no_green_cnt = 0
                    if dt > 200:
                        phase = 1; t0 = now

                # ---- 1：判断误差，决定转不转 ----
                elif phase == 1:
                    if safe is None:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        ax_lost += 1
                        if ax_lost > AX_LOST_MAX:
                            log(">>> ALIGN_X：连续丢帧，回 FIND_SAFE")
                            ax_lost = 0
                            state = "APPROACH_SAFE"; phase = 0; t0 = now
                    else:
                        ax_lost = 0
                        err = sx - 320          # <0 = 安全区在画面左边

                        if abs(err) <= AX_TOL_PX or ax_iter >= AX_MAX_ITER:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            log(">>> ALIGN_X 完成: sx=%d err=%d 转%d次"
                                % (sx, err, ax_iter))
                            state = "SHIFT" if SHIFT_ENABLE else "GO_SAFE"
                            phase = 0; t0 = now
                            _SH = []
                        else:
                            d = abs(err) * AX_GAIN
                            if d > AX_MAX_DEG: d = AX_MAX_DEG
                            if d < AX_MIN_DEG: d = AX_MIN_DEG
                            ax_deg = int(d + 0.5)
                            # 安全区在左边 → 左转；在右边 → 右转
                            ax_cmd = CMD_TURN_L if err < 0 else CMD_TURN_R
                            send_frame(ax_cmd, ax_deg, 0, 0, sx, sy)
                            log(">>> ALIGN_X: err=%d 转 %s %d°"
                                % (err, "L" if err < 0 else "R", ax_deg))
                            phase = 2; t0 = now

                # ---- 2：等这次转向完成（每帧重发，喂 500ms 看门狗）----
                elif phase == 2:
                    send_frame(ax_cmd, ax_deg, 0, 0, sx, sy)     # 每帧重发
                    ax_w = ax_deg * AX_MS_PER_DEG + AX_MARGIN    # ★ 按角度算
                    if ax_w > AX_WAIT_MS: ax_w = AX_WAIT_MS
                    if dt > ax_w:
                        if ack != ax_cmd:
                            log(">>> ALIGN_X: 超时，强制继续")
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        ax_iter += 1
                        phase = 3; t0 = now

                # ---- 3：等画面稳定后重新测 ----
                elif phase == 3:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    if dt > AX_SETTLE_MS:
                        phase = 1; t0 = now

            # ---------- 侧移：转(90-θ) → 直行 → 转回/掉头 ----------
            elif state == "SHIFT":
                dt = time.ticks_diff(now, t0)

                # ---- 0：测 θ / Zq / 颜色（每帧拟合，存进 _SH） ----
                if phase == 0:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    # ★ 先停稳再采样（车从 ALIGN/APPROACH 转过来有惯性，前几百 ms 画面还在动）
                    if dt < SH_SAMPLE_SETTLE_MS:
                        continue
                    g = _g if EDGE_DRAW else safe_long_edge(img, safe, SH_EDGE_DBG)
                    if g is not None and safe is not None:
                        a = (g[2][1] - g[1][1]) / float(g[2][0] - g[1][0])
                        b = g[1][1] - a * g[1][0]
                        geom = shift_geom(a, b)
                        if geom is not None:
                            _SH.append((geom[0], geom[1], safe_cls))   # (θ, Zq, 颜色)
                            while len(_SH) > MED_N:
                                _SH.pop(0)   # ★ 滑动窗口：保留停稳后最后 5 帧

                    # ★ 角度稳定锁定：最近 N 帧 θ 极差 < 阈值 → 提前结束采样（小角度自动提速）
                    stable = False
                    if len(_SH) >= SH_STABLE_N and len(_SH) >= MED_N:
                        st = _SH[-SH_STABLE_N:]
                        sp = sorted([x[0] for x in st])
                        stable = (sp[-1] - sp[0]) < SH_STABLE_TH

                    if (dt > SH_SAMPLE_SETTLE_MS + SH_MEAS_MS) or stable:
                        if len(_SH) < 1:
                            # 保底：还没采到就再等一会，实在采不到才跳过
                            if dt > SH_SAMPLE_SETTLE_MS + SH_MEAS_MAX_MS:
                                log(">>> SHIFT 跳过: 0 样本（等了 %dms）" % dt)
                                state = "GO_SAFE"; phase = 0; t0 = now
                        else:
                            # ★ 统一取整个窗口的中位数（锁定只提前结束等待，不缩小样本）
                            use = _SH
                            ths = sorted([x[0] for x in use])
                            th = ths[len(ths) // 2]
                            Zq = use[-1][1]
                            _c = use[-1][2]
                            th -= TH_ZERO
                            S  = shift_S(th, Zq)
                            # ★ 角度过大（面对角）：S 估偏小，放大一点补偿横向误差
                            if abs(th) > SH_BIG_TH:
                                S *= SH_BIG_GAIN
                            S *= SH_S_GAIN
                            sh_theta = th
                            sh_S     = S

                            # 颜色投票：多数是自己颜色 = 自己区；否则 = 对方区（最后掉头）
                            votes = [x[2] for x in _SH]
                            sh_flip = (SAFE_USE is not None) and \
                                      (votes.count(SAFE_USE) * 2 <= len(votes))

                            log(">>> SHIFT 测角: n=%d θ=%.1f Zq=%.2f S=%.3f %s"
                                % (len(_SH), th, Zq, S,
                                   "对方区→掉头" if sh_flip else "自己区"))

                            if abs(th) > SH_MAX_TH:
                                log(">>> SHIFT 跳过: |θ|>%d 拟合可能抓到短边/角" % SH_MAX_TH)
                                state = "GO_SAFE"; phase = 0; t0 = now
                            elif abs(th) <= SH_MIN_DEG and not sh_flip:
                                # 自己区且已正对 → 不用横移，直接送
                                log(">>> SHIFT 跳过: 已正对")
                                state = "GO_SAFE"; phase = 0; t0 = now
                            else:
                                sh_d1 = int(90 - abs(th) - SH_TRIM_DEG + 0.5)
                                if sh_d1 < 10: sh_d1 = 10
                                if sh_d1 > 90: sh_d1 = 90

                                # 横移距离：S 为主，TRIM 补偿方向偏差，再夹保险丝
                                sh_L = min(S, SH_S_MAX) / math.cos(math.radians(SH_TRIM_DEG))
                                sh_dms = int(1000.0 * sh_L / SH_V)
                                if sh_dms > SH_DRIVE_MAX_MS: sh_dms = SH_DRIVE_MAX_MS

                                if (th * SH_DIR_SIGN) > 0:      # 方向开关
                                    sh_c1, sh_c2 = CMD_TURN_L, CMD_TURN_R
                                else:
                                    sh_c1, sh_c2 = CMD_TURN_R, CMD_TURN_L

                                # 按角度算等待（ack 没通，只能靠时间；转慢了会方向错 → 标 SH_MS_PER_DEG）
                                sh_w1 = sh_d1 * SH_MS_PER_DEG + SH_TURN_MARGIN
                                sh_w2 = 90   * SH_MS_PER_DEG + SH_TURN_MARGIN
                                if sh_w1 > SH_MAX_MS: sh_w1 = SH_MAX_MS
                                if sh_w2 > SH_MAX_MS: sh_w2 = SH_MAX_MS

                                send_frame(sh_c1, sh_d1, 0, 0, sx, sy)
                                log(">>> SHIFT: θ=%.1f → %s %d°(等%dms) 直行%dms"
                                    % (th, "L" if sh_c1 == CMD_TURN_L else "R",
                                       sh_d1, sh_w1, sh_dms))
                                phase = 1; t0 = now

                # ---- 1：等第 1 次转向（每帧重发，喂看门狗） ----
                elif phase == 1:
                    send_frame(sh_c1, sh_d1, 0, 0, sx, sy)
                    if dt > sh_w1:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        log(">>> SHIFT: 第1次转向完成 %dms" % dt)
                        phase = 2; t0 = now

                # ---- 2：停稳 → 直行侧移（每帧重发 CMD_STRAIGHT，喂看门狗） ----
                elif phase == 2:
                    if dt < SH_SETTLE_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    elif dt < SH_SETTLE_MS + sh_dms:
                        send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)
                    else:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        flush_rx()
                        log(">>> SHIFT 直行 %dms 完成" % sh_dms)
                        phase = 3; t0 = now

                # ---- 3：停稳后转回 90°；对方区则反向转 90°（= 掉头） ----
                elif phase == 3:
                    if dt < SH_SETTLE_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    else:
                        # 自己区：转回 90°（sh_c2，面朝安全区）；对方区：同方向再转 90°（sh_c1，掉头）
                        send_frame(sh_c1 if sh_flip else sh_c2, 90, 0, 0, sx, sy)
                        if dt > SH_SETTLE_MS + sh_w2:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            flush_rx()
                            if sh_flip:
                                state = "TO_OWN_SAFE"; phase = 0; t0 = now
                                log(">>> SHIFT 完成（对方区）→ 掉头直行找自己区")
                            else:
                                state = "GO_SAFE"; phase = 0; t0 = now
                                log(">>> SHIFT 完成，进 GO_SAFE")

            # ---------- 对方区掉头后：直行直到看见自己区 ----------
            elif state == "TO_OWN_SAFE":
                dt = time.ticks_diff(now, t0)
                send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)   # 已在中线上、朝向自己区，直行

                # 看到自己区且够近 → 直接 GO_SAFE（不用再 align/shift）
                if safe is not None and safe_cls == SAFE_USE and sy >= AP_Y_NEAR:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    flush_rx()
                    state = "GO_SAFE"; phase = 0; t0 = now
                    log(">>> 掉头后看到自己区(sy=%d) → 直接 GO_SAFE" % sy)
                elif dt > TO_OWN_TIMEOUT:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                    flush_rx()
                    state = "FIND_SAFE"; phase = 0; t0 = now
                    log(">>> 掉头后 %dms 没看到自己区 → 回 FIND_SAFE" % TO_OWN_TIMEOUT)

            elif state == "GO_SAFE":
                dt = time.ticks_diff(now, t0)

                # ---------- ① 对准安全区中心（不再加偏移） ----------
                if phase == 0:
                    if safe is not None:
                        nav_sx, nav_sy = sx, sy
                    elif last_sx or last_sy:
                        nav_sx, nav_sy = last_sx, last_sy      # 丢帧时用最后已知位置
                    else:
                        nav_sx = nav_sy = 0

                    if nav_sx == 0 and nav_sy == 0:
                        send_frame(CMD_STOP, 0, 0, 0, 0, 0)    # 从没见过，别乱动
                    else:
                        # ★ 关键：tx/ty 都直接用安全区中心，不做任何像素偏移
                        send_frame(CMD_GO_SAFE, last_cls, nav_sx, nav_sy, nav_sx, nav_sy)

                        if (safe is not None and sy >= SAFE_Y_NEAR) or dt > GO_SAFE_TIMEOUT:
                            send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                            if not (safe is not None and sy >= SAFE_Y_NEAR):
                                log(">>> GO_SAFE 超时 %dms(sy=%d) → 强制转向送"
                                    % (GO_SAFE_TIMEOUT, sy))
                            # 按目标类别决定往哪边送：橙→右，其余→左
                            if target_side(last_cls) == "left":
                                deliver_cmd, restore_cmd = CMD_TURN_L, CMD_TURN_R
                            else:
                                deliver_cmd, restore_cmd = CMD_TURN_R, CMD_TURN_L

                            phase = 1; t0 = now
                            safe_back_cnt = 0
                            flush_rx()
                            log(">>> 到达安全区前方，准备 %s 转 %d°" %
                                ("左" if deliver_cmd == CMD_TURN_L else "右", SAFE_DELIVER_DEG))

                # ---------- ② 转 ±15° ----------
                elif phase == 1:
                    send_frame(deliver_cmd, SAFE_DELIVER_DEG, 0, 0, sx, sy)   # 角度放在 cls
                    if dt > TURN_ACK_TIMEOUT:
                        if ack != deliver_cmd:
                            log(">>> 转向 ack 超时，强制继续")
                        phase = 2; t0 = now
                        flush_rx()

                # ---------- ③ 前进一小段，把目标横向推进安全区 ----------
                elif phase == 2:
                    send_frame(CMD_STRAIGHT, 0, 0, 0, sx, sy)
                    if dt > SAFE_PUSH_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        phase = 3; t0 = now
                        flush_rx()

                # ---------- ④ 松爪 ----------
                elif phase == 3:
                    send_frame(CMD_OPEN, 0, 0, 0, sx, sy)
                    if dt > 1000:
                        phase = 4; t0 = now
                        flush_rx()
                        log(">>> 松爪完成，后退")

                # ---------- ⑤ 后退脱离 ----------
                elif phase == 4:
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > SAFE_BACK_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        phase = 8; t0 = now
                        flush_rx()
                        log(">>> 后退完成 → 落爪")

                # ---------- ⑥ 停车验证：绿色是否进了安全区 + 是否有目标卡在门口 ----------
                elif phase == 5:
                    send_frame(CMD_STOP, 0, 0, 0, sx, sy)     # 保持不动
                    verify_frames += 1

                    # ★ 安全区闪烁时，用最后已知的安全区框做验证，不因为 safe=None 漏掉
                    if safe is not None:
                        verify_seen += 1
                        vsx, vsy, vsw, vsh = safe
                    else:
                        vsx, vsy, vsw, vsh = last_sx, last_sy, last_sw, last_sh

                    if vsw > 0 and vsh > 0:
                        rx = vsw / 2.0 * VERIFY_SHRINK_X
                        ry = vsh / 2.0 * VERIFY_SHRINK_Y
                        if rx < VERIFY_MIN_R: rx = VERIFY_MIN_R
                        if ry < VERIFY_MIN_R: ry = VERIFY_MIN_R

                        # ① 本次搬的目标(last_cls)是否进了内切椭圆
                        for d in dets:
                            if d["cls"] != last_cls:
                                continue
                            ux = (d["cx"] - vsx) / rx
                            uy = (d["cy"] - vsy) / ry
                            if ux * ux + uy * uy <= 1.0:      # ★ 内切椭圆
                                verify_in += 1
                                break

                        # ② 有没有目标卡在安全区外面：x 在安全区宽度内、y 在"下 1/4 线"之下
                        for d in dets:
                            if abs(d["cx"] - vsx) < vsw / 2.0 and \
                               d["cy"] > vsy + vsh * VERIFY_LEFTOVER_FRAC:
                                leftover_frames += 1
                                break

                    if verify_frames >= VERIFY_FRAMES:
                        if leftover_frames >= 3 and safe_retry < SAFE_RETRY_MAX:
                            # ★ 卡门口优先：中心低于下1/4线 = 没推进去，先补推
                            #   （不能先判"验证通过"，否则椭圆把门口区域也算进去，补推永远不触发）
                            safe_retry += 1
                            phase = 7; t0 = now
                            flush_rx()
                            log(">>> 目标卡在安全区外(遗留%d/%d) → 补推 %d/%d（先框住）"
                                % (leftover_frames, verify_frames, safe_retry, SAFE_RETRY_MAX))
                        elif verify_in * 2 >= verify_frames:
                            # 本次搬的目标确认进了安全区；只有绿才标记 green_done/green_ever
                            if last_cls == GREEN_CLS:
                                green_done = True
                                green_ever = True          # ★ 上锁，永不解锁
                            log(">>> 验证通过：cls=%d 已进安全区 (%d/%d)"
                                % (last_cls, verify_in, verify_frames))
                            phase = 6; t0 = now
                        elif verify_seen * 2 >= verify_frames and last_cls == GREEN_CLS:
                            # 只有搬的是绿、看到区、绿没进去 → 才判定"绿还没送成"
                            green_done = False
                            log(">>> 验证失败：看到安全区但绿没进去 (%d/%d)，继续找绿"
                                % (verify_in, verify_frames))
                            phase = 6; t0 = now
                        else:
                            # 黑/橙没送进去，或没看到区：先离场，不动 green_done
                            if last_cls == GREEN_CLS:
                                if safe_retry >= SAFE_RETRY_MAX:
                                    # ★ 补抓都补完了还是没确认进 → 绿大概率还在外面，不设 True，继续找绿
                                    green_done = False
                                    log(">>> 补抓 %d/%d 次仍未确认送入，继续找绿"
                                        % (safe_retry, SAFE_RETRY_MAX))
                                else:
                                    green_done = True          # 没补抓满，保守认为绿已送入
                                    green_ever = True
                            log(">>> 未确认送入(cls=%d)，离场" % last_cls)
                            phase = 6; t0 = now

                # ---------- 补抓：目标卡门口（phase3 已松爪、phase4 已后退，框是空的），
                #   后退太远直接 GRAB 抓不住 → 先刷新目标位置，进 TRACK 追近，再 GRAB 抓，抓完回 phase1 送 ----------
                elif phase == 7:
                    same = [d for d in dets if d["cls"] == last_cls]
                    if same:
                        tt = min(same, key=car_dist)   # 最近同色目标 = 卡门口那个
                        last_tx, last_ty = int(tt["cx"]), int(tt["cy"])
                    regrab_after_safe = True
                    state = "TRACK"; phase = 0; t0 = now
                    lost_count = 0
                    track_aligning = False
                    multi_hits = 0
                    multi_mode = False
                    flush_rx()
                    log(">>> 目标卡门口 → 先 TRACK 追近再抓（抓完再送）")

                # ---------- 后退后先落爪（闭合），再验证（开框会干扰检测） ----------
                elif phase == 8:
                    send_frame(CMD_GRAB, 0, 0, 0, sx, sy)
                    if dt > GRAB_CLOSE_MS:
                        verify_frames = verify_in = verify_seen = 0
                        leftover_frames = 0
                        phase = 5; t0 = now
                        flush_rx()
                        log(">>> 落爪完成 → 开始验证")

                # ---------- 离太近：先后退一点，再回 phase 0 重新接近 ----------
                elif phase == 9:
                    send_frame(CMD_BACK, 0, 0, 0, sx, sy)
                    if dt > SAFE_BACK_AREA_MS:
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)
                        phase = 0; t0 = now
                        flush_rx()
                        log(">>> 后退完成，重新接近安全区")

                # ---------- ⑦ 送完直接左转 90°，把安全区甩出视野 ----------
                elif phase == 6:
                    send_frame(CMD_TURN_L, LEAVE_TURN_DEG, 0, 0, sx, sy)
                    if dt > LEAVE_TURN_WAIT:
                        if ack != CMD_TURN_L:
                            log(">>> 离场转向 ack 超时，按 %dms 强制继续"
                                % LEAVE_TURN_WAIT)
                        send_frame(CMD_STOP, 0, 0, 0, sx, sy)   # ★ 必须清 last_cmd
                        safe_retry = 0
                        state = "SEARCH"; phase = 0; t0 = now
                        flush_rx()
                        log(">>> 离场转向 %d° 结束 (耗时 %dms)"
                            % (LEAVE_TURN_DEG, dt))

                            
            
            ob_det.draw_result(pl, res)
            if EDGE_DRAW:
                draw_edge_overlay(pl, safe, _g)      # ★ 必须在 clear 之后
            # ★ GO_SAFE 验证时的调试显示：椭圆 + 卡门口判据线
            if state == "GO_SAFE" and phase == 5:
                if safe is not None:
                    _vb = safe
                elif last_sx:
                    _vb = (last_sx, last_sy, last_sw, last_sh)
                else:
                    _vb = None
                draw_verify_debug(pl, _vb)
                pl.osd_img.draw_string_advanced(8, 360, 22,
                    "left=%d in=%d seen=%d" % (leftover_frames, verify_in, verify_seen),
                    color=(255, 255, 0))
            # ★ 状态常驻显示（左上角）
            pl.osd_img.draw_string_advanced(8, 8, 26,
                "%s/%d  %dms" % (state, phase, time.ticks_diff(now, t0)),
                color=(255, 255, 0))
            if SHOW_THETA:
                if _theta_show is not None:
                    pl.osd_img.draw_string_advanced(8, 44, 24,
                        "theta=%.1f" % _theta_show, color=(0, 255, 255))
                else:
                    pl.osd_img.draw_string_advanced(8, 44, 24, "theta=--", color=(0, 255, 255))
            if state == "SHIFT":
                pl.osd_img.draw_string_advanced(8, 400, 22,
                    "theta=%.1f S=%.3f d1=%d drv=%d" % (sh_theta, sh_S, sh_d1, sh_dms),
                    color=(255, 0, 255))
            if CALIB_BOX:
                draw_grab_box(pl)
            pl.show_image()
            gc.collect()
        
        except Exception as e:
            try:
                sys.print_exception(e)                  # ★ 打印完整堆栈（含行号）
            except:
                print("!!! 异常:", e)

            send_frame(CMD_CAM, CAM_UP, 0, 0, 0, 0)     # ★ 异常时务必抬头
            time.sleep_ms(50)
            send_frame(CMD_STOP, 0, 0, 0, 0, 0)
            state = "SEARCH"
            phase = 0
            t0 = time.ticks_ms()
