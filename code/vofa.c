#include "vofa.h"
#include "W_Motor.h"
#include "Y_Motor.h"
#include "param.h"
#include "attitude.h"
#include "control.h"
#include <stdio.h>

// 全局状态
volatile vofa_mode_t g_vofa_mode = VOFA_OFF;    // 当前波形模式
volatile uint8       g_vofa_div  = 20;          // 发送分频，单位为 1ms
volatile tune_axis_t g_tune_axis = TUNE_AXIS_ROLL;  // 当前调参轴
volatile tune_ring_t g_tune_ring = TUNE_RING_RATE;  // 当前调参环

// 波形快照
// 单帧最大通道数。16 通道每帧约 170 字节，20ms 一帧就是 8.5KB/s，
// 115200 的线速率是 11.5KB/s，够用。再往上加通道要同时把 g_vofa_div 调大
#define VOFA_CH_MAX     (16)

static volatile uint32      s_seq = 0;          // 快照序号
static volatile float       s_ch[VOFA_CH_MAX];  // 通道值
static volatile uint8       s_n = 0;            // 通道数
static volatile const char *s_tag = "off";      // 帧标签

// 下行命令缓冲
#define VOFA_CMD_LINE_MAX   (64)                // 单行命令长度

static char  s_cmd_line[VOFA_CMD_LINE_MAX];     // 命令行缓冲
static uint8 s_cmd_len = 0;                     // 命令行长度
static uint8 s_cmd_ovf = 0;                     // 命令行溢出标志

// 收发环形缓冲。
// 刷屏走 SPI，一次实时行刷新要十几毫秒，主循环被它拖住的时候串口不能跟着停：
// 发慢了波形会一阵一阵地涌，收慢了库里那 64 字节接收缓冲会溢出，
// 命令中间少几个字节还能解析成功，"r_rate_kp 12.5" 少个 2 就变成 1.5，静默改错增益。
// 所以两个方向都由 1ms 中断搬运，前台只负责格式化和解析。
//
// 单生产者单消费者，前台只写 head、中断只写 tail，各自是自己那个下标的唯一写者，
// 所以不用关中断。长度取 2 的幂，uint32 下标自然回绕后再取模。
#define VOFA_TX_FIFO_DEPTH  (16u)               // ASCLIN 硬件发送 FIFO 深度
#define VOFA_TX_SIZE        (2048u)             // 发送环长度，必须是 2 的幂
#define VOFA_RX_SIZE        (256u)              // 接收环长度，必须是 2 的幂
#define VOFA_TX_REPLY_RESERVE (96u)             // 为命令 ACK/回读保留空间，波形不能占用
#define VOFA_TX_MASK        (VOFA_TX_SIZE - 1u)
#define VOFA_RX_MASK        (VOFA_RX_SIZE - 1u)

typedef char vofa_ring_size_is_pow2[((VOFA_TX_SIZE & VOFA_TX_MASK) == 0u &&
                                     (VOFA_RX_SIZE & VOFA_RX_MASK) == 0u &&
                                     VOFA_TX_REPLY_RESERVE < VOFA_TX_SIZE) ? 1 : -1];

static uint8 s_tx_buf[VOFA_TX_SIZE];            // 上行字节环
static volatile uint32 s_tx_head;               // 写下标，只由前台写
static volatile uint32 s_tx_tail;               // 读下标，只由 1ms 中断写

static uint8 s_rx_buf[VOFA_RX_SIZE];            // 下行字节环
static volatile uint32 s_rx_head;               // 写下标，只由 1ms 中断写
static volatile uint32 s_rx_tail;               // 读下标，只由前台写
static volatile uint8  s_rx_overflow;           // 接收环溢出，本行命令必须作废

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把一整帧压进上行环，装不下就整帧丢弃，绝不写半截
// 参数说明     dat/len         数据地址与长度
// 返回参数     uint8           1 写入成功，0 空间不足已丢帧
// 使用示例     (void)tx_push((const uint8 *)line, (uint32)len);
//-------------------------------------------------------------------------------------------------------------------
static uint8 tx_push(const uint8 *dat, uint32 len)
{
    uint32 head = s_tx_head;
    uint32 used = head - s_tx_tail;             // 无符号相减，回绕后依然是真实占用量
    uint32 i;

    if (len == 0u || len > (VOFA_TX_SIZE - used)) return 0;

    // 半截帧比丢帧更糟：VOFA+ 会把残行当成一帧解析，通道全部错位。
    for (i = 0; i < len; i++) s_tx_buf[(head + i) & VOFA_TX_MASK] = dat[i];
    s_tx_head = head + len;                     // 数据写完再发布下标
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将一整帧波形压入上行环，并为命令回复预留空间
// 参数说明     dat/len         波形数据地址与长度
// 返回参数     uint8           1=写入成功 0=空间不足已丢帧
// 使用示例     (void)tx_push_wave((const uint8 *)line, (uint32)len);
//-------------------------------------------------------------------------------------------------------------------
static uint8 tx_push_wave(const uint8 *dat, uint32 len)
{
    uint32 used = s_tx_head - s_tx_tail;
    uint32 wave_capacity = VOFA_TX_SIZE - VOFA_TX_REPLY_RESERVE;

    if (used >= wave_capacity || len > (wave_capacity - used)) return 0;
    return tx_push(dat, len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将一行 ASCII 数据写入上行环
// 参数说明     line            以 '\0' 结尾且包含换行符的字符串
// 返回参数     void
// 使用示例     cmd_send("ack:1.000\n");
//-------------------------------------------------------------------------------------------------------------------
static void cmd_send(const char *line)
{
    uint32 n = 0;
    while (line[n] != '\0' && n < 200u) n++;
    if (n) (void)tx_push((const uint8 *)line, n);
}

// 调参轴、环与参数名的对应表
static const char *const s_tune_tbl[TUNE_AXIS_MAX][TUNE_RING_MAX][3] =
{
    // Roll
                { { "r_rate_kp",  "r_rate_ki",  "r_rate_kd"  },
                  { "r_angle_kp", "r_angle_ki", "r_angle_kd" },
                  { "r_rcy_kp",   "r_rcy_ki",   "r_rcy_kd"   } },
    // Pitch
                { { "p_rate_kp",  "p_rate_ki",  "p_rate_kd"  },
                  { "p_angle_kp", "p_angle_ki", "p_angle_kd" },
                  { "p_vel_kp",   "p_vel_ki",   "p_vel_kd"   } },
    // Yaw
                { { "y_rate_kp",  "y_rate_ki",  "y_rate_kd"  },
                  { "y_angle_kp", "y_angle_ki", "y_angle_kd" },
                  { 0,            0,            0            } },
};

// 下行放行的就是上表这 24 个环 PID，没有别的。
// 回收环额外低通、回收环输出限幅、角度环输出限幅和反电动势前馈均未启用。

// 字符串工具

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     比较两个 ASCII 字符串并忽略大小写
// 参数说明     a 第一个字符串，b 第二个字符串
// 返回参数     int             0 表示相等
// 使用示例     if (cmd_icmp(tok, "r_rate_kp") == 0)
//-------------------------------------------------------------------------------------------------------------------
static int cmd_icmp(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
        if (ca != cb) return (int)ca - (int)cb;
        a++; b++;
    }
    return (int)(uint8)*a - (int)(uint8)*b;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断单精度浮点数是否为有限值
// 参数说明     value           待检查数值
// 返回参数     uint8           1 为有限值, 0 为 NaN 或正负无穷
// 使用示例     if (!cmd_float_is_finite(value)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_float_is_finite(float value)
{
    union
    {
        float  f;
        uint32 u;
    } bits;

    bits.f = value;
    return (uint8)((bits.u & 0x7F800000u) != 0x7F800000u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     严格解析一个十进制浮点参数
// 参数说明     text/value      输入字符串与解析结果地址
// 返回参数     uint8           1 解析成功, 0 表示格式或数值非法
// 使用示例     if (!cmd_parse_float_strict(tok[1], &value)) cmd_ack(0);
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_parse_float_strict(const char *text, float *value)
{
    const char *p = text;
    float result = 0.0f;
    float fraction = 0.1f;
    int sign = 1;
    int exponent_sign = 1;
    uint8 digit_seen = 0;
    uint8 exponent_seen = 0;
    uint16 exponent = 0;

    if (p == 0 || value == 0 || *p == '\0') return 0;
    if (*p == '-' || *p == '+')
    {
        if (*p == '-') sign = -1;
        p++;
    }

    while (*p >= '0' && *p <= '9')
    {
        digit_seen = 1;
        result = result * 10.0f + (float)(*p - '0');
        if (!cmd_float_is_finite(result)) return 0;
        p++;
    }
    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9')
        {
            digit_seen = 1;
            result += (float)(*p - '0') * fraction;
            fraction *= 0.1f;
            p++;
        }
    }
    if (!digit_seen) return 0;

    if (*p == 'e' || *p == 'E')
    {
        p++;
        if (*p == '-' || *p == '+')
        {
            if (*p == '-') exponent_sign = -1;
            p++;
        }
        while (*p >= '0' && *p <= '9')
        {
            exponent_seen = 1;
            exponent = (uint16)(exponent * 10u + (uint16)(*p - '0'));
            if (exponent > 38u) return 0;
            p++;
        }
        if (!exponent_seen) return 0;
    }
    if (*p != '\0') return 0;

    while (exponent > 0u)
    {
        result = (exponent_sign > 0) ? result * 10.0f : result * 0.1f;
        if (!cmd_float_is_finite(result)) return 0;
        exponent--;
    }
    result = (sign < 0) ? -result : result;
    if (!cmd_float_is_finite(result)) return 0;
    *value = result;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将命令行严格拆分为不超过三个字段
// 参数说明     line/tokens      可写命令行与字段指针数组
// 返回参数     int             字段数, -1 表示字段过多
// 使用示例     ntok = cmd_tokenize(line, tok);
//-------------------------------------------------------------------------------------------------------------------
static int cmd_tokenize(char *line, char **tokens)
{
    char *p = line;
    int count = 0;

    while (*p)
    {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') break;
        if (count >= 3) return -1;
        tokens[count++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    return count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断当前运行状态是否禁止在线改 PID
// 参数说明     void
// 返回参数     uint8           1 表示拒绝改参数
// 使用示例     if (cmd_motion_active()) cmd_ack(0);
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_motion_active(void)
{
    // 架空点动是开环下发固定占空比，PID 根本不在环里，改了没有意义，拒
    if (control_jog_running() != MOTOR_JOG_NONE) return 1;

    // START_BALANCE 是三轴一起跑的整定状态，八个环全在环里，整定本来就要边跑边改，放行。
    // 越界值仍由 param.c 钳位，NaN/Inf 仍由 cmd_parse_float_strict() 拒掉。
    // 其余发车状态照旧拒绝：START_DRIVE_ONLY 只有 C 轮在跑，改 Roll/Yaw 的增益看不出任何变化
    return (uint8)(start_flag != START_STOP && start_flag != START_BALANCE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断参数名是否属于当前测试轴已启用的串级范围
// 参数说明     name            参数名
// 返回参数     uint8           1 表示测试中允许在线修改
// 使用示例     if (!cmd_test_param_allowed(tok[1])) cmd_ack(0);
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_test_param_allowed(const char *name)
{
    uint8 ring;
    uint8 gain;

    if (name == 0 || g_tune_axis >= TUNE_AXIS_MAX ||
        g_tune_ring >= TUNE_RING_MAX)
        return 0;

    for (ring = 0; ring <= (uint8)g_tune_ring; ring++)
    {
        for (gain = 0; gain < 3u; gain++)
        {
            const char *allowed = s_tune_tbl[g_tune_axis][ring][gain];
            if (allowed != 0 && cmd_icmp(name, allowed) == 0) return 1;
        }
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送命令执行状态帧
// 参数说明     ok              1 成功，0 失败
// 返回参数     void
// 使用示例     cmd_ack(1);
//-------------------------------------------------------------------------------------------------------------------
static void cmd_ack(uint8 ok)
{
    cmd_send(ok ? "ack:1.000\n" : "ack:0.000\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     返回参数当前实际值，用于确认串口修改和范围处理结果
// 参数说明     value           参数当前值
// 返回参数     void
// 使用示例     cmd_value(0.002f);
//-------------------------------------------------------------------------------------------------------------------
static void cmd_value(float value)
{
    char line[48];
    int len = snprintf(line, sizeof(line), "value:%.6f\n", (double)value);

    if (len > 0 && len < (int)sizeof(line))
        (void)tx_push((const uint8 *)line, (uint32)len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断参数名是否属于串口调参白名单
// 参数说明     name            参数名
// 返回参数     uint8           1=允许串口读写 0=不允许
// 使用示例     if (!cmd_pid_param(tok[0])) cmd_ack(0);
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_pid_param(const char *name)
{
    uint8 axis, ring, gain;

    if (name == 0) return 0;
    for (axis = 0; axis < (uint8)TUNE_AXIS_MAX; axis++)
        for (ring = 0; ring < (uint8)TUNE_RING_MAX; ring++)
            for (gain = 0; gain < 3u; gain++)
            {
                const char *entry = s_tune_tbl[axis][ring][gain];
                if (entry != 0 && cmd_icmp(name, entry) == 0) return 1;
            }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并执行一行下行命令，支持修改参数和 get 读取参数
// 参数说明     line            已去掉换行符的命令字符串
// 返回参数     void
// 使用示例     cmd_execute("r_rate_kp 12.5");
//-------------------------------------------------------------------------------------------------------------------
static void cmd_execute(char *line)
{
    char *tok[3];
    int ntok = cmd_tokenize(line, tok);
    float value;
    const param_desc_t *desc;

    if (ntok == 0) return;                      // 空行不当错误
    if (ntok != 2) { cmd_ack(0); return; }

    // 查询不改变车辆状态，运行中也允许。回复先给 ACK，再给 value:<实际值>。
    if (cmd_icmp(tok[0], "get") == 0)
    {
        if (!cmd_pid_param(tok[1]) ||
            !param_get_by_name(tok[1], &value))
        {
            cmd_ack(0);
            return;
        }
        cmd_ack(1);
        cmd_value(value);
        return;
    }

    // 只放行八个环的 PID 与整定相关限幅/滤波参数
    if (!cmd_pid_param(tok[0])) { cmd_ack(0); return; }

    // 发车或架空点动期间不接受改参数
    if (cmd_motion_active()) { cmd_ack(0); return; }

    // 闭环测试期间锁定调参轴，只放行当前轴、当前最高启用环之内的 PID
    if (control_test_running() && !cmd_test_param_allowed(tok[0])) { cmd_ack(0); return; }

    if (!cmd_parse_float_strict(tok[1], &value)) { cmd_ack(0); return; }
    desc = param_find(tok[0]);
    if (desc == 0 || value < desc->vmin || value > desc->vmax)
    {
        cmd_ack(0);
        return;
    }
    if (!param_set_by_name(tok[0], value) ||
        !param_get_by_name(tok[0], &value))
    {
        cmd_ack(0);
        return;
    }
    cmd_ack(1);
    cmd_value(value);
}


// 外部接口
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据波形模式更新通道快照
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_snapshot();
//-------------------------------------------------------------------------------------------------------------------
void vofa_snapshot(void)
{
    vofa_mode_t m = g_vofa_mode;
    if (m == VOFA_OFF) return;

    s_seq++;                                    // 奇数表示正在写入
    switch (m)
    {
    case VOFA_ATT:
        // FireWater 通道顺序为 Roll、Pitch、Yaw
        s_tag = "att"; s_n = 3;
        s_ch[0] = att.roll;
        s_ch[1] = att.pitch;
        s_ch[2] = att.yaw;
        break;
    case VOFA_ROLL:
        // r_rcy_fb 是 B、A 两个飞轮的转速差(RPM)，由 CYT2BL3 每 10ms 回传
        s_tag = "roll"; s_n = 15;
        s_ch[0] = g_bal_dbg.r_rcy_set;  s_ch[1] = g_bal_dbg.r_rcy_fb;  s_ch[2] = g_bal_dbg.r_rcy_out;
        s_ch[3] = g_bal_dbg.r_ang_fb;   s_ch[4] = g_bal_dbg.r_ang_out;
        s_ch[5] = g_bal_dbg.r_rate_fb;  s_ch[6] = g_bal_dbg.r_pwm;
        s_ch[7] = (float)g_motor_a;     s_ch[8] = (float)g_motor_b;
        s_ch[9] = (float)W_Motor_GetSpeed1();
        s_ch[10] = (float)W_Motor_GetSpeed2();
        // 末尾四路是当前正在用的增益本身。整定时波形和参数得对得上，
        // 否则回放录下来的曲线根本分不清哪一段是哪组增益
        // Roll 内环是位置式，R_RATE_KI 恒为 0，发它没信息量，改发正在用的 kd
        s_ch[11] = R_RATE_KP;  s_ch[12] = R_RATE_KD;
        s_ch[13] = R_ANGLE_KP; s_ch[14] = R_RCY_KP;
        break;
    case VOFA_PITCH:
        s_tag = "pit"; s_n = 11;
        s_ch[0] = g_bal_dbg.p_vel_set;  s_ch[1] = g_bal_dbg.p_vel_fb;  s_ch[2] = g_bal_dbg.p_vel_out;
        s_ch[3] = g_bal_dbg.p_ang_fb;   s_ch[4] = g_bal_dbg.p_ang_out;
        s_ch[5] = g_bal_dbg.p_rate_fb;  s_ch[6] = g_bal_dbg.p_pwm;
        s_ch[7] = P_RATE_KP;   s_ch[8] = P_RATE_KI;
        s_ch[9] = P_ANGLE_KP;  s_ch[10] = P_VEL_KP;
        break;
    case VOFA_YAW:
        s_tag = "yaw"; s_n = 8;
        s_ch[0] = g_bal_dbg.y_set;      s_ch[1] = g_bal_dbg.y_fb;      s_ch[2] = g_bal_dbg.y_out;
        s_ch[3] = g_bal_dbg.y_rate_fb;  s_ch[4] = g_bal_dbg.y_pwm;     s_ch[5] = g_lean_offset;
        // Yaw 内环是位置式，ki/kd 恒 0，只发两个真正在调的
        s_ch[6] = Y_RATE_KP;   s_ch[7] = Y_ANGLE_KP;
        break;
    case VOFA_TRACK:
        // track_valid=0 时 err 被强制回 0，看 err 必须同时看 valid 和 both_lost
        s_tag = "trk"; s_n = 13;
        s_ch[0] = g_dbg_error;                  // 中线偏差，右偏为正
        s_ch[1] = (float)g_track_valid;         // 本帧循迹是否有效
        s_ch[2] = (float)g_vision_search_stop;  // 有效前瞻行数
        s_ch[3] = (float)g_vision_left_lost;    // 左边线丢线行数
        s_ch[4] = (float)g_vision_right_lost;   // 右边线丢线行数
        s_ch[5] = (float)g_vision_both_lost;    // 双边丢线行数
        s_ch[6] = (float)g_vision_threshold;    // 大津阈值
        s_ch[7] = (float)g_track_lost_frames;   // 连续无效帧计数
        s_ch[8] = (float)g_vision_heartbeat;    // CPU1 心跳
        s_ch[9] = (float)g_vision_age_ms;       // 距上一帧的时间(ms)
        s_ch[10] = (float)g_vision_active_elem; // 元素编号
        s_ch[11] = g_vision_speed_scale;        // 元素建议速度倍率
        s_ch[12] = (float)g_vision_stop_request;// 元素停车请求
        break;
    case VOFA_MOTOR:
        // 架空验方向用: 指令与回传转速同号才说明 MOTOR_DIR 配对，
        // 指令过零时看 rpm 是平滑穿零还是被驱动硬刹车拽到 0。
        s_tag = "mot"; s_n = 5;
        s_ch[0] = (float)g_motor_a;             // A 轮占空比指令
        s_ch[1] = (float)g_motor_b;             // B 轮占空比指令
        s_ch[2] = (float)W_Motor_GetSpeed1();   // A 轮回传转速(RPM)
        s_ch[3] = (float)W_Motor_GetSpeed2();   // B 轮回传转速(RPM)
        s_ch[4] = (float)Y_Motor_GetSpeed20ms();// C 轮编码器增量(counts/20ms)
        break;
    case VOFA_BAL:
        // 三轴一起跑时看的是"站没站住"和"飞轮有没有单向堆积"，各环内部量看单轴波形。
        // CH0/CH1 与 CH2/CH3 成对，角度贴住目标就是站住了；
        // CH8/CH9 同向一起爬升说明回收环压不住，再跑下去就要闩驱动堵转保护。
        s_tag = "bal"; s_n = 15;
        s_ch[0] = att.roll;                     // 横滚角(°)
        s_ch[1] = g_roll_zero + g_lean_offset;  // 横滚有效目标(°)，含压弯动态零点
        s_ch[2] = att.pitch;                    // 俯仰角(°)
        s_ch[3] = g_pitch_zero;                 // 俯仰机械零点(°)
        s_ch[4] = g_bal_dbg.y_set - g_bal_dbg.y_fb;     // 航向误差(°)，连续角相减
        s_ch[5] = (float)g_motor_a;             // 混控后 A 轮指令
        s_ch[6] = (float)g_motor_b;             // 混控后 B 轮指令
        s_ch[7] = (float)g_motor_c;             // 混控后 C 轮指令
        s_ch[8] = (float)W_Motor_GetSpeed1();   // A 轮回传转速(RPM)
        s_ch[9] = (float)W_Motor_GetSpeed2();   // B 轮回传转速(RPM)
        s_ch[10] = (float)Y_Motor_GetSpeed20ms();       // C 轮编码器增量(counts/20ms)
        // 三轴一起跑的时候只带两个内环的增益，它俩决定站不站得住；
        // 外环增益去对应轴的单轴波形上看
        s_ch[11] = R_RATE_KP;  s_ch[12] = R_RATE_KI;
        s_ch[13] = P_RATE_KP;  s_ch[14] = P_RATE_KI;
        break;
    default:
        s_n = 0;
        break;
    }
    s_seq++;                                    // 偶数表示写入完成
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按发送分频输出最新波形快照
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_poll();
//-------------------------------------------------------------------------------------------------------------------
void vofa_poll(void)
{
    static uint32 last_seq = 0;                 // 上次已发送的快照序号
    float  ch[VOFA_CH_MAX];
    char   line[256];
    uint32 seq1, seq2;
    uint8  n, i;
    const char *tag;
    int    len;

    if (g_vofa_mode == VOFA_OFF) { last_seq = s_seq; return; }

    seq1 = s_seq;
    if (seq1 & 1u) return;                                          // 快照正在更新
    if ((seq1 - last_seq) < (uint32)(2u * g_vofa_div)) return;      // 未达到发送分频

    n = s_n; tag = (const char *)s_tag;
    if (n == 0 || n > VOFA_CH_MAX) return;
    for (i = 0; i < n; i++) ch[i] = s_ch[i];
    seq2 = s_seq;
    if (seq1 != seq2) return;                                       // 快照读取不完整

    last_seq = seq1;

    // FireWater 格式为 <tag>:v0,v1,...\n
    len = snprintf(line, sizeof(line), "%s:", tag);
    for (i = 0; i < n && len > 0 && len < (int)sizeof(line) - 16; i++)
        len += snprintf(line + len, sizeof(line) - (uint32)len, (i == 0) ? "%.3f" : ",%.3f", (double)ch[i]);
    if (len <= 0) return;                                           // 格式化失败
    if (len > (int)sizeof(line) - 2) len = (int)sizeof(line) - 2;   // 限制实际缓冲长度
    line[len++] = '\n'; line[len] = '\0';
    (void)tx_push_wave((const uint8 *)line, (uint32)len);           // 写入上行环，装不下就丢帧并保留回复空间
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化无线转串口模块与上行发送队列
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_init();
//-------------------------------------------------------------------------------------------------------------------
void vofa_init(void)
{
    // 模块 RX 接 P10_5(MCU TX)、TX 接 P10_6(MCU RX)、RTS 接 P10_2，引脚与波特率都在
    // zf_device_wireless_uart.h 里定义。自动波特率关着，模块保持出厂 115200，
    // 这条路径里 wireless_uart_init() 不会失败，返回值没有信息量。
    (void)wireless_uart_init();

    s_tx_head = 0; s_tx_tail = 0;
    s_rx_head = 0; s_rx_tail = 0;
    s_rx_overflow = 0;
    s_cmd_len = 0; s_cmd_ovf = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     搬运一拍串口字节，由 1ms 控制中断调用。只搬字节，不做格式化和解析
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_tick1ms();
//-------------------------------------------------------------------------------------------------------------------
void vofa_tick1ms(void)
{
    uint8  buf[32];
    uint32 got, i;
    uint32 head;
    uint32 tail;

    // 收：把库里那 64 字节缓冲倒进自己的环。1ms 一次，115200 下一拍最多来 12 字节，
    // 无论前台在刷屏还是在解析都不会溢出。读的时候要挡住 UART2 接收中断，
    // 那个中断优先级 17 低于本中断的 30，会被本中断打断在 fifo 写一半的位置上。
    {
        uint32 interrupt_state = interrupt_global_disable();
        got = wireless_uart_read_buffer(buf, (uint32)sizeof(buf));
        interrupt_global_enable(interrupt_state);
    }

    head = s_rx_head;
    for (i = 0; i < got; i++)
    {
        if ((uint32)(head - s_rx_tail) >= VOFA_RX_SIZE)
        {
            s_rx_overflow = 1;                                // 当前命令已截断，前台必须整行作废
            break;
        }
        s_rx_buf[head & VOFA_RX_MASK] = buf[i];
        head++;
    }
    s_rx_head = head;

    // 发：RTS 是无线模块的流控输出，P10_2 带上拉。高电平表示模块内部缓冲满，
    // 此时继续灌数据会被模块丢掉，所以整拍不发，字节留在环里下一拍再说。
    // 模块没插、没供电或者挂了，这里会一直读到高电平。
    // 库里的 wireless_uart_send_buffer() 遇到这种情况靠 system_delay_ms 死等，中断里不能用。
    if (gpio_get_level(WIRELESS_UART_RTS_PIN)) return;

    // 一拍最多补满 16 字节硬件 FIFO。115200 下一拍只能发走约 12 字节，
    // 所以每拍都能把 FIFO 顶满，实际吞吐就是波特率本身，与主循环刷屏彻底无关。
    tail = s_tx_tail;
    while (tail != s_tx_head &&
           IfxAsclin_getTxFifoFillLevel(uart2_handle.asclin) < VOFA_TX_FIFO_DEPTH)
    {
        IfxAsclin_writeTxData(uart2_handle.asclin, s_tx_buf[tail & VOFA_TX_MASK]);
        tail++;
    }
    s_tx_tail = tail;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并执行无线串口下行调参命令，由主循环调用
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_cmd_poll();
//-------------------------------------------------------------------------------------------------------------------
void vofa_cmd_poll(void)
{
    uint32 tail = s_rx_tail;
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    if (s_rx_overflow)
    {
        s_rx_overflow = 0;
        s_cmd_ovf = 1;                      // 丢弃直到本行结束，防止截断命令被误执行
    }
    interrupt_global_enable(interrupt_state);

    while (tail != s_rx_head)
    {
        char c = (char)s_rx_buf[tail & VOFA_RX_MASK];
        tail++;

        if (c == '\n' || c == '\r')
        {
            if (s_cmd_ovf)
            {
                cmd_ack(0);
            }
            else if (s_cmd_len > 0)
            {
                s_cmd_line[s_cmd_len] = '\0';
                cmd_execute(s_cmd_line);
            }
            s_cmd_len = 0;
            s_cmd_ovf = 0;
            continue;
        }
        if (s_cmd_len < (VOFA_CMD_LINE_MAX - 1))
            s_cmd_line[s_cmd_len++] = c;
        else
            s_cmd_ovf = 1;                      // 标记命令行溢出，整行作废
    }
    s_rx_tail = tail;
}
