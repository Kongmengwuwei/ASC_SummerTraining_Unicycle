#include "vofa.h"
#include "W_Motor.h"
#include "Y_Motor.h"
#include "board_config.h"
#include "param.h"
#include "imu.h"
#include "attitude.h"
#include "control.h"
#include "display.h"
#include <stdio.h>

// 全局状态
volatile vofa_mode_t g_vofa_mode = VOFA_OFF;    // 当前波形模式
volatile uint8       g_vofa_div  = 20;          // 发送分频，单位为 1ms
volatile tune_axis_t g_tune_axis = TUNE_AXIS_ROLL;  // 当前调参轴
volatile tune_ring_t g_tune_ring = TUNE_RING_RATE;  // 当前调参环

// 波形快照
#define VOFA_CH_MAX     (13)                    // 单帧最大通道数

static volatile uint32      s_seq = 0;          // 快照序号
static volatile float       s_ch[VOFA_CH_MAX];  // 通道值
static volatile uint8       s_n = 0;            // 通道数
static volatile const char *s_tag = "off";      // 帧标签

// 下行命令缓冲
#define VOFA_CMD_LINE_MAX   (64)                // 单行命令长度
#define VOFA_CMD_READ_MAX   (32)                // 单次读取长度

static char  s_cmd_line[VOFA_CMD_LINE_MAX];     // 命令行缓冲
static uint8 s_cmd_len = 0;                     // 命令行长度
static uint8 s_cmd_ovf = 0;                     // 命令行溢出标志

// UART0 非阻塞发送队列
#define VOFA_TX_FIFO_DEPTH  (16)                // 硬件发送 FIFO 深度
#define VOFA_TX_UART0_SIZE  (2048)              // 软件发送队列长度

typedef struct
{
    fifo_struct  fifo;                          // 字节环形缓冲
    Ifx_ASCLIN  *asclin;                        // 串口寄存器组
} vofa_tx_t;

static vofa_tx_t s_tx_uart0;                    // UART0 发送状态
static uint8     s_tx_uart0_buf[VOFA_TX_UART0_SIZE];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将完整数据帧写入发送队列
// 参数说明     tx 目标队列，dat 数据地址，len 数据长度
// 返回参数     uint8           1 写入成功，0 写入失败
// 使用示例     (void)tx_push(&s_tx_uart0, (const uint8 *)line, len);
//-------------------------------------------------------------------------------------------------------------------
static uint8 tx_push(vofa_tx_t *tx, const uint8 *dat, uint32 len)
{
    return (uint8)(len != 0u &&
                   fifo_write_buffer(&tx->fifo, (void *)dat, len) == FIFO_SUCCESS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将队列数据写入硬件发送 FIFO
// 参数说明     tx              目标队列
// 返回参数     void
// 使用示例     tx_pump(&s_tx_uart0);
//-------------------------------------------------------------------------------------------------------------------
static void tx_pump(vofa_tx_t *tx)
{
    uint8 b;

    while (fifo_used(&tx->fifo) != 0u &&
           IfxAsclin_getTxFifoFillLevel(tx->asclin) < VOFA_TX_FIFO_DEPTH)
    {
        if (fifo_read_element(&tx->fifo, &b, FIFO_READ_AND_CLEAN) != FIFO_SUCCESS) break;
        IfxAsclin_writeTxData(tx->asclin, b);
    }
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

// 字符串工具

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     比较两个 ASCII 字符串并忽略大小写
// 参数说明     a 第一个字符串，b 第二个字符串
// 返回参数     int             0 表示相等
// 使用示例     if (cmd_icmp(tok, "axis") == 0)
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
            if (exponent > 38u) return 0;
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
// 函数简介     判断点动或完整发车流程是否正在占用电机
// 参数说明     void
// 返回参数     uint8           1 表示禁止改变调参对象
// 使用示例     if (cmd_motion_active()) cmd_ack(0);
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_motion_active(void)
{
    return (uint8)(start_flag != START_STOP ||
                   control_jog_running() != MOTOR_JOG_NONE);
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
// 函数简介     将一行 ASCII 数据写入 UART0 发送队列
// 参数说明     line            以 '\0' 结尾且包含换行符的字符串
// 返回参数     void
// 使用示例     cmd_send("ack:1\n");
//-------------------------------------------------------------------------------------------------------------------
static void cmd_send(const char *line)
{
    uint32 n = 0;
    while (line[n] != '\0' && n < 200u) n++;
    if (n) (void)tx_push(&s_tx_uart0, (const uint8 *)line, n);
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
// 函数简介     发送当前调参轴和调参环的 PID 参数
// 参数说明     void
// 返回参数     void
// 使用示例     cmd_report_pid();
//-------------------------------------------------------------------------------------------------------------------
static void cmd_report_pid(void)
{
    char  line[96];
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    const char *const *names = s_tune_tbl[g_tune_axis][g_tune_ring];

    if (names[0]) (void)param_get_by_name(names[0], &kp);
    if (names[1]) (void)param_get_by_name(names[1], &ki);
    if (names[2]) (void)param_get_by_name(names[2], &kd);

    (void)snprintf(line, sizeof(line), "pid:%d.000,%d.000,%.4f,%.4f,%.4f\n",
                   (int)g_tune_axis, (int)g_tune_ring, (double)kp, (double)ki, (double)kd);
    cmd_send(line);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送指定参数的当前值
// 参数说明     index           参数表下标
// 返回参数     void
// 使用示例     cmd_report_param(3);
//-------------------------------------------------------------------------------------------------------------------
static void cmd_report_param(uint16 index)
{
    char  line[64];
    float v = 0.0f;
    if (index >= param_table_count()) return;
    (void)param_get_by_name(g_param_table[index].name, &v);
    (void)snprintf(line, sizeof(line), "par:%u.000,%.4f\n", (unsigned)index, (double)v);
    cmd_send(line);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并执行一行下行命令
// 参数说明     line            已移除换行符的命令字符串
// 返回参数     void
// 使用示例     cmd_execute("kp 12.5");
//-------------------------------------------------------------------------------------------------------------------
static void cmd_execute(char *line)
{
    char *tok[3];
    int ntok = cmd_tokenize(line, tok);

    if (ntok < 0) { cmd_ack(0); return; }
    if (ntok == 0) return;

    // 切换调参轴
    if (cmd_icmp(tok[0], "axis") == 0)
    {
        tune_axis_t axis;

        if (ntok != 2 || cmd_motion_active() || control_test_running())
        {
            cmd_ack(0);
            return;
        }
        if      (cmd_icmp(tok[1], "roll")  == 0) axis = TUNE_AXIS_ROLL;
        else if (cmd_icmp(tok[1], "pitch") == 0) axis = TUNE_AXIS_PITCH;
        else if (cmd_icmp(tok[1], "yaw")   == 0) axis = TUNE_AXIS_YAW;
        else if (cmd_icmp(tok[1], "next")  == 0)
            axis = (tune_axis_t)(((int)g_tune_axis + 1) % (int)TUNE_AXIS_MAX);
        else { cmd_ack(0); return; }

        if (axis == TUNE_AXIS_YAW && g_tune_ring == TUNE_RING_VEL)
        {
            cmd_ack(0);
            return;
        }
        g_tune_axis = axis;
        cmd_report_pid(); cmd_ack(1); return;
    }

    // 切换调参环
    if (cmd_icmp(tok[0], "ring") == 0)
    {
        tune_ring_t ring;

        if (ntok != 2 || cmd_motion_active() || control_test_running())
        {
            cmd_ack(0);
            return;
        }
        if      (cmd_icmp(tok[1], "rate")  == 0) ring = TUNE_RING_RATE;
        else if (cmd_icmp(tok[1], "angle") == 0) ring = TUNE_RING_ANGLE;
        else if (cmd_icmp(tok[1], "vel")   == 0) ring = TUNE_RING_VEL;
        else if (cmd_icmp(tok[1], "next")  == 0)
            ring = (tune_ring_t)(((int)g_tune_ring + 1) % (int)TUNE_RING_MAX);
        else { cmd_ack(0); return; }

        if (g_tune_axis == TUNE_AXIS_YAW && ring == TUNE_RING_VEL)
        {
            cmd_ack(0);
            return;
        }
        g_tune_ring = ring;
        cmd_report_pid(); cmd_ack(1); return;
    }

    // 修改当前轴和环的增益
    if (cmd_icmp(tok[0], "kp")  == 0 || cmd_icmp(tok[0], "kpm") == 0 ||
        cmd_icmp(tok[0], "ki")  == 0 || cmd_icmp(tok[0], "kd")  == 0)
    {
        int which = (cmd_icmp(tok[0], "ki") == 0) ? 1 : ((cmd_icmp(tok[0], "kd") == 0) ? 2 : 0);
        const char *name;
        float value;

        if (ntok != 2 || cmd_motion_active() ||
            g_tune_axis >= TUNE_AXIS_MAX || g_tune_ring >= TUNE_RING_MAX ||
            !cmd_parse_float_strict(tok[1], &value))
        {
            cmd_ack(0);
            return;
        }
        name = s_tune_tbl[g_tune_axis][g_tune_ring][which];
        if (name == 0 || !param_set_by_name(name, value)) { cmd_ack(0); return; }
        cmd_report_pid(); cmd_ack(1); return;
    }

    // 按名称修改参数
    if (cmd_icmp(tok[0], "set") == 0)
    {
        float value;
        uint8 test_active = control_test_running();

        if (ntok != 3 || cmd_motion_active() ||
            (test_active && !cmd_test_param_allowed(tok[1])) ||
            !cmd_parse_float_strict(tok[2], &value))
        {
            cmd_ack(0);
            return;
        }
        if (param_set_by_name(tok[1], value))
        {
            if (cmd_icmp(tok[1], "cam_exposure") == 0)
                display_set_exposure((uint16)g_param.cam_exposure);
            cmd_ack(1);
        }
        else
        {
            cmd_ack(0);
        }
        return;
    }

    // 读取指定参数
    if (cmd_icmp(tok[0], "get") == 0)
    {
        uint16 i;
        if (ntok != 2) { cmd_ack(0); return; }
        for (i = 0; i < param_table_count(); i++)
            if (cmd_icmp(tok[1], g_param_table[i].name) == 0) { cmd_report_param(i); cmd_ack(1); return; }
        cmd_ack(0); return;
    }

    // 回传全部参数
    if (cmd_icmp(tok[0], "list") == 0)
    {
        uint16 i;
        if (ntok != 1) { cmd_ack(0); return; }
        for (i = 0; i < param_table_count(); i++) cmd_report_param(i);
        cmd_ack(1); return;
    }

    // 保存参数
    if (cmd_icmp(tok[0], "save") == 0)
    {
        if (ntok != 1 || cmd_motion_active() || control_test_running())
        {
            cmd_ack(0);
            return;
        }
        cmd_ack(param_save());
        return;
    }

    // 切换波形模式
    if (cmd_icmp(tok[0], "wave") == 0)
    {
        int m;
        if (ntok != 2 || cmd_motion_active() || control_test_running())
        {
            cmd_ack(0);
            return;
        }
        if      (cmd_icmp(tok[1], "off")   == 0) m = (int)VOFA_OFF;
        else if (cmd_icmp(tok[1], "imu")   == 0) m = (int)VOFA_IMU_RAW;
        else if (cmd_icmp(tok[1], "att")   == 0) m = (int)VOFA_ATT;
        else if (cmd_icmp(tok[1], "roll")  == 0) m = (int)VOFA_ROLL;
        else if (cmd_icmp(tok[1], "pitch") == 0 ||
                 cmd_icmp(tok[1], "pit")   == 0) m = (int)VOFA_PITCH;
        else if (cmd_icmp(tok[1], "yaw")   == 0) m = (int)VOFA_YAW;
        else if (cmd_icmp(tok[1], "track") == 0 ||
                 cmd_icmp(tok[1], "trk")   == 0) m = (int)VOFA_TRACK;
        else if (cmd_icmp(tok[1], "motor") == 0 ||
                 cmd_icmp(tok[1], "mot")   == 0) m = (int)VOFA_MOTOR;
        else if (cmd_icmp(tok[1], "dash")  == 0) m = (int)VOFA_DASH;
        else { cmd_ack(0); return; }
        g_vofa_mode = (vofa_mode_t)m;
        cmd_ack(1); return;
    }

    // 通信心跳
    if (cmd_icmp(tok[0], "ping") == 0)
    {
        cmd_ack((uint8)(ntok == 1));
        return;
    }

    cmd_ack(0);
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
    case VOFA_IMU_RAW:
        s_tag = "imu"; s_n = 6;
        s_ch[0] = imu.acc_x;  s_ch[1] = imu.acc_y;  s_ch[2] = imu.acc_z;
        s_ch[3] = imu.gyro_x; s_ch[4] = imu.gyro_y; s_ch[5] = imu.gyro_z;
        break;
    case VOFA_ATT:
        // FireWater 通道顺序为 Roll、Pitch、Yaw
        s_tag = "att"; s_n = 3;
        s_ch[0] = att.roll;
        s_ch[1] = att.pitch;
        s_ch[2] = att.yaw;
        break;
    case VOFA_ROLL:
        // r_rcy_fb 是 B、A 两个飞轮的转速差(RPM)，由 CYT2BL3 每 10ms 回传
        s_tag = "roll"; s_n = 7;
        s_ch[0] = g_bal_dbg.r_rcy_set;  s_ch[1] = g_bal_dbg.r_rcy_fb;  s_ch[2] = g_bal_dbg.r_rcy_out;
        s_ch[3] = g_bal_dbg.r_ang_fb;   s_ch[4] = g_bal_dbg.r_ang_out;
        s_ch[5] = g_bal_dbg.r_rate_fb;  s_ch[6] = g_bal_dbg.r_pwm;
        break;
    case VOFA_PITCH:
        s_tag = "pit"; s_n = 7;
        s_ch[0] = g_bal_dbg.p_vel_set;  s_ch[1] = g_bal_dbg.p_vel_fb;  s_ch[2] = g_bal_dbg.p_vel_out;
        s_ch[3] = g_bal_dbg.p_ang_fb;   s_ch[4] = g_bal_dbg.p_ang_out;
        s_ch[5] = g_bal_dbg.p_rate_fb;  s_ch[6] = g_bal_dbg.p_pwm;
        break;
    case VOFA_YAW:
        s_tag = "yaw"; s_n = 6;
        s_ch[0] = g_bal_dbg.y_set;      s_ch[1] = g_bal_dbg.y_fb;      s_ch[2] = g_bal_dbg.y_out;
        s_ch[3] = g_bal_dbg.y_rate_fb;  s_ch[4] = g_bal_dbg.y_pwm;     s_ch[5] = g_lean_offset;
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
    case VOFA_DASH:
        // 前三通道用于三维姿态显示
        s_tag = "dash"; s_n = 9;
        s_ch[0] = att.roll;
        s_ch[1] = att.pitch;
        s_ch[2] = att.yaw;
        s_ch[3] = g_bal_dbg.p_vel_fb;           // 行进轮速度
        s_ch[4] = g_dbg_error;                  // 中线偏差
        s_ch[5] = (float)g_motor_a;             // 电机 A 输出
        s_ch[6] = (float)g_motor_b;             // 电机 B 输出
        s_ch[7] = (float)g_motor_c;             // 电机 C 输出
        s_ch[8] = g_bal_dbg.p_vel_set;          // 目标速度
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
    (void)tx_push(&s_tx_uart0, (const uint8 *)line, (uint32)len);   // 写入发送队列
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     上行发送队列初始化
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_init();
//-------------------------------------------------------------------------------------------------------------------
void vofa_init(void)
{
    (void)fifo_init(&s_tx_uart0.fifo, FIFO_DATA_8BIT, s_tx_uart0_buf, VOFA_TX_UART0_SIZE);
    s_tx_uart0.asclin = uart0_handle.asclin;        // 烧录器虚拟串口 UART0
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将发送队列数据写入 UART0 硬件 FIFO
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_tx_pump();
//-------------------------------------------------------------------------------------------------------------------
void vofa_tx_pump(void)
{
    tx_pump(&s_tx_uart0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收并执行 UART0 下行调参命令
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_cmd_poll();
//-------------------------------------------------------------------------------------------------------------------
void vofa_cmd_poll(void)
{
    uint8  buf[VOFA_CMD_READ_MAX];
    uint32 got, k;

    got = debug_read_ring_buffer(buf, (uint32)VOFA_CMD_READ_MAX);
    for (k = 0; k < got; k++)
    {
        char c = (char)buf[k];

        if (c == '\n' || c == '\r')
        {
            if (!s_cmd_ovf && s_cmd_len > 0)
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
            s_cmd_ovf = 1;                      // 标记命令行溢出
    }
}
