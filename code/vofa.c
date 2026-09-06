#include "vofa.h"
#include "attitude.h"
#include "board_config.h"
#include "control.h"
#include "param.h"
#include "imu.h"
#include "W_Motor.h"
#include "Y_Motor.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// 全局状态
volatile vofa_mode_t g_vofa_mode = VOFA_OFF;    // 当前波形模式
volatile uint8       g_vofa_div  = 20;          // 发送分频，单位为 1ms
volatile tune_axis_t g_tune_axis = TUNE_AXIS_ROLL;  // 当前调参轴
volatile tune_ring_t g_tune_ring = TUNE_RING_RATE;  // 当前调参环
volatile uint32      g_vofa_rx_bytes = 0;       // 无线下行累计接收字节数
volatile uint32      g_vofa_cmd_lines = 0;      // 已提交解析的整行数，只由主循环写
volatile uint32      g_vofa_cmd_ok = 0;         // 已写入控制目标的命令数，只由主循环写
volatile vofa_cmd_result_t g_vofa_cmd_last = VOFA_CMD_NONE;  // 最近一行的处理结果
volatile float       g_vofa_cmd_turn = 0.0f;    // 最近一次解析成功的转向值(°)，与是否被接受无关
volatile float       g_vofa_cmd_speed = 0.0f;   // 最近一次解析成功的速度原始值(-90~90)

// 波形快照。ISR 只更新定长字段，前台复制完整快照后再格式化。
#define VOFA_ATT_CH_COUNT   (3u)

static volatile uint32             s_seq = 0;   // 快照序号，奇数=正在写
static volatile float              s_att_ch[VOFA_ATT_CH_COUNT];
static volatile control_run_diag_t s_run_diag;

// 下行命令缓冲
#define VOFA_CMD_LINE_MAX   (64)                // 单行命令长度
#define VOFA_CMD_STALE_MS   (1000u)             // 未收到 CR/LF 的残行作废时间

static char  s_cmd_line[VOFA_CMD_LINE_MAX];     // 命令行缓冲
static uint8 s_cmd_len = 0;                     // 命令行长度
static uint8 s_cmd_ovf = 0;                     // 命令行溢出标志
static volatile uint16 s_rx_idle_ms = 0;        // 距最后一个下行字节的时间

#define VOFA_TX_FIFO_DEPTH  (16u)               // ASCLIN 硬件发送 FIFO 深度
#define VOFA_TX_SIZE        (2048u)             // 发送环长度，必须是 2 的幂
#define VOFA_RX_SIZE        (1024u)             // 接收环长度，必须是 2 的幂。
                                                // 115200 下 1024 字节约 89ms，够盖住一次满屏刷新
#define VOFA_TX_MASK        (VOFA_TX_SIZE - 1u)
#define VOFA_RX_MASK        (VOFA_RX_SIZE - 1u)

typedef char vofa_ring_size_is_pow2[((VOFA_TX_SIZE & VOFA_TX_MASK) == 0u &&
                                     (VOFA_RX_SIZE & VOFA_RX_MASK) == 0u) ? 1 : -1];

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
static uint32 s_cfg_rx_error, s_cfg_tx_drop;
static uint16 s_schema_seq, s_schema_index, s_save_seq;
static uint32 s_schema_next_ms, s_stop_since, s_cfg_last_status;
static uint8 s_stop_seen, s_station_att;
static char s_save_group[24];

static uint8 tx_push(const uint8 *dat, uint32 len)
{
    uint32 head = s_tx_head;
    uint32 used = head - s_tx_tail;           
    uint32 i;

    if (len == 0u || len > (VOFA_TX_SIZE - used)) { s_cfg_tx_drop++; return 0; }

    for (i = 0; i < len; i++) s_tx_buf[(head + i) & VOFA_TX_MASK] = dat[i];
    __dsync();
    s_tx_head = head + len;                     // 数据写完再发布下标
    return 1;
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
// 使用示例     if (!cmd_parse_float_strict(text, &value)) return;
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
// 函数简介     判断命令是否以 speed: 开头，忽略 speed 的大小写
// 参数说明     line            命令行
// 返回参数     uint8           1=无线遥控命令 0=其他命令
// 使用示例     if (cmd_is_speed(line)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_is_speed(const char *line)
{
    static const char prefix[] = "speed:";
    uint8 i;

    if (line == 0) return 0;
    for (i = 0; i < 6u; i++)
    {
        char c = line[i];
        char p = prefix[i];
        if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
        if (c != p) return 0;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     去掉字段首尾的空格和制表符
// 参数说明     text            可写字符串
// 返回参数     char*           去掉前导空白后的首地址
// 使用示例     field = cmd_trim(field);
//-------------------------------------------------------------------------------------------------------------------
static char *cmd_trim(char *text)
{
    char *end;

    while (*text == ' ' || *text == '\t') text++;
    end = text;
    while (*end != '\0') end++;
    while (end > text && (end[-1] == ' ' || end[-1] == '\t')) end--;
    *end = '\0';
    return text;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并执行 speed:<相对航向角>,<速度原始值> 无线遥控命令
// 参数说明     line            已去掉换行符的命令字符串
// 返回参数     vofa_cmd_result_t 处理结果，APPLIED 表示目标已写入控制层
// 使用示例     result = cmd_execute_speed(line);
//-------------------------------------------------------------------------------------------------------------------
static vofa_cmd_result_t cmd_execute_speed(char *line)
{
    char *turn_text;
    char *speed_text;
    char *comma;
    float steer_angle;
    float speed_raw;
    float speed_mps;

    if (!cmd_is_speed(line)) return VOFA_CMD_PREFIX;

    turn_text = line + 6;
    comma = turn_text;
    while (*comma != '\0' && *comma != ',') comma++;
    if (*comma != ',')
    {
        return VOFA_CMD_FORMAT;                 // 没有逗号分隔
    }
    *comma = '\0';
    speed_text = comma + 1;
    if (*speed_text == '\0')
    {
        return VOFA_CMD_FORMAT;                 // 逗号后面是空的
    }
    comma = speed_text;
    while (*comma != '\0')
    {
        if (*comma == ',')
        {
            return VOFA_CMD_FORMAT;             // 多于一个逗号
        }
        comma++;
    }

    turn_text = cmd_trim(turn_text);
    speed_text = cmd_trim(speed_text);
    if (!cmd_parse_float_strict(turn_text, &steer_angle) ||
        !cmd_parse_float_strict(speed_text, &speed_raw))
    {
        return VOFA_CMD_FORMAT;                 // 数字本身非法
    }

    // 解析成功即回显原始输入，便于在停车状态检查无线链路与命令格式。
    g_vofa_cmd_turn = steer_angle;
    g_vofa_cmd_speed = speed_raw;

    if (steer_angle < -REMOTE_STEER_INPUT_LIMIT ||
        steer_angle > REMOTE_STEER_INPUT_LIMIT ||
        speed_raw < -REMOTE_SPEED_INPUT_LIMIT ||
        speed_raw > REMOTE_SPEED_INPUT_LIMIT)
        return VOFA_CMD_RANGE;

    speed_mps = speed_raw / REMOTE_SPEED_INPUT_DIVISOR;
    if (control_remote_command(steer_angle, speed_mps)) return VOFA_CMD_APPLIED;

    return control_remote_running() ? VOFA_CMD_RANGE : VOFA_CMD_NOT_RUNNING;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     提交一整行由 CR/LF 结束的下行命令并记账
// 参数说明     void
// 返回参数     void
// 使用示例     cmd_submit_line();
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断整行是不是 stop 命令，忽略大小写
// 参数说明     line            已去掉首尾空白的命令行
// 返回参数     uint8           1=是 stop
// 使用示例     if (cmd_is_stop(line)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_is_stop(const char *line)
{
    static const char word[] = "stop";
    uint8 i;

    if (line == 0) return 0;
    for (i = 0; i < 4u; i++)
    {
        char c = line[i];

        if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
        if (c != word[i]) return 0;
    }
    return (uint8)(line[4] == '\0');            // 必须整行就是 stop，不接受后缀
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断命令行缓冲的末尾是不是某条命令的开头
// 参数说明     void
// 返回参数     uint8           匹配到的前缀长度，0=没匹配上
// 使用示例     uint8 n = cmd_tail_prefix_len();
//-------------------------------------------------------------------------------------------------------------------
//
// 用来在上位机没发 CR/LF 时重新同步。两条命令的前缀都要认，否则
// "speed:1,2stop" 这种粘连里的 stop 会被当成上一条命令的一部分丢掉。
// 浮点字段里只可能出现数字和 . - + e，拼不出 stop 或 speed:，不会误切。
/* UART2 cfg v1. All functions below execute only in CPU0 foreground. */
static const char *const s_cfg_pid_names[] = {
    "r_rate_kp", "r_rate_ki", "r_rate_kd", "r_angle_kp", "r_angle_ki", "r_angle_kd", "r_rcy_kp", "r_rcy_ki", "r_rcy_kd",
    "p_rate_kp", "p_rate_ki", "p_rate_kd", "p_angle_kp", "p_angle_ki", "p_angle_kd", "p_vel_kp", "p_vel_ki", "p_vel_kd",
    "y_rate_kp", "y_rate_ki", "y_rate_kd", "y_angle_kp", "y_angle_ki", "y_angle_kd"
};

static int cfg_pid_index(const char *name)
{
    uint8 i;
    for (i = 0; i < 24u; i++) if (strcmp(name, s_cfg_pid_names[i]) == 0) return (int)i;
    return -1;
}

static const char *cfg_group(const char *n)
{
    if (strncmp(n, "r_", 2) == 0) return "Roll";
    if (strncmp(n, "p_", 2) == 0) return "Pitch";
    if (strncmp(n, "y_", 2) == 0) return "Yaw";
    if (strncmp(n, "lean_", 5) == 0) return "Lean";
    if (strncmp(n, "ipm_", 4) == 0) return "IPM";
    if (strncmp(n, "odom_", 5) == 0) return "Odometry";
    if (strncmp(n, "run_", 4) == 0 || strncmp(n, "direction_", 10) == 0) return "Run";
    if (strncmp(n, "elem_", 5) == 0 || strncmp(n, "ring_", 5) == 0 || strncmp(n, "zebra_", 6) == 0) return "Element";
    if (strncmp(n, "cam_", 4) == 0 || strcmp(n, "err_front_row") == 0) return "Camera";
    if (strncmp(n, "roll_", 5) == 0 || strncmp(n, "pitch_", 6) == 0) return "Zero";
    if (strncmp(n, "motor_", 6) == 0 || strncmp(n, "enc_", 4) == 0 || strncmp(n, "fly_", 4) == 0 ||
        strncmp(n, "jog_", 4) == 0 || strcmp(n, "steer_dir") == 0) return "Motor";
    return "Other";
}

static uint8 cfg_dangerous(const char *name)
{
    const char *group = cfg_group(name);
    return (uint8)(strcmp(group, "Motor") == 0 || strcmp(group, "Zero") == 0 || strcmp(group, "IPM") == 0);
}

static uint8 cfg_stopped(void)
{
    return (uint8)(start_flag == START_STOP && !control_test_running() &&
        control_jog_running() == MOTOR_JOG_NONE && !control_run_running() && !control_remote_running());
}

static uint32 cfg_pid_mask(void)
{
    uint32 mask;
    uint8 rings, shift;
    if (control_jog_running() != MOTOR_JOG_NONE) return 0;
    if (control_test_running())
    {
        rings = (uint8)g_tune_ring + 1u;
        if (rings > 3u || g_tune_axis >= TUNE_AXIS_MAX) return 0;
        if (g_tune_axis == TUNE_AXIS_YAW && rings > 2u) rings = 2u;
        shift = (uint8)g_tune_axis * 9u;
        mask = ((1u << (rings * 3u)) - 1u) << shift;
        return mask;
    }
    if (start_flag == START_BALANCE || cfg_stopped()) return 0x00FFFFFFu;
    return 0;
}

static uint8 cfg_reply(uint16 seq, const char *state, const char *fields)
{
    char line[192];
    int len = snprintf(line, sizeof(line), "rsp:%u,%s,%s\n", (unsigned int)seq, state, fields);
    if (len <= 0 || len >= (int)sizeof(line)) { s_cfg_tx_drop++; return 0; }
    return tx_push((const uint8 *)line, (uint32)len);
}

static uint8 cfg_status(void)
{
    uint32 v[15];
    uint32 irq;
    char line[240];
    int len;
    /* Fixed-size atomic snapshot; no formatting or Flash with interrupts masked. */
    irq = interrupt_global_disable();
    v[0] = g_control_uptime_ms;
    v[1] = (uint32)start_flag;
    v[2] = control_run_running();
    v[3] = control_test_running() ? 1u + (uint32)g_tune_axis * 3u + (uint32)g_tune_ring : 0u;
    v[4] = (uint32)control_jog_running();
    v[5] = (uint32)g_imu_ok | ((uint32)(imu_calib_state() == IMU_CALIB_OK) << 1) | ((uint32)!imu_link_lost() << 2);
    v[6] = (uint32)!W_Motor_LinkLost();
    v[7] = g_cam_ok;
    v[8] = (uint32)attitude_converged() | ((uint32)attitude_diverged() << 1);
    v[9] = (uint32)control_run_stop_reason();
    v[10] = (uint32)control_test_last_status();
    v[11] = g_param_revision;
    v[12] = s_cfg_rx_error;
    v[13] = s_cfg_tx_drop;
    v[14] = cfg_pid_mask();
    interrupt_global_enable(irq);
    len = snprintf(line, sizeof(line), "stat:%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)v[0], (unsigned long)v[1], (unsigned long)v[2], (unsigned long)v[3], (unsigned long)v[4],
        (unsigned long)v[5], (unsigned long)v[6], (unsigned long)v[7], (unsigned long)v[8], (unsigned long)v[9],
        (unsigned long)v[10], (unsigned long)v[11], (unsigned long)v[12], (unsigned long)v[13], (unsigned long)v[14]);
    if (len <= 0 || len >= (int)sizeof(line)) { s_cfg_tx_drop++; return 0; }
    return tx_push((const uint8 *)line, (uint32)len);
}

static vofa_cmd_result_t cfg_execute(char *line)
{
    char *fields[5];
    char *p = line + 4;
    char payload[144];
    uint8 count = 1;
    uint32 seq32 = 0;
    uint16 seq;
    const param_desc_t *d;
    float value, actual;
    uint32 irq;
    int pid_index;
    uint8 allowed;
    fields[0] = p;
    for (; *p; p++)
    {
        if (*p == ',')
        {
            *p = 0;
            if (count >= 5u) return VOFA_CMD_FORMAT;
            fields[count++] = p + 1;
        }
    }
    if (count < 2u || fields[1][0] == 0) return VOFA_CMD_FORMAT;
    for (p = fields[1]; *p; p++)
    {
        if (*p < '0' || *p > '9') return VOFA_CMD_FORMAT;
        seq32 = seq32 * 10u + (uint32)(*p - '0');
        if (seq32 > 65535u) return VOFA_CMD_FORMAT;
    }
    if (seq32 == 0u) return VOFA_CMD_FORMAT;
    seq = (uint16)seq32;
    if (strcmp(fields[0], "hello") == 0 && count == 2u)
    {
        s_station_att = 1;
        (void)cfg_reply(seq, "ok", "hello,1,tc264-cfg1-task1,63");
    }
    else if (strcmp(fields[0], "status") == 0 && count == 2u)
    {
        if (cfg_status()) (void)cfg_reply(seq, "ok", "status");
        else (void)cfg_reply(seq, "err", "BUSY,Status TX unavailable");
    }
    else if (strcmp(fields[0], "schema") == 0 && count == 2u)
    {
        if (s_schema_seq || s_save_seq) (void)cfg_reply(seq, "err", "BUSY,Operation pending");
        else { s_schema_seq = seq; s_schema_index = 0; s_schema_next_ms = g_control_uptime_ms; }
    }
    else if ((strcmp(fields[0], "get") == 0 && count == 3u) || (strcmp(fields[0], "set") == 0 && count == 4u))
    {
        d = param_find(fields[2]);
        if (!d) { (void)cfg_reply(seq, "err", "UNKNOWN_PARAM,Unknown name"); return VOFA_CMD_FORMAT; }
        if (count == 4u)
        {
            if (!cmd_parse_float_strict(fields[3], &value) || (!d->is_float && value != floorf(value)))
            { (void)cfg_reply(seq, "err", "INVALID_VALUE,Finite typed value required"); return VOFA_CMD_FORMAT; }
            if ((strncmp(d->name, "motor_dir_", 10) == 0 || strcmp(d->name, "enc_dir_c") == 0 || strcmp(d->name, "steer_dir") == 0) && value != 1.0f && value != -1.0f)
            { (void)cfg_reply(seq, "err", "OUT_OF_RANGE,Polarity must be plus or minus one"); return VOFA_CMD_FORMAT; }
            if (s_save_seq) { (void)cfg_reply(seq, "err", "BUSY,Save pending"); return VOFA_CMD_FORMAT; }
            pid_index = cfg_pid_index(d->name);
            irq = interrupt_global_disable();
            allowed = (uint8)(cfg_stopped() || (pid_index >= 0 && (cfg_pid_mask() & (1u << pid_index)) != 0u));
            if (allowed)
            {
                /* Descriptor already resolved. Bounded writes; no table search in the critical section. */
                actual = value < d->vmin ? d->vmin : (value > d->vmax ? d->vmax : value);
                if (d->is_float) *(float *)d->ptr = actual;
                else *(int *)d->ptr = (int)actual;
                if (d->ptr == &g_param.roll_zero_init || d->ptr == &g_param.pitch_zero_init) param_sync_zero();
                g_param_revision++;
            }
            interrupt_global_enable(irq);
            if (!allowed)
            { (void)cfg_reply(seq, "err", cfg_dangerous(d->name) ? "UNSAFE_PARAM,Stop required" : "RUNNING_LOCKED,Not an active PID"); return VOFA_CMD_FORMAT; }
            (void)param_get_by_name(d->name, &actual);
            (void)snprintf(payload, sizeof(payload), "set,%s,%.9g,%s", d->name, (double)actual,
                (value < d->vmin || value > d->vmax) ? "CLAMPED" : "APPLIED");
        }
        else
        {
            (void)param_get_by_name(d->name, &actual);
            (void)snprintf(payload, sizeof(payload), "get,%s,%.9g", d->name, (double)actual);
        }
        (void)cfg_reply(seq, "ok", payload);
    }
    else if (strcmp(fields[0], "save") == 0 && count == 3u)
    {
        if (!cfg_stopped()) (void)cfg_reply(seq, "err", "SAVE_BLOCKED,Stop required");
        else if (s_save_seq || s_schema_seq) (void)cfg_reply(seq, "err", "BUSY,Operation pending");
        else if (strlen(fields[2]) >= sizeof(s_save_group)) (void)cfg_reply(seq, "err", "BAD_FORMAT,Group too long");
        else { s_save_seq = seq; strcpy(s_save_group, fields[2]); }
    }
    else (void)cfg_reply(seq, "err", "UNKNOWN_COMMAND,Unsupported operation or arity");
    return VOFA_CMD_APPLIED;
}

#include "vofa_task.inc"

static void cfg_poll(void)
{
    uint32 now = g_control_uptime_ms;
    uint8 physically_stopped = (uint8)(cfg_stopped() && !W_Motor_LinkLost() &&
        abs((int)W_Motor_GetSpeed1()) <= 50 && abs((int)W_Motor_GetSpeed2()) <= 50 && Y_Motor_GetSpeed20ms() == 0);
    if (!physically_stopped) s_stop_seen = 0;
    else if (!s_stop_seen) { s_stop_seen = 1; s_stop_since = now; }
    if ((uint32)(now - s_cfg_last_status) >= 100u)
    {
        s_cfg_last_status = now;
        (void)cfg_status();
    }
    if (s_station_att && g_vofa_mode != VOFA_ATT)
    {
        static uint32 last_att;
        float a[3];
        char line[96];
        int len;
        uint32 irq;
        /* 10Hz auxiliary attitude leaves room for the 25-channel Run stream. */
        if ((uint32)(now - last_att) >= 100u)
        {
            last_att = now;
            irq = interrupt_global_disable();
            a[0] = att.roll; a[1] = att.pitch; a[2] = att.yaw;
            interrupt_global_enable(irq);
            len = snprintf(line, sizeof(line), "att:%.3f,%.3f,%.3f\n", (double)a[0], (double)a[1], (double)a[2]);
            if (len > 0 && len < (int)sizeof(line)) (void)tx_push((const uint8 *)line, (uint32)len);
        }
    }
    if (s_schema_seq && (uint32)(now - s_schema_next_ms) >= 30u &&
        (uint32)(s_tx_head - s_tx_tail) < VOFA_TX_SIZE - 640u)
    {
        char line[192];
        int len;
        if (s_schema_index < param_count())
        {
            const param_desc_t *d = &g_param_table[s_schema_index];
            float value;
            uint8 flags = 4u | (cfg_pid_index(d->name) >= 0 ? 1u : 0u) | (cfg_dangerous(d->name) ? 2u : 0u);
            (void)param_get_by_name(d->name, &value);
            len = snprintf(line, sizeof(line), "par:%u,%s,%s,%.9g,%.9g,%.9g,%s,%.6g,%u\n",
                (unsigned int)s_schema_seq, d->name, d->is_float ? "float" : "int", (double)value,
                (double)d->vmin, (double)d->vmax, cfg_group(d->name), d->is_float ? 0.0001 : 1.0, (unsigned int)flags);
            if (len > 0 && len < (int)sizeof(line) && tx_push((const uint8 *)line, (uint32)len)) s_schema_index++;
        }
        else
        {
            (void)snprintf(line, sizeof(line), "schema,%u", (unsigned int)param_count());
            if (cfg_reply(s_schema_seq, "ok", line)) s_schema_seq = 0;
        }
        s_schema_next_ms = now;
    }
    if (s_save_seq && s_rx_head == s_rx_tail && s_cmd_len == 0u && s_rx_idle_ms >= 100u)
    {
        const char *names[128];
        uint16 i, count = 0, seq = s_save_seq;
        uint8 ok = 0;
        char reply[80];
        s_save_seq = 0; /* Never auto retry a Flash operation. */
        if (!physically_stopped || !s_stop_seen || (uint32)(now - s_stop_since) < 500u || control_ipm_pending())
        { (void)cfg_reply(seq, "err", "SAVE_BLOCKED,Require stable stopped wheels and no IPM save"); return; }
        if (strcmp(s_save_group, "all") == 0) ok = param_save();
        else
        {
            for (i = 0; i < param_count(); i++)
                if (strcmp(cfg_group(g_param_table[i].name), s_save_group) == 0)
                {
                    if (count == 128u) { (void)cfg_reply(seq, "err", "BUSY,Too many group parameters"); return; }
                    names[count++] = g_param_table[i].name;
                }
            if (!count) { (void)cfg_reply(seq, "err", "BAD_FORMAT,Unknown group"); return; }
            ok = param_save_names(names, count);
        }
        if (ok)
        {
            (void)snprintf(reply, sizeof(reply), "save,%s,VERIFIED", s_save_group);
            (void)cfg_reply(seq, "ok", reply);
        }
        else (void)cfg_reply(seq, "err", "FLASH_FAILED,Readback verification failed");
    }
}

static vofa_cmd_result_t cmd_execute_line(char *line)
{
    if (cmd_is_stop(line))
    {
        s_schema_seq = 0; s_save_seq = 0;
        control_stop();                 // 三电机清零 + A/B 刹车锁死，与返回键急停同一条路
        return VOFA_CMD_STOPPED;
    }
    if (strncmp(line, "cfg:", 4) == 0) return cfg_execute(line);
    return cmd_execute_speed(line);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     提交一整行下行命令并记账
// 参数说明     void
// 返回参数     void
// 使用示例     cmd_submit_line();
//-------------------------------------------------------------------------------------------------------------------
static void cmd_submit_line(void)
{
    vofa_cmd_result_t result;

    g_vofa_cmd_lines++;
    if (s_cmd_ovf)
    {
        s_cfg_rx_error++;
        g_vofa_cmd_last = VOFA_CMD_OVERFLOW;    // 整行作废，绝不拿截断的命令去发车
        return;
    }

    s_cmd_line[s_cmd_len] = '\0';
    result = cmd_execute_line(cmd_trim(s_cmd_line));
    g_vofa_cmd_last = result;
    if (result != VOFA_CMD_APPLIED && result != VOFA_CMD_STOPPED) s_cfg_rx_error++;
    if (result == VOFA_CMD_APPLIED || result == VOFA_CMD_STOPPED) g_vofa_cmd_ok++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按当前模式更新姿态或正式 Run 诊断快照
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_snapshot();
//-------------------------------------------------------------------------------------------------------------------
void vofa_snapshot(void)
{
    vofa_mode_t mode = g_vofa_mode;

    task_capture();
    if (mode == VOFA_OFF) return;

    s_seq++;                                    // 奇数表示正在写入
    if (mode == VOFA_ATT)
    {
        s_att_ch[0] = att.roll;
        s_att_ch[1] = att.pitch;
        s_att_ch[2] = att.yaw;
    }
    else if (mode == VOFA_RUN)
    {
        control_run_diag_snapshot(&s_run_diag);
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
    static vofa_mode_t last_mode = VOFA_OFF;
    vofa_mode_t mode = g_vofa_mode;
    float  att_ch[VOFA_ATT_CH_COUNT];
    control_run_diag_t run;
    char   line[256];
    uint32 seq1, seq2;
    int    len;

    cfg_poll();
    task_poll();
    if (mode == VOFA_OFF) { last_seq = s_seq; last_mode = mode; return; }
    if (mode != last_mode)
    {
        last_seq = s_seq;
        last_mode = mode;
        return;
    }

    seq1 = s_seq;
    if (seq1 & 1u) return;                                          // 快照正在更新
    if ((seq1 - last_seq) < (uint32)(2u * g_vofa_div)) return;      // 未达到发送分频

    if (mode == VOFA_ATT)
    {
        att_ch[0] = s_att_ch[0];
        att_ch[1] = s_att_ch[1];
        att_ch[2] = s_att_ch[2];
    }
    else
    {
        run = s_run_diag;
    }
    seq2 = s_seq;
    if (seq1 != seq2) return;                                       // 快照读取不完整

    last_seq = seq1;

    if (mode == VOFA_ATT)
    {
        len = snprintf(line, sizeof(line), "att:%.3f,%.3f,%.3f\n",
                       (double)att_ch[0], (double)att_ch[1], (double)att_ch[2]);
    }
    else
    {
        len = snprintf(line, sizeof(line),
                       "run:%lu,%.3f,%.3f,%.2f,%.0f,%.3f,%.0f,%.3f,"
                       "%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.3f,%.3f,"
                       "%.2f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u\n",
                       (unsigned long)run.uptime_ms,
                       (double)run.roll,
                       (double)run.roll_target,
                       (double)run.roll_rate,
                       (double)run.recovery_feedback,
                       (double)run.recovery_output,
                       (double)run.roll_output,
                       (double)run.lean_offset,
                       (double)run.yaw_rate_target,
                       (double)run.yaw_rate_command,
                       (double)run.yaw_rate_actual,
                       (double)run.yaw_output_raw,
                       (double)run.yaw_output_applied,
                       (double)run.flywheel_common_rpm,
                       (double)run.direction_offset,
                       (double)run.lateral_error,
                       (double)run.heading_error,
                       (double)run.curvature,
                       (double)run.speed_plan_mps,
                       (double)run.speed_ramp_mps,
                       (double)run.speed_actual_mps,
                       (double)run.momentum_scale,
                       (double)run.vision_quality,
                       (unsigned int)run.vision_age_ms,
                       (unsigned int)run.state_flags);
    }
    if (len <= 0) return;                                           // 格式化失败
    if (len >= (int)sizeof(line)) return;                            // 格式化结果不完整
    if ((uint32)(s_tx_head - s_tx_tail) + (uint32)len > VOFA_TX_SIZE - 384u) { s_cfg_tx_drop++; return; }
    (void)tx_push((const uint8 *)line, (uint32)len);                // 写入上行环，装不下就丢弃整帧
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化无线转串口模块与上行发送队列
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_init();
//-------------------------------------------------------------------------------------------------------------------
void vofa_init(void)
{
    s_cfg_rx_error = s_cfg_tx_drop = 0;
    s_schema_seq = s_schema_index = s_save_seq = 0;
    s_schema_next_ms = s_stop_since = s_cfg_last_status = 0;
    s_stop_seen = s_station_att = 0;
    task_reset();
    (void)wireless_uart_init();

    s_tx_head = 0; s_tx_tail = 0;
    s_rx_head = 0; s_rx_tail = 0;
    s_rx_overflow = 0;
    s_cmd_len = 0; s_cmd_ovf = 0;
    s_rx_idle_ms = 0;
    g_vofa_rx_bytes = 0;
    g_vofa_cmd_lines = 0;
    g_vofa_cmd_ok = 0;
    g_vofa_cmd_last = VOFA_CMD_NONE;
    g_vofa_cmd_turn = 0.0f;
    g_vofa_cmd_speed = 0.0f;
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

    {
        uint32 interrupt_state = interrupt_global_disable();
        got = wireless_uart_read_buffer(buf, (uint32)sizeof(buf));
        interrupt_global_enable(interrupt_state);
    }
    if (got > 0u)
    {
        g_vofa_rx_bytes += got;
        s_rx_idle_ms = 0;
    }
    else if (s_rx_idle_ms < 0xFFFFu)
    {
        s_rx_idle_ms++;
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

    if (gpio_get_level(WIRELESS_UART_RTS_PIN)) return;

    tail = s_tx_tail;
    for (i = 0u; i < VOFA_TX_FIFO_DEPTH && tail != s_tx_head &&
           IfxAsclin_getTxFifoFillLevel(uart2_handle.asclin) < VOFA_TX_FIFO_DEPTH; i++)
    {
        IfxAsclin_writeTxData(uart2_handle.asclin, s_tx_buf[tail & VOFA_TX_MASK]);
        tail++;
    }
    s_tx_tail = tail;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并执行无线串口 Run Test 命令，由主循环调用
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_cmd_poll();
//-------------------------------------------------------------------------------------------------------------------
void vofa_cmd_poll(void)
{
    uint32 tail;
    uint32 budget;
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    if (s_rx_overflow)
    {
        s_rx_overflow = 0;
        s_rx_tail = s_rx_head;
        s_cmd_len = 0;
        s_cmd_ovf = 1; /* Discard until a line terminator after RX loss. */
        s_cfg_rx_error++;
        g_vofa_cmd_lines++;
        g_vofa_cmd_last = VOFA_CMD_RX_FULL;     // 和"单行超长"分开报，否则查不出是哪一种
        interrupt_global_enable(interrupt_state);
        return;
    }
    tail = s_rx_tail;
    interrupt_global_enable(interrupt_state);

    // 每消费一个字节就立刻发布 s_rx_tail。
    // 1ms 中断按 (head - s_rx_tail) 判接收环满不满，如果等整个循环跑完才发布一次，
    // 上位机一旦连续发字节这个循环就一直不退出(消费速度≈到达速度)，
    // 中断那边看到的 tail 永远停在循环开始时的值，累计过 VOFA_RX_SIZE 就误判溢出 ——
    // 而前台其实早就把这些字节吃掉了。实车表现是"命令格式完全正确，一持续发就一直报 OVERFLOW"。
    // budget 给单次调用设字节上限，别让前台被上位机的连续流锁在这个循环里出不去。
    budget = VOFA_RX_SIZE;
    while (tail != s_rx_head && budget > 0u)
    {
        char c = (char)s_rx_buf[tail & VOFA_RX_MASK];

        tail++;
        budget--;
        s_rx_tail = tail;                       // 立刻发布，让中断看到真实的剩余空间

        if (c == '\n' || c == '\r')
        {
            // 长度为 0 说明是 "\r\n" 的第二个字符或空行，不算一行命令
            if (s_cmd_len > 0) cmd_submit_line();
            s_cmd_len = 0;
            s_cmd_ovf = 0;
            continue;
        }
        if (((uint8)c < 32u && c != '\t') || (uint8)c > 126u) s_cmd_ovf = 1;
        if (s_cmd_len < (VOFA_CMD_LINE_MAX - 1))
            s_cmd_line[s_cmd_len++] = c;
        else
            s_cmd_ovf = 1;                      // 标记命令行溢出，整行作废

        /* Never execute partial lines or resynchronize inside an overflowed command. */

    }

    if (s_cmd_len > 0u && tail == s_rx_head)
    {
        if (s_rx_idle_ms >= VOFA_CMD_STALE_MS)
        {
            g_vofa_cmd_lines++;
            g_vofa_cmd_last = s_cmd_ovf ? VOFA_CMD_OVERFLOW : VOFA_CMD_FORMAT;
            s_cmd_len = 0;
            s_cmd_ovf = 1; /* Remain poisoned until CR/LF. */
        }
    }
}
