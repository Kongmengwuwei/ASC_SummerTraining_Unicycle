#include "vofa.h"
#include "attitude.h"
#include "board_config.h"
#include "control.h"
#include <stdio.h>

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

// 姿态波形快照
#define VOFA_CH_COUNT   (3u)

static volatile uint32      s_seq = 0;          // 快照序号
static volatile float       s_ch[VOFA_CH_COUNT];// Roll、Pitch、Yaw

// 下行命令缓冲
#define VOFA_CMD_LINE_MAX   (64)                // 单行命令长度
#define VOFA_CMD_STALE_MS   (1000u)             // 未收到 CR/LF 的残行作废时间

static char  s_cmd_line[VOFA_CMD_LINE_MAX];     // 命令行缓冲
static uint8 s_cmd_len = 0;                     // 命令行长度
static uint8 s_cmd_ovf = 0;                     // 命令行溢出标志
static volatile uint16 s_rx_idle_ms = 0;        // 距最后一个下行字节的时间

#define VOFA_TX_FIFO_DEPTH  (16u)               // ASCLIN 硬件发送 FIFO 深度
#define VOFA_TX_SIZE        (2048u)             // 发送环长度，必须是 2 的幂
#define VOFA_RX_SIZE        (256u)              // 接收环长度，必须是 2 的幂
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
static uint8 tx_push(const uint8 *dat, uint32 len)
{
    uint32 head = s_tx_head;
    uint32 used = head - s_tx_tail;           
    uint32 i;

    if (len == 0u || len > (VOFA_TX_SIZE - used)) return 0;

    for (i = 0; i < len; i++) s_tx_buf[(head + i) & VOFA_TX_MASK] = dat[i];
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
static void cmd_submit_line(void)
{
    vofa_cmd_result_t result;

    g_vofa_cmd_lines++;
    if (s_cmd_ovf)
    {
        g_vofa_cmd_last = VOFA_CMD_OVERFLOW;    // 整行作废，绝不拿截断的命令去发车
        return;
    }

    s_cmd_line[s_cmd_len] = '\0';
    result = cmd_execute_speed(cmd_trim(s_cmd_line));
    g_vofa_cmd_last = result;
    if (result == VOFA_CMD_APPLIED) g_vofa_cmd_ok++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新 Roll、Pitch、Yaw 姿态波形快照
// 参数说明     void
// 返回参数     void
// 使用示例     vofa_snapshot();
//-------------------------------------------------------------------------------------------------------------------
void vofa_snapshot(void)
{
    if (g_vofa_mode != VOFA_ATT) return;

    s_seq++;                                    // 奇数表示正在写入
    s_ch[0] = att.roll;
    s_ch[1] = att.pitch;
    s_ch[2] = att.yaw;
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
    float  ch[VOFA_CH_COUNT];
    char   line[96];
    uint32 seq1, seq2;
    int    len;

    if (g_vofa_mode != VOFA_ATT) { last_seq = s_seq; return; }

    seq1 = s_seq;
    if (seq1 & 1u) return;                                          // 快照正在更新
    if ((seq1 - last_seq) < (uint32)(2u * g_vofa_div)) return;      // 未达到发送分频

    ch[0] = s_ch[0];
    ch[1] = s_ch[1];
    ch[2] = s_ch[2];
    seq2 = s_seq;
    if (seq1 != seq2) return;                                       // 快照读取不完整

    last_seq = seq1;

    len = snprintf(line, sizeof(line), "att:%.3f,%.3f,%.3f\n",
                   (double)ch[0], (double)ch[1], (double)ch[2]);
    if (len <= 0) return;                                           // 格式化失败
    if (len >= (int)sizeof(line)) return;                            // 格式化结果不完整
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
    while (tail != s_tx_head &&
           IfxAsclin_getTxFifoFillLevel(uart2_handle.asclin) < VOFA_TX_FIFO_DEPTH)
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
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    if (s_rx_overflow)
    {
        s_rx_overflow = 0;
        s_rx_tail = s_rx_head;
        s_cmd_len = 0;
        s_cmd_ovf = 0;
        g_vofa_cmd_lines++;
        g_vofa_cmd_last = VOFA_CMD_OVERFLOW;
        interrupt_global_enable(interrupt_state);
        return;
    }
    tail = s_rx_tail;
    interrupt_global_enable(interrupt_state);

    while (tail != s_rx_head)
    {
        char c = (char)s_rx_buf[tail & VOFA_RX_MASK];
        tail++;

        if (c == '\n' || c == '\r')
        {
            // 长度为 0 说明是 "\r\n" 的第二个字符或空行，不算一行命令
            if (s_cmd_len > 0) cmd_submit_line();
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

    if (s_cmd_len > 0u && tail == s_rx_head)
    {
        if (s_rx_idle_ms >= VOFA_CMD_STALE_MS)
        {
            g_vofa_cmd_lines++;
            g_vofa_cmd_last = s_cmd_ovf ? VOFA_CMD_OVERFLOW : VOFA_CMD_FORMAT;
            s_cmd_len = 0;
            s_cmd_ovf = 0;
        }
    }
}
