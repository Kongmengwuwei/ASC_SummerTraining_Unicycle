#include "vofa.h"
#include "attitude.h"
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
volatile float       g_vofa_cmd_speed = 0.0f;   // 最近一次解析成功的速度值(m/s)，与是否被接受无关

// 姿态波形快照
#define VOFA_CH_COUNT   (3u)

static volatile uint32      s_seq = 0;          // 快照序号
static volatile float       s_ch[VOFA_CH_COUNT];// Roll、Pitch、Yaw

// 下行命令缓冲
#define VOFA_CMD_LINE_MAX   (64)                // 单行命令长度
// 无 CR/LF 时靠接收静默断行，这个窗口是双边约束，两头都不能碰：
//   下界 —— 上位机和无线模块会把一行拆成两段发，间隔几到几十毫秒。窗口小于这个间隔时
//           半截 "speed:0," 会被当成一行提交并判 FORMAT 错，紧跟的 "0.1" 又判 PREFIX 错，
//           两半都丢，只有整行恰好落进同一个窗口才偶尔成功一次。
//   上界 —— 不带换行的周期发送，命令间隔就是发送周期(10Hz 即 100ms)。窗口大于它的话
//           静默永远不触发，两条命令拼成一行、逗号变两个，同样一条都收不到。
// 所以取 40ms：够长，能扛住行内拆包；够短，10Hz 无换行的周期发送每条都能断出来。
// 另外提交的判据是"缓冲已像一条完整命令"而不是单纯静默够久，见 cmd_line_looks_complete()
#define VOFA_CMD_IDLE_MS    (40u)               // 缓冲已像完整命令时的断行静默时间
#define VOFA_CMD_STALE_MS   (1000u)             // 残行作废时间：这么久还凑不成完整命令就丢掉

static char  s_cmd_line[VOFA_CMD_LINE_MAX];     // 命令行缓冲
static uint8 s_cmd_len = 0;                     // 命令行长度
static uint8 s_cmd_ovf = 0;                     // 命令行溢出标志
static volatile uint16 s_rx_idle_ms = 0;        // 距最后一个下行字节的时间

// 收发环形缓冲。
// 刷屏走 SPI，一次实时行刷新要十几毫秒，主循环被它拖住的时候串口不能跟着停：
// 发慢了波形会一阵一阵地涌，收慢了库里那 64 字节接收缓冲会溢出，
// speed 命令中间少字节可能变成另一组合法目标，因此接收溢出时必须整行作废。
// 所以两个方向都由 1ms 中断搬运，前台只负责格式化和解析。
//
// 单生产者单消费者，前台只写 head、中断只写 tail，各自是自己那个下标的唯一写者，
// 所以不用关中断。长度取 2 的幂，uint32 下标自然回绕后再取模。
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
    uint32 used = head - s_tx_tail;             // 无符号相减，回绕后依然是真实占用量
    uint32 i;

    if (len == 0u || len > (VOFA_TX_SIZE - used)) return 0;

    // 半截帧比丢帧更糟：VOFA+ 会把残行当成一帧解析，通道全部错位。
    for (i = 0; i < len; i++) s_tx_buf[(head + i) & VOFA_TX_MASK] = dat[i];
    s_tx_head = head + len;                     // 数据写完再发布下标
    return 1;
}

// 字符串工具

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
// 函数简介     解析并执行 speed:<相对航向角>,<线速度> 无线遥控命令
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
    float speed_value;

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
        !cmd_parse_float_strict(speed_text, &speed_value))
    {
        return VOFA_CMD_FORMAT;                 // 数字本身非法
    }

    // 解析成功就先回显，不管控制层收不收。停车状态下也能在屏幕上看到发进来的数值，
    // 这样"链路通不通"和"发车了没有"是两件能分开验证的事
    g_vofa_cmd_turn = steer_angle;
    g_vofa_cmd_speed = speed_value;

    if (control_remote_command(steer_angle, speed_value)) return VOFA_CMD_APPLIED;

    // control_remote_command() 只有两种失败：数值越界，或者 Run Test 没在跑。
    // 用已导出的运行标志把这两种分开，不必改它的签名
    return control_remote_running() ? VOFA_CMD_RANGE : VOFA_CMD_NOT_RUNNING;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断当前命令行缓冲是否已经凑成一条完整的 speed 命令，供静默断行判据用
// 参数说明     void
// 返回参数     uint8           1=前缀、逗号和逗号后的数字都齐了 0=还是半截
// 使用示例     if (cmd_line_looks_complete()) cmd_submit_line();
//-------------------------------------------------------------------------------------------------------------------
static uint8 cmd_line_looks_complete(void)
{
    uint8 i;
    uint8 comma = 0;                    // 逗号个数
    uint8 tail_digit = 0;               // 逗号后面出现过数字

    if (s_cmd_len < 8u) return 0;       // "speed:" 之后至少还要各有一个字符
    s_cmd_line[s_cmd_len] = '\0';       // 写入侧钳在 VOFA_CMD_LINE_MAX-1，这里写 NUL 不越界
    if (!cmd_is_speed(s_cmd_line)) return 0;

    for (i = 6u; i < s_cmd_len; i++)
    {
        char c = s_cmd_line[i];

        if (c == ',') { comma++; continue; }
        if (comma == 1u && c >= '0' && c <= '9') tail_digit = 1;
    }
    return (uint8)(comma == 1u && tail_digit);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     提交一整行下行命令并记账，两个断行入口共用
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

// 外部接口
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
    // 模块 RX 接 P10_5(MCU TX)、TX 接 P10_6(MCU RX)、RTS 接 P10_2，引脚与波特率都在
    // zf_device_wireless_uart.h 里定义。自动波特率关着，模块保持出厂 115200，
    // 这条路径里 wireless_uart_init() 不会失败，返回值没有信息量。
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

    // 收：把库里那 64 字节缓冲倒进自己的环。1ms 一次，115200 下一拍最多来 12 字节，
    // 无论前台在刷屏还是在解析都不会溢出。读的时候要挡住 UART2 接收中断，
    // 那个中断优先级 17 低于本中断的 30，会被本中断打断在 fifo 写一半的位置上。
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
// 函数简介     解析并执行无线串口 Run Test 命令，由主循环调用
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

    // 某些串口上位机不追加 CR/LF，这里靠接收静默断行。判据是"缓冲已经像一条完整命令"，
    // 不是"静默够久"：一行被拆成两段发进来时，前半段永远凑不齐逗号后的数字，
    // 于是继续等后半段，而不是把半截命令提交上去判错。
    if (s_cmd_len > 0u && tail == s_rx_head)
    {
        if (s_rx_idle_ms >= VOFA_CMD_IDLE_MS && (s_cmd_ovf || cmd_line_looks_complete()))
        {
            cmd_submit_line();
            s_cmd_len = 0;
            s_cmd_ovf = 0;
        }
        else if (s_rx_idle_ms >= VOFA_CMD_STALE_MS)
        {
            // 等到这里还凑不成完整命令，说明本来就是错的或者后半段丢了。
            // 必须丢掉，否则残字节会和下一条命令拼在一起，把后面每一条都带坏
            g_vofa_cmd_lines++;
            g_vofa_cmd_last = VOFA_CMD_FORMAT;
            s_cmd_len = 0;
            s_cmd_ovf = 0;
        }
    }
}
