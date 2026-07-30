#include "W_Motor.h"

#include "board_config.h"

#define W_MOTOR_FRAME_HEAD       (0xA5u)    // 帧头
#define W_MOTOR_SET_DUTY_CMD     (0x01u)    // 设占空比功能字
#define W_MOTOR_GET_SPEED_CMD    (0x02u)    // 请求/回传转速功能字
#define W_MOTOR_FRAME_SIZE       (7u)       // 定长帧字节数
#define W_MOTOR_TX_FIFO_DEPTH    (16u)      // ASCLIN 发送 FIFO 深度

#pragma section all "cpu0_dsram"
static uint8  w_motor_rx_buffer[W_MOTOR_FRAME_SIZE];    // 接收拼帧缓冲，只由 UART3 中断写
static uint8  w_motor_rx_length;                        // 接收拼帧长度，只由 UART3 中断写
static volatile int16  w_motor_speed_1;                 // 动量轮 A 转速(RPM)，UART3 中断写
static volatile int16  w_motor_speed_2;                 // 动量轮 B 转速(RPM)，UART3 中断写
static volatile uint32 w_motor_rx_frames;               // 合法转速帧计数，UART3 中断写
static uint32 w_motor_seen_frames;                      // 1ms 中断上次看到的帧计数
static uint16 w_motor_link_age_ms;                      // 距上一帧回传的时间(ms)
static uint16 w_motor_request_age_ms;                   // 断链后补发转速请求的计时(ms)
static volatile uint8 w_motor_brake_locked;             // 软件刹车闩，前台与中断都会读写
static int16  w_motor_last_duty_1;                      // 上一拍实际下发的 A 占空比，斜坡用
static int16  w_motor_last_duty_2;                      // 上一拍实际下发的 B 占空比，斜坡用
#pragma section all restore

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将占空比限制在 CYT2BL3 有符号协议范围内
// 参数说明     duty            待限制占空比
// 返回参数     int16           限幅后的有符号占空比
// 使用示例     duty = W_Motor_LimitDuty(duty);
//-------------------------------------------------------------------------------------------------------------------
static int16 W_Motor_LimitDuty(int32 duty)
{
    return (int16)func_limit_ab(duty, -W_MOTOR_DUTY_MAX, W_MOTOR_DUTY_MAX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按每拍变化率上限逼近目标占空比，避免零转速下的阶跃电流被驱动误判成堵转
// 参数说明     last/target     上一拍实际下发值与本拍目标值
// 返回参数     int16           本拍允许下发的占空比
// 使用示例     duty = W_Motor_Slew(w_motor_last_duty_1, target);
//-------------------------------------------------------------------------------------------------------------------
static int16 W_Motor_Slew(int16 last, int16 target)
{
    int32 step = FLY_SLEW;
    int32 delta;

    if (step <= 0) return target;               // 0 表示不限变化率
    delta = (int32)target - (int32)last;
    if (delta >  step) return (int16)((int32)last + step);
    if (delta < -step) return (int16)((int32)last - step);
    return target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算 CYT2BL3 定长帧前六字节的八位累加校验
// 参数说明     frame           七字节协议帧
// 返回参数     uint8           累加和低八位
// 使用示例     frame[6] = W_Motor_Checksum(frame);
//-------------------------------------------------------------------------------------------------------------------
static uint8 W_Motor_Checksum(const uint8 frame[W_MOTOR_FRAME_SIZE])
{
    uint8 checksum = 0;
    uint8 index;

    for (index = 0; index < (W_MOTOR_FRAME_SIZE - 1u); index++)
        checksum = (uint8)(checksum + frame[index]);
    return checksum;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     非阻塞发送一帧 CYT2BL3 命令，硬件 FIFO 空间不足时整帧丢弃
// 参数说明     command         功能字
// 参数说明     data0~data3     四字节负载
// 返回参数     void
// 使用示例     W_Motor_SendFrame(W_MOTOR_SET_DUTY_CMD, h1, l1, h2, l2);
//-------------------------------------------------------------------------------------------------------------------
static void W_Motor_SendFrame(uint8 command, uint8 data0, uint8 data1, uint8 data2, uint8 data3)
{
    uint8 frame[W_MOTOR_FRAME_SIZE];
    uint8 index;
    uint32 interrupt_state;

    frame[0] = W_MOTOR_FRAME_HEAD;
    frame[1] = command;
    frame[2] = data0;
    frame[3] = data1;
    frame[4] = data2;
    frame[5] = data3;
    frame[6] = W_Motor_Checksum(frame);

    interrupt_state = interrupt_global_disable();
    if (IfxAsclin_getTxFifoFillLevel(uart3_handle.asclin) >
        (W_MOTOR_TX_FIFO_DEPTH - W_MOTOR_FRAME_SIZE))
    {
        interrupt_global_enable(interrupt_state);
        return;
    }

    for (index = 0; index < W_MOTOR_FRAME_SIZE; index++)
        IfxAsclin_writeTxData(uart3_handle.asclin, frame[index]);
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 CYT2BL3 双路无刷驱动 UART3、锁定软件刹车并请求周期回传转速
// 参数说明     void
// 返回参数     void
// 使用示例     W_Motor_Init();
//-------------------------------------------------------------------------------------------------------------------
void W_Motor_Init(void)
{
    w_motor_rx_length = 0;
    w_motor_speed_1 = 0;
    w_motor_speed_2 = 0;
    w_motor_rx_frames = 0;
    w_motor_seen_frames = 0;
    w_motor_link_age_ms = W_MOTOR_LINK_TIMEOUT_MS;
    w_motor_request_age_ms = 0;
    w_motor_brake_locked = 1;
    w_motor_last_duty_1 = 0;
    w_motor_last_duty_2 = 0;

    uart_init(W_MOTOR_UART,
              W_MOTOR_UART_BAUDRATE,
              W_MOTOR_UART_TX_PIN,
              W_MOTOR_UART_RX_PIN);
    uart_rx_interrupt(W_MOTOR_UART, 1);

    W_Motor_Stop();
    W_Motor_SendFrame(W_MOTOR_GET_SPEED_CMD, 0, 0, 0, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置两只动量轮的有符号占空比，软件刹车锁定时强制发送双路零输出
// 参数说明     motor1_duty    动量轮 A 占空比
// 参数说明     motor2_duty    动量轮 B 占空比
// 返回参数     void
// 使用示例     W_Motor_SetDuty(g_motor_a, g_motor_b);
//-------------------------------------------------------------------------------------------------------------------
void W_Motor_SetDuty(int32 motor1_duty, int32 motor2_duty)
{
    uint16 motor1_data;
    uint16 motor2_data;

    if (w_motor_brake_locked)
    {
        // 刹车是安全动作，必须立即到 0，不走斜坡
        motor1_duty = 0;
        motor2_duty = 0;
        w_motor_last_duty_1 = 0;
        w_motor_last_duty_2 = 0;
    }
    else
    {
        // 斜坡在极性之前做，这样限的是"实际加在电机上的"变化率
        motor1_duty = W_Motor_Slew(w_motor_last_duty_1, W_Motor_LimitDuty(motor1_duty));
        motor2_duty = W_Motor_Slew(w_motor_last_duty_2, W_Motor_LimitDuty(motor2_duty));
        w_motor_last_duty_1 = (int16)motor1_duty;
        w_motor_last_duty_2 = (int16)motor2_duty;
    }

    motor1_duty = (int32)W_Motor_LimitDuty(motor1_duty) * MOTOR_DIR_A;
    motor2_duty = (int32)W_Motor_LimitDuty(motor2_duty) * MOTOR_DIR_B;
    motor1_data = (uint16)(int16)motor1_duty;
    motor2_data = (uint16)(int16)motor2_duty;

    W_Motor_SendFrame(W_MOTOR_SET_DUTY_CMD,
                      (uint8)(motor1_data >> 8),
                      (uint8)(motor1_data & 0xFFu),
                      (uint8)(motor2_data >> 8),
                      (uint8)(motor2_data & 0xFFu));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     锁定动量轮软件刹车并立即发送双路零输出
// 参数说明     void
// 返回参数     void
// 使用示例     W_Motor_Stop();
//-------------------------------------------------------------------------------------------------------------------
void W_Motor_Stop(void)
{
    w_motor_brake_locked = 1;
    W_Motor_SetDuty(0, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解除动量轮软件刹车闩，允许后续占空比命令生效
// 参数说明     void
// 返回参数     void
// 使用示例     W_Motor_Release();
//-------------------------------------------------------------------------------------------------------------------
void W_Motor_Release(void)
{
    w_motor_brake_locked = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析 CYT2BL3 回传字节，由 UART3 接收中断调用
// 参数说明     void
// 返回参数     void
// 使用示例     W_Motor_RxHandler();
//-------------------------------------------------------------------------------------------------------------------
void W_Motor_RxHandler(void)
{
    uint8 byte;
    uint8 checksum;
    uint8 index;

    while (uart_query_byte(W_MOTOR_UART, &byte))
    {
        if (w_motor_rx_length == 0u && byte != W_MOTOR_FRAME_HEAD)
            continue;

        w_motor_rx_buffer[w_motor_rx_length++] = byte;
        if (w_motor_rx_length < W_MOTOR_FRAME_SIZE)
            continue;
        w_motor_rx_length = 0;

        checksum = 0;
        for (index = 0; index < (W_MOTOR_FRAME_SIZE - 1u); index++)
            checksum = (uint8)(checksum + w_motor_rx_buffer[index]);
        if (checksum != w_motor_rx_buffer[6])
            continue;
        if (w_motor_rx_buffer[1] != W_MOTOR_GET_SPEED_CMD)
            continue;

        w_motor_speed_1 =
            (int16)((int16)(((uint16)w_motor_rx_buffer[2] << 8) | w_motor_rx_buffer[3]) * MOTOR_DIR_A);
        w_motor_speed_2 =
            (int16)((int16)(((uint16)w_motor_rx_buffer[4] << 8) | w_motor_rx_buffer[5]) * MOTOR_DIR_B);
        w_motor_rx_frames++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新无刷驱动通信看门狗，断链时周期补发转速请求
// 参数说明     void
// 返回参数     void
// 使用示例     W_Motor_Tick1ms();
//-------------------------------------------------------------------------------------------------------------------
void W_Motor_Tick1ms(void)
{
    uint32 frames = w_motor_rx_frames;

    if (frames != w_motor_seen_frames)
    {
        w_motor_seen_frames = frames;
        w_motor_link_age_ms = 0;
    }
    else if (w_motor_link_age_ms < W_MOTOR_LINK_TIMEOUT_MS)
    {
        w_motor_link_age_ms++;
    }

    // 转速请求无条件周期补发：驱动收到一次 0x02 后是否持续 10ms 一帧取决于固件版本，
    // 只在断链时补发的话，一旦第一帧请求发早于驱动上电就再也要不回转速了。
    // 0x02 不喂失控保护看门狗，占空比帧由 control_loop 每 1ms 单独下发。
    if (w_motor_request_age_ms < W_MOTOR_SPEED_REQUEST_MS)
    {
        w_motor_request_age_ms++;
    }
    else
    {
        w_motor_request_age_ms = 0;
        W_Motor_SendFrame(W_MOTOR_GET_SPEED_CMD, 0, 0, 0, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取动量轮 A 的驱动回传转速
// 参数说明     void
// 返回参数     int16           动量轮 A 转速(RPM，已应用 MOTOR_DIR_A)
// 使用示例     int16 speed = W_Motor_GetSpeed1();
//-------------------------------------------------------------------------------------------------------------------
int16 W_Motor_GetSpeed1(void)
{
    return w_motor_speed_1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取动量轮 B 的驱动回传转速
// 参数说明     void
// 返回参数     int16           动量轮 B 转速(RPM，已应用 MOTOR_DIR_B)
// 使用示例     int16 speed = W_Motor_GetSpeed2();
//-------------------------------------------------------------------------------------------------------------------
int16 W_Motor_GetSpeed2(void)
{
    return w_motor_speed_2;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 CYT2BL3 是否超过规定时间没有回传合法转速帧
// 参数说明     void
// 返回参数     uint8           1=通信中断 0=通信正常
// 使用示例     if (W_Motor_LinkLost()) control_stop();
//-------------------------------------------------------------------------------------------------------------------
uint8 W_Motor_LinkLost(void)
{
    return (uint8)(w_motor_link_age_ms >= W_MOTOR_LINK_TIMEOUT_MS);
}
