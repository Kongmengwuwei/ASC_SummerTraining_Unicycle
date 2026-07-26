#include "W_Motor.h"

#include "zf_common_headfile.h"

/*
 * P15.6/P15.7 对应逐飞 TC264 库的 UART3 引脚：
 *
 *     P15.6：TC264 发送，连接 CYT2BL3 接收；
 *     P15.7：TC264 接收，连接 CYT2BL3 发送。
 *
 * 当前模块只实现控制命令发送，不开启 UART3 接收中断，也不会与
 * 工程模板中遗留的 GNSS 接收回调发生冲突。后续若需要读取驱动板
 * 返回的转速，再单独增加 UART3 接收解析和对应中断回调。
 */
#define W_MOTOR_UART       (UART_3)
#define W_MOTOR_UART_TX    (UART3_TX_P15_6)
#define W_MOTOR_UART_RX    (UART3_RX_P15_7)

#define W_MOTOR_FRAME_HEAD      (0xA5U)
#define W_MOTOR_SET_DUTY_CMD    (0x01U)
#define W_MOTOR_FRAME_SIZE      (7U)

/* 将用户命令限制在 CYT2BL3 占空比协议允许的范围内。 */
static int16_t W_Motor_LimitDuty(int32_t duty)
{
    if (duty > W_MOTOR_DUTY_MAX)
    {
        return (int16_t)W_MOTOR_DUTY_MAX;
    }

    if (duty < -W_MOTOR_DUTY_MAX)
    {
        return (int16_t)(-W_MOTOR_DUTY_MAX);
    }

    return (int16_t)duty;
}

/*
 * 计算前 6 字节的八位累加和。
 * uint8_t 自然保留累加结果的低 8 位，与 CYT2BL3 官方协议一致。
 */
static uint8_t W_Motor_Checksum(const uint8_t frame[W_MOTOR_FRAME_SIZE])
{
    uint8_t checksum = 0U;
    uint8_t index;

    for (index = 0U; index < (W_MOTOR_FRAME_SIZE - 1U); index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }

    return checksum;
}

void W_Motor_Init(void)
{
    /*
     * uart_init() 的参数顺序为：串口号、波特率、TX 引脚、RX 引脚。
     * 逐飞 UART 初始化默认关闭接收中断，符合当前仅发送控制命令的
     * 精简实现。
     */
    uart_init(W_MOTOR_UART,
              W_MOTOR_UART_BAUDRATE,
              W_MOTOR_UART_TX,
              W_MOTOR_UART_RX);

    /* 上电初始化完成后首先明确发送双路零输出。 */
    W_Motor_Stop();
}

void W_Motor_SetDuty(int32_t motor1_duty, int32_t motor2_duty)
{
    uint8_t frame[W_MOTOR_FRAME_SIZE];
    uint16_t motor1_data;
    uint16_t motor2_data;

    /*
     * 先限幅，再转成 uint16_t 取得有符号 int16_t 的原始二进制编码。
     * 这样正数和负数都能按高字节在前的顺序装入通讯帧。
     */
    motor1_data = (uint16_t)W_Motor_LimitDuty(motor1_duty);
    motor2_data = (uint16_t)W_Motor_LimitDuty(motor2_duty);

    /*
     * CYT2BL3 设置双路占空比帧，共 7 字节：
     *
     * [0] 0xA5                         帧头
     * [1] 0x01                         设置占空比功能字
     * [2] [3] 第一台电机有符号占空比   高字节、低字节
     * [4] [5] 第二台电机有符号占空比   高字节、低字节
     * [6] 前 6 字节的八位累加和
     */
    frame[0] = W_MOTOR_FRAME_HEAD;
    frame[1] = W_MOTOR_SET_DUTY_CMD;
    frame[2] = (uint8_t)(motor1_data >> 8U);
    frame[3] = (uint8_t)(motor1_data & 0x00FFU);
    frame[4] = (uint8_t)(motor2_data >> 8U);
    frame[5] = (uint8_t)(motor2_data & 0x00FFU);
    frame[6] = W_Motor_Checksum(frame);

    /*
     * 一帧仅 7 字节。UART 驱动在这里完成发送，不在函数中加入延时、
     * 重试或动态内存操作。应由 CPU0 单独拥有该模块，避免多个执行
     * 上下文同时发送而造成帧交错。
     */
    uart_write_buffer(W_MOTOR_UART, frame, W_MOTOR_FRAME_SIZE);
}

void W_Motor_Stop(void)
{
    W_Motor_SetDuty(0, 0);
}
