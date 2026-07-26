#ifndef CODE_Y_MOTOR_H_
#define CODE_Y_MOTOR_H_

#include <stdint.h>

/*
 * 逐飞 PWM 驱动使用 0～10000 表示 0%～100% 占空比。
 * Y_Motor_SetDuty() 使用正负号表示方向：
 *     5000  表示 50% 正向输出；
 *    -5000  表示 50% 反向输出；
 *        0  表示停止输出。
 */
#define Y_MOTOR_PWM_MAX_DUTY        (10000)
#define Y_MOTOR_PWM_FREQUENCY_HZ    (17000U)
#define Y_MOTOR_ENCODER_PERIOD_MS   (5U)

typedef struct
{
    /* 最近一个 5 ms 周期内的带方向编码器计数。 */
    int16_t count_5ms;

    /* 自上次清零以来的累计编码器计数。 */
    int32_t total_count;
} y_motor_encoder_data_t;

/*
 * 初始化 P21.2 方向输出、P21.3 单路 PWM 和 P33.7/P33.6 编码器。
 * 初始化完成后 PWM 占空比为 0，电机不会主动转动。
 */
void Y_Motor_Init(void);

/*
 * 设置单路有刷电机输出。
 * duty > 0：正向；duty < 0：反向；duty == 0：停止输出。
 * 超出 [-10000, 10000] 的输入会被自动限幅。
 */
void Y_Motor_SetDuty(int32_t duty);

/* 将唯一一路电机 PWM 占空比设为 0。 */
void Y_Motor_Stop(void);

/* 每 5 ms 在 CPU0 的 CCU60_CH0 中断中调用一次。 */
void Y_Motor_EncoderUpdate5ms(void);

/* 清除硬件编码器计数、最近周期计数和累计计数。 */
void Y_Motor_EncoderClear(void);

/* 原子读取最近周期计数和累计计数。 */
void Y_Motor_GetEncoder(y_motor_encoder_data_t *data);

#endif
