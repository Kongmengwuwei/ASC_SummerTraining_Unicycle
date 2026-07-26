#include "Motor_Test.h"

#include "W_Motor.h"
#include "Y_Motor.h"
#include "zf_common_headfile.h"

/*
 * 维持一条 CYT2BL3 飞轮测试命令。
 *
 * CYT2BL3 通过 UART 控制，不像普通 PWM 那样由硬件持续保持输出。为了兼容
 * 驱动板的通信失控保护，本函数每 MOTOR_TEST_W_REFRESH_MS 重新发送一次命令，
 * 直到达到指定测试时间。
 */
static void Motor_Test_RunFlywheel(int32_t motor1_duty,
                                   int32_t motor2_duty,
                                   uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < duration_ms)
    {
        uint32_t remaining_ms = duration_ms - elapsed_ms;
        uint32_t delay_ms =
            (remaining_ms < MOTOR_TEST_W_REFRESH_MS)
                ? remaining_ms
                : MOTOR_TEST_W_REFRESH_MS;

        W_Motor_SetDuty(motor1_duty, motor2_duty);
        system_delay_ms(delay_ms);
        elapsed_ms += delay_ms;
    }

    /* 到达测试时间后立即清零双路输出，不能等待下一测试阶段再停止。 */
    W_Motor_Stop();
}

/*
 * 测试一个飞轮的正反转，另一个飞轮在整个过程中始终保持零输出。
 *
 * motor_index == 1：测试 CYT2BL3 一号电机；
 * motor_index == 2：测试 CYT2BL3 二号电机。
 */
static void Motor_Test_OneFlywheel(uint8_t motor_index)
{
    if (motor_index == 1U)
    {
        Motor_Test_RunFlywheel(MOTOR_TEST_W_DUTY,
                               0,
                               MOTOR_TEST_RUN_TIME_MS);
        system_delay_ms(MOTOR_TEST_STOP_TIME_MS);

        Motor_Test_RunFlywheel(-MOTOR_TEST_W_DUTY,
                               0,
                               MOTOR_TEST_RUN_TIME_MS);
    }
    else
    {
        Motor_Test_RunFlywheel(0,
                               MOTOR_TEST_W_DUTY,
                               MOTOR_TEST_RUN_TIME_MS);
        system_delay_ms(MOTOR_TEST_STOP_TIME_MS);

        Motor_Test_RunFlywheel(0,
                               -MOTOR_TEST_W_DUTY,
                               MOTOR_TEST_RUN_TIME_MS);
    }

    W_Motor_Stop();
    system_delay_ms(MOTOR_TEST_STOP_TIME_MS);
}

void Motor_Test_StopAll(void)
{
    Y_Motor_Stop();
    W_Motor_Stop();
}

void Motor_Test_RunOnBoot(void)
{
#if MOTOR_TEST_ENABLE
    /*
     * 重要安全要求：
     *     - 必须架空行进轮；
     *     - 两个高速飞轮必须安装牢固并带防护罩；
     *     - 人员、导线和工具必须远离所有旋转件；
     *     - 随时可以切断主电源。
     *
     * 本测试故意在 PIT 和平衡控制启动前执行，因此不会与 Control 模块竞争
     * 电机输出。测试期间 CPU1 只是在同步点等待 CPU0，属于预期行为。
     */
    Motor_Test_StopAll();
    system_delay_ms(MOTOR_TEST_START_DELAY_MS);

    /*
     * 第一阶段：有刷行进轮。
     * 正负号仅表示 Y_Motor 驱动中定义的两个方向，哪一边是车辆真正的“前进”
     * 必须通过本次架空测试观察后记录。
     */
    Y_Motor_SetDuty(MOTOR_TEST_Y_DUTY);
    system_delay_ms(MOTOR_TEST_RUN_TIME_MS);
    Y_Motor_Stop();
    system_delay_ms(MOTOR_TEST_STOP_TIME_MS);

    Y_Motor_SetDuty(-MOTOR_TEST_Y_DUTY);
    system_delay_ms(MOTOR_TEST_RUN_TIME_MS);
    Y_Motor_Stop();
    system_delay_ms(MOTOR_TEST_STOP_TIME_MS);

    /* 第二阶段：只测试一号动量轮，二号动量轮保持停止。 */
    Motor_Test_OneFlywheel(1U);

    /* 第三阶段：只测试二号动量轮，一号动量轮保持停止。 */
    Motor_Test_OneFlywheel(2U);

    /*
     * 无论前面测试顺序如何结束，最后统一清零三个执行器。
     * 随后的 Control_Init() 还会再次清零，形成双重保险。
     */
    Motor_Test_StopAll();
#else
    /*
     * 关闭测试后仍显式停止全部电机，使本函数可以安全保留在上电流程中，
     * 无需为了关闭测试而修改 cpu0_main.c。
     */
    Motor_Test_StopAll();
#endif
}
