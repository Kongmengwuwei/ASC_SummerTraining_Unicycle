#ifndef CODE_MOTOR_TEST_H_
#define CODE_MOTOR_TEST_H_

#include <stdint.h>

/*
 * 上电电机测试总开关。
 *
 * 1：每次上电都自动执行三个电机的正反转测试；
 * 0：保留测试程序，但上电调用会直接返回，不产生电机输出。
 *
 * 完成首次接线和方向确认后，建议将其改为 0，避免日常调试时电机自动转动。
 */
#define MOTOR_TEST_ENABLE                 (1U)

/*
 * 测试参数。
 *
 * 逐飞电机接口用 0~10000 表示 0%~100% 占空比。
 * 行进轮初始使用 15%，飞轮初始使用 20%，目的是降低首次测试风险。
 * 如果机械静摩擦导致电机不能启动，应在架空条件下小幅增加，而不是直接满占空比。
 */
#define MOTOR_TEST_Y_DUTY                 (1500)
#define MOTOR_TEST_W_DUTY                 (2000)

/* 上电后先保持所有电机停止，留出断电和检查机械环境的时间。 */
#define MOTOR_TEST_START_DELAY_MS         (2000U)

/* 每个方向的持续时间，以及正反转之间的强制停止时间。 */
#define MOTOR_TEST_RUN_TIME_MS            (800U)
#define MOTOR_TEST_STOP_TIME_MS           (500U)

/*
 * CYT2BL3 非零命令刷新周期。
 * 某些驱动板固件带通信失控保护，只发送一次命令可能在测试尚未结束时自动停机。
 */
#define MOTOR_TEST_W_REFRESH_MS           (20U)

/*
 * 上电顺序测试：
 *     1. 行进轮正转、停止、反转、停止；
 *     2. 一号动量轮正转、停止、反转、停止；
 *     3. 二号动量轮正转、停止、反转、停止。
 *
 * 该函数包含毫秒级阻塞延时，只能在 CPU0 初始化阶段调用，禁止放入中断。
 * 调用前必须完成 Y_Motor_Init() 和 W_Motor_Init()。
 */
void Motor_Test_RunOnBoot(void);

/* 无条件停止行进轮和两个动量轮，可作为测试阶段的统一收尾函数。 */
void Motor_Test_StopAll(void);

#endif
