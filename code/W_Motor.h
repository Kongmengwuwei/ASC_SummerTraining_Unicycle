#ifndef CODE_W_MOTOR_H_
#define CODE_W_MOTOR_H_

#include <stdint.h>

/*
 * CYT2BL3 双路无刷驱动通讯参数。
 *
 * 一块 CYT2BL3 双驱板通过一组 UART TX/RX 同时控制两台无刷电机：
 *     TC264 P15.6（UART3 TX）-> CYT2BL3 RX；
 *     TC264 P15.7（UART3 RX）<- CYT2BL3 TX；
 *     波特率 460800。
 *
 * 因为 UART3 已由飞轮驱动占用，所以不能再用同一串口连接 GNSS。
 */
#define W_MOTOR_UART_BAUDRATE      (460800U)
#define W_MOTOR_DUTY_MAX           (10000)

/*
 * 初始化 UART3，并立即向 CYT2BL3 发送“两台电机均为零输出”的命令。
 * 初始化不会主动启动飞轮。
 */
void W_Motor_Init(void);

/*
 * 同时设置两个动量轮的有符号输出，输入会自动限制到
 * [-10000, 10000]：
 *
 *     正数：CYT2BL3 定义的正转；
 *     负数：CYT2BL3 定义的反转；
 *       0 ：停止输出。
 *
 * 两只动量轮的实际正方向必须在架空条件下分别验证。
 *
 * 如果 CYT2BL3 固件开启了失控保护，非零输出需要由上层控制任务
 * 周期刷新；不要只发送一次非零命令后长期不再通讯。
 */
void W_Motor_SetDuty(int32_t motor1_duty, int32_t motor2_duty);

/* 向 CYT2BL3 发送双路零输出命令。 */
void W_Motor_Stop(void);

#endif
