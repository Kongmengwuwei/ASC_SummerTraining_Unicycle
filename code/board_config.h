#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#include "zf_common_headfile.h"

// 硬件常量与各模块的默认值。
// 本文件里的量是编译期常量；能在线整定的量在 param.h 里有对应的运行参数，
// 菜单和 UART0 改的是运行参数，本文件的 *_DEFAULT 只是上电初值。
//
// MCU  TC264D          IMU IMU660RB(LSM6DSR, SPI0)   Camera MT9V03X   Display IPS200(SPI)
// A/B  动量轮          CYT2BL3 双路无刷驱动，霍尔六步换相，UART3 @460800
// C    行进轮          DRV8701E 通道 1，PWM=P21_3 DIR=P21_2，编码器 TIM2(P33_7 脉冲 / P33_6 方向)
// Roll 车体 X 左右倾   A/B 动量轮差动
// Pitch 车体 Y 前后倾  C 行进轮
// Yaw  车体 Z 航向     A/B 动量轮同向

// 控制调度：1ms 角速度环，5ms 姿态与角度环、编码器，10ms 按键，20ms 速度环。
#define CTRL_PERIOD_MS          (1)            // 控制中断周期(ms)
#define CTRL_DT                 (0.001f)       // 陀螺仪积分步长(s)
#define CTRL_PIT_CH             (CCU60_CH0)    // 控制定时器通道

#define CTRL_DIV_ATT            (5)            // 姿态与角度环分频
#define ATT_DT                  (0.005f)       // 姿态解算周期(s)
#define CTRL_DIV_KEY            (10)           // 按键扫描分频
#define CTRL_DIV_SPEED          (20)           // 速度环与飞轮回收环分频

// W_Motor：动量轮 A/B 使用 CYT2BL3 双路无刷驱动，霍尔六步换相模式。
#define W_MOTOR_UART                    (UART_3)
#define W_MOTOR_UART_BAUDRATE           (460800)
#define W_MOTOR_UART_TX_PIN             (UART3_TX_P15_6)
#define W_MOTOR_UART_RX_PIN             (UART3_RX_P15_7)
#define W_MOTOR_DUTY_MAX                (10000)
#define W_MOTOR_LINK_TIMEOUT_MS         (60)
#define W_MOTOR_SPEED_REQUEST_MS        (200)

// Y_Motor：行进轮 C 使用 DRV8701E 通道 1，编码器为 P33.7 脉冲 + P33.6 方向。
#define Y_MOTOR_DIR_PIN                 (P21_2)
#define Y_MOTOR_PWM_PIN                 (ATOM0_CH1_P21_3)
#define Y_MOTOR_PWM_FREQUENCY_HZ        (17000)
#define Y_MOTOR_PWM_MAX_DUTY            (10000)
#define Y_MOTOR_FORWARD_DIR_LEVEL       (GPIO_LOW)
#define Y_MOTOR_ENCODER_INDEX           (TIM2_ENCODER)
#define Y_MOTOR_ENCODER_A_PIN           (TIM2_ENCODER_CH1_P33_7)
#define Y_MOTOR_ENCODER_B_PIN           (TIM2_ENCODER_CH2_P33_6)
#define Y_MOTOR_ENCODER_PERIOD_MS       (5)

// 电机与编码器极性，取值为 ±1。
// A/B 的极性同时作用于占空比下发和转速回读，两者必须同源，否则回收环变正反馈。
#define MOTOR_DIR_A_DEFAULT     (1)             // A轮输出与转速极性
#define MOTOR_DIR_B_DEFAULT     (1)             // B轮输出与转速极性
#define MOTOR_DIR_C_DEFAULT     (1)             // C轮输出极性
#define ENC_DIR_C_DEFAULT       (1)             // C轮脉冲/方向编码器计数极性

// 菜单按键
#define MENU_KEY_UP             (KEY_1)        // P20_6，上
#define MENU_KEY_DOWN           (KEY_2)        // P20_7，下
#define MENU_KEY_ENTER          (KEY_3)        // P11_2，确认
#define MENU_KEY_RETURN         (KEY_4)        // P11_3，返回

// 机械零点
#define ROLL_ZERO_INIT_DEFAULT  (0.0f)         // 横滚机械零点(°)
#define PITCH_ZERO_INIT_DEFAULT (0.0f)         // 俯仰机械零点(°)

// 平衡串级 PID 参数
// Roll：飞轮回收环 -> 角度环 -> 角速度环。
#define R_RCY_KP_DEFAULT        (0.0f)         // 飞轮回收环 P
#define R_RCY_KI_DEFAULT        (0.0f)         // 飞轮回收环 I
#define R_RCY_KD_DEFAULT        (0.0f)         // 飞轮回收环 D
#define R_RCY_IMAX              (50.0f)        // 飞轮回收环积分限幅
#define R_ANGLE_KP_DEFAULT      (0.0f)         // 横滚角度环 P
#define R_ANGLE_KI_DEFAULT      (0.0f)         // 横滚角度环 I
#define R_ANGLE_KD_DEFAULT      (0.0f)         // 横滚角度环 D
#define R_ANGLE_IMAX            (50.0f)        // 横滚角度环积分限幅
#define R_RATE_KP_DEFAULT       (0.0f)         // 横滚角速度环 P
#define R_RATE_KI_DEFAULT       (0.0f)         // 横滚角速度环 I
#define R_RATE_KD_DEFAULT       (0.0f)         // 横滚角速度环 D
#define R_RATE_IMAX             (100.0f)       // 横滚角速度环积分限幅

// Pitch：速度环 -> 角度环 -> 角速度环。
#define P_VEL_KP_DEFAULT        (0.0f)         // 俯仰速度环 P
#define P_VEL_KI_DEFAULT        (0.0f)         // 俯仰速度环 I
#define P_VEL_KD_DEFAULT        (0.0f)         // 俯仰速度环 D
#define P_VEL_IMAX              (50.0f)        // 俯仰速度环积分限幅
#define P_ANGLE_KP_DEFAULT      (0.0f)         // 俯仰角度环 P
#define P_ANGLE_KI_DEFAULT      (0.0f)         // 俯仰角度环 I
#define P_ANGLE_KD_DEFAULT      (0.0f)         // 俯仰角度环 D
#define P_ANGLE_IMAX            (50.0f)        // 俯仰角度环积分限幅
#define P_RATE_KP_DEFAULT       (0.0f)         // 俯仰角速度环 P
#define P_RATE_KI_DEFAULT       (0.0f)         // 俯仰角速度环 I
#define P_RATE_KD_DEFAULT       (0.0f)         // 俯仰角速度环 D
#define P_RATE_IMAX             (100.0f)       // 俯仰角速度环积分限幅

// Yaw：转向外环 -> 角速度内环。
#define Y_ANGLE_KP_DEFAULT      (0.0f)         // 转向外环 P
#define Y_ANGLE_KI_DEFAULT      (0.0f)         // 转向外环 I
#define Y_ANGLE_KD_DEFAULT      (0.0f)         // 转向外环 D
#define Y_ANGLE_IMAX            (50.0f)        // 转向外环积分限幅
#define Y_RATE_KP_DEFAULT       (0.0f)         // 航向角速度内环 P
#define Y_RATE_KI_DEFAULT       (0.0f)         // 航向角速度内环 I
#define Y_RATE_KD_DEFAULT       (0.0f)         // 航向角速度内环 D
#define Y_RATE_IMAX             (100.0f)       // 航向角速度内环积分限幅

// 压弯动态零点
#define LEAN_K1_DEFAULT         (0.002f)       // 转向比例系数
#define LEAN_K2_DEFAULT         (0.0001f)      // 速度相关限幅系数
#define LEAN_LIMIT_DEFAULT      (10.0f)        // 固定限幅(°)
#define LEAN_LIMIT_MODE_DEFAULT (0)            // 0=固定限幅，1=动态限幅
#define LEAN_SLEW_DEFAULT       (0.08f)        // 零点变化速率(°/5ms 拍)，0=不限速率
#define LEAN_TURN_DEAD          (1.0f)         // 转向死区
#define LEAN_DECAY              (0.98f)        // 回零衰减系数
#define LEAN_LIMIT_MAX          (15.0f)        // 最大限幅(°)

// 电机输出限幅与死区
// A/B 的起转死区由驱动内部处理，主控不做死区补偿。
#define FLYWHEEL_OUT_LIMIT      (10000)        // A/B 输出限幅
#define DRIVE_OUT_LIMIT         (3500)         // C轮输出限幅
#define DRIVE_DEAD_ZONE         (120)          // C轮死区补偿

// Test/Wave 调试限幅
#define BAL_TEST_FLY_LIMIT      (1500)          // 飞轮测试输出限幅
#define BAL_TEST_DRIVE_LIMIT    (800)           // 行进轮测试输出限幅

// Motor 页架空点动测试
#define MOTOR_JOG_FLY_DUTY      (800)           // A/B 点动占空比
#define MOTOR_JOG_DRIVE_DUTY    (600)           // C 点动占空比
#define MOTOR_JOG_MS            (1500)          // 单次点动时长(ms)

// 姿态保护阈值
#define ROLL_PROTECT_ANGLE_DEFAULT  (20.0f)    // 横滚误差阈值(°)
#define PITCH_PROTECT_ANGLE_DEFAULT (15.0f)    // 俯仰误差阈值(°)

// 摄像头循迹与赛道元素
// 裁剪后第 0 行为远端，第 IMG_H-1 行为近端。
#define IMG_W                   180            // 算法图像宽度
#define IMG_H                   80             // 算法图像高度
#define IMG_COL_OFFSET          ((MT9V03X_W - IMG_W) / 2)   // 居中裁剪列偏移
#define IMG_ROW_OFFSET          (MT9V03X_H - IMG_H)         // 底部裁剪行偏移
#define IMG_MID_COL             (IMG_W / 2)    // 图像中心列
#if (IMG_W > MT9V03X_W) || (IMG_H > MT9V03X_H)
    #error "IMG_W/IMG_H 不能超过采图缓冲 MT9V03X_W/MT9V03X_H"
#endif

// 大津阈值保护
#define OTSU_TH_MIN             30             // 阈值下限
#define OTSU_TH_MAX             220            // 阈值上限
#define OTSU_CONTRAST_MIN       25             // 最小灰度跨度

// 八邻域提边参数
#define EN_START_ROW_BOTTOM     (IMG_H - 3)    // 起点搜索下界
#define EN_START_ROW_TOP        (IMG_H - 20)   // 起点搜索上界
#define EN_MIN_ROAD_WIDE        25             // 起点最小赛道宽度
#define EN_ROW_TOP_LIMIT        2              // 提边终止行
#define EN_COL_MIN_LIMIT        2              // 爬线最左列
#define EN_COL_MAX_LIMIT        (IMG_W - 3)    // 爬线最右列
#define EN_MAX_PTS              (IMG_H * 4)    // 单侧边线点上限

// 中线与有效性
#define ROAD_WIDE_NEAR          133            // 近端标准赛道宽度
#define ROAD_WIDE_FAR           30             // 远端标准赛道宽度
#define ROAD_WIDE_MIN_RATIO     (0.45f)        // 最小有效宽度比例
#define ROAD_WIDE_MAX_RATIO     (1.90f)        // 最大有效宽度比例
#define TRACK_MIN_VALID_ROWS    12             // 最少有效中线行数

// 中线偏差取样
#define ERR_FRONT_ROW           27             // 起始行
#define ERR_AVG_ROWS            7              // 平均行数

// 元素使能默认值。全关，普通循迹跑稳后在 Params -> Element 页一次只开一个
#define ELEM_EN_ZEBRA_DEFAULT    0
#define ELEM_EN_CROSS_DEFAULT    0
#define ELEM_EN_RING_DEFAULT     0
#define ELEM_EN_RAMP_DEFAULT     0
#define ELEM_EN_OBSTACLE_DEFAULT 0

// 元素通用保护。摄像头约 50 帧/秒，帧数换算成时间除以 50
#define RING_TIMEOUT_CNT_DEFAULT 250           // 环岛单个状态最长停留帧数，约 5 秒
#define ELEM_GUARD_CNT_DEFAULT   40            // 元素退出后的屏蔽帧数，约 0.8 秒

// 斑马线
#define ZEBRA_JUMP_CNT_DEFAULT  8              // 黑白跳变阈值

// 十字
#define CROSS_LOST_CNT_DEFAULT  15             // 丢线行数阈值
#define CORNER_JUMP             8              // 拐点跳变阈值

// 坡道
#define RAMP_SEARCH_LINE        75             // 有效搜索行阈值
#define RAMP_WIDE_OVER          10             // 路宽余量
#define RAMP_WIDE_ROWS          20             // 路宽累计行数
#define RAMP_ERR_LIMIT          5              // 偏差阈值

// 环岛状态机
#define RING_CONTINUITY         7              // 连续性判据阈值
#define RING_LOST_MIN           12             // 丢线计数下界
#define RING_LOST_MAX           50             // 丢线计数上界
#define RING_OPP_LOST           5              // 对侧丢线阈值
#define RING_VIEW               (IMG_H - EN_ROW_TOP_LIMIT) // 环岛要求边线提取到当前可达最远行
#define RING_S2_CNT_R_DEFAULT   400            // 状态2 右环编码器累计阈值
#define RING_S2_CNT_L_DEFAULT   300            // 状态2 左环编码器累计阈值
#define RING_ANGLE_DEFAULT      340            // 元素积分角阈值(°)
#define RING_S4_CNT             2000           // 状态4 计数阈值
#define RING_S5_CNT             1000           // 状态5 计数阈值
#define RING_SIDE_OFFSET_DEFAULT 20            // 单边巡线横向补偿偏移

// 路障
#define OBS_NARROW_RATIO_DEFAULT (0.8f)        // 路宽收窄比例
#define OBS_RECOVER_RATIO       (0.95f)        // 路宽恢复比例
#define OBS_NARROW_CNT          5              // 收窄需连续计数
#define OBS_ROW_MIN             10             // 路障检测行下界
#define OBS_ROW_MAX             40             // 路障检测行上界
#define OBS_LINE_OFFSET_DEFAULT 15             // 避障横向补偿偏移

// 循迹速度与转向
#define TRACK_BASE_SPEED_DEFAULT 0            // 基准速度目标

#define SPEED_UP_RATE_DEFAULT   (1.2f)         // 加速速率
#define SPEED_DOWN_RATE_DEFAULT (1.9f)         // 减速速率
#define SPEED_RAMP_GAIN_DEFAULT (0.8f)         // 坡道速度倍率
#define SPEED_RING_GAIN_DEFAULT (0.8f)         // 环岛降速倍率
#define TRACK_ERR_GAIN_DEFAULT  (1.0f)         // 中线偏差转向增益
#define CAM_EXPOSURE_DEFAULT    (512)          // 摄像头曝光时间

// 斑马线、十字、环岛、坡道和路障均在 CPU1 每个有效图像帧中执行。
// Run 尚未接入，因此元素结果当前只发布给 CPU0，不直接改变电机输出。

#include "param.h"

#endif 
