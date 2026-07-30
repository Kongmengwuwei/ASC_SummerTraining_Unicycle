#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#include "zf_common_headfile.h"

// 硬件常量与各模块的默认值。
// 本文件里的量是编译期常量；能在线整定的量在 param.h 里有对应的运行参数，
// 菜单和无线串口改的是运行参数，本文件的 *_DEFAULT 只是上电初值。
//
// MCU  TC264D          IMU IMU660RB(LSM6DSR, SPI0)   Camera MT9V03X   Display IPS200(SPI)
// A/B  动量轮          CYT2BL3 双路无刷驱动，霍尔六步换相，UART3 @460800
// C    行进轮          DRV8701E 通道 1，DIR=P21_4 PWM=P21_5，编码器 TIM2(P33_7 脉冲 / P33_6 方向)
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
#define Y_MOTOR_DIR_PIN                 (P21_4)
#define Y_MOTOR_PWM_PIN                 (ATOM0_CH3_P21_5)
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
#define ENC_DIR_C_DEFAULT       (-1)            // C轮脉冲/方向编码器计数极性

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
// 横滚角度环积分限幅。这一路是整条 Roll 链里唯一的回正来源，必须留够量程。
// 反作用轮力矩 = -J·dω/dt，占空比≈飞轮转速，所以：
//   占空比 ∝ 角度   -> 力矩 ∝ 角速度 = 阻尼，回正为 0
//   占空比 ∝ ∫角度  -> 力矩 ∝ 角度   = 回正      <- 只有积分项给得出
// 角度环 5ms 一拍，imax=50 时 5° 倾角十拍就顶满，I 项退化成常数偏置等于没有。
// 2000 允许 1° 持续 10 秒或 5° 持续 2 秒。这一环的输出不再单独限幅，
// 由内环 pid_loc_calc_limited(±FLYWHEEL_OUT_LIMIT) 兜底，两个参考工程也是只限积分
#define R_ANGLE_IMAX            (2000.0f)      // 横滚角度环积分限幅
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
#define DRIVE_OUT_LIMIT         (8000)         // C轮输出限幅
#define DRIVE_DEAD_ZONE         (120)          // C轮死区补偿

// Test 专用限幅，与实跑限幅一致，台架整定完的值落地后饱和点不变。
// 两轴都不要再压低：反作用轮的极速正比于占空比上限，压低限幅等于直接砍掉动量预算。
// 实测 8000 占空比下飞轮极速约 4335 RPM，到极速 dω/dt=0 就完全没有力矩，车必倒；
// 放到 10000 极速约 5400 RPM，动量和峰值力矩各多约 25%
#define BAL_TEST_FLY_LIMIT      (10000)         // 飞轮测试输出限幅，与 FLYWHEEL_OUT_LIMIT 一致
#define BAL_TEST_DRIVE_LIMIT    (8000)          // 行进轮测试输出限幅，与 DRIVE_OUT_LIMIT 一致

// Motor 页架空点动测试。占空比满量程 10000，运行参数，Params -> Motor 可调。
// 点动没有时限，按同一行或返回键停；MCU 跑飞时 A/B 靠驱动固件的失控保护兜底
#define JOG_DUTY_FLY_DEFAULT    (2500)          // A/B 点动占空比
#define JOG_DUTY_DRIVE_DEFAULT  (800)           // C 点动占空比，架空空载，验方向不需要转快

// 飞轮超速保护。Rate/Angle 测试时回收环被旁路，增量式输出停在非零值上飞轮就会一路加速，
// 到极速会触发驱动的堵转保护而且不报原因。这里先一步停测试并在菜单上说明。0 = 关闭
#define FLY_SPEED_LIMIT_DEFAULT (7000)         // A/B 转速上限(RPM)

// 这里曾经有过一个反电动势前馈 FLY_BEMF_K：下发 = PID输出 + k × 转速(RPM)。
// 已删除，不要再加回来。稳态拟合 占空比 = 1.85 × 转速 (R²=1.00) 本身没错，
// 但反电动势不是干扰项，它是飞轮唯一的被动转速阻尼 —— 转得越快、同样占空比下电流越小。
// 把它前馈掉等于拆掉限制飞轮转速的物理机制：k=1.6 时电气阻尼只剩 13%，
// 自然衰减时间常数从 0.1s 变成 0.77s，PID 输出为 0、飞轮 4000RPM 时下发占空比是 6400，
// 在**主动维持**转速。症状正好是"平衡点附近飞轮转得飞快、容易饱和"。
// 十几份完赛的独轮技术报告和两个参考工程都没有这一项。要管飞轮转速就靠回收环。

// A/B 占空比变化率上限(每 1ms)。动量轮从 0 转速被一脚踩到满占空比时的电流波形
// 和真堵转几乎一样，会误触发 CYT2BL3 的堵转保护。0 = 关闭斜坡
#define FLY_SLEW_DEFAULT        (800)          // 0 到 8000 约 10ms

// 姿态保护阈值
#define ROLL_PROTECT_ANGLE_DEFAULT  (50.0f)    // 横滚误差阈值(°)，台架整定值
#define PITCH_PROTECT_ANGLE_DEFAULT (50.0f)    // 俯仰误差阈值(°)，台架整定值

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
#define ROAD_WIDE_NEAR_DEFAULT  133            // 近端标准赛道宽度，运行参数，Params -> Camera 可调
#define ROAD_WIDE_FAR_DEFAULT   30             // 远端标准赛道宽度，运行参数，Params -> Camera 可调
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

// 元素通用保护。摄像头约 50 帧/秒，帧数换算成时间除以 50
#define RING_TIMEOUT_CNT_DEFAULT 250           // 环岛单个状态最长停留帧数，约 5 秒
#define ELEM_GUARD_CNT_DEFAULT   40            // 元素退出后的屏蔽帧数，约 0.8 秒

// 斑马线
#define ZEBRA_JUMP_CNT_DEFAULT  8              // 黑白跳变阈值
#define ZEBRA_CONFIRM_FRAMES    2              // 连续确认帧数

// 十字
#define CROSS_LOST_CNT_DEFAULT  15             // 丢线行数阈值
#define CORNER_JUMP             8              // 拐点跳变阈值
#define CROSS_CONFIRM_FRAMES    2              // 连续确认帧数
#define CROSS_RELEASE_FRAMES    3              // 双边恢复退出帧数
#define CROSS_WIDE_OVER         18             // 中段赛宽余量
#define CROSS_WIDE_ROWS         4              // 中段超宽行数

// 坡道
#define RAMP_SEARCH_LINE        75             // 有效搜索行阈值
#define RAMP_WIDE_OVER          10             // 路宽余量
#define RAMP_WIDE_ROWS          20             // 路宽累计行数
#define RAMP_ERR_LIMIT          5              // 偏差阈值
#define RAMP_PITCH_MIN          (3.0f)          // 坡道最小俯仰角
#define RAMP_RATE_MIN           (8.0f)          // 坡道最小俯仰角速度
#define RAMP_CONFIRM_FRAMES     2              // 连续确认帧数

// 环岛状态机
#define RING_CONTINUITY         7              // 连续性判据阈值
#define RING_LOST_MIN           12             // 丢线计数下界
#define RING_LOST_MAX           50             // 丢线计数上界
#define RING_OPP_LOST           5              // 对侧丢线阈值
#define RING_VIEW               60             // 环岛最小有效前瞻行数
#define RING_CONFIRM_FRAMES     2              // 连续确认帧数
#define RING_S2_CNT_R_DEFAULT   400            // 状态2 右环编码器累计阈值
#define RING_S2_CNT_L_DEFAULT   300            // 状态2 左环编码器累计阈值
#define RING_ANGLE_DEFAULT      340            // 元素积分角阈值(°)
#define RING_S4_CNT             2000           // 状态4 计数阈值
#define RING_S5_CNT             1000           // 状态5 计数阈值
#define RING_SIDE_OFFSET_DEFAULT 20            // 单边巡线横向补偿偏移

// 循迹速度与转向
#define TRACK_BASE_SPEED_DEFAULT 0            // 基准速度目标

#define SPEED_UP_RATE_DEFAULT   (1.2f)         // 加速速率
#define SPEED_DOWN_RATE_DEFAULT (1.9f)         // 减速速率
#define SPEED_RAMP_GAIN_DEFAULT (0.8f)         // 坡道速度倍率
#define SPEED_RING_GAIN_DEFAULT (0.8f)         // 环岛降速倍率
#define TRACK_ERR_GAIN_DEFAULT  (1.0f)         // 中线偏差转向增益
#define CAM_EXPOSURE_DEFAULT    (48)           // 摄像头曝光时间，实车可用值在 48 附近
#define VISION_FPS_WIN_MS       (500u)         // 帧率统计窗口(ms)，窗口越长读数越稳、跟随越慢

// C 轮里程标定。初值只用于首次进入页面，必须用实车直行 1m 后按累计脉冲修正。
#define ODOM_COUNTS_PER_M_DEFAULT   (11695.0f)
#define ODOM_TEST_SPEED_DEFAULT     (0.10f)
#define ODOM_TEST_DISTANCE_M        (1.0f)
#define ODOM_TEST_SLOW_DISTANCE_M   (0.20f)
#define ODOM_TEST_MIN_SPEED_MPS     (0.05f)

// 无线 Run Test。speed:<turn>,<speed> 中 turn 是相对发车航向的目标角度(°)，
// speed 是车速(m/s)。命令超时只停止移动，三轴平衡继续运行。
#define REMOTE_STEER_ANGLE_LIMIT    (180.0f)
#define REMOTE_SPEED_LIMIT_MPS      (1.0f)
#define REMOTE_SPEED_COUNT_LIMIT    (200.0f)
#define REMOTE_CMD_TIMEOUT_MS       (1000u)

// 斑马线、十字、环岛和坡道均在 CPU1 每个有效图像帧中执行。
// Run Test 只接受无线遥控目标，正式视觉 Run 后续再接元素结果。

#include "param.h"

#endif 
