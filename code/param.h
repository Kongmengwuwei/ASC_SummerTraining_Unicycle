#ifndef PARAM_H_
#define PARAM_H_

#include "zf_common_headfile.h"

#define PARAM_MAGIC             (0x51435452u)   // 参数区魔数

// 参数结构
typedef struct
{
    // 正式跑车与循迹
    float run_speed_straight;           // 直道速度(m/s)
    float run_speed_curve;              // 弯道最低速度(m/s)
    float run_speed_cross;              // 十字速度(m/s)
    float run_speed_ring;               // 环岛速度(m/s)
    float run_speed_ramp;               // 坡道速度(m/s)
    float run_speed_lost;               // 低质量或短时丢线速度(m/s)
    float run_accel_mps2;               // 正式跑车加速度上限(m/s^2)
    float run_decel_mps2;               // 正式跑车减速度上限(m/s^2)
    float direction_balance_kp;         // 方向偏差 P
    float direction_balance_kd;         // 方向偏差 D
    float zebra_stop_offset_m;          // 识别斑马线后的前行距离(m)
    int   err_front_row;                // 前瞻行，拟合线在这一行求值
    float ipm_h[9];                     // 逆透视矩阵，行优先。不进菜单，由 Calib IPM 写
    int   cam_exposure;                 // 摄像头曝光时间
    float odom_counts_per_m;            // C 轮每行进 1m 的编码器脉冲数
    float odom_test_speed;              // 1m 里程验证速度(m/s)

    // Roll 串级
    float r_rcy_kp;                     // 飞轮回收环 P
    float r_rcy_ki;                     // 飞轮回收环 I
    float r_rcy_kd;                     // 飞轮回收环 D
    float r_angle_kp;                   // 横滚角度环 P
    float r_angle_ki;                   // 横滚角度环 I
    float r_angle_kd;                   // 横滚角度环 D
    float r_rate_kp;                    // 横滚角速度环 P
    float r_rate_ki;                    // 横滚角速度环 I
    float r_rate_kd;                    // 横滚角速度环 D

    // Pitch 串级
    float p_vel_kp;                     // 俯仰速度环 P
    float p_vel_ki;                     // 俯仰速度环 I
    float p_vel_kd;                     // 俯仰速度环 D
    float p_angle_kp;                   // 俯仰角度环 P
    float p_angle_ki;                   // 俯仰角度环 I
    float p_angle_kd;                   // 俯仰角度环 D
    float p_rate_kp;                    // 俯仰角速度环 P
    float p_rate_ki;                    // 俯仰角速度环 I
    float p_rate_kd;                    // 俯仰角速度环 D

    // Yaw 串级
    float y_angle_kp;                   // 航向角度环 P
    float y_angle_ki;                   // 转向外环 I
    float y_angle_kd;                   // 转向外环 D
    float y_rate_kp;                    // 航向角速度环 P
    float y_rate_ki;                    // 航向角速度内环 I
    float y_rate_kd;                    // 航向角速度内环 D

    // 压弯
    float direction_roll_kp;            // 山大压弯公式方向倾角 Kp
    float lean_max_angle;               // 压弯动态零点最大值(°)

    // 元素使能，0=关 1=开。默认全关，普通循迹跑稳后一次只开一个
    int   elem_en_zebra;                // 斑马线
    int   elem_en_cross;                // 十字
    int   elem_en_ring;                 // 环岛
    int   elem_en_ramp;                 // 坡道
    // 现场需要调整的元素路径参数
    int   ring_angle;                   // 环岛转角阈值
    int   ring_s2_cnt_l;                // 左环编码器阈值
    int   ring_s2_cnt_r;                // 右环编码器阈值

    // 零点、标定与保护
    float roll_zero_init;               // 横滚机械零点初值
    float pitch_zero_init;              // 俯仰机械零点初值
    float roll_protect_angle;           // 横滚保护角度
    float pitch_protect_angle;          // 俯仰保护角度

    // 电机与编码器极性
    int   motor_dir_a;                  // 动量轮A占空比与转速回读极性
    int   motor_dir_b;                  // 动量轮B占空比与转速回读极性
    int   motor_dir_c;                  // 行进轮C输出极性
    int   enc_dir_c;                    // C轮脉冲/方向编码器计数符号
    int   steer_dir;                    // 转向极性，遥控与视觉转向共用
    int   jog_duty_fly;                 // A/B 架空点动占空比，满量程 10000
    int   jog_duty_drive;               // C 架空点动占空比，满量程 10000
    int   fly_speed_limit;              // A/B 转速上限(RPM)，超了停测试，0=关闭
    int   fly_slew;                     // A/B 占空比变化率上限(每1ms)，0=不限
} param_t;

extern param_t g_param;                 // 运行参数
extern volatile uint32 g_param_revision;// 参数修订号

// 运行参数映射
// 正式跑车与循迹
#define RUN_SPEED_STRAIGHT      (g_param.run_speed_straight)
#define RUN_SPEED_CURVE         (g_param.run_speed_curve)
#define RUN_SPEED_CROSS         (g_param.run_speed_cross)
#define RUN_SPEED_RING          (g_param.run_speed_ring)
#define RUN_SPEED_RAMP          (g_param.run_speed_ramp)
#define RUN_SPEED_LOST          (g_param.run_speed_lost)
#define RUN_ACCEL_MPS2          (g_param.run_accel_mps2)
#define RUN_DECEL_MPS2          (g_param.run_decel_mps2)
#define DIRECTION_BALANCE_KP    (g_param.direction_balance_kp)
#define DIRECTION_BALANCE_KD    (g_param.direction_balance_kd)
#define ZEBRA_STOP_OFFSET_M     (g_param.zebra_stop_offset_m)
#define ERR_FRONT_ROW           (g_param.err_front_row)
#define CAM_EXPOSURE            (g_param.cam_exposure)
#define ODOM_COUNTS_PER_M       (g_param.odom_counts_per_m)
#define ODOM_TEST_SPEED         (g_param.odom_test_speed)

// Roll 串级
#define R_RCY_KP                (g_param.r_rcy_kp)
#define R_RCY_KI                (g_param.r_rcy_ki)
#define R_RCY_KD                (g_param.r_rcy_kd)
#define R_ANGLE_KP              (g_param.r_angle_kp)
#define R_ANGLE_KI              (g_param.r_angle_ki)
#define R_ANGLE_KD              (g_param.r_angle_kd)
#define R_RATE_KP               (g_param.r_rate_kp)
#define R_RATE_KI               (g_param.r_rate_ki)
#define R_RATE_KD               (g_param.r_rate_kd)

// Pitch 串级
#define P_VEL_KP                (g_param.p_vel_kp)
#define P_VEL_KI                (g_param.p_vel_ki)
#define P_VEL_KD                (g_param.p_vel_kd)
#define P_ANGLE_KP              (g_param.p_angle_kp)
#define P_ANGLE_KI              (g_param.p_angle_ki)
#define P_ANGLE_KD              (g_param.p_angle_kd)
#define P_RATE_KP               (g_param.p_rate_kp)
#define P_RATE_KI               (g_param.p_rate_ki)
#define P_RATE_KD               (g_param.p_rate_kd)

// Yaw 串级
#define Y_ANGLE_KP              (g_param.y_angle_kp)
#define Y_ANGLE_KI              (g_param.y_angle_ki)
#define Y_ANGLE_KD              (g_param.y_angle_kd)
#define Y_RATE_KP               (g_param.y_rate_kp)
#define Y_RATE_KI               (g_param.y_rate_ki)
#define Y_RATE_KD               (g_param.y_rate_kd)

// 压弯
#define DIRECTION_ROLL_KP       (g_param.direction_roll_kp)
#define LEAN_MAX_ANGLE          (g_param.lean_max_angle)

// 零点、标定与保护
#define ROLL_ZERO_INIT          (g_param.roll_zero_init)
#define PITCH_ZERO_INIT         (g_param.pitch_zero_init)
#define ROLL_PROTECT_ANGLE      (g_param.roll_protect_angle)
#define PITCH_PROTECT_ANGLE     (g_param.pitch_protect_angle)

// 电机与编码器极性
#define MOTOR_DIR_A             (g_param.motor_dir_a)
#define MOTOR_DIR_B             (g_param.motor_dir_b)
#define MOTOR_DIR_C             (g_param.motor_dir_c)
#define ENC_DIR_C               (g_param.enc_dir_c)
#define STEER_DIR               (g_param.steer_dir)
#define JOG_DUTY_FLY            (g_param.jog_duty_fly)
#define JOG_DUTY_DRIVE          (g_param.jog_duty_drive)
#define FLY_SPEED_LIMIT         (g_param.fly_speed_limit)
#define FLY_SLEW                (g_param.fly_slew)

// 参数描述
typedef struct
{
    const char *name;       // 参数名
    void       *ptr;        // 参数地址
    uint8       is_float;   // 数据类型
    float       vmin;       // 下限
    float       vmax;       // 上限
} param_desc_t;

extern const param_desc_t g_param_table[];      // 参数描述表

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按名称查找参数描述
// 参数说明     name            参数名
// 返回参数     const param_desc_t*  参数描述, 未找到返回 0
// 使用示例     const param_desc_t *d = param_find("p_angle_kp");
//-------------------------------------------------------------------------------------------------------------------
const param_desc_t *param_find(const char *name);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按名称设置参数
// 参数说明     name/value      参数名和目标值
// 返回参数     uint8           1 表示成功, 0 表示参数不存在
// 使用示例     param_set_by_name("r_rate_kp", -45.0f);
//-------------------------------------------------------------------------------------------------------------------
uint8 param_set_by_name(const char *name, float value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按名称读取参数
// 参数说明     name/value      参数名和输出地址
// 返回参数     uint8           1 表示成功, 0 表示参数不存在
// 使用示例     float v; param_get_by_name("r_rate_kp", &v);
//-------------------------------------------------------------------------------------------------------------------
uint8 param_get_by_name(const char *name, float *value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 Flash 初始化运行参数
// 参数说明     void
// 返回参数     void
// 使用示例     param_init();
//-------------------------------------------------------------------------------------------------------------------
void param_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入 board_config.h 中的默认参数
// 参数说明     void
// 返回参数     void
// 使用示例     param_load_defaults();
//-------------------------------------------------------------------------------------------------------------------
void param_load_defaults(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将全部运行参数保存到 Flash，对应 Params 页最底下那个 Save
// 参数说明     void
// 返回参数     uint8           1 表示成功, 0 表示失败
// 使用示例     uint8 saved = param_save();
//-------------------------------------------------------------------------------------------------------------------
uint8 param_save(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     只保存名字表里的参数，其余参数保持 Flash 里已有的值，对应各子页里的 Save。
//              Flash 里从没存过、又不在名字表里的参数不写记录，下次上电仍回默认值
// 参数说明     names/count     参数名表与表长，名字要和 g_param_table 里的完全一致
// 返回参数     uint8           1 表示成功, 0 表示失败或表为空
// 使用示例     uint8 saved = param_save_names(names, 9);
//-------------------------------------------------------------------------------------------------------------------
uint8 param_save_names(const char *const *names, uint16 count);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     擦除 Flash 参数页并把运行参数恢复成默认值
// 参数说明     void
// 返回参数     uint8           1=擦除后页确实是空的 0=擦除失败
// 使用示例     uint8 ok = param_erase();
//-------------------------------------------------------------------------------------------------------------------
uint8 param_erase(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同步横滚与俯仰机械零点
// 参数说明     void
// 返回参数     void
// 使用示例     param_sync_zero();
//-------------------------------------------------------------------------------------------------------------------
void param_sync_zero(void);

#endif
