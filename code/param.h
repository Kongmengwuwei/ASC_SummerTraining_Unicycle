#ifndef PARAM_H_
#define PARAM_H_

#include "zf_common_headfile.h"

#define PARAM_MAGIC             (0x51435452u)   // 参数区魔数
#define PARAM_VERSION           (7u)            // 参数结构版本

// 参数结构
typedef struct
{
    uint32 magic;                       // 参数魔数
    uint32 version;                     // 参数版本

    // 速度与循迹
    int   track_base_speed;             // 基准速度
    float track_err_gain;               // 循迹误差增益
    float speed_ramp_gain;              // 坡道降速倍率
    float speed_ring_gain;              // 环岛降速倍率
    int   cam_exposure;                 // 摄像头曝光时间

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
    float lean_k1;                      // 压弯系数 K1
    float lean_k2;                      // 压弯系数 K2
    float lean_limit;                   // 压弯角限幅
    int   lean_limit_mode;              // 压弯限幅模式

    // 元素阈值
    int   zebra_jump_cnt;               // 斑马线底行跳变阈值
    int   cross_lost_cnt;               // 十字丢线行数阈值
    int   ring_angle;                   // 环岛转角阈值
    int   ring_s2_cnt_l;                // 左环编码器阈值
    int   ring_s2_cnt_r;                // 右环编码器阈值
    int   ring_side_offset;             // 环岛单边巡线横向补偿
    float obs_narrow_ratio;             // 路障路宽收窄判据比例
    int   obs_line_offset;              // 路障避障横向补偿

    // 零点、标定与保护
    float roll_zero_init;               // 横滚机械零点初值
    float pitch_zero_init;              // 俯仰机械零点初值
    float roll_protect_angle;           // 横滚保护角度
    float pitch_protect_angle;          // 俯仰保护角度
    float err_offset;                   // 中线偏差零点

    // 电机与编码器极性
    int   motor_dir_a;                  // 动量轮A占空比与转速回读极性
    int   motor_dir_b;                  // 动量轮B占空比与转速回读极性
    int   motor_dir_c;                  // 行进轮C输出极性
    int   enc_dir_c;                    // C轮脉冲/方向编码器计数符号
} param_t;

extern param_t g_param;                 // 运行参数
extern volatile uint32 g_param_revision;// 参数修订号

// 运行参数映射
// 速度与循迹
#define TRACK_BASE_SPEED        (g_param.track_base_speed)
#define TRACK_ERR_GAIN          (g_param.track_err_gain)
#define SPEED_RAMP_GAIN         (g_param.speed_ramp_gain)
#define SPEED_RING_GAIN         (g_param.speed_ring_gain)
#define CAM_EXPOSURE            (g_param.cam_exposure)

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
#define LEAN_K1                 (g_param.lean_k1)
#define LEAN_K2                 (g_param.lean_k2)
#define LEAN_LIMIT              (g_param.lean_limit)
#define LEAN_LIMIT_MODE         (g_param.lean_limit_mode)

// 元素阈值
#define ZEBRA_JUMP_CNT          (g_param.zebra_jump_cnt)
#define CROSS_LOST_CNT          (g_param.cross_lost_cnt)
#define RING_ANGLE              (g_param.ring_angle)
#define RING_S2_CNT_L           (g_param.ring_s2_cnt_l)
#define RING_S2_CNT_R           (g_param.ring_s2_cnt_r)
#define RING_SIDE_OFFSET        (g_param.ring_side_offset)
#define OBS_NARROW_RATIO        (g_param.obs_narrow_ratio)
#define OBS_LINE_OFFSET         (g_param.obs_line_offset)

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
// 函数简介     获取参数描述表条目数
// 参数说明     void
// 返回参数     uint16          条目数
// 使用示例     uint16 count = param_table_count();
//-------------------------------------------------------------------------------------------------------------------
uint16 param_table_count(void);

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
// 函数简介     将运行参数保存到 Flash
// 参数说明     void
// 返回参数     uint8           1 表示成功, 0 表示失败
// 使用示例     uint8 saved = param_save();
//-------------------------------------------------------------------------------------------------------------------
uint8 param_save(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同步横滚与俯仰机械零点
// 参数说明     void
// 返回参数     void
// 使用示例     param_sync_zero();
//-------------------------------------------------------------------------------------------------------------------
void param_sync_zero(void);

#endif /* PARAM_H_ */
