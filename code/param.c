#include "param.h"
#include "board_config.h"
#include "balance.h"

// 参数描述表
const param_desc_t g_param_table[] =
{
    // 速度与循迹
    { "track_base_speed", &g_param.track_base_speed, 0, 0.0f,    200.0f  },
    { "track_err_gain",   &g_param.track_err_gain,   1, -20.0f,  20.0f   },
    { "speed_ramp_gain",  &g_param.speed_ramp_gain,  1, 0.0f,    1.5f    },
    { "speed_ring_gain",  &g_param.speed_ring_gain,  1, 0.0f,    1.5f    },
    { "cam_exposure",     &g_param.cam_exposure,     0, 16.0f,   1600.0f },
    // Roll 串级
    { "r_rcy_kp",         &g_param.r_rcy_kp,         1, -50.0f,   50.0f  },
    { "r_rcy_ki",         &g_param.r_rcy_ki,         1, -20.0f,   20.0f  },
    { "r_rcy_kd",         &g_param.r_rcy_kd,         1, -50.0f,   50.0f  },
    { "r_angle_kp",       &g_param.r_angle_kp,       1, -2000.0f, 2000.0f},
    { "r_angle_ki",       &g_param.r_angle_ki,       1, -200.0f,  200.0f },
    { "r_angle_kd",       &g_param.r_angle_kd,       1, -500.0f,  500.0f },
    { "r_rate_kp",        &g_param.r_rate_kp,        1, -2000.0f, 2000.0f},
    { "r_rate_ki",        &g_param.r_rate_ki,        1, -200.0f,  200.0f },
    { "r_rate_kd",        &g_param.r_rate_kd,        1, -500.0f,  500.0f },
    // Pitch 串级
    { "p_vel_kp",         &g_param.p_vel_kp,         1, -50.0f,   50.0f  },
    { "p_vel_ki",         &g_param.p_vel_ki,         1, -20.0f,   20.0f  },
    { "p_vel_kd",         &g_param.p_vel_kd,         1, -50.0f,   50.0f  },
    { "p_angle_kp",       &g_param.p_angle_kp,       1, -2000.0f, 2000.0f},
    { "p_angle_ki",       &g_param.p_angle_ki,       1, -200.0f,  200.0f },
    { "p_angle_kd",       &g_param.p_angle_kd,       1, -500.0f,  500.0f },
    { "p_rate_kp",        &g_param.p_rate_kp,        1, -500.0f,  500.0f },
    { "p_rate_ki",        &g_param.p_rate_ki,        1, -50.0f,   50.0f  },
    { "p_rate_kd",        &g_param.p_rate_kd,        1, -500.0f,  500.0f },
    // Yaw 串级
    { "y_angle_kp",       &g_param.y_angle_kp,       1, -50.0f,   50.0f  },
    { "y_angle_ki",       &g_param.y_angle_ki,       1, -50.0f,   50.0f  },
    { "y_angle_kd",       &g_param.y_angle_kd,       1, -200.0f,  200.0f },
    { "y_rate_kp",        &g_param.y_rate_kp,        1, -200.0f,  200.0f },
    { "y_rate_ki",        &g_param.y_rate_ki,        1, -50.0f,   50.0f  },
    { "y_rate_kd",        &g_param.y_rate_kd,        1, -200.0f,  200.0f },
    // 压弯
    { "lean_k1",          &g_param.lean_k1,          1, -1.0f,    1.0f   },
    { "lean_k2",          &g_param.lean_k2,          1, -0.1f,    0.1f   },
    { "lean_limit",       &g_param.lean_limit,       1, 0.0f,     15.0f  },
    { "lean_limit_mode",  &g_param.lean_limit_mode,  0, 0.0f,     1.0f   },
    // 元素阈值
    { "zebra_jump_cnt",   &g_param.zebra_jump_cnt,   0, 0.0f,     60.0f  },
    { "cross_lost_cnt",   &g_param.cross_lost_cnt,   0, 0.0f,     80.0f  },
    { "ring_angle",       &g_param.ring_angle,       0, 0.0f,     720.0f },
    { "ring_s2_cnt_l",    &g_param.ring_s2_cnt_l,    0, 0.0f,     5000.0f},
    { "ring_s2_cnt_r",    &g_param.ring_s2_cnt_r,    0, 0.0f,     5000.0f},
    { "ring_side_offset", &g_param.ring_side_offset, 0, 0.0f,     80.0f  },
    { "obs_narrow_ratio", &g_param.obs_narrow_ratio, 1, 0.0f,     1.0f   },
    { "obs_line_offset",  &g_param.obs_line_offset,  0, 0.0f,     80.0f  },
    // 零点、标定与保护
    { "roll_zero_init",   &g_param.roll_zero_init,   1, -45.0f,   45.0f  },
    { "pitch_zero_init",  &g_param.pitch_zero_init,  1, -45.0f,   45.0f  },
    { "roll_protect",     &g_param.roll_protect_angle, 1, 1.0f,   90.0f  },
    { "pitch_protect",    &g_param.pitch_protect_angle,1, 1.0f,   90.0f  },
    { "err_offset",       &g_param.err_offset,       1, -90.0f,   90.0f  },
    // 电机与编码器极性
    { "motor_dir_a",      &g_param.motor_dir_a,      0, -1.0f,    1.0f   },
    { "motor_dir_b",      &g_param.motor_dir_b,      0, -1.0f,    1.0f   },
    { "motor_dir_c",      &g_param.motor_dir_c,      0, -1.0f,    1.0f   },
    { "enc_dir_c",        &g_param.enc_dir_c,        0, -1.0f,    1.0f   },
};

#define PARAM_TABLE_NUM  ((uint16)(sizeof(g_param_table) / sizeof(g_param_table[0])))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     比较两个不区分大小写的 ASCII 字符串
// 参数说明     a/b             两个字符串
// 返回参数     int             0 表示相等
// 使用示例     int equal = (str_icmp(a, b) == 0);
//-------------------------------------------------------------------------------------------------------------------
static int str_icmp(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
        if (ca != cb) return (int)ca - (int)cb;
        a++; b++;
    }
    return (int)(uint8)*a - (int)(uint8)*b;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取参数描述表条目数
// 参数说明     void
// 返回参数     uint16          条目数
// 使用示例     uint16 count = param_table_count();
//-------------------------------------------------------------------------------------------------------------------
uint16 param_table_count(void)
{
    return PARAM_TABLE_NUM;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按名称查找参数描述
// 参数说明     name            参数名
// 返回参数     const param_desc_t*  参数描述, 未找到返回 0
// 使用示例     const param_desc_t *d = param_find("p_angle_kp");
//-------------------------------------------------------------------------------------------------------------------
const param_desc_t *param_find(const char *name)
{
    uint16 i;
    if (name == 0) return 0;
    for (i = 0; i < PARAM_TABLE_NUM; i++)
        if (str_icmp(name, g_param_table[i].name) == 0) return &g_param_table[i];
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按名称设置参数
// 参数说明     name/value      参数名和目标值
// 返回参数     uint8           1 表示成功, 0 表示参数不存在
// 使用示例     param_set_by_name("r_rate_kp", -45.0f);
//-------------------------------------------------------------------------------------------------------------------
uint8 param_set_by_name(const char *name, float value)
{
    const param_desc_t *d = param_find(name);
    if (d == 0) return 0;

    if (value < d->vmin) value = d->vmin;
    if (value > d->vmax) value = d->vmax;

    if (d->is_float) *(float *)d->ptr = value;
    else             *(int *)d->ptr   = (int)(value + (value >= 0.0f ? 0.5f : -0.5f));

    // 极性字段归一化
    if (d->ptr == &g_param.motor_dir_a || d->ptr == &g_param.motor_dir_b ||
        d->ptr == &g_param.motor_dir_c || d->ptr == &g_param.enc_dir_c)
        *(int *)d->ptr = (*(int *)d->ptr >= 0) ? 1 : -1;

    // 同步机械零点
    if (d->ptr == &g_param.roll_zero_init || d->ptr == &g_param.pitch_zero_init)
        param_sync_zero();

    g_param_revision++;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按名称读取参数
// 参数说明     name/value      参数名和输出地址
// 返回参数     uint8           1 表示成功, 0 表示参数不存在
// 使用示例     float v; param_get_by_name("r_rate_kp", &v);
//-------------------------------------------------------------------------------------------------------------------
uint8 param_get_by_name(const char *name, float *value)
{
    const param_desc_t *d = param_find(name);
    if (d == 0 || value == 0) return 0;
    *value = d->is_float ? (*(float *)d->ptr) : (float)(*(int *)d->ptr);
    return 1;
}

// Flash 数据区
#define PARAM_FLASH_SECTOR      (0u)            // DFlash 扇区
#define PARAM_FLASH_PAGE        (11u)           // DFlash 页

param_t g_param;                                // 运行参数
volatile uint32 g_param_revision = 0;            // 参数修订号

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将运行参数写入 Flash 缓冲区
// 参数说明     void
// 返回参数     uint32          已写入的缓冲单元数
// 使用示例     uint32 count = param_pack_to_buffer();
//-------------------------------------------------------------------------------------------------------------------
static uint32 param_pack_to_buffer(void)
{
    uint32 i = 0;
    flash_buffer_clear();
    flash_union_buffer[i++].uint32_type = g_param.magic;
    flash_union_buffer[i++].uint32_type = g_param.version;

    flash_union_buffer[i++].int32_type  = g_param.track_base_speed;
    flash_union_buffer[i++].float_type  = g_param.track_err_gain;
    flash_union_buffer[i++].float_type  = g_param.speed_ramp_gain;
    flash_union_buffer[i++].float_type  = g_param.speed_ring_gain;
    flash_union_buffer[i++].int32_type  = g_param.cam_exposure;

    flash_union_buffer[i++].float_type  = g_param.r_rcy_kp;
    flash_union_buffer[i++].float_type  = g_param.r_rcy_ki;
    flash_union_buffer[i++].float_type  = g_param.r_rcy_kd;
    flash_union_buffer[i++].float_type  = g_param.r_angle_kp;
    flash_union_buffer[i++].float_type  = g_param.r_angle_ki;
    flash_union_buffer[i++].float_type  = g_param.r_angle_kd;
    flash_union_buffer[i++].float_type  = g_param.r_rate_kp;
    flash_union_buffer[i++].float_type  = g_param.r_rate_ki;
    flash_union_buffer[i++].float_type  = g_param.r_rate_kd;

    flash_union_buffer[i++].float_type  = g_param.p_vel_kp;
    flash_union_buffer[i++].float_type  = g_param.p_vel_ki;
    flash_union_buffer[i++].float_type  = g_param.p_vel_kd;
    flash_union_buffer[i++].float_type  = g_param.p_angle_kp;
    flash_union_buffer[i++].float_type  = g_param.p_angle_ki;
    flash_union_buffer[i++].float_type  = g_param.p_angle_kd;
    flash_union_buffer[i++].float_type  = g_param.p_rate_kp;
    flash_union_buffer[i++].float_type  = g_param.p_rate_ki;
    flash_union_buffer[i++].float_type  = g_param.p_rate_kd;

    flash_union_buffer[i++].float_type  = g_param.y_angle_kp;
    flash_union_buffer[i++].float_type  = g_param.y_angle_ki;
    flash_union_buffer[i++].float_type  = g_param.y_angle_kd;
    flash_union_buffer[i++].float_type  = g_param.y_rate_kp;
    flash_union_buffer[i++].float_type  = g_param.y_rate_ki;
    flash_union_buffer[i++].float_type  = g_param.y_rate_kd;

    flash_union_buffer[i++].float_type  = g_param.lean_k1;
    flash_union_buffer[i++].float_type  = g_param.lean_k2;
    flash_union_buffer[i++].float_type  = g_param.lean_limit;
    flash_union_buffer[i++].int32_type  = g_param.lean_limit_mode;

    flash_union_buffer[i++].int32_type  = g_param.zebra_jump_cnt;
    flash_union_buffer[i++].int32_type  = g_param.cross_lost_cnt;
    flash_union_buffer[i++].int32_type  = g_param.ring_angle;
    flash_union_buffer[i++].int32_type  = g_param.ring_s2_cnt_l;
    flash_union_buffer[i++].int32_type  = g_param.ring_s2_cnt_r;
    flash_union_buffer[i++].int32_type  = g_param.ring_side_offset;
    flash_union_buffer[i++].float_type  = g_param.obs_narrow_ratio;
    flash_union_buffer[i++].int32_type  = g_param.obs_line_offset;

    flash_union_buffer[i++].float_type  = g_param.roll_zero_init;
    flash_union_buffer[i++].float_type  = g_param.pitch_zero_init;
    flash_union_buffer[i++].float_type  = g_param.roll_protect_angle;
    flash_union_buffer[i++].float_type  = g_param.pitch_protect_angle;
    flash_union_buffer[i++].float_type  = g_param.err_offset;


    flash_union_buffer[i++].int32_type  = g_param.motor_dir_a;
    flash_union_buffer[i++].int32_type  = g_param.motor_dir_b;
    flash_union_buffer[i++].int32_type  = g_param.motor_dir_c;
    flash_union_buffer[i++].int32_type  = g_param.enc_dir_c;
    return i;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 Flash 缓冲区读取运行参数
// 参数说明     void
// 返回参数     void
// 使用示例     param_unpack_from_buffer();
//-------------------------------------------------------------------------------------------------------------------
static void param_unpack_from_buffer(void)
{
    uint32 i = 0;
    g_param.magic   = flash_union_buffer[i++].uint32_type;
    g_param.version = flash_union_buffer[i++].uint32_type;

    g_param.track_base_speed = flash_union_buffer[i++].int32_type;
    g_param.track_err_gain   = flash_union_buffer[i++].float_type;
    g_param.speed_ramp_gain  = flash_union_buffer[i++].float_type;
    g_param.speed_ring_gain  = flash_union_buffer[i++].float_type;
    g_param.cam_exposure     = flash_union_buffer[i++].int32_type;

    g_param.r_rcy_kp   = flash_union_buffer[i++].float_type;
    g_param.r_rcy_ki   = flash_union_buffer[i++].float_type;
    g_param.r_rcy_kd   = flash_union_buffer[i++].float_type;
    g_param.r_angle_kp = flash_union_buffer[i++].float_type;
    g_param.r_angle_ki = flash_union_buffer[i++].float_type;
    g_param.r_angle_kd = flash_union_buffer[i++].float_type;
    g_param.r_rate_kp  = flash_union_buffer[i++].float_type;
    g_param.r_rate_ki  = flash_union_buffer[i++].float_type;
    g_param.r_rate_kd  = flash_union_buffer[i++].float_type;

    g_param.p_vel_kp   = flash_union_buffer[i++].float_type;
    g_param.p_vel_ki   = flash_union_buffer[i++].float_type;
    g_param.p_vel_kd   = flash_union_buffer[i++].float_type;
    g_param.p_angle_kp = flash_union_buffer[i++].float_type;
    g_param.p_angle_ki = flash_union_buffer[i++].float_type;
    g_param.p_angle_kd = flash_union_buffer[i++].float_type;
    g_param.p_rate_kp  = flash_union_buffer[i++].float_type;
    g_param.p_rate_ki  = flash_union_buffer[i++].float_type;
    g_param.p_rate_kd  = flash_union_buffer[i++].float_type;

    g_param.y_angle_kp = flash_union_buffer[i++].float_type;
    g_param.y_angle_ki = flash_union_buffer[i++].float_type;
    g_param.y_angle_kd = flash_union_buffer[i++].float_type;
    g_param.y_rate_kp  = flash_union_buffer[i++].float_type;
    g_param.y_rate_ki  = flash_union_buffer[i++].float_type;
    g_param.y_rate_kd  = flash_union_buffer[i++].float_type;

    g_param.lean_k1         = flash_union_buffer[i++].float_type;
    g_param.lean_k2         = flash_union_buffer[i++].float_type;
    g_param.lean_limit      = flash_union_buffer[i++].float_type;
    g_param.lean_limit_mode = flash_union_buffer[i++].int32_type;

    g_param.zebra_jump_cnt   = flash_union_buffer[i++].int32_type;
    g_param.cross_lost_cnt   = flash_union_buffer[i++].int32_type;
    g_param.ring_angle       = flash_union_buffer[i++].int32_type;
    g_param.ring_s2_cnt_l    = flash_union_buffer[i++].int32_type;
    g_param.ring_s2_cnt_r    = flash_union_buffer[i++].int32_type;
    g_param.ring_side_offset = flash_union_buffer[i++].int32_type;
    g_param.obs_narrow_ratio = flash_union_buffer[i++].float_type;
    g_param.obs_line_offset  = flash_union_buffer[i++].int32_type;

    g_param.roll_zero_init      = flash_union_buffer[i++].float_type;
    g_param.pitch_zero_init     = flash_union_buffer[i++].float_type;
    g_param.roll_protect_angle  = flash_union_buffer[i++].float_type;
    g_param.pitch_protect_angle = flash_union_buffer[i++].float_type;
    g_param.err_offset          = flash_union_buffer[i++].float_type;


    g_param.motor_dir_a = flash_union_buffer[i++].int32_type;
    g_param.motor_dir_b = flash_union_buffer[i++].int32_type;
    g_param.motor_dir_c = flash_union_buffer[i++].int32_type;
    g_param.enc_dir_c   = flash_union_buffer[i++].int32_type;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入 board_config.h 中的默认参数
// 参数说明     void
// 返回参数     void
// 使用示例     param_load_defaults();
//-------------------------------------------------------------------------------------------------------------------
void param_load_defaults(void)
{
    g_param.magic   = PARAM_MAGIC;
    g_param.version = PARAM_VERSION;

    g_param.track_base_speed = TRACK_BASE_SPEED_DEFAULT;
    g_param.track_err_gain   = TRACK_ERR_GAIN_DEFAULT;
    g_param.speed_ramp_gain  = SPEED_RAMP_GAIN_DEFAULT;
    g_param.speed_ring_gain  = SPEED_RING_GAIN_DEFAULT;
    g_param.cam_exposure     = CAM_EXPOSURE_DEFAULT;

    g_param.r_rcy_kp   = R_RCY_KP_DEFAULT;
    g_param.r_rcy_ki   = R_RCY_KI_DEFAULT;
    g_param.r_rcy_kd   = R_RCY_KD_DEFAULT;
    g_param.r_angle_kp = R_ANGLE_KP_DEFAULT;
    g_param.r_angle_ki = R_ANGLE_KI_DEFAULT;
    g_param.r_angle_kd = R_ANGLE_KD_DEFAULT;
    g_param.r_rate_kp  = R_RATE_KP_DEFAULT;
    g_param.r_rate_ki  = R_RATE_KI_DEFAULT;
    g_param.r_rate_kd  = R_RATE_KD_DEFAULT;

    g_param.p_vel_kp   = P_VEL_KP_DEFAULT;
    g_param.p_vel_ki   = P_VEL_KI_DEFAULT;
    g_param.p_vel_kd   = P_VEL_KD_DEFAULT;
    g_param.p_angle_kp = P_ANGLE_KP_DEFAULT;
    g_param.p_angle_ki = P_ANGLE_KI_DEFAULT;
    g_param.p_angle_kd = P_ANGLE_KD_DEFAULT;
    g_param.p_rate_kp  = P_RATE_KP_DEFAULT;
    g_param.p_rate_ki  = P_RATE_KI_DEFAULT;
    g_param.p_rate_kd  = P_RATE_KD_DEFAULT;

    g_param.y_angle_kp = Y_ANGLE_KP_DEFAULT;
    g_param.y_angle_ki = Y_ANGLE_KI_DEFAULT;
    g_param.y_angle_kd = Y_ANGLE_KD_DEFAULT;
    g_param.y_rate_kp  = Y_RATE_KP_DEFAULT;
    g_param.y_rate_ki  = Y_RATE_KI_DEFAULT;
    g_param.y_rate_kd  = Y_RATE_KD_DEFAULT;

    g_param.lean_k1         = LEAN_K1_DEFAULT;
    g_param.lean_k2         = LEAN_K2_DEFAULT;
    g_param.lean_limit      = LEAN_LIMIT_DEFAULT;
    g_param.lean_limit_mode = LEAN_LIMIT_MODE_DEFAULT;

    g_param.zebra_jump_cnt   = ZEBRA_JUMP_CNT_DEFAULT;
    g_param.cross_lost_cnt   = CROSS_LOST_CNT_DEFAULT;
    g_param.ring_angle       = RING_ANGLE_DEFAULT;
    g_param.ring_s2_cnt_l    = RING_S2_CNT_L_DEFAULT;
    g_param.ring_s2_cnt_r    = RING_S2_CNT_R_DEFAULT;
    g_param.ring_side_offset = RING_SIDE_OFFSET_DEFAULT;
    g_param.obs_narrow_ratio = OBS_NARROW_RATIO_DEFAULT;
    g_param.obs_line_offset  = OBS_LINE_OFFSET_DEFAULT;

    g_param.roll_zero_init      = ROLL_ZERO_INIT_DEFAULT;
    g_param.pitch_zero_init     = PITCH_ZERO_INIT_DEFAULT;
    g_param.roll_protect_angle  = ROLL_PROTECT_ANGLE_DEFAULT;
    g_param.pitch_protect_angle = PITCH_PROTECT_ANGLE_DEFAULT;
    g_param.err_offset          = 0.0f;            // 中线偏差零点


    g_param.motor_dir_a = MOTOR_DIR_A_DEFAULT;
    g_param.motor_dir_b = MOTOR_DIR_B_DEFAULT;
    g_param.motor_dir_c = MOTOR_DIR_C_DEFAULT;
    g_param.enc_dir_c   = ENC_DIR_C_DEFAULT;
    g_param_revision++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同步横滚与俯仰机械零点
// 参数说明     void
// 返回参数     void
// 使用示例     param_sync_zero();
//-------------------------------------------------------------------------------------------------------------------
void param_sync_zero(void)
{
    g_roll_zero  = g_param.roll_zero_init;
    g_pitch_zero = g_param.pitch_zero_init;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 Flash 初始化运行参数
// 参数说明     void
// 返回参数     void
// 使用示例     param_init();
//-------------------------------------------------------------------------------------------------------------------
void param_init(void)
{
    if (flash_check(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE))   // 检查页数据
    {
        flash_read_page_to_buffer(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE);    // 读取参数页
        param_unpack_from_buffer();
    }

    if (g_param.magic != PARAM_MAGIC || g_param.version != PARAM_VERSION)
        param_load_defaults();                     // 载入默认参数
    else
        g_param_revision++;                        // 更新修订号
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将运行参数保存到 Flash
// 参数说明     void
// 返回参数     uint8           1 表示成功, 0 表示失败
// 使用示例     uint8 saved = param_save();
//-------------------------------------------------------------------------------------------------------------------
uint8 param_save(void)
{
    g_param.magic   = PARAM_MAGIC;
    g_param.version = PARAM_VERSION;
    param_pack_to_buffer();                        // 打包参数
    return (uint8)(0 == flash_write_page_from_buffer(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE));
}
