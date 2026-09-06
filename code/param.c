#include "control.h"
#include "param.h"
#include "board_config.h"

// 参数描述表
const param_desc_t g_param_table[] =
{
    // 正式跑车与循迹
    { "run_speed_straight",     &g_param.run_speed_straight,     1, 0.0f,    RUN_SPEED_MAX_MPS },
    { "run_speed_curve",        &g_param.run_speed_curve,        1, 0.0f,    RUN_SPEED_MAX_MPS },
    { "run_speed_cross",        &g_param.run_speed_cross,        1, 0.0f,    RUN_SPEED_MAX_MPS },
    { "run_speed_ring",         &g_param.run_speed_ring,         1, 0.0f,    RUN_SPEED_MAX_MPS },
    { "run_speed_ramp",         &g_param.run_speed_ramp,         1, 0.0f,    RUN_SPEED_MAX_MPS },
    { "run_speed_lost",         &g_param.run_speed_lost,         1, 0.0f,    RUN_SPEED_MAX_MPS },
    { "run_accel_mps2",         &g_param.run_accel_mps2,         1, 0.05f,   10.0f   },
    { "run_decel_mps2",         &g_param.run_decel_mps2,         1, 0.05f,   10.0f   },
    { "direction_pixel_kp",     &g_param.direction_pixel_kp,     1, 0.0f,    10.0f   },
    { "direction_heading_kp",   &g_param.direction_heading_kp,   1, 0.0f,    10.0f   },
    { "direction_curve_kff",    &g_param.direction_curve_kff,    1, 0.0f,    100.0f  },
    { "direction_rate_kd",      &g_param.direction_balance_kd,   1, 0.0f,    1.0f    },
    { "zebra_stop_offset_m",    &g_param.zebra_stop_offset_m,    1, 0.0f,    2.0f    },
    { "err_front_row",    &g_param.err_front_row,    0, 10.0f,   115.0f  },
    // 逆透视矩阵九个系数。不在任何菜单页里，由 Calib IPM 动作行写入并存 Flash
    { "ipm_h0",           &g_param.ipm_h[0],         1, -1e8f,   1e8f    },
    { "ipm_h1",           &g_param.ipm_h[1],         1, -1e8f,   1e8f    },
    { "ipm_h2",           &g_param.ipm_h[2],         1, -1e8f,   1e8f    },
    { "ipm_h3",           &g_param.ipm_h[3],         1, -1e8f,   1e8f    },
    { "ipm_h4",           &g_param.ipm_h[4],         1, -1e8f,   1e8f    },
    { "ipm_h5",           &g_param.ipm_h[5],         1, -1e8f,   1e8f    },
    { "ipm_h6",           &g_param.ipm_h[6],         1, -1e8f,   1e8f    },
    { "ipm_h7",           &g_param.ipm_h[7],         1, -1e8f,   1e8f    },
    { "ipm_h8",           &g_param.ipm_h[8],         1, -1e8f,   1e8f    },
    { "cam_exposure",     &g_param.cam_exposure,     0,  4.0f,   1600.0f },
    { "odom_counts_per_m",&g_param.odom_counts_per_m,1, 100.0f,  50000.0f },
    { "odom_test_speed",  &g_param.odom_test_speed,  1, 0.05f,   0.50f    },
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
    { "lean_roll_kp",      &g_param.direction_roll_kp, 1, 0.0f, 20.0f },
    { "lean_max_angle",    &g_param.lean_max_angle,    1, 0.0f, 8.0f  },
    // 元素现场参数
    { "elem_en_zebra",    &g_param.elem_en_zebra,    0, 0.0f,     1.0f   },
    { "elem_en_cross",    &g_param.elem_en_cross,    0, 0.0f,     1.0f   },
    { "elem_en_ring",     &g_param.elem_en_ring,     0, 0.0f,     1.0f   },
    { "elem_en_ramp",     &g_param.elem_en_ramp,     0, 0.0f,     1.0f   },
    { "ring_angle",       &g_param.ring_angle,       0, 0.0f,     720.0f },
    { "ring_s2_cnt_l",    &g_param.ring_s2_cnt_l,    0, 0.0f,     5000.0f},
    { "ring_s2_cnt_r",    &g_param.ring_s2_cnt_r,    0, 0.0f,     5000.0f},
    // 零点、标定与保护
    { "roll_zero_init",   &g_param.roll_zero_init,   1, -45.0f,   45.0f  },
    { "pitch_zero_init",  &g_param.pitch_zero_init,  1, -45.0f,   45.0f  },
    { "roll_protect",     &g_param.roll_protect_angle, 1, 1.0f,   90.0f  },
    { "pitch_protect",    &g_param.pitch_protect_angle,1, 1.0f,   90.0f  },
    // 电机与编码器极性
    { "motor_dir_a",      &g_param.motor_dir_a,      0, -1.0f,    1.0f   },
    { "motor_dir_b",      &g_param.motor_dir_b,      0, -1.0f,    1.0f   },
    { "motor_dir_c",      &g_param.motor_dir_c,      0, -1.0f,    1.0f   },
    { "enc_dir_c",        &g_param.enc_dir_c,        0, -1.0f,    1.0f   },
    { "steer_dir",        &g_param.steer_dir,        0, -1.0f,    1.0f   },
    { "jog_duty_fly",     &g_param.jog_duty_fly,     0,  0.0f,    10000.0f },
    // 点动只用来验方向，架空空载 3500 已经转得很快了，所以卡在 DRIVE_OUT_LIMIT(8000) 之下
    { "jog_duty_drive",   &g_param.jog_duty_drive,   0,  0.0f,    3500.0f  },
    { "fly_speed_limit",  &g_param.fly_speed_limit,  0,  0.0f,    20000.0f },
    { "fly_slew",         &g_param.fly_slew,         0,  0.0f,    10000.0f },
};

#define PARAM_TABLE_NUM  ((uint16)(sizeof(g_param_table) / sizeof(g_param_table[0])))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     比较两个不区分大小写的 ASCII 字符串
// 参数说明     a/b             两个字符串
// 返回参数     int             0 表示相等
// 使用示例     int equal = (str_icmp(a, b) == 0);
//-------------------------------------------------------------------------------------------------------------------
uint16 param_count(void) { return PARAM_TABLE_NUM; }

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
// 函数简介     判断单精度浮点数是否为有限值
// 参数说明     value           待检查数值
// 返回参数     uint8           1 为有限值, 0 为 NaN 或正负无穷
// 使用示例     if (!param_float_is_finite(value)) return 0;
//-------------------------------------------------------------------------------------------------------------------
static uint8 param_float_is_finite(float value)
{
    union
    {
        float  f;
        uint32 u;
    } bits;

    bits.f = value;
    return (uint8)((bits.u & 0x7F800000u) != 0x7F800000u);
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
    if (d == 0 || !param_float_is_finite(value)) return 0;

    if (value < d->vmin) value = d->vmin;
    if (value > d->vmax) value = d->vmax;

    if (d->is_float) *(float *)d->ptr = value;
    else             *(int *)d->ptr   = (int)(value + (value >= 0.0f ? 0.5f : -0.5f));

    // 极性字段归一化
    if (d->ptr == &g_param.motor_dir_a || d->ptr == &g_param.motor_dir_b ||
        d->ptr == &g_param.motor_dir_c || d->ptr == &g_param.enc_dir_c ||
        d->ptr == &g_param.steer_dir)
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

#define PARAM_FLASH_SECTOR       (0u)           // DFlash 扇区
#define PARAM_FLASH_PAGE         (11u)          // DFlash 页
#define PARAM_MAGIC_INDEX        (0u)           // 魔数所在字
#define PARAM_COUNT_INDEX        (1u)           // 条数所在字
#define PARAM_FIRST_RECORD_INDEX (2u)           // 第一组键值对所在字
#define PARAM_MAX_RECORDS        ((uint32)((EEPROM_PAGE_LENGTH - 3u) / 2u))
#define PARAM_RECORD_WORDS       ((uint32)(3u + (uint32)PARAM_TABLE_NUM * 2u))  // 整条记录字数

// 参数表放不下一页 Flash 时下面这行会因为数组长度为负而编译失败
typedef char param_record_fits_one_page[(PARAM_TABLE_NUM <= PARAM_MAX_RECORDS) ? 1 : -1];

param_t g_param;                                // 运行参数
volatile uint32 g_param_revision = 0;            // 参数修订号

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算若干 32 位字的标准 CRC32
// 参数说明     data/count      数据首地址与 32 位字数
// 返回参数     uint32          CRC32 校验值
// 使用示例     crc = param_crc32_words(words, PARAM_DATA_WORDS);
//-------------------------------------------------------------------------------------------------------------------
static uint32 param_crc32_words(const uint32 *data, uint32 count)
{
    uint32 crc = 0xFFFFFFFFu;
    uint32 i;

    for (i = 0; i < count; i++)
    {
        uint32 word = data[i];
        uint8 byte_index;

        for (byte_index = 0; byte_index < 4u; byte_index++)
        {
            uint8 bit;
            crc ^= (uint8)(word >> ((uint32)byte_index * 8u));
            for (bit = 0; bit < 8u; bit++)
                crc = (crc >> 1u) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查当前运行参数的有限值、范围与方向字段
// 参数说明     void
// 返回参数     uint8           1 表示全部合法, 0 表示至少一项非法
// 使用示例     if (!param_validate_current()) param_load_defaults();
//-------------------------------------------------------------------------------------------------------------------
static uint8 param_validate_current(void)
{
    uint16 i;

    for (i = 0; i < PARAM_TABLE_NUM; i++)
    {
        const param_desc_t *d = &g_param_table[i];

        if (d->is_float)
        {
            float value = *(float *)d->ptr;
            if (!param_float_is_finite(value) || value < d->vmin || value > d->vmax)
                return 0;
        }
        else
        {
            int value = *(int *)d->ptr;
            if ((float)value < d->vmin || (float)value > d->vmax)
                return 0;
        }
    }

    if ((g_param.motor_dir_a != 1 && g_param.motor_dir_a != -1) ||
        (g_param.motor_dir_b != 1 && g_param.motor_dir_b != -1) ||
        (g_param.motor_dir_c != 1 && g_param.motor_dir_c != -1) ||
        (g_param.enc_dir_c   != 1 && g_param.enc_dir_c   != -1) ||
        (g_param.steer_dir   != 1 && g_param.steer_dir   != -1))
        return 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     由参数名和类型算出存储键，Flash 记录按键索引而不是按表内顺序
// 参数说明     name/is_float   参数名与是否浮点
// 返回参数     uint32          FNV-1a 哈希，最后把类型折进去
// 使用示例     key = param_name_key("r_rate_kp", 1);
//-------------------------------------------------------------------------------------------------------------------
static uint32 param_name_key(const char *name, uint8 is_float)
{
    uint32 hash = 2166136261u;                  // FNV-1a 32 位偏移基

    while (*name != 0)
    {
        hash ^= (uint32)(uint8)*name++;
        hash *= 16777619u;
    }
    // 类型折进键里：某个参数从 int 改成 float 时旧值自动失配，只有它回默认值
    return hash ^ (is_float ? 0x5A5A5A5Au : 0u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查参数表内所有存储键互不相同
// 参数说明     void
// 返回参数     uint8           1=全部唯一 0=有重名或哈希碰撞
// 使用示例     if (!param_keys_unique()) return;
//-------------------------------------------------------------------------------------------------------------------
static uint8 param_keys_unique(void)
{
    uint16 i, j;

    for (i = 0; i < PARAM_TABLE_NUM; i++)
    {
        uint32 key = param_name_key(g_param_table[i].name, g_param_table[i].is_float);

        for (j = (uint16)(i + 1u); j < PARAM_TABLE_NUM; j++)
            if (key == param_name_key(g_param_table[j].name, g_param_table[j].is_float))
                return 0;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把一条键值对写进对应参数，越界的钳到该参数允许范围内
// 参数说明     key/value       存储键与原始 32 位值
// 返回参数     void
// 使用示例     param_apply_record(key, value);
//-------------------------------------------------------------------------------------------------------------------
static void param_apply_record(uint32 key, flash_data_union value)
{
    uint16 i;

    for (i = 0; i < PARAM_TABLE_NUM; i++)
    {
        const param_desc_t *d = &g_param_table[i];

        if (param_name_key(d->name, d->is_float) != key) continue;

        if (d->is_float)
        {
            float v = value.float_type;
            if (!param_float_is_finite(v)) return;      // 这一项存坏了就保持默认值
            if (v < d->vmin) v = d->vmin;
            if (v > d->vmax) v = d->vmax;
            *(float *)d->ptr = v;
        }
        else
        {
            int v = value.int32_type;
            if ((float)v < d->vmin) v = (int)d->vmin;
            if ((float)v > d->vmax) v = (int)d->vmax;
            *(int *)d->ptr = v;
        }
        return;
    }
    // 键不在当前参数表里：这个参数已经删掉或改名，直接忽略
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把四个方向参数归一到 +1 或 -1
// 参数说明     void
// 返回参数     void
// 使用示例     param_normalize_dirs();
//-------------------------------------------------------------------------------------------------------------------
static void param_normalize_dirs(void)
{
    g_param.motor_dir_a = (g_param.motor_dir_a >= 0) ? 1 : -1;
    g_param.motor_dir_b = (g_param.motor_dir_b >= 0) ? 1 : -1;
    g_param.motor_dir_c = (g_param.motor_dir_c >= 0) ? 1 : -1;
    g_param.enc_dir_c   = (g_param.enc_dir_c   >= 0) ? 1 : -1;
    g_param.steer_dir   = (g_param.steer_dir   >= 0) ? 1 : -1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入 board_config.h 中的默认参数
// 参数说明     void
// 返回参数     void
// 使用示例     param_load_defaults();
//-------------------------------------------------------------------------------------------------------------------
void param_load_defaults(void)
{
    g_param.run_speed_straight     = RUN_SPEED_STRAIGHT_DEFAULT;
    g_param.run_speed_curve        = RUN_SPEED_CURVE_DEFAULT;
    g_param.run_speed_cross        = RUN_SPEED_CROSS_DEFAULT;
    g_param.run_speed_ring         = RUN_SPEED_RING_DEFAULT;
    g_param.run_speed_ramp         = RUN_SPEED_RAMP_DEFAULT;
    g_param.run_speed_lost         = RUN_SPEED_LOST_DEFAULT;
    g_param.run_accel_mps2         = RUN_ACCEL_MPS2_DEFAULT;
    g_param.run_decel_mps2         = RUN_DECEL_MPS2_DEFAULT;
    g_param.direction_pixel_kp     = DIRECTION_PIXEL_KP_DEFAULT;
    g_param.direction_heading_kp   = DIRECTION_HEADING_KP_DEFAULT;
    g_param.direction_curve_kff    = DIRECTION_CURVE_KFF_DEFAULT;
    g_param.direction_balance_kd   = DIRECTION_BALANCE_KD_DEFAULT;
    g_param.zebra_stop_offset_m    = ZEBRA_STOP_OFFSET_M_DEFAULT;
    g_param.err_front_row    = ERR_FRONT_ROW_DEFAULT;
    g_param.ipm_h[0] = IPM_H0_DEFAULT;
    g_param.ipm_h[1] = IPM_H1_DEFAULT;
    g_param.ipm_h[2] = IPM_H2_DEFAULT;
    g_param.ipm_h[3] = IPM_H3_DEFAULT;
    g_param.ipm_h[4] = IPM_H4_DEFAULT;
    g_param.ipm_h[5] = IPM_H5_DEFAULT;
    g_param.ipm_h[6] = IPM_H6_DEFAULT;
    g_param.ipm_h[7] = IPM_H7_DEFAULT;
    g_param.ipm_h[8] = IPM_H8_DEFAULT;
    g_param.cam_exposure     = CAM_EXPOSURE_DEFAULT;
    g_param.odom_counts_per_m = ODOM_COUNTS_PER_M_DEFAULT;
    g_param.odom_test_speed   = ODOM_TEST_SPEED_DEFAULT;

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

    g_param.direction_roll_kp = DIRECTION_ROLL_KP_DEFAULT;
    g_param.lean_max_angle    = LEAN_MAX_ANGLE_DEFAULT;
    g_param.elem_en_zebra    = ELEM_EN_ZEBRA_DEFAULT;
    g_param.elem_en_cross    = ELEM_EN_CROSS_DEFAULT;
    g_param.elem_en_ring     = ELEM_EN_RING_DEFAULT;
    g_param.elem_en_ramp     = ELEM_EN_RAMP_DEFAULT;
    g_param.ring_angle       = RING_ANGLE_DEFAULT;
    g_param.ring_s2_cnt_l    = RING_S2_CNT_L_DEFAULT;
    g_param.ring_s2_cnt_r    = RING_S2_CNT_R_DEFAULT;

    g_param.roll_zero_init      = ROLL_ZERO_INIT_DEFAULT;
    g_param.pitch_zero_init     = PITCH_ZERO_INIT_DEFAULT;
    g_param.roll_protect_angle  = ROLL_PROTECT_ANGLE_DEFAULT;
    g_param.pitch_protect_angle = PITCH_PROTECT_ANGLE_DEFAULT;


    g_param.motor_dir_a = MOTOR_DIR_A_DEFAULT;
    g_param.motor_dir_b = MOTOR_DIR_B_DEFAULT;
    g_param.motor_dir_c = MOTOR_DIR_C_DEFAULT;
    g_param.enc_dir_c   = ENC_DIR_C_DEFAULT;
    g_param.steer_dir   = STEER_DIR_DEFAULT;
    g_param.jog_duty_fly   = JOG_DUTY_FLY_DEFAULT;
    g_param.jog_duty_drive = JOG_DUTY_DRIVE_DEFAULT;
    g_param.fly_speed_limit = FLY_SPEED_LIMIT_DEFAULT;
    g_param.fly_slew        = FLY_SLEW_DEFAULT;
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
// 函数简介     擦除 Flash 参数页并把运行参数恢复成默认值
// 参数说明     void
// 返回参数     uint8           1=擦除后页确实是空的 0=擦除失败
// 使用示例     uint8 ok = param_erase();
//-------------------------------------------------------------------------------------------------------------------
uint8 param_erase(void)
{
    if (flash_check(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE))
        flash_erase_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE);

    param_load_defaults();
    // flash_check 返回 0 表示整页全是 0，也就是擦干净了
    return (uint8)(flash_check(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE) == 0u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 Flash 初始化运行参数
// 参数说明     void
// 返回参数     void
// 使用示例     param_init();
//-------------------------------------------------------------------------------------------------------------------
void param_init(void)
{
    uint32 count;
    uint32 words;
    uint32 i;

    // 先铺一遍默认值。Flash 里没有的键就保持默认，所以新加的参数不影响旧记录
    param_load_defaults();

    // 存储键撞了会把值写进错的参数，这种情况宁可整体不加载
    if (!param_keys_unique()) return;
    if (!flash_check(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE)) return;

    flash_read_page_to_buffer(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE);
    if (flash_union_buffer[PARAM_MAGIC_INDEX].uint32_type != PARAM_MAGIC) return;

    count = flash_union_buffer[PARAM_COUNT_INDEX].uint32_type;
    if (count == 0u || count > PARAM_MAX_RECORDS) return;

    words = PARAM_FIRST_RECORD_INDEX + count * 2u;
    if (flash_union_buffer[words].uint32_type !=
        param_crc32_words((const uint32 *)flash_union_buffer, words)) return;

    for (i = 0; i < count; i++)
        param_apply_record(flash_union_buffer[PARAM_FIRST_RECORD_INDEX + i * 2u].uint32_type,
                           flash_union_buffer[PARAM_FIRST_RECORD_INDEX + i * 2u + 1u]);

    param_normalize_dirs();
    g_param_revision++;
}

static flash_data_union s_stored[PARAM_TABLE_NUM];  // Flash 里现存的值，按参数表下标铺开
static uint8            s_stored_ok[PARAM_TABLE_NUM];   // 该参数在 Flash 里有记录

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把 Flash 里现存的键值对按参数表下标铺进 s_stored / s_stored_ok
// 参数说明     void
// 返回参数     void
// 使用示例     param_load_stored();
//-------------------------------------------------------------------------------------------------------------------
static void param_load_stored(void)
{
    uint32 count, words, i;
    uint16 t;

    for (t = 0; t < PARAM_TABLE_NUM; t++) s_stored_ok[t] = 0;

    // 下面每一条校验不过就当整页没记录，此时按页保存等价于只写本页那几个参数
    if (!flash_check(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE)) return;
    flash_read_page_to_buffer(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE);
    if (flash_union_buffer[PARAM_MAGIC_INDEX].uint32_type != PARAM_MAGIC) return;

    count = flash_union_buffer[PARAM_COUNT_INDEX].uint32_type;
    if (count == 0u || count > PARAM_MAX_RECORDS) return;

    words = PARAM_FIRST_RECORD_INDEX + count * 2u;
    if (flash_union_buffer[words].uint32_type !=
        param_crc32_words((const uint32 *)flash_union_buffer, words)) return;

    for (i = 0; i < count; i++)
    {
        uint32 key = flash_union_buffer[PARAM_FIRST_RECORD_INDEX + i * 2u].uint32_type;

        for (t = 0; t < PARAM_TABLE_NUM; t++)
        {
            const param_desc_t *d = &g_param_table[t];

            if (param_name_key(d->name, d->is_float) != key) continue;
            s_stored[t]    = flash_union_buffer[PARAM_FIRST_RECORD_INDEX + i * 2u + 1u];
            s_stored_ok[t] = 1;
            break;
        }
        // 键不在当前参数表里：这个参数已经删掉或改名，不搬回去
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断参数名是否在给定的名字表里
// 参数说明     name/names/count 参数名、名字表与表长
// 返回参数     uint8           1 表示在表里
// 使用示例     if (param_name_listed(d->name, names, count)) ...
//-------------------------------------------------------------------------------------------------------------------
static uint8 param_name_listed(const char *name, const char *const *names, uint16 count)
{
    uint16 i;

    for (i = 0; i < count; i++)
        if (names[i] != 0 && strcmp(names[i], name) == 0) return 1;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     组装并写入一条 Flash 记录，names 为 0 表示保存全部参数
// 参数说明     names/count     要取内存当前值的参数名表与表长
// 返回参数     uint8           1 表示写入并回读比对成功
// 使用示例     ok = param_save_impl(0, 0);
//-------------------------------------------------------------------------------------------------------------------
static uint8 param_save_impl(const char *const *names, uint16 count)
{
    static uint32 expected[PARAM_RECORD_WORDS];     // 回读比对用，放静态区不占栈
    uint32 words;
    uint16 t;
    uint16 n = 0;                                   // 实际写进去的记录条数
    uint32 i;

    if (!param_keys_unique()) return 0;
    if (!param_validate_current()) return 0;

    // 全量保存每一条都取内存值，不需要旧记录；按页保存才要先读出来
    if (names != 0) param_load_stored();

    flash_union_buffer[PARAM_MAGIC_INDEX].uint32_type = PARAM_MAGIC;
    for (t = 0; t < PARAM_TABLE_NUM; t++)
    {
        const param_desc_t *d = &g_param_table[t];
        uint8  from_ram = (uint8)((names == 0) ? 1 : param_name_listed(d->name, names, count));
        uint32 at;

        // 不在本页、Flash 里也没存过 → 这一条整个不写
        if (!from_ram && !s_stored_ok[t]) continue;

        at = PARAM_FIRST_RECORD_INDEX + (uint32)n * 2u;
        flash_union_buffer[at].uint32_type = param_name_key(d->name, d->is_float);
        if (!from_ram)
        {
            flash_union_buffer[at + 1u] = s_stored[t];      // 别的页的旧值原样搬回去
        }
        else if (d->is_float)
        {
            flash_union_buffer[at + 1u].float_type = *(float *)d->ptr;
        }
        else
        {
            flash_union_buffer[at + 1u].int32_type = *(int *)d->ptr;
        }
        n++;
    }
    if (n == 0u) return 0;                          // 空记录会让 param_init() 判无效

    flash_union_buffer[PARAM_COUNT_INDEX].uint32_type = (uint32)n;
    words = PARAM_FIRST_RECORD_INDEX + (uint32)n * 2u;
    flash_union_buffer[words].uint32_type =
        param_crc32_words((const uint32 *)flash_union_buffer, words);

    for (i = 0; i <= words; i++)
        expected[i] = flash_union_buffer[i].uint32_type;

    if (flash_write_page_from_buffer(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE) != 0u)
        return 0;
    flash_read_page_to_buffer(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE);

    // 逐字比对而不是只验 CRC：写入整页没生效时旧记录的 CRC 一样是对的
    for (i = 0; i <= words; i++)
        if (flash_union_buffer[i].uint32_type != expected[i]) return 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将全部运行参数保存到 Flash
// 参数说明     void
// 返回参数     uint8           1 表示成功, 0 表示失败
// 使用示例     uint8 saved = param_save();
//-------------------------------------------------------------------------------------------------------------------
uint8 param_save(void)
{
    return param_save_impl(0, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     只把名字表里的参数存进 Flash，其余参数保持 Flash 里已有的值
// 参数说明     names/count     参数名表与表长，名字要和 g_param_table 里的完全一致
// 返回参数     uint8           1 表示成功, 0 表示失败或表为空
// 使用示例     uint8 saved = param_save_names(names, 9);
//-------------------------------------------------------------------------------------------------------------------
uint8 param_save_names(const char *const *names, uint16 count)
{
    if (names == 0 || count == 0u) return 0;
    return param_save_impl(names, count);
}
