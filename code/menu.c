#include "menu.h"

#include <stdio.h>
#include <string.h>

#include "W_Motor.h"
#include "Y_Motor.h"
#include "attitude.h"
#include "board_config.h"
#include "control.h"
#include "display.h"
#include "element.h"
#include "imu.h"
#include "param.h"
#include "vision_core.h"
#include "vofa.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"

// 菜单配色：黑色背景、黄色光标、绿色参数、红色编辑状态。
#define UI_BLACK       ((uint16)0x0000u)
#define UI_WHITE       ((uint16)0xFFFFu)
#define UI_YELLOW      ((uint16)0xFFE0u)
#define UI_GREEN       ((uint16)0x07E0u)
#define UI_RED         ((uint16)0xF800u)
#define UI_GRAY        ((uint16)0x7BEFu)
#define UI_CYAN        ((uint16)0x07FFu)

#define UI_TITLE_X             (5u)
#define UI_TITLE_Y             (2u)
#define UI_LIST_Y              (21u)
#define UI_ROW_H               (16u)
#define UI_VALUE_X             (112u)
#define UI_PAGE_X              (176u)
#define UI_FOOTER_Y            (302u)
#define UI_STATUS_Y            (277u)
#define UI_VISIBLE_ROWS        (13u)
#define UI_LONG_REPEAT_MS      (120u)
#define UI_LIVE_PERIOD_MS      (100u)
#define UI_IMAGE_PERIOD_MS     (100u)    // 横屏整帧刷新周期
#define UI_IMAGE_STALL_MS      (500u)    // 停帧时的状态栏刷新周期，保证 FPS 能掉到 0
#define UI_RESET_CONFIRM_MS    (3000u)   // Reset 二次确认窗口
#define UI_BALANCE_CONFIRM_MS  (3000u)   // Balance 二次确认窗口
#define UI_MAX_GROUP_ITEMS     (16u)     // 单页参数行上限，子页 Save 组名字表用
#define UI_CAM_FRAME_TIMEOUT_MS (100u)   // 超过该时间没有新帧视为摄像头帧失联

typedef enum
{
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_PARAMS,
    MENU_PAGE_ATTITUDE,
    MENU_PAGE_GROUP,
    MENU_PAGE_IMAGE,
    MENU_PAGE_RUN_TEST,
    MENU_PAGE_COUNT
} menu_page_t;

// 参数组的动作类型，决定组内 Action 行的语义与实时显示内容
typedef enum
{
    GROUP_KIND_AXIS = 0,    // 分环 Test，闭环驱动对应轴
    GROUP_KIND_CAMERA,      // 循迹参数，实时显示阈值、丢线和帧率，无动作行
    GROUP_KIND_MOTOR,       // 架空点动，验证转向与转速回读符号
    GROUP_KIND_ZERO,        // 在线抓当前姿态角当机械零点
    GROUP_KIND_ELEMENT,     // 元素使能与阈值，实时显示识别到的元素，无动作行
    GROUP_KIND_ODOMETRY,    // C 轮每米脉冲标定与 1m 验证
} group_kind_t;

// 一个可编辑参数行
typedef struct
{
    const char *label;      // 屏幕上显示的名字
    const char *name;       // 对应 g_param_table 里的参数名
    float       step;       // 单次按键的调整步长
    uint8       decimals;   // 显示小数位数
} menu_param_item_t;

// 一个参数组，对应 Params 下的一页
typedef struct
{
    const char              *title;         // 页标题
    const menu_param_item_t *items;         // 参数行数组
    uint8                    item_count;    // 参数行数量
    tune_axis_t              axis;          // 该页对应的调参轴，只有 GROUP_KIND_AXIS 会用到
    uint8                    action_count;  // 动作行数量
    group_kind_t             kind;          // 动作行语义
} menu_group_t;

// PID 三页的步长按各环的实际落点定，规则是"步长 ≈ 落点下限的 1/10"：
// 从 0 按十下就到起手值，调到落点上限一百下以内，停在最终值附近时一下改动 1~5%。
// 落点区间见 控制调试指南.md 第 5 节。步长比落点还大或者小两个数量级都没法用键调，
// 前者一按就冲过头，后者要按几百下。
// 恒为 0 的 ki/kd 行跟同一个环的 kp 取同一尺度，免得一页之内量级看着是乱的。
static const menu_param_item_t s_roll_items[] =
{
    { "Rate Kp",  "r_rate_kp",  1.0f,   2 },    // 落点 10~80
    { "Rate Ki",  "r_rate_ki",  0.01f,  3 },    // 落点 0.2~1.0
    { "Rate Kd",  "r_rate_kd",  0.5f,   2 },
    { "Angle Kp", "r_angle_kp", 0.2f,   2 },    // 落点 2~20
    { "Angle Ki", "r_angle_ki", 0.01f,  3 },
    { "Angle Kd", "r_angle_kd", 0.2f,   2 },    // 位置式内环之后这一路可用
    // 回收环 kp。实测落点在 0.002 附近，比原先估的 0.003~0.03 小一个量级，
    // 所以步长不按"落点下限的 1/10"给：0.001 一按就改 50%，停不到 0.0021 这种值上。
    // 0.0001 是一按约 5%，显示同步到 4 位小数才看得见改动
    { "Speed Kp", "r_rcy_kp",   0.0001f, 4 },   // 飞轮转速差(RPM)换倾角(°)，符号为正
    { "Speed Ki", "r_rcy_ki",   0.0001f, 4 },  // 与同环 kp 同尺度，见本表头的约定
    { "Speed Kd", "r_rcy_kd",   0.0001f, 4 }   // 同上
};

static const menu_param_item_t s_pitch_items[] =
{
    { "Rate Kp",  "p_rate_kp",  0.5f,   2 },    // 落点 5~50
    { "Rate Ki",  "p_rate_ki",  0.01f,  3 },    // 落点 0.1~0.8
    { "Rate Kd",  "p_rate_kd",  0.5f,   2 },
    { "Angle Kp", "p_angle_kp", 0.2f,   2 },    // 落点 2~20
    { "Angle Ki", "p_angle_ki", 0.01f,  3 },
    { "Angle Kd", "p_angle_kd", 0.2f,   2 },
    { "Speed Kp", "p_vel_kp",   0.001f, 4 },    // 落点 0.005~0.05，counts/20ms 换倾角(°)
    { "Speed Ki", "p_vel_ki",   0.001f, 4 },
    { "Speed Kd", "p_vel_kd",   0.001f, 4 }
};

static const menu_param_item_t s_yaw_items[] =
{
    { "Rate Kp",  "y_rate_kp",  0.5f,   2 },    // 落点 5~50，位置式，只调这一个
    { "Rate Ki",  "y_rate_ki",  0.01f,  3 },
    { "Rate Kd",  "y_rate_kd",  0.5f,   2 },
    { "Angle Kp", "y_angle_kp", 0.1f,   2 },    // 落点 1~5
    { "Angle Ki", "y_angle_ki", 0.01f,  3 },
    { "Angle Kd", "y_angle_kd", 0.1f,   2 }
};

static const menu_param_item_t s_camera_items[] =
{
    { "Exposure",   "cam_exposure",      2.0f, 0 },
    { "Road Near",  "road_wide_near",    1.0f,  0 },
    { "Road Far",   "road_wide_far",     1.0f,  0 },
    { "Error Zero", "err_offset",        0.5f,  1 },
    { "Track Gain", "track_err_gain",    0.05f, 2 },
    { "Base Speed", "track_base_speed",  1.0f,  0 },
    { "Spd Up",     "speed_up_rate",     0.1f,  2 },
    { "Spd Down",   "speed_down_rate",   0.1f,  2 }
};

// 元素页。前四行是使能，步长 2 大于取值范围，所以上键一定开、下键一定关。
// 默认全 0，普通循迹跑稳后一次只开一个。
static const menu_param_item_t s_element_items[] =
{
    { "En Zebra",   "elem_en_zebra",    2.0f, 0 },
    { "En Cross",   "elem_en_cross",    2.0f, 0 },
    { "En Ring",    "elem_en_ring",     2.0f, 0 },
    { "En Ramp",    "elem_en_ramp",     2.0f, 0 },
    { "Zebra Jump", "zebra_jump_cnt",   1.0f, 0 },
    { "Cross Lost", "cross_lost_cnt",   1.0f, 0 },
    { "Ring Angle", "ring_angle",       5.0f, 0 },
    { "Ring CntL",  "ring_s2_cnt_l",   10.0f, 0 },
    { "Ring CntR",  "ring_s2_cnt_r",   10.0f, 0 },
    { "Ring Ofs",   "ring_side_offset", 1.0f, 0 },
    { "Ring TmO",   "ring_timeout_cnt",10.0f, 0 },
    { "Guard Cnt",  "elem_guard_cnt",   5.0f, 0 },
    { "Ramp Gain",  "speed_ramp_gain",  0.05f, 2 },
    { "Ring Gain",  "speed_ring_gain",  0.05f, 2 }
};

// 极性取值只有 ±1，步长给 2 保证一次按键就翻符号。
static const menu_param_item_t s_motor_items[] =
{
    { "Dir A",    "motor_dir_a",    2.0f,   0 },
    { "Dir B",    "motor_dir_b",    2.0f,   0 },
    { "Dir C",    "motor_dir_c",    2.0f,   0 },
    { "Enc C",    "enc_dir_c",      2.0f,   0 },
    { "Duty Fly", "jog_duty_fly",   100.0f, 0 },
    { "Duty Drv", "jog_duty_drive", 100.0f, 0 },
    { "Fly RpmMx","fly_speed_limit",100.0f, 0 },
    { "Fly Slew", "fly_slew",       100.0f, 0 }
};

static const menu_param_item_t s_zero_items[] =
{
    { "Roll Zero",  "roll_zero_init",  0.1f, 2 },
    { "Pitch Zero", "pitch_zero_init", 0.1f, 2 },
    { "Roll Prot",  "roll_protect",    1.0f, 1 },
    { "Pitch Prot", "pitch_protect",   1.0f, 1 }
};

static const menu_param_item_t s_odometry_items[] =
{
    { "Counts / m", "odom_counts_per_m", 10.0f, 0 },
    { "Test Speed", "odom_test_speed",     0.01f, 2 }
};

// 点动动作行的顺序与 motor_jog_action() 的解码一一对应，改一处必须改另一处。
static const char * const s_motor_action_names[] =
{
    "Jog A Fwd", "Jog A Rev",
    "Jog B Fwd", "Jog B Rev",
    "Jog C Fwd", "Jog C Rev"
};

#define MENU_ITEMS_OF(a) ((uint8)(sizeof(a) / sizeof((a)[0])))

static const menu_group_t s_groups[] =
{
    { "Roll",   s_roll_items,   MENU_ITEMS_OF(s_roll_items),   TUNE_AXIS_ROLL,  3, GROUP_KIND_AXIS   },
    { "Pitch",  s_pitch_items,  MENU_ITEMS_OF(s_pitch_items),  TUNE_AXIS_PITCH, 3, GROUP_KIND_AXIS   },
    { "Yaw",    s_yaw_items,    MENU_ITEMS_OF(s_yaw_items),    TUNE_AXIS_YAW,   2, GROUP_KIND_AXIS   },
    // 下面五页不走 control_test_start()，axis 填什么都不会被读到
    { "Camera",  s_camera_items,  MENU_ITEMS_OF(s_camera_items),  TUNE_AXIS_YAW,  0, GROUP_KIND_CAMERA  },
    { "Element", s_element_items, MENU_ITEMS_OF(s_element_items), TUNE_AXIS_YAW,  0, GROUP_KIND_ELEMENT },
    { "Motor",   s_motor_items,   MENU_ITEMS_OF(s_motor_items),   TUNE_AXIS_ROLL, 6, GROUP_KIND_MOTOR   },
    { "Zero",    s_zero_items,    MENU_ITEMS_OF(s_zero_items),    TUNE_AXIS_ROLL, 1, GROUP_KIND_ZERO    },
    { "Odometry",s_odometry_items,MENU_ITEMS_OF(s_odometry_items),TUNE_AXIS_PITCH,2, GROUP_KIND_ODOMETRY}
};

// save_group_action() 在栈上开 UI_MAX_GROUP_ITEMS 个名字指针，任何一页的行数超了都会越界写
typedef char menu_group_items_fit[
    (MENU_ITEMS_OF(s_roll_items)    <= UI_MAX_GROUP_ITEMS &&
     MENU_ITEMS_OF(s_pitch_items)   <= UI_MAX_GROUP_ITEMS &&
     MENU_ITEMS_OF(s_yaw_items)     <= UI_MAX_GROUP_ITEMS &&
     MENU_ITEMS_OF(s_camera_items)  <= UI_MAX_GROUP_ITEMS &&
     MENU_ITEMS_OF(s_element_items) <= UI_MAX_GROUP_ITEMS &&
     MENU_ITEMS_OF(s_motor_items)   <= UI_MAX_GROUP_ITEMS &&
     MENU_ITEMS_OF(s_zero_items)    <= UI_MAX_GROUP_ITEMS &&
     MENU_ITEMS_OF(s_odometry_items)<= UI_MAX_GROUP_ITEMS) ? 1 : -1];

static const char * const s_param_page_names[] =
{
    "Attitude",
    "Roll",
    "Pitch",
    "Yaw",
    "Camera",
    "Element",
    "Motor",
    "Zero",
    "Odometry",
    "Save",
    "Reset",
    "Back"
};

#define PARAM_PAGE_COUNT ((uint8)(sizeof(s_param_page_names) / sizeof(s_param_page_names[0])))
#define GROUP_COUNT      ((uint8)(sizeof(s_groups) / sizeof(s_groups[0])))

static menu_page_t s_page;                          // 当前页面
static uint8       s_group_index;                   // 当前参数组下标
static uint8       s_cursor;                        // 当前光标行
static uint8       s_top;                           // 当前滚动窗口的首行
static uint8       s_editing;                       // 正在编辑参数值
static uint8       s_test_on;                       // 当前页开着 Test
static uint8       s_active_test;                   // 开着的是哪一个动作行，0xFF 表示没有
static uint8       s_dirty;                         // 1=重绘本页 2=先整屏清再重绘
static uint8       s_image_ok;                      // 图像页看到的摄像头就绪状态
static uint32      s_reset_armed_until_ms;          // Reset 二次确认截止时刻，0=未进入确认态
static uint32      s_balance_armed_until_ms;        // Balance 二次确认截止时刻，0=未进入确认态
static uint8       s_balance_on;                    // 上次看到的三轴平衡运行状态，用来发现安全闸自动停机
static uint8       s_page_cursor[MENU_PAGE_COUNT];  // 各页面记住的光标位置
static uint8       s_page_top[MENU_PAGE_COUNT];     // 各页面记住的滚动位置
static uint8       s_group_cursor[GROUP_COUNT];     // 各参数组记住的光标位置
static uint8       s_group_top[GROUP_COUNT];        // 各参数组记住的滚动位置
static uint8       s_long_active[KEY_NUMBER];       // 该键正处于长按状态
static uint32      s_last_repeat_ms[KEY_NUMBER];    // 该键上次长按连发的时刻
static uint32      s_last_live_ms;                  // 上次刷新实时行的时刻
static uint32      s_last_image_draw_ms;            // 上次刷新横屏图像的时刻
static uint32      s_last_image_frame_seq;          // 上次提交显示的视觉帧序号
static uint32      s_last_param_revision;           // 上次看到的参数修订号
static uint32      s_status_until_ms;               // 状态行到期时刻
static uint8       s_image_redraw_pending;          // 图像页有新内容待刷
static uint8       s_last_image_state;              // 上次显示的 CPU1 摄像头状态
static char        s_status[30];                    // 限时状态行文本

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在指定位置绘制一段菜单文本
// 参数说明     x/y/text/color  横坐标/纵坐标/文本/前景色
// 返回参数     void
// 使用示例     ui_text(0, 0, "MENU", UI_WHITE);
//-------------------------------------------------------------------------------------------------------------------
static void ui_text(uint16 x, uint16 y, const char *text, uint16 color)
{
    ips200_set_color(color, UI_BLACK);
    ips200_show_string(x, y, text);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除菜单中的一行文本
// 参数说明     y                行起始纵坐标
// 返回参数     void
// 使用示例     ui_clear_line(UI_STATUS_Y);
//-------------------------------------------------------------------------------------------------------------------
static void ui_clear_line(uint16 y)
{
    ui_text(0, y, "                              ", UI_WHITE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清行后绘制完整菜单行
// 参数说明     y/text/color     纵坐标/文本/前景色
// 返回参数     void
// 使用示例     ui_line(21, "Params", UI_WHITE);
//-------------------------------------------------------------------------------------------------------------------
static void ui_line(uint16 y, const char *text, uint16 color)
{
    ui_clear_line(y);
    ui_text(0, y, text, color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制菜单页标题和当前位置
// 参数说明     title/current/total 标题/当前索引/项目总数
// 返回参数     void
// 使用示例     ui_header("Params", s_cursor, PARAM_PAGE_COUNT);
//-------------------------------------------------------------------------------------------------------------------
static void ui_header(const char *title, uint8 current, uint8 total)
{
    char page_text[12];

    ui_clear_line(UI_TITLE_Y);
    ui_text(UI_TITLE_X, UI_TITLE_Y, title, s_editing ? UI_RED : UI_WHITE);
    if (total > 0)
    {
        (void)snprintf(page_text, sizeof(page_text), "%u/%u",
                       (unsigned)(current + 1u), (unsigned)total);
        ui_text(UI_PAGE_X, UI_TITLE_Y, page_text, UI_GRAY);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制菜单底部按键提示
// 参数说明     void
// 返回参数     void
// 使用示例     ui_footer();
//-------------------------------------------------------------------------------------------------------------------
static void ui_footer(void)
{
    if (s_editing)
        ui_line(UI_FOOTER_Y, "UP/DN Adjust  ENT/BACK Done", UI_GRAY);
    else
        ui_line(UI_FOOTER_Y, "UP/DN Move    ENT Select", UI_GRAY);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置限时显示的菜单状态信息
// 参数说明     text             状态文本
// 返回参数     void
// 使用示例     menu_status("SAVED TO FLASH");
//-------------------------------------------------------------------------------------------------------------------
static void menu_status(const char *text)
{
    (void)snprintf(s_status, sizeof(s_status), "%s", text);
    s_status_until_ms = g_control_uptime_ms + 2000u;
    s_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按状态类型绘制菜单状态信息
// 参数说明     void
// 返回参数     void
// 使用示例     draw_status();
//-------------------------------------------------------------------------------------------------------------------
static void draw_status(void)
{
    uint16 color = UI_GREEN;

    if (s_status[0] == '\0')
    {
        ui_clear_line(UI_STATUS_Y);
        return;
    }
    if (strstr(s_status, "FAIL") != 0 ||
        strstr(s_status, "ERROR") != 0 ||
        strstr(s_status, "CONFLICT") != 0 ||
        strstr(s_status, "DIVERGED") != 0 ||
        strstr(s_status, "LOST") != 0 ||
        strstr(s_status, "PROTECT") != 0 ||
        strstr(s_status, "SAFETY") != 0)
        color = UI_RED;
    else if (strstr(s_status, "NOT") != 0 ||
             strstr(s_status, "INVALID") != 0 ||
             strstr(s_status, "BLOCKED") != 0)
        color = UI_YELLOW;
    ui_line(UI_STATUS_Y, s_status, color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取一次菜单按键事件并处理长按连发
// 参数说明     key/repeat      按键编号/是否允许长按重复
// 返回参数     uint8           1=本周期产生事件 0=无事件
// 使用示例     if (key_event(MENU_KEY_RETURN, 0)) set_page(MENU_PAGE_MAIN);
//-------------------------------------------------------------------------------------------------------------------
static uint8 key_event(key_index_enum key, uint8 repeat)
{
    key_state_enum state = key_get_state(key);
    uint8 index = (uint8)key;
    uint32 now = g_control_uptime_ms;

    if (state == KEY_SHORT_PRESS)
    {
        key_clear_state(key);
        s_long_active[index] = 0;
        return 1;
    }

    if (state == KEY_LONG_PRESS)
    {
        if (!s_long_active[index])
        {
            s_long_active[index] = 1;
            s_last_repeat_ms[index] = now;
            return 1;
        }
        if (repeat && (uint32)(now - s_last_repeat_ms[index]) >= UI_LONG_REPEAT_MS)
        {
            s_last_repeat_ms[index] = now;
            return 1;
        }
    }
    else if (state == KEY_RELEASE)
    {
        s_long_active[index] = 0;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前页面可选择的项目数量
// 参数说明     void
// 返回参数     uint8           当前页面项目数
// 使用示例     uint8 count = current_count();
//-------------------------------------------------------------------------------------------------------------------
static uint8 current_count(void)
{
    if (s_page == MENU_PAGE_MAIN)            return 4;
    if (s_page == MENU_PAGE_PARAMS)          return PARAM_PAGE_COUNT;
    if (s_page == MENU_PAGE_ATTITUDE)        return 2;
    if (s_page == MENU_PAGE_RUN_TEST)        return 2;
    if (s_page == MENU_PAGE_GROUP)
        return (uint8)(s_groups[s_group_index].item_count +
                       s_groups[s_group_index].action_count + 2u);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止当前页面的测试
// 参数说明     void
// 返回参数     void
// 使用示例     stop_local_test();
//-------------------------------------------------------------------------------------------------------------------
static void stop_local_test(void)
{
    // control_jog_stop() 会锁刹车, 平衡运行中翻页不能误触发, 所以先确认确实在点动。
    if (control_jog_running() != MOTOR_JOG_NONE) control_jog_stop();
    if (control_odometry_test_state() != ODOM_TEST_IDLE)
    {
        control_odometry_test_stop();
        s_balance_on = 0;
    }
    if (s_test_on || control_test_running())
    {
        control_test_stop();
        g_vofa_mode = VOFA_OFF;
        s_test_on = 0;
        s_active_test = 0xFFu;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断当前能不能写 Flash，不能写时顺带把原因写进状态行
// 参数说明     void
// 返回参数     uint8           1 表示被挡住，调用方直接返回
// 使用示例     if (save_blocked()) return;
//-------------------------------------------------------------------------------------------------------------------
static uint8 save_blocked(void)
{
    if (control_test_running() ||
        control_jog_running() != MOTOR_JOG_NONE ||
        start_flag != START_STOP)
    {
        menu_status("SAVE BLOCKED: RUNNING");
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Params 页最底下的 Save：把全部参数写进 Flash
// 参数说明     void
// 返回参数     void
// 使用示例     save_params_action();
//-------------------------------------------------------------------------------------------------------------------
static void save_params_action(void)
{
    if (save_blocked()) return;
    menu_status(param_save() ? "ALL SAVED TO FLASH" : "SAVE FAILED");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     子页里的 Save：只把当前这一页的参数写进 Flash，别的页保持 Flash 里已有的值
// 参数说明     void
// 返回参数     void
// 使用示例     save_group_action();
//-------------------------------------------------------------------------------------------------------------------
static void save_group_action(void)
{
    const menu_group_t *group = &s_groups[s_group_index];
    const char *names[UI_MAX_GROUP_ITEMS];
    char text[sizeof(s_status)];
    uint8 i;

    if (save_blocked()) return;

    // item_count 不会超过 UI_MAX_GROUP_ITEMS，文件末尾有编译期检查
    for (i = 0; i < group->item_count; i++)
        names[i] = group->items[i].name;

    (void)snprintf(text, sizeof(text), "%s %s",
                   group->title,
                   param_save_names(names, i) ? "PAGE SAVED" : "SAVE FAILED");
    menu_status(text);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     擦除 Flash 参数页并回默认值，第一次按只提示，UI_RESET_CONFIRM_MS 内再按一次才执行
// 参数说明     void
// 返回参数     void
// 使用示例     reset_params_action();
//-------------------------------------------------------------------------------------------------------------------
static void reset_params_action(void)
{
    uint32 now = g_control_uptime_ms;

    if (control_test_running() ||
        control_jog_running() != MOTOR_JOG_NONE ||
        start_flag != START_STOP)
    {
        menu_status("RESET BLOCKED: RUNNING");
        s_reset_armed_until_ms = 0;
        return;
    }

    // 擦除会丢掉全部标定，所以要连按两次确认
    if (s_reset_armed_until_ms == 0 || (int32)(now - s_reset_armed_until_ms) >= 0)
    {
        s_reset_armed_until_ms = now + UI_RESET_CONFIRM_MS;
        menu_status("PRESS AGAIN TO ERASE");
        return;
    }

    s_reset_armed_until_ms = 0;
    menu_status(param_erase() ? "ERASED, DEFAULTS" : "ERASE FAILED");
    param_sync_zero();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存当前页面的光标和滚动位置
// 参数说明     void
// 返回参数     void
// 使用示例     save_page_position();
//-------------------------------------------------------------------------------------------------------------------
static void save_page_position(void)
{
    if (s_page == MENU_PAGE_GROUP)
    {
        s_group_cursor[s_group_index] = s_cursor;
        s_group_top[s_group_index] = s_top;
    }
    else
    {
        s_page_cursor[s_page] = s_cursor;
        s_page_top[s_page] = s_top;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入目标页面上次保存的光标和滚动位置
// 参数说明     page            目标页面
// 返回参数     void
// 使用示例     load_page_position(MENU_PAGE_PARAMS);
//-------------------------------------------------------------------------------------------------------------------
static void load_page_position(menu_page_t page)
{
    if (page == MENU_PAGE_GROUP)
    {
        s_cursor = s_group_cursor[s_group_index];
        s_top = s_group_top[s_group_index];
    }
    else
    {
        s_cursor = s_page_cursor[page];
        s_top = s_page_top[page];
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     切换菜单页面并恢复目标页面上次光标
// 参数说明     page            目标页面
// 返回参数     void
// 使用示例     set_page(MENU_PAGE_PARAMS);
//-------------------------------------------------------------------------------------------------------------------
static void set_page(menu_page_t page)
{
    stop_local_test();
    save_page_position();
    s_page = page;
    load_page_position(page);

    // 只有 Attitude 页开启 Test 时发送姿态波形，离开页面立即关闭。
    g_vofa_mode = VOFA_OFF;

    s_editing = 0;
    s_dirty = 2;
    s_status[0] = '\0';
    s_status_until_ms = 0;
    s_last_live_ms = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     循环移动光标并维护滚动起点
// 参数说明     direction        正数向下，负数向上
// 返回参数     void
// 使用示例     move_cursor(1);
//-------------------------------------------------------------------------------------------------------------------
static void move_cursor(int direction)
{
    uint8 count = current_count();

    if (count == 0) return;
    if (direction > 0)
        s_cursor = (uint8)((s_cursor + 1u) % count);
    else
        s_cursor = (s_cursor == 0u) ? (uint8)(count - 1u) : (uint8)(s_cursor - 1u);

    if (s_cursor < s_top) s_top = s_cursor;
    if (s_cursor >= (uint8)(s_top + UI_VISIBLE_ROWS))
        s_top = (uint8)(s_cursor - UI_VISIBLE_ROWS + 1u);
    s_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制一个可执行菜单项
// 参数说明     row/absolute_index/label 显示行/项目索引/项目名称
// 返回参数     void
// 使用示例     draw_action_row(0, 0, "Params");
//-------------------------------------------------------------------------------------------------------------------
static void draw_action_row(uint8 row, uint8 absolute_index, const char *label)
{
    char line[30];
    uint16 color = (absolute_index == s_cursor) ? UI_YELLOW : UI_WHITE;

    (void)snprintf(line, sizeof(line), "%c %s",
                   (absolute_index == s_cursor) ? '>' : ' ', label);
    ui_line((uint16)(UI_LIST_Y + (uint16)row * UI_ROW_H), line, color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制一个可编辑参数及其当前值
// 参数说明     row/absolute_index/item 显示行/项目索引/参数描述
// 返回参数     void
// 使用示例     draw_param_row(0, 0, &s_roll_items[0]);
//-------------------------------------------------------------------------------------------------------------------
static void draw_param_row(uint8 row, uint8 absolute_index, const menu_param_item_t *item)
{
    char label[18];
    char value[18];
    float current = 0.0f;
    uint16 label_color = (absolute_index == s_cursor) ? UI_YELLOW : UI_WHITE;
    uint16 value_color = (s_editing && absolute_index == s_cursor) ? UI_RED : UI_GREEN;
    uint16 y = (uint16)(UI_LIST_Y + (uint16)row * UI_ROW_H);

    (void)param_get_by_name(item->name, &current);
    (void)snprintf(label, sizeof(label), "%c %s",
                   (s_editing && absolute_index == s_cursor) ? '*' :
                   ((absolute_index == s_cursor) ? '>' : ' '),
                   item->label);
    (void)snprintf(value, sizeof(value), "%.*f", (int)item->decimals, (double)current);

    ui_clear_line(y);
    ui_text(0, y, label, label_color);
    ui_text(UI_VALUE_X, y, value, value_color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     刷新主菜单上 IMU、摄像头与无刷驱动的在线状态
// 参数说明     void
// 返回参数     void
// 使用示例     render_main_live();
//-------------------------------------------------------------------------------------------------------------------
static void render_main_live(void)
{
    vision_core_state_t camera_state = vision_core_state();

    if (!g_imu_ok)                                      ui_line(120, "IMU  INIT FAILED", UI_RED);
    else if (imu_link_lost())                           ui_line(120, "IMU  LINK LOST", UI_RED);
    else if (imu_calib_state() == IMU_CALIB_MOVED)      ui_line(120, "IMU  CALIB MOVED", UI_RED);
    else if (imu_calib_state() != IMU_CALIB_OK)         ui_line(120, "IMU  CALIB WAIT", UI_YELLOW);
    else if (attitude_diverged())                       ui_line(120, "IMU  ATT DIVERGED", UI_RED);
    else if (!attitude_converged())                     ui_line(120, "IMU  CONVERGING", UI_YELLOW);
    else                                                ui_line(120, "IMU  READY", UI_GREEN);

    if (W_Motor_LinkLost())         ui_line(144, "BLDC LINK LOST", UI_RED);
    else                            ui_line(144, "BLDC UART OK", UI_GREEN);

    if (camera_state == VISION_CORE_OFF)
        ui_line(168, "CAM  NOT INITIALIZED", UI_YELLOW);
    else if (camera_state == VISION_CORE_STARTING)
        ui_line(168, "CAM  STARTING", UI_YELLOW);
    else if (camera_state == VISION_CORE_FAILED)
        ui_line(168, "CAM  INIT FAILED", UI_RED);
    else if (g_vision_frame_seq == 0u)
        ui_line(168, "CAM  WAITING FRAME", UI_YELLOW);
    else if (g_vision_age_ms > UI_CAM_FRAME_TIMEOUT_MS)
        ui_line(168, "CAM  FRAME LOST", UI_RED);
    else
        ui_line(168, "CAM  READY", UI_GREEN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制主菜单
// 参数说明     void
// 返回参数     void
// 使用示例     render_main();
//-------------------------------------------------------------------------------------------------------------------
static void render_main(void)
{
    ui_header("MENU", s_cursor, 4);
    draw_action_row(0, 0, "Params");
    draw_action_row(1, 1, "Image");
    draw_action_row(2, 2, (start_flag == START_BALANCE) ? "Balance: ON" : "Balance: OFF");
    draw_action_row(3, 3, control_remote_running() ? "Run Test: ON" : "Run Test");
    render_main_live();
    draw_status();
    ui_footer();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把最近一行下行命令的处理结果翻译成屏幕文本
// 参数说明     result          vofa 侧记录的分类结果
// 返回参数     const char*     结果说明
// 使用示例     ui_line(184, cmd_result_text(g_vofa_cmd_last), UI_YELLOW);
//-------------------------------------------------------------------------------------------------------------------
static const char *cmd_result_text(vofa_cmd_result_t result)
{
    switch (result)
    {
        case VOFA_CMD_APPLIED:     return "CMD APPLIED";
        case VOFA_CMD_NOT_RUNNING: return "CMD DROP: NOT RUNNING";
        case VOFA_CMD_RANGE:       return "CMD DROP: OUT OF RANGE";
        case VOFA_CMD_PREFIX:      return "CMD DROP: BAD PREFIX";
        case VOFA_CMD_FORMAT:      return "CMD DROP: BAD FORMAT";
        case VOFA_CMD_OVERFLOW:    return "CMD DROP: OVERFLOW";
        default:                   return "CMD NONE";
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     刷新无线 Run Test 的命令与反馈状态
// 参数说明     void
// 返回参数     void
// 使用示例     render_run_test_live();
//-------------------------------------------------------------------------------------------------------------------
static void render_run_test_live(void)
{
    char line[30];
    float steer_angle = 0.0f;
    float speed_mps = 0.0f;
    uint16 age_ms = 0;
    uint8 seen = 0;
    vofa_cmd_result_t result = g_vofa_cmd_last;

    control_remote_status(&steer_angle, &speed_mps, &age_ms, &seen);
    (void)snprintf(line, sizeof(line), "Steer %8.1f deg", (double)steer_angle);
    ui_line(72, line, UI_CYAN);
    (void)snprintf(line, sizeof(line), "Speed %8.2f m/s", (double)speed_mps);
    ui_line(96, line, UI_CYAN);
    (void)snprintf(line, sizeof(line), "C Fdb %8.2f m/s", (double)Y_Motor_GetSpeedMps());
    ui_line(120, line, UI_CYAN);
    // RX 是收到的字节，LN 是断出的整行，OK 是真正写进控制目标的条数。
    // 三个数分别对应链路、断行和接受三级，哪一级不涨就说明卡在那一级
    (void)snprintf(line, sizeof(line), "RX%6lu LN%5lu OK%5lu",
                   (unsigned long)g_vofa_rx_bytes,
                   (unsigned long)g_vofa_cmd_lines,
                   (unsigned long)g_vofa_cmd_ok);
    ui_line(144, line, UI_CYAN);
    ui_line(184, cmd_result_text(result),
            (result == VOFA_CMD_APPLIED) ? UI_GREEN
                                         : ((result == VOFA_CMD_NONE) ? UI_GRAY : UI_YELLOW));
    // 解析成功就回显，与是否被接受无关：停车状态下也能靠这一行确认链路和格式都对了
    (void)snprintf(line, sizeof(line), "Parsed %+7.1f %+6.2f",
                   (double)g_vofa_cmd_turn, (double)g_vofa_cmd_speed);
    ui_line(232, line, UI_GRAY);

    if (!control_remote_running())
        ui_line(160, "REMOTE STOPPED", UI_YELLOW);
    else if (!seen)
        ui_line(160, "REMOTE WAIT COMMAND", UI_YELLOW);
    else if (age_ms > REMOTE_CMD_TIMEOUT_MS)
        ui_line(160, "REMOTE COMMAND TIMEOUT", UI_RED);
    else
    {
        (void)snprintf(line, sizeof(line), "REMOTE OK  AGE %u ms", (unsigned)age_ms);
        ui_line(160, line, UI_GREEN);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制无线 Run Test 页面
// 参数说明     void
// 返回参数     void
// 使用示例     render_run_test();
//-------------------------------------------------------------------------------------------------------------------
static void render_run_test(void)
{
    ui_header("Run Test", s_cursor, 2);
    draw_action_row(0, 0, control_remote_running() ? "Remote: ON" : "Remote: OFF");
    draw_action_row(1, 1, "Back");
    render_run_test_live();
    ui_line(208, "speed:turn,speed", UI_WHITE);
    // 232 行由 render_run_test_live() 画最近一次解析到的数值
    ui_line(256, "Send command at least 2 Hz", UI_GRAY);
    draw_status();
    ui_footer();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制参数分类菜单
// 参数说明     void
// 返回参数     void
// 使用示例     render_params();
//-------------------------------------------------------------------------------------------------------------------
static void render_params(void)
{
    uint8 i;

    ui_header("Params", s_cursor, PARAM_PAGE_COUNT);
    for (i = 0; i < PARAM_PAGE_COUNT; i++)
        draw_action_row(i, i, s_param_page_names[i]);
    draw_status();
    ui_footer();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     刷新姿态页面的实时角度
// 参数说明     void
// 返回参数     void
// 使用示例     render_attitude_live();
//-------------------------------------------------------------------------------------------------------------------
static void render_attitude_live(void)
{
    char line[30];

    (void)snprintf(line, sizeof(line), "Roll   %9.1f deg", (double)att.roll);
    ui_line(64, line, UI_CYAN);
    (void)snprintf(line, sizeof(line), "Pitch  %9.1f deg", (double)att.pitch);
    ui_line(88, line, UI_CYAN);
    (void)snprintf(line, sizeof(line), "Yaw    %9.1f deg", (double)att.yaw);
    ui_line(112, line, UI_CYAN);

    if (!g_imu_ok)
        ui_line(152, "IMU INIT FAILED", UI_RED);
    else if (imu_link_lost())
        ui_line(152, "IMU LINK LOST", UI_RED);
    else if (attitude_diverged())
        ui_line(152, "ATTITUDE DIVERGED", UI_RED);
    else if (attitude_converged())
        ui_line(152, "ATTITUDE READY", UI_GREEN);
    else
        ui_line(152, "ATTITUDE CONVERGING", UI_YELLOW);

    // 上电标定期间车动过，零偏不可信，必须断电静置重来
    if (imu_calib_state() == IMU_CALIB_MOVED)
        ui_line(176, "CALIB MOVED / REBOOT", UI_RED);
    else
        ui_clear_line(176);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制姿态页面
// 参数说明     void
// 返回参数     void
// 使用示例     render_attitude();
//-------------------------------------------------------------------------------------------------------------------
static void render_attitude(void)
{
    ui_header("Attitude", s_cursor, 2);
    draw_action_row(0, 0, s_test_on ? "Wave: ON" : "Wave: OFF");
    draw_action_row(1, 1, "Back");
    render_attitude_live();
    ui_line(224, "WIRELESS 115200 / FIREWATER", UI_GRAY);
    ui_footer();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示当前参数组的主要反馈量
// 参数说明     void
// 返回参数     void
// 使用示例     render_group_live();
//-------------------------------------------------------------------------------------------------------------------
static void render_group_live(void)
{
    const menu_group_t *group = &s_groups[s_group_index];
    char line[30];

    // Motor 页一直显示转速用于对方向, Zero 页一直显示角度用于对零点, 与是否在跑测试无关。
    // 两行都停在 261 以上, 把 UI_STATUS_Y 留给 draw_status()。
    if (group->kind == GROUP_KIND_MOTOR)
    {
        motor_jog_t jog = control_jog_running();

        if (W_Motor_LinkLost())
            ui_line(229, "BLDC LINK LOST", UI_RED);
        else
        {
            // 指令值带符号，和下面两行反馈对着看就能判断极性对不对
            (void)snprintf(line, sizeof(line), "AIRBORNE JOG %s CMD%+6d",
                           (jog == MOTOR_JOG_A) ? "A" :
                           (jog == MOTOR_JOG_B) ? "B" :
                           (jog == MOTOR_JOG_C) ? "C" : "-",
                           (int)((jog == MOTOR_JOG_A) ? g_motor_a :
                                 (jog == MOTOR_JOG_B) ? g_motor_b :
                                 (jog == MOTOR_JOG_C) ? g_motor_c : 0));
            ui_line(229, line, UI_YELLOW);
        }

        // A/B 没有编码器，转子霍尔接在 CYT2BL3 上，主控只能拿到驱动每 10ms 回传的转速。
        // 霍尔六步在低转速下回传是阶梯值，慢转时读数跳变属正常
        (void)snprintf(line, sizeof(line), "RPM  A%+6d B%+6d",
                       (int)W_Motor_GetSpeed1(), (int)W_Motor_GetSpeed2());
        ui_line(245, line, UI_CYAN);

        // C 轮编码器 TIM2：20ms 增量和上电累计值，都已按 enc_dir_c 取过符号
        (void)snprintf(line, sizeof(line), "ENC  C%+5d TOT%+8ld",
                       (int)Y_Motor_GetSpeed20ms(),
                       (long)Y_Motor_GetTotalCount());
        ui_line(261, line, UI_CYAN);
        return;
    }

    // Zero 页一直显示当前姿态角和已保存的零点，方便对比后再抓零点
    if (group->kind == GROUP_KIND_ZERO)
    {
        (void)snprintf(line, sizeof(line), "NOW  R%7.2f P%7.2f",
                       (double)att.roll, (double)att.pitch);
        ui_line(245, line, UI_CYAN);
        (void)snprintf(line, sizeof(line), "ZERO R%7.2f P%7.2f",
                       (double)g_roll_zero, (double)g_pitch_zero);
        ui_line(261, line, UI_CYAN);
        return;
    }

    // Camera 页显示采集、处理和忙丢帧频率，便于区分相机与算法瓶颈。
    // Camera 页只有 8 个参数行，列表最多画到 y=165，下面四行都是空的
    if (group->kind == GROUP_KIND_CAMERA)
    {
        // 分段耗时(us)：ROI 复制 / 大津二值化 / 八邻域提边 / 元素处理
        (void)snprintf(line, sizeof(line), "G%4u B%4u E%5u M%4u",
                       (unsigned)g_vision_grab_us, (unsigned)g_vision_binarize_us,
                       (unsigned)g_vision_edge_us, (unsigned)g_vision_element_us);
        ui_line(213, line, UI_GRAY);
        // 总耗时与本次开机峰值。超过 16670us 就会被 VSYNC 隔帧丢弃，帧率对半掉
        (void)snprintf(line, sizeof(line), "US%6u MAX%6u",
                       (unsigned)g_vision_process_us, (unsigned)g_vision_process_max_us);
        ui_line(229, line, UI_GRAY);

        (void)snprintf(line, sizeof(line), "F%3d VS%3d DM%3d DP%3d",
                       (int)(g_vision_fps + 0.5f),
                       (int)(g_vision_vsync_fps + 0.5f),
                       (int)(g_vision_dma_fps + 0.5f),
                       (int)(g_vision_drop_fps + 0.5f));
        ui_line(245, line, UI_CYAN);
        (void)snprintf(line, sizeof(line), "V%d E%+7.1f L%2u R%2u",
                       (int)g_track_valid, (double)g_dbg_error,
                       (unsigned)g_vision_left_lost, (unsigned)g_vision_right_lost);
        ui_line(261, line, UI_CYAN);
        return;
    }

    // Element 页直接显示识别到的元素与环岛状态。
    // 两行停在 261 以上，把 UI_STATUS_Y 留给 draw_status()
    if (group->kind == GROUP_KIND_ELEMENT)
    {
        (void)snprintf(line, sizeof(line), "ELEM %s   RING %d",
                       element_name(g_vision_active_elem), (int)g_vision_island_state);
        ui_line(245, line, UI_CYAN);
        (void)snprintf(line, sizeof(line), "SCALE%5.2f  STOP %d",
                       (double)g_vision_speed_scale, (int)g_vision_stop_request);
        ui_line(261, line, UI_CYAN);
        return;
    }

    if (group->kind == GROUP_KIND_ODOMETRY)
    {
        int32 count = 0;
        float distance = 0.0f;
        float speed = 0.0f;
        odom_test_state_t state = control_odometry_test_state();

        control_odometry_status(&count, &distance, &speed);
        (void)snprintf(line, sizeof(line), "COUNT %+10ld", (long)count);
        ui_line(197, line, UI_CYAN);
        (void)snprintf(line, sizeof(line), "DIST  %+8.3f m", (double)distance);
        ui_line(213, line, UI_CYAN);
        (void)snprintf(line, sizeof(line), "SPEED %+8.3f m/s", (double)speed);
        ui_line(229, line, UI_CYAN);
        if (state == ODOM_TEST_RUNNING)
            ui_line(253, "1M TEST RUNNING", UI_YELLOW);
        else if (state == ODOM_TEST_DONE)
            ui_line(253, "1M DONE / BALANCING", UI_GREEN);
        else
            ui_line(253, "RESET, PUSH 1M, READ COUNT", UI_GRAY);
        return;
    }

    if (!s_test_on)
    {
        ui_clear_line(245);
        ui_clear_line(261);
        ui_clear_line(277);
        return;
    }

    switch (group->axis)
    {
        case TUNE_AXIS_ROLL:
            (void)snprintf(line, sizeof(line), "RATE%+7.2f ANG%+7.2f",
                           (double)att.roll_rate, (double)att.roll);
            ui_line(245, line, UI_CYAN);
            (void)snprintf(line, sizeof(line), "CMD A%+5d B%+5d",
                           (int)g_motor_a, (int)g_motor_b);
            ui_line(261, line, UI_YELLOW);
            (void)snprintf(line, sizeof(line), "RPM A%+6d B%+6d",
                           (int)W_Motor_GetSpeed1(), (int)W_Motor_GetSpeed2());
            ui_line(277, line, UI_CYAN);
            break;
        case TUNE_AXIS_PITCH:
            ui_line(245, "TEST ACTIVE / BACK STOP", UI_YELLOW);
            (void)snprintf(line, sizeof(line), "RATE %8.3f", (double)att.pitch_rate);
            ui_line(261, line, UI_CYAN);
            (void)snprintf(line, sizeof(line), "ANGLE%8.3f PWM%6d",
                           (double)att.pitch, (int)g_motor_c);
            ui_line(277, line, UI_CYAN);
            break;
        case TUNE_AXIS_YAW:
            (void)snprintf(line, sizeof(line), "RATE%+7.2f YAW%+7.2f",
                           (double)att.yaw_rate, (double)att.yaw);
            ui_line(245, line, UI_CYAN);
            (void)snprintf(line, sizeof(line), "CMD A%+5d B%+5d",
                           (int)g_motor_a, (int)g_motor_b);
            ui_line(261, line, UI_YELLOW);
            (void)snprintf(line, sizeof(line), "RPM A%+6d B%+6d",
                           (int)W_Motor_GetSpeed1(), (int)W_Motor_GetSpeed2());
            ui_line(277, line, UI_CYAN);
            break;
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取动作行的显示名称
// 参数说明     action_index    组内动作索引
// 返回参数     const char*     动作名称
// 使用示例     const char *name = action_name(0);
//-------------------------------------------------------------------------------------------------------------------
static const char *action_name(uint8 action_index)
{
    const menu_group_t *group = &s_groups[s_group_index];

    if (group->kind == GROUP_KIND_MOTOR) return s_motor_action_names[action_index];
    if (group->kind == GROUP_KIND_ZERO) return "Capture Zero";
    if (group->kind == GROUP_KIND_ODOMETRY)
    {
        if (action_index == 0u) return "Reset Counter";
        return (control_odometry_test_state() == ODOM_TEST_IDLE) ?
               "Start 1m Test" : "Stop 1m Test";
    }
    if (action_index == 0u) return "Rate";
    if (action_index == 1u) return "Angle";
    return "Speed";
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取测试启动或安全停止状态文本
// 参数说明     status           测试状态
// 返回参数     const char*      菜单提示文本
// 使用示例     menu_status(test_status_text(control_test_last_status()));
//-------------------------------------------------------------------------------------------------------------------
static const char *test_status_text(control_test_status_t status)
{
    switch (status)
    {
        case CTRL_TEST_STATUS_INVALID:        return "INVALID TEST";
        case CTRL_TEST_STATUS_IMU_FAIL:       return "IMU INIT FAILED";
        case CTRL_TEST_STATUS_IMU_CALIB:      return "IMU CALIB INVALID";
        case CTRL_TEST_STATUS_ATT_CONVERGING: return "ATTITUDE NOT READY";
        case CTRL_TEST_STATUS_ATT_DIVERGED:   return "ATTITUDE DIVERGED";
        case CTRL_TEST_STATUS_IMU_LOST:       return "IMU LINK LOST";
        case CTRL_TEST_STATUS_BLDC_LOST:      return "BLDC LINK LOST";
        case CTRL_TEST_STATUS_ROLL_PROT:      return "ROLL OVER PROTECT";
        case CTRL_TEST_STATUS_PITCH_PROT:     return "PITCH OVER PROTECT";
        case CTRL_TEST_STATUS_FLY_OVERSPEED:  return "FLYWHEEL OVERSPEED";
        case CTRL_TEST_STATUS_OUTPUT_INVALID: return "PID OUTPUT INVALID";
        case CTRL_TEST_STATUS_SAFETY:         return "SAFETY STOP";
        default:                              return "TEST BLOCKED";
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制当前轴的参数与分环测试入口
// 参数说明     void
// 返回参数     void
// 使用示例     render_group();
//-------------------------------------------------------------------------------------------------------------------
static void render_group(void)
{
    const menu_group_t *group = &s_groups[s_group_index];
    char label[28];
    uint8 row;
    uint8 index;
    uint8 action_begin = group->item_count;
    uint8 save_index   = (uint8)(action_begin + group->action_count);
    uint8 total        = (uint8)(save_index + 2u);

    ui_header(group->title, s_cursor, total);

    // 只画 s_top 开始的一屏，Element 页有十几个参数，装不下要滚动
    for (row = 0; row < UI_VISIBLE_ROWS; row++)
    {
        index = (uint8)(s_top + row);
        if (index >= total)
        {
            ui_clear_line((uint16)(UI_LIST_Y + (uint16)row * UI_ROW_H));
            continue;
        }

        if (index < action_begin)
        {
            draw_param_row(row, index, &group->items[index]);
        }
        else if (index < save_index)
        {
            uint8 action = (uint8)(index - action_begin);

            if (group->kind == GROUP_KIND_MOTOR || group->kind == GROUP_KIND_ZERO ||
                group->kind == GROUP_KIND_ODOMETRY)
                (void)snprintf(label, sizeof(label), "%s", action_name(action));
            else
                (void)snprintf(label, sizeof(label), "%s %s Test",
                               (s_test_on && s_active_test == action) ? "Stop" : "Start",
                               action_name(action));
            draw_action_row(row, index, label);
        }
        else if (index == save_index)
        {
            draw_action_row(row, index, "Save");
        }
        else
        {
            draw_action_row(row, index, "Back");
        }
    }

    render_group_live();
    if (!s_test_on || s_status[0] != '\0') draw_status();
    ui_footer();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据当前页面绘制完整菜单
// 参数说明     void
// 返回参数     void
// 使用示例     render_page();
//-------------------------------------------------------------------------------------------------------------------
static void render_page(void)
{
    if (s_dirty == 2)
        ips200_full(UI_BLACK);

    switch (s_page)
    {
        case MENU_PAGE_MAIN:            render_main(); break;
        case MENU_PAGE_PARAMS:          render_params(); break;
        case MENU_PAGE_ATTITUDE:        render_attitude(); break;
        case MENU_PAGE_GROUP:           render_group(); break;
        case MENU_PAGE_IMAGE:           break;
        case MENU_PAGE_RUN_TEST:        render_run_test(); break;
        default:                        break;
    }
    s_dirty = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一次架空点动，动作索引偶数为正转、奇数为反转
// 参数说明     action_index    组内动作索引
// 返回参数     void
// 使用示例     motor_jog_action(0);
//-------------------------------------------------------------------------------------------------------------------
static void motor_jog_action(uint8 action_index)
{
    static const motor_jog_t targets[3] = { MOTOR_JOG_A, MOTOR_JOG_B, MOTOR_JOG_C };
    motor_jog_t target;
    uint8 forward;

    if (action_index >= 6u) return;
    if (control_jog_running() != MOTOR_JOG_NONE)
    {
        control_jog_stop();
        menu_status("JOG OFF");
        return;
    }

    target = targets[action_index / 2u];
    forward = (uint8)((action_index % 2u) == 0u);
    if (control_jog_start(target, forward))
        menu_status("JOG RUNNING");
    else
    {
        menu_status(test_status_text(control_test_last_status()));
    }
    s_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把当前横滚角与俯仰角写进机械零点，车必须先按平衡姿态摆好并静止
// 参数说明     void
// 返回参数     void
// 使用示例     zero_capture_action();
//-------------------------------------------------------------------------------------------------------------------
static void zero_capture_action(void)
{
    if (!g_imu_ok || imu_link_lost() || attitude_diverged())
    {
        menu_status("IMU NOT READY");
        return;
    }
    if (imu_calib_state() != IMU_CALIB_OK)
    {
        menu_status("CALIB MOVED / REBOOT");
        return;
    }
    if (!attitude_converged())
    {
        menu_status("ATTITUDE NOT READY");
        return;
    }
    if (start_flag != START_STOP || control_test_running() ||
        control_jog_running() != MOTOR_JOG_NONE)
    {
        menu_status("STOP MOTORS FIRST");     // 电机在动时姿态不是静态零点
        return;
    }

    (void)param_set_by_name("roll_zero_init", att.roll);
    (void)param_set_by_name("pitch_zero_init", att.pitch);   // 内部会调 param_sync_zero()
    menu_status("ZERO CAPTURED");
    s_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启停当前参数组指定动作行
// 参数说明     action_index    组内动作索引
// 返回参数     void
// 使用示例     toggle_group_action(0);
//-------------------------------------------------------------------------------------------------------------------
static void toggle_group_action(uint8 action_index)
{
    const menu_group_t *group = &s_groups[s_group_index];
    tune_ring_t ring = (tune_ring_t)action_index;

    if (action_index >= group->action_count)
        return;

    if (group->kind == GROUP_KIND_MOTOR)
    {
        motor_jog_action(action_index);
        return;
    }

    if (group->kind == GROUP_KIND_ZERO)
    {
        zero_capture_action();
        return;
    }

    if (group->kind == GROUP_KIND_ODOMETRY)
    {
        if (action_index == 0u)
        {
            if (control_odometry_counter_reset())
                menu_status("COUNTER RESET / PUSH 1M");
            else
                menu_status("STOP MOTORS FIRST");
        }
        else if (control_odometry_test_state() != ODOM_TEST_IDLE)
        {
            control_odometry_test_stop();
            s_balance_on = 0;
            menu_status("1M TEST STOPPED");
        }
        else if (control_odometry_test_start())
        {
            s_balance_on = 1;
            menu_status("1M TEST RUNNING");
        }
        else
        {
            menu_status(test_status_text(control_test_last_status()));
        }
        s_dirty = 1;
        return;
    }

    if (s_test_on && s_active_test == action_index)
    {
        stop_local_test();
        menu_status("TEST OFF");
        return;
    }
    if (group->kind == GROUP_KIND_AXIS && control_test_running())
    {
        menu_status("STOP TEST FIRST");
        return;
    }

    stop_local_test();
    if (!control_test_start(group->axis, ring))
    {
        menu_status(test_status_text(control_test_last_status()));
        return;
    }

    // g_tune_axis / g_tune_ring 由 control_test_start() 记录，供菜单标识当前测试范围。
    s_test_on = 1;
    s_active_test = action_index;
    menu_status("TEST ON");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按当前步长修改选中的参数
// 参数说明     direction        正数增加，负数减小
// 返回参数     void
// 使用示例     edit_current_parameter(1.0f);
//-------------------------------------------------------------------------------------------------------------------
static void edit_current_parameter(float direction)
{
    const menu_group_t *group = &s_groups[s_group_index];
    const menu_param_item_t *item;
    float value = 0.0f;

    if (s_cursor >= group->item_count) return;
    item = &group->items[s_cursor];
    if (!param_get_by_name(item->name, &value)) return;
    (void)param_set_by_name(item->name, value + direction * item->step);
    if (strcmp(item->name, "cam_exposure") == 0)
        display_set_exposure((uint16)g_param.cam_exposure);
    s_dirty = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断测试运行时当前参数行是否属于已启用串级
// 参数说明     group/item_index 参数组与组内参数行
// 返回参数     uint8           1=允许编辑 0=会与真实测试环脱节
// 使用示例     if (!group_param_edit_allowed(group, s_cursor)) return;
//-------------------------------------------------------------------------------------------------------------------
static uint8 group_param_edit_allowed(const menu_group_t *group, uint8 item_index)
{
    uint8 item_ring;

    if (group->kind == GROUP_KIND_ODOMETRY &&
        control_odometry_test_state() != ODOM_TEST_IDLE)
        return 0;
    if (group->kind != GROUP_KIND_AXIS || !control_test_running()) return 1;
    if (group->axis != g_tune_axis) return 0;

    // Roll 在角度环之后插入 Ang Lim，不能再用 item_index / 3 推断所属环。
    // 0~2 为角速度环，3~6 为角度环，7~11 为飞轮速度回收环。
    if (group->axis == TUNE_AXIS_ROLL)
    {
        if (item_index <= 2u)      item_ring = (uint8)TUNE_RING_RATE;
        else if (item_index <= 6u) item_ring = (uint8)TUNE_RING_ANGLE;
        else                       item_ring = (uint8)TUNE_RING_VEL;
    }
    else
    {
        item_ring = (uint8)(item_index / 3u);
    }
    return (uint8)(item_ring <= (uint8)g_tune_ring);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主菜单 Balance 行：连按两次启动三轴平衡，运行中再按一次停车
// 参数说明     void
// 返回参数     void
// 使用示例     balance_action();
//-------------------------------------------------------------------------------------------------------------------
static void balance_action(void)
{
    uint32 now = g_control_uptime_ms;

    // 跑着的时候再按一次就是停，和三轴页的分环 Test、Motor 页的点动一个手感
    if (start_flag == START_BALANCE)
    {
        s_balance_armed_until_ms = 0;
        s_balance_on = 0;
        control_stop();
        menu_status("BALANCE STOPPED");
        return;
    }

    if (control_test_running() || control_jog_running() != MOTOR_JOG_NONE)
    {
        s_balance_armed_until_ms = 0;
        menu_status("BALANCE BLOCKED: RUNNING");
        return;
    }

    // 按下去 A/B 就松刹车，车会自己立起来，所以要连按两次确认
    if (s_balance_armed_until_ms == 0 || (int32)(now - s_balance_armed_until_ms) >= 0)
    {
        s_balance_armed_until_ms = now + UI_BALANCE_CONFIRM_MS;
        menu_status("PRESS AGAIN TO BALANCE");
        return;
    }

    s_balance_armed_until_ms = 0;
    if (control_balance_start())
    {
        s_balance_on = 1;
        menu_status("BALANCE RUNNING");
    }
    else
    {
        menu_status(test_status_text(control_test_last_status()));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理主菜单按键
// 参数说明     up/down/enter   上移/下移/确认事件
// 返回参数     void
// 使用示例     handle_main(up, down, enter);
//-------------------------------------------------------------------------------------------------------------------
static void handle_main(uint8 up, uint8 down, uint8 enter)
{
    if (up) move_cursor(-1);
    if (down) move_cursor(1);
    // 光标一离开 Balance 行就撤销确认态，避免停在别处按确认时误发车
    if (up || down) s_balance_armed_until_ms = 0;
    if (!enter) return;

    if (s_cursor == 0)
    {
        set_page(MENU_PAGE_PARAMS);
    }
    else if (s_cursor == 1u)
    {
        set_page(MENU_PAGE_IMAGE);
        (void)control_camera_debug_start();
        s_image_ok = (uint8)(vision_core_state() == VISION_CORE_READY);
        display_init();
        display_image_page_enter();
        s_last_image_draw_ms = 0u;
        s_last_image_frame_seq = 0u;
        s_image_redraw_pending = 1u;
        s_last_image_state = 0xFFu;
        // 图像页只看屏幕，不发送波形。
    }
    else if (s_cursor == 2u)
    {
        balance_action();
    }
    else
    {
        set_page(MENU_PAGE_RUN_TEST);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理无线 Run Test 页的启停与返回
// 参数说明     up/down/enter/back 上移/下移/确认/返回事件
// 返回参数     void
// 使用示例     handle_run_test(up, down, enter, back);
//-------------------------------------------------------------------------------------------------------------------
static void handle_run_test(uint8 up, uint8 down, uint8 enter, uint8 back)
{
    if (back)
    {
        if (control_remote_running()) control_remote_stop();
        s_balance_on = 0;
        set_page(MENU_PAGE_MAIN);
        return;
    }

    if (up) move_cursor(-1);
    if (down) move_cursor(1);
    if (!enter) return;

    if (s_cursor == 0u)
    {
        if (control_remote_running())
        {
            control_remote_stop();
            s_balance_on = 0;
            menu_status("REMOTE STOPPED");
        }
        else if (control_remote_start())
        {
            s_balance_on = 1;
            menu_status("REMOTE RUNNING");
        }
        else
        {
            menu_status(test_status_text(control_test_last_status()));
        }
        s_dirty = 1;
    }
    else
    {
        if (control_remote_running()) control_remote_stop();
        s_balance_on = 0;
        set_page(MENU_PAGE_MAIN);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理参数分类页面按键
// 参数说明     up/down/enter/back 上移/下移/确认/返回事件
// 返回参数     void
// 使用示例     handle_params(up, down, enter, back);
//-------------------------------------------------------------------------------------------------------------------
static void handle_params(uint8 up, uint8 down, uint8 enter, uint8 back)
{
    if (back)
    {
        set_page(MENU_PAGE_MAIN);
        return;
    }
    if (up) move_cursor(-1);
    if (down) move_cursor(1);
    // 光标一离开 Reset 行就撤销确认态，避免停在别处按确认时误擦
    if (up || down) s_reset_armed_until_ms = 0;
    if (!enter) return;

    if (s_cursor == 0)
    {
        set_page(MENU_PAGE_ATTITUDE);
    }
    else if (s_cursor <= GROUP_COUNT)
    {
        s_group_index = (uint8)(s_cursor - 1u);
        set_page(MENU_PAGE_GROUP);
        // 这两页靠视觉实时数据，进来就把摄像头拉起来，不用再按一次动作行
        if (s_groups[s_group_index].kind == GROUP_KIND_CAMERA ||
            s_groups[s_group_index].kind == GROUP_KIND_ELEMENT)
        {
            (void)control_camera_debug_start();
            if (vision_core_state() == VISION_CORE_FAILED)
                menu_status("CAM INIT FAILED");
        }
    }
    else if (s_cursor == (uint8)(GROUP_COUNT + 1u))
    {
        save_params_action();
    }
    else if (s_cursor == (uint8)(GROUP_COUNT + 2u))
    {
        reset_params_action();
    }
    else
    {
        set_page(MENU_PAGE_MAIN);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理姿态页面按键和波形开关
// 参数说明     up/down/enter/back 上移/下移/确认/返回事件
// 返回参数     void
// 使用示例     handle_attitude(up, down, enter, back);
//-------------------------------------------------------------------------------------------------------------------
static void handle_attitude(uint8 up, uint8 down, uint8 enter, uint8 back)
{
    if (back)
    {
        set_page(MENU_PAGE_PARAMS);
        return;
    }
    if (up) move_cursor(-1);
    if (down) move_cursor(1);
    if (!enter) return;

    if (s_cursor == 0)
    {
        if (s_test_on)
            stop_local_test();
        else
        {
            g_vofa_mode = VOFA_ATT;
            s_test_on = 1;
        }
        s_dirty = 1;
    }
    else
    {
        set_page(MENU_PAGE_PARAMS);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理控制轴参数页面按键
// 参数说明     up/down/enter/back 上移/下移/确认/返回事件
// 返回参数     void
// 使用示例     handle_group(up, down, enter, back);
//-------------------------------------------------------------------------------------------------------------------
static void handle_group(uint8 up, uint8 down, uint8 enter, uint8 back)
{
    const menu_group_t *group = &s_groups[s_group_index];
    uint8 action_begin = group->item_count;
    uint8 save_index = (uint8)(action_begin + group->action_count);

    if (s_editing)
    {
        if (back)
        {
            s_editing = 0;
            if (s_test_on)
            {
                stop_local_test();
                menu_status("TEST OFF");
            }
            s_dirty = 1;
            return;
        }
        if (up) edit_current_parameter(1.0f);
        if (down) edit_current_parameter(-1.0f);
        if (enter)
        {
            s_editing = 0;
            s_dirty = 1;
        }
        return;
    }

    if (back)
    {
        set_page(MENU_PAGE_PARAMS);
        return;
    }
    if (up) move_cursor(-1);
    if (down) move_cursor(1);
    if (!enter) return;

    if (s_cursor < group->item_count)
    {
        if (!group_param_edit_allowed(group, s_cursor))
        {
            if (group->kind == GROUP_KIND_ODOMETRY)
                menu_status("STOP 1M TEST FIRST");
            else
                menu_status("PARAM NOT IN TEST");
            return;
        }
        s_editing = 1;
        s_dirty = 1;
    }
    else if (s_cursor >= action_begin && s_cursor < save_index)
    {
        toggle_group_action((uint8)(s_cursor - action_begin));
    }
    else if (s_cursor == save_index)
    {
        save_group_action();            // 子页的 Save 只存本页参数，全量存去 Params 页的 Save
    }
    else
    {
        set_page(MENU_PAGE_PARAMS);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理横屏图像页面按键和刷新
// 参数说明     up/down/enter/back 曝光增/曝光减/切换模式/返回事件
// 返回参数     void
// 使用示例     handle_image(up, down, enter, back);
//-------------------------------------------------------------------------------------------------------------------
static void handle_image(uint8 up, uint8 down, uint8 enter, uint8 back)
{
    uint8 view_dirty;
    uint32 now = g_control_uptime_ms;
    vision_core_state_t state;

    if (back)
    {
        s_image_redraw_pending = 0u;
        display_image_page_exit();
        set_page(MENU_PAGE_MAIN);
        return;
    }

    if (up) display_exposure_step(CAM_EXP_STEP);
    if (down) display_exposure_step(-CAM_EXP_STEP);
    if (enter)
        display_next_mode();

    state = vision_core_state();
    s_image_ok = (uint8)(state == VISION_CORE_READY);
    if ((uint8)state != s_last_image_state)
    {
        s_last_image_state = (uint8)state;
        s_last_image_draw_ms = 0u;
        s_image_redraw_pending = 1u;
        if (!s_image_ok)
        {
            ips200_full(UI_BLACK);
            ui_text(5, 2, "Image", UI_WHITE);
            if (state == VISION_CORE_FAILED)
            {
                ui_line(48, "CAMERA INIT FAILED", UI_RED);
                ui_line(72, "Check MT9V03X wiring.", UI_YELLOW);
            }
            else
            {
                ui_line(48, "CAMERA STARTING", UI_YELLOW);
                ui_line(72, "Waiting for CPU1...", UI_GRAY);
            }
            ui_line(216, "BACK Return", UI_GRAY);
        }
    }

    if (!s_image_ok)
        return;

    if (g_vision_frame_seq != 0u &&
        g_vision_frame_seq != s_last_image_frame_seq)
    {
        s_last_image_frame_seq = g_vision_frame_seq;
        s_image_redraw_pending = 1u;
    }

    // 停帧时上面那个条件不再成立，页面会一直停在最后一帧，FPS 也跟着冻住。
    // 这里定期补一次重画请求；此时快照不会 ready，display_track_view() 只刷状态栏。
    if ((uint32)(now - s_last_image_draw_ms) >= UI_IMAGE_STALL_MS)
        s_image_redraw_pending = 1u;

    view_dirty = display_view_dirty();
    if (view_dirty)
        s_image_redraw_pending = 1u;

    if (s_image_redraw_pending &&
        (s_last_image_draw_ms == 0u ||
         (uint32)(now - s_last_image_draw_ms) >= UI_IMAGE_PERIOD_MS))
    {
        s_last_image_draw_ms = now;
        s_image_redraw_pending = 0u;
        display_track_view();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化菜单状态和IPS200显示
// 参数说明     void
// 返回参数     void
// 使用示例     menu_init();
//-------------------------------------------------------------------------------------------------------------------
void menu_init(void)
{
    memset(s_long_active, 0, sizeof(s_long_active));
    memset(s_last_repeat_ms, 0, sizeof(s_last_repeat_ms));
    memset(s_page_cursor, 0, sizeof(s_page_cursor));
    memset(s_page_top, 0, sizeof(s_page_top));
    memset(s_group_cursor, 0, sizeof(s_group_cursor));
    memset(s_group_top, 0, sizeof(s_group_top));
    s_page = MENU_PAGE_MAIN;
    s_group_index = 0;
    s_cursor = 0;
    s_top = 0;
    s_editing = 0;
    s_test_on = 0;
    s_active_test = 0xFFu;
    s_balance_on = 0;
    s_balance_armed_until_ms = 0;
    s_image_ok = 0;
    s_dirty = 2;
    s_last_live_ms = 0;
    s_last_image_draw_ms = 0;
    s_last_image_frame_seq = 0;
    s_last_param_revision = g_param_revision;
    s_status_until_ms = 0;
    s_image_redraw_pending = 0;
    s_last_image_state = 0xFFu;
    s_status[0] = '\0';

    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(UI_WHITE, UI_BLACK);
    ips200_full(UI_BLACK);
    render_page();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     非阻塞处理菜单按键、页面刷新和测试状态
// 参数说明     void
// 返回参数     void
// 使用示例     menu_run();
//-------------------------------------------------------------------------------------------------------------------
void menu_run(void)
{
    uint8 up = key_event(MENU_KEY_UP, 1);
    uint8 down = key_event(MENU_KEY_DOWN, 1);
    uint8 enter = 0;
    uint8 back = key_event(MENU_KEY_RETURN, 0);
    uint32 now = g_control_uptime_ms;

    enter = key_event(MENU_KEY_ENTER, 0);
    // 返回键在任何页面都是急停。先清 s_balance_on，否则下面那段自动停机检测会把
    // 上一次的停机原因当成本次的原因显示出来
    if (back && start_flag != START_STOP)
    {
        s_balance_on = 0;
        control_stop();
        menu_status("STOPPED");
    }

    if (g_param_revision != s_last_param_revision)
    {
        s_last_param_revision = g_param_revision;
        s_dirty = 1;
    }

    if (s_status[0] != '\0' &&
        (int32)(now - s_status_until_ms) >= 0)
    {
        s_status[0] = '\0';
        s_dirty = 1;
    }

    if (s_page == MENU_PAGE_GROUP &&
        s_test_on &&
        s_groups[s_group_index].kind == GROUP_KIND_AXIS &&
        !control_test_running())
    {
        s_test_on = 0;
        s_active_test = 0xFFu;
        g_vofa_mode = VOFA_OFF;
        menu_status(test_status_text(control_test_last_status()));
    }

    // 三轴平衡可能被 1ms 中断里的安全闸直接打回 START_STOP，这里发现并显示原因。
    // 不限页面：平衡是唯一跨页面存活的运行状态
    if (s_balance_on && start_flag != START_BALANCE)
    {
        s_balance_on = 0;
        menu_status(test_status_text(control_test_last_status()));
        s_dirty = 1;
    }

    switch (s_page)
    {
        case MENU_PAGE_MAIN:
            handle_main(up, down, enter);
            break;
        case MENU_PAGE_PARAMS:
            handle_params(up, down, enter, back);
            break;
        case MENU_PAGE_ATTITUDE:
            handle_attitude(up, down, enter, back);
            break;
        case MENU_PAGE_GROUP:
            handle_group(up, down, enter, back);
            break;
        case MENU_PAGE_IMAGE:
            handle_image(up, down, enter, back);
            return;
        case MENU_PAGE_RUN_TEST:
            handle_run_test(up, down, enter, back);
            break;
        default:
            set_page(MENU_PAGE_MAIN);
            break;
    }

    if ((s_page == MENU_PAGE_MAIN ||
         s_page == MENU_PAGE_ATTITUDE ||
         s_page == MENU_PAGE_GROUP ||
         s_page == MENU_PAGE_RUN_TEST) &&
        (uint32)(now - s_last_live_ms) >= UI_LIVE_PERIOD_MS)
    {
        s_last_live_ms = now;
        if (s_page == MENU_PAGE_MAIN)          render_main_live();
        else if (s_page == MENU_PAGE_ATTITUDE) render_attitude_live();
        else if (s_page == MENU_PAGE_RUN_TEST) render_run_test_live();
        else
        {
            render_group_live();
            if (s_status[0] != '\0') draw_status();
        }
    }

    if (s_dirty)
        render_page();
}
