#ifndef ELEMENT_H
#define ELEMENT_H

#include "zf_common_headfile.h"
#include "board_config.h"

typedef enum {
    ELEM_NONE = 0,
    ELEM_ZEBRA,                                // 斑马线
    ELEM_CROSS,                                // 十字
    ELEM_RING_LEFT,                            // 左环岛
    ELEM_RING_RIGHT,                           // 右环岛
    ELEM_RAMP,                                 // 坡道
    ELEM_OBSTACLE,                             // 路障
} elem_type_t;

typedef struct {
    volatile int cross;         // 十字处理中
    volatile int island;        // 环岛处理中
    volatile int ramp;          // 坡道处理中
    volatile int zebra;         // 斑马线确认状态
} order_t;

typedef struct {
    volatile int island_state;  // 0空闲 1入环 2进环 3沿环 4环内 5出环
    volatile int detect;        // 0无环岛 1左环岛 2右环岛
    volatile int state2_count;  // 状态2总里程基准
    volatile int state3_count;  // 状态3总里程基准
    volatile int state4_count;  // 状态4总里程基准
    volatile int state5_count;  // 状态5总里程基准
    float        state3_angle;  // 状态3元素角基准
    int          state_frames;  // 当前状态已停留帧数，超时用
} island_t;

typedef struct {
    volatile int state;         // 0无路障 1有路障
    volatile int direction;     // 1左避 2右避
    volatile int narrow_count;  // 收窄行计数
} obstacle_t;

typedef struct {
    float       speed_scale;    // 元素速度倍率
    elem_type_t active_elem;    // 当前元素
    uint8       stop_request;   // 停车请求
    int         ring_side_offset; // 环岛单边循迹补偿
} elem_action_t;

typedef struct
{
    int32  drive_count_total;
    float  element_yaw;
    float  pitch;
    float  pitch_rate;
    float  track_error;
    float  speed_ramp_gain;
    float  speed_ring_gain;
    float  obs_narrow_ratio;
    uint32 uptime_ms;
    int    zebra_jump_cnt;
    int    cross_lost_cnt;
    int    ring_angle;
    int    ring_s2_cnt_l;
    int    ring_s2_cnt_r;
    int    ring_side_offset;
    int    ring_timeout_cnt;    // 环岛单状态最长停留帧数，超了强制回空闲
    int    elem_guard_cnt;      // 元素退出后的屏蔽帧数
    int    obs_line_offset;
    uint8  en_zebra;            // 各元素使能，0=本帧不检测并清自己的旗标
    uint8  en_cross;
    uint8  en_ring;
    uint8  en_ramp;
    uint8  en_obstacle;
} element_motion_t;

extern order_t       g_order;
extern island_t      g_island;
extern obstacle_t    g_obstacle;
extern elem_action_t g_elem_action;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把元素编号翻译成定长 5 字符的显示名字，供 CPU0 的图像页与菜单用
// 参数说明     elem            elem_type_t 取值
// 返回参数     const char*     定长 5 字符名字，越界返回 "NONE "
// 使用示例     ips200_show_string(160, 0, element_name(frame->active_elem));
//-------------------------------------------------------------------------------------------------------------------
const char *element_name(uint8 elem);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化元素识别与控制状态
// 参数说明     void
// 返回参数     void
// 使用示例     element_init();
//-------------------------------------------------------------------------------------------------------------------
void element_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新元素状态机使用的 CPU0 里程与姿态快照
// 参数说明     motion          总里程、元素角、Pitch 与基础偏差
// 返回参数     void
// 使用示例     element_set_motion(&motion);
//-------------------------------------------------------------------------------------------------------------------
void element_set_motion(const element_motion_t *motion);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行单帧元素检测与控制量更新
// 参数说明     void
// 返回参数     void
// 使用示例     element_process();
//-------------------------------------------------------------------------------------------------------------------
void element_process(void);

#endif
