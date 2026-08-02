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
} elem_type_t;

typedef struct {
    volatile int cross;         // 十字处理中
    volatile int island;        // 环岛处理中
    volatile int zebra;         // 斑马线确认状态
    volatile int ramp;          // 坡道处理中
} order_t;

typedef struct {
    volatile int island_state;  // 0空闲 1入环 2进环 3沿环 4环内 5出环
    volatile int detect;        // 0无环岛 1左环岛 2右环岛
    volatile int state2_count;  // 状态2总里程基准
    volatile int state3_count;  // 状态3总里程基准
    volatile int state4_count;  // 状态4总里程基准
    volatile int state5_count;  // 状态5总里程基准
    float        state3_angle;  // 状态3元素角基准
    uint32       state_since_ms;// 当前状态进入时间(ms)
} island_t;

typedef struct {
    float       speed_limit_mps;  // 当前元素绝对限速(m/s)
    elem_type_t active_elem;      // 当前元素
    uint8       stop_request;     // 终点停车请求
    int         ring_side_offset; // 环岛单边循迹补偿
} elem_action_t;

typedef struct
{
    int32  drive_count_total;
    float  element_yaw;
    float  pitch;
    float  pitch_rate;
    float  drive_speed_mps;
    float  drive_output;
    float  track_error;
    uint8  track_valid;          // 上一帧循迹结果有效
    float  speed_cross_mps;
    float  speed_ring_mps;
    float  speed_ramp_mps;
    uint32 uptime_ms;
    int    ring_angle;
    int    ring_s2_cnt_l;
    int    ring_s2_cnt_r;
    int    ring_side_offset;
    uint8  en_zebra;            // 各元素使能，0=本帧不检测并清自己的旗标
    uint8  en_cross;
    uint8  en_ring;
    uint8  en_ramp;
    uint8  run_active;           // 正式 Run 状态，上升沿清理旧元素
} element_motion_t;

extern order_t       g_order;
extern island_t      g_island;
extern elem_action_t g_elem_action;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把元素编号转换为屏幕显示名称
// 参数说明     elem            元素编号，类型为 uint8
// 返回参数     const char*     定长 5 字符名称，未知编号返回 "NONE "
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
