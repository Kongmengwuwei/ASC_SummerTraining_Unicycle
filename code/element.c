#include "element.h"
#include "image.h"
#include <string.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把元素编号翻译成定长 5 字符的显示名字，供 CPU0 的图像页与菜单用
// 参数说明     elem            elem_type_t 取值
// 返回参数     const char*     定长 5 字符名字，越界返回 "NONE "
// 使用示例     ips200_show_string(160, 0, element_name(frame->active_elem));
//-------------------------------------------------------------------------------------------------------------------
const char *element_name(uint8 elem)
{
    // 定长 5 字符，短名字补空格，屏幕上不用先清行
    switch ((elem_type_t)elem)
    {
        case ELEM_ZEBRA:      return "ZEBRA";
        case ELEM_CROSS:      return "CROSS";
        case ELEM_RING_LEFT:  return "RINGL";
        case ELEM_RING_RIGHT: return "RINGR";
        case ELEM_RAMP:       return "RAMP ";
        case ELEM_OBSTACLE:   return "OBST ";
        default:              return "NONE ";
    }
}

#pragma section all "cpu1_dsram"

order_t         g_order      = {0};             // 元素调度状态
island_t        g_island     = {0};             // 环岛状态
obstacle_t      g_obstacle   = {0};             // 路障状态
elem_action_t   g_elem_action = {0};            // 元素控制量
static element_motion_t s_motion;                // CPU0 反馈快照
static int s_guard_frames;                       // 元素退出后的剩余屏蔽帧数，倒数到 0 才允许再次进入
static int s_island_last_state;                  // 上一帧的环岛状态，用来识别状态切换并重新计时

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素退出时起屏蔽期，防止刚出环岛或十字就立刻被同一处特征再次触发
// 参数说明     void
// 返回参数     void
// 使用示例     element_guard_start();
//-------------------------------------------------------------------------------------------------------------------
static void element_guard_start(void)
{
    s_guard_frames = (s_motion.elem_guard_cnt > 0) ? s_motion.elem_guard_cnt : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询是否处于元素屏蔽期
// 参数说明     void
// 返回参数     int              1=屏蔽中，本帧不许进入任何元素
// 使用示例     if (element_guard_active()) return;
//-------------------------------------------------------------------------------------------------------------------
static int element_guard_active(void)
{
    return (s_guard_frames > 0) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把环岛状态机整个打回空闲
// 参数说明     void
// 返回参数     void
// 使用示例     island_reset();
//-------------------------------------------------------------------------------------------------------------------
static void island_reset(void)
{
    g_island.island_state = 0;
    g_island.detect = 0;
    g_island.state2_count = 0;
    g_island.state3_count = 0;
    g_island.state4_count = 0;
    g_island.state5_count = 0;
    g_island.state3_angle = 0.0f;
    g_island.state_frames = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     环岛超时看门狗：单个状态停留超过 ring_timeout_cnt 帧就强制回空闲并起屏蔽期
// 参数说明     void
// 返回参数     void
// 使用示例     island_watchdog();
//-------------------------------------------------------------------------------------------------------------------
static void island_watchdog(void)
{
    if (g_island.island_state != s_island_last_state)    // 状态变了，重新计时
    {
        if (g_island.island_state == 0) element_guard_start();   // 正常出环也起屏蔽期
        s_island_last_state = g_island.island_state;
        g_island.state_frames = 0;
        return;
    }
    if (g_island.island_state == 0) return;

    g_island.state_frames++;
    if (s_motion.ring_timeout_cnt > 0 &&
        g_island.state_frames >= s_motion.ring_timeout_cnt)
    {
        // 卡在某个状态出不来，多半是判据没命中，强制退出比一直卡着安全
        island_reset();
        s_island_last_state = 0;
        element_guard_start();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算当前总里程相对状态入口的绝对增量
// 参数说明     base_count      状态入口总里程
// 返回参数     int32           绝对里程增量
// 使用示例     if (element_distance_from(base) >= threshold) { ... }
//-------------------------------------------------------------------------------------------------------------------
static int32 element_distance_from(int32 base_count)
{
    int32 distance = s_motion.drive_count_total - base_count;
    return (distance < 0) ? -distance : distance;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检测近端图像的斑马线跳变特征
// 参数说明     void
// 返回参数     int              1=检测到 0=未检测到
// 使用示例     if (black_stop()) g_order.zebra = 1;
//-------------------------------------------------------------------------------------------------------------------
static int black_stop(void)
{
    int i, j, count, best = 0;
    for (i = IMG_H - 1; i >= IMG_H - 2; i--)
    {
        count = 0;
        for (j = 30; j < IMG_W - 31; j++)
            if (my_image.image_two_value[i][j] != my_image.image_two_value[i][j + 1])
                count++;
        if (count > best) best = count;
    }
    return (best >= s_motion.zebra_jump_cnt) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新斑马线三段确认状态
// 参数说明     void
// 返回参数     void
// 使用示例     zebra();
//-------------------------------------------------------------------------------------------------------------------
static void zebra(void)
{
    if (black_stop() && g_order.zebra == 0)       g_order.zebra = 1;
    if (g_order.zebra == 1 && black_stop() == 0)  g_order.zebra = 2;
    if (g_order.zebra == 2 && black_stop())       g_order.zebra = 3;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据双边丢线与上角点检测十字
// 参数说明     void
// 返回参数     void
// 使用示例     Cross_Detect();
//-------------------------------------------------------------------------------------------------------------------
static void Cross_Detect(void)
{
    my_image.Left_Up_Find = 0;
    my_image.Right_Up_Find = 0;
    Find_Up_Point(IMG_H - 1, 0);

    if (g_order.island == 0)
    {
        if (my_image.Left_Lost_Counter >= s_motion.cross_lost_cnt &&
            my_image.Right_Lost_Counter >= s_motion.cross_lost_cnt)
        {
            if (my_image.Left_Up_Find != 0 || my_image.Right_Up_Find != 0)
                g_order.cross = 1;              // 双边丢线且存在角点
            else
                g_order.cross = 0;
        }
        else
            g_order.cross = 0;
    }
    else
        g_order.cross = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据赛宽与中线偏差检测坡道
// 参数说明     void
// 返回参数     void
// 使用示例     Ramp_Detect();
//-------------------------------------------------------------------------------------------------------------------
static void Ramp_Detect(void)
{
    int i, count = 0;

    if (my_image.Search_Stop_Line >= RAMP_SEARCH_LINE)
    {
        for (i = IMG_H - 1; i > IMG_H - my_image.Search_Stop_Line; i--)
            if (my_image.Road_Wide[i] - Standard_Road_Wide[i] > RAMP_WIDE_OVER)
                count++;                        // 统计超宽行
    }

    if (count >= RAMP_WIDE_ROWS &&
        func_abs((int)s_motion.track_error) <= RAMP_ERR_LIMIT &&
        g_order.cross == 0 &&
        g_island.detect == 0 && g_island.island_state == 0 &&
        my_image.Right_Lost_Counter <= 15 && my_image.Left_Lost_Counter <= 15)
        g_order.ramp = 1;
    else
        g_order.ramp = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     搜索右边线不连续位置
// 参数说明     start/end        搜索起始/终止行
// 返回参数     void
// 使用示例     Continuity_Change_Right(30, IMG_H-1-10);
//-------------------------------------------------------------------------------------------------------------------
static void Continuity_Change_Right(int start, int end)
{
    int i, t;
    if (my_image.Right_Lost_Counter >= (int)(0.9f * IMG_H) ||
        my_image.Search_Stop_Line <= 5)
    {
        my_image.continuity_change_flag_right = 0;
        return;
    }
    if (start >= IMG_H - 5) start = IMG_H - 5;
    if (end <= 5) end = 5;
    if (start < end) { t = start; start = end; end = t; }

    for (i = start; i >= end; i--)
    {
        if (func_abs(my_image.Right_Line[i] - my_image.Right_Line[i - 5]) >= RING_CONTINUITY)
        { my_image.continuity_change_flag_right = i; break; }
        else
            my_image.continuity_change_flag_right = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     搜索左边线不连续位置
// 参数说明     start/end        搜索起始/终止行
// 返回参数     void
// 使用示例     Continuity_Change_Left(30, IMG_H-1-10);
//-------------------------------------------------------------------------------------------------------------------
static void Continuity_Change_Left(int start, int end)
{
    int i, t;
    if (my_image.Both_Lost_Counter >= (int)(0.9f * IMG_H) ||
        my_image.Search_Stop_Line <= 5)
    {
        my_image.continuity_change_flag_left = 0;
        return;
    }
    if (start >= IMG_H - 1 - 5) start = IMG_H - 1 - 5;
    if (end <= 5) end = 5;
    if (start < end) { t = start; start = end; end = t; }

    for (i = start; i >= end; i--)
    {
        if (func_abs(my_image.Left_Line[i] - my_image.Left_Line[i - 2]) >= RING_CONTINUITY)
        { my_image.continuity_change_flag_left = i; break; }
        else
            my_image.continuity_change_flag_left = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据边线连续性与丢线状态检测环岛方向
// 参数说明     void
// 返回参数     void
// 使用示例     island_detect();
//-------------------------------------------------------------------------------------------------------------------
static void island_detect(void)
{
    if (g_island.detect == 0)
    {
        my_image.continuity_change_flag_left = 0;
        my_image.continuity_change_flag_right = 0;
        Continuity_Change_Left(30, IMG_H - 1 - 10);
        Continuity_Change_Right(30, IMG_H - 1 - 10);
        if (my_image.continuity_change_flag_right >= 20 &&
            my_image.continuity_change_flag_left  <= RING_OPP_LOST &&
            my_image.Right_Lost_Counter >= RING_LOST_MIN &&
            my_image.Right_Lost_Counter <= RING_LOST_MAX &&
            my_image.Left_Lost_Counter  <= RING_OPP_LOST &&
            my_image.Search_Stop_Line   >= RING_VIEW &&
            my_image.Both_Lost_Counter  <= RING_OPP_LOST)
            g_island.detect = 2;                // 右环岛
        else if (my_image.continuity_change_flag_left >= 20 &&
                 my_image.continuity_change_flag_right <= RING_OPP_LOST &&
                 my_image.Left_Lost_Counter  >= RING_LOST_MIN &&
                 my_image.Left_Lost_Counter  <= RING_LOST_MAX &&
                 my_image.Right_Lost_Counter <= RING_OPP_LOST &&
                 my_image.Search_Stop_Line   >= RING_VIEW &&
                 my_image.Both_Lost_Counter  <= RING_OPP_LOST)
            g_island.detect = 1;                // 左环岛
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新左环岛六状态流程
// 参数说明     void
// 返回参数     void
// 使用示例     island_detect_left();
//-------------------------------------------------------------------------------------------------------------------
static void island_detect_left(void)
{
    my_image.continuity_change_flag_left = 0;
    my_image.continuity_change_flag_right = 0;
    Continuity_Change_Right(30, IMG_H - 1 - 10);
    Continuity_Change_Left(30, IMG_H - 1 - 10);
    if (g_order.cross == 0 && g_order.ramp == 0)
    {
        // 状态0: 确认入环特征
        if (g_island.island_state == 0 && func_abs((int)s_motion.track_error) <= 20)
        {
            if (my_image.continuity_change_flag_left >= 20 &&
                my_image.continuity_change_flag_right <= RING_OPP_LOST &&
                my_image.Left_Lost_Counter  >= RING_LOST_MIN &&
                my_image.Left_Lost_Counter  <= RING_LOST_MAX &&
                my_image.Right_Lost_Counter <= RING_OPP_LOST &&
                my_image.Search_Stop_Line   >= RING_VIEW &&
                my_image.Both_Lost_Counter  <= RING_OPP_LOST)
            { g_island.island_state = 1; }
        }
        if (g_island.island_state == 1)         // 状态1: 等待左边界起点上移
        {
            if (my_image.Boundry_Start_Left > 0 && my_image.Boundry_Start_Left < 50)
            { g_island.island_state = 2; g_island.state2_count = s_motion.drive_count_total; }
        }
        if (g_island.island_state == 2)         // 状态2: 按编码器累计进环
        {
            if (element_distance_from(g_island.state2_count) >= s_motion.ring_s2_cnt_l)
            {
                g_island.island_state = 3;
                g_island.state3_count = s_motion.drive_count_total;
                g_island.state3_angle = s_motion.element_yaw;
            }
        }
        if (g_island.island_state == 3)         // 状态3: 按元素转角沿环
        {
            if ((s_motion.element_yaw - g_island.state3_angle) >= (float)s_motion.ring_angle)
            { g_island.island_state = 4; g_island.state4_count = s_motion.drive_count_total; }
        }
        if (g_island.island_state == 4)         // 状态4: 按编码器累计出环
        {
            if (element_distance_from(g_island.state4_count) >= RING_S4_CNT)
            { g_island.island_state = 5; g_island.state5_count = s_motion.drive_count_total; }
        }
        if (g_island.island_state == 5)         // 状态5: 完成出环
        {
            if (element_distance_from(g_island.state5_count) >= RING_S5_CNT)
            { g_island.island_state = 0; g_island.state5_count = 0; g_island.detect = 0; }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新右环岛六状态流程
// 参数说明     void
// 返回参数     void
// 使用示例     island_detect_right();
//-------------------------------------------------------------------------------------------------------------------
static void island_detect_right(void)
{
    my_image.continuity_change_flag_left = 0;
    my_image.continuity_change_flag_right = 0;
    Continuity_Change_Left(30, IMG_H - 1 - 10);
    Continuity_Change_Right(30, IMG_H - 1 - 10);
    if (g_order.cross == 0 && g_order.ramp == 0)
    {
        if (g_island.island_state == 0 && func_abs((int)s_motion.track_error) <= 20)
        {
            if (my_image.continuity_change_flag_right >= 20 &&
                my_image.continuity_change_flag_left  <= RING_OPP_LOST &&
                my_image.Right_Lost_Counter >= RING_LOST_MIN &&
                my_image.Right_Lost_Counter <= RING_LOST_MAX &&
                my_image.Left_Lost_Counter  <= RING_OPP_LOST &&
                my_image.Search_Stop_Line   >= RING_VIEW &&
                my_image.Both_Lost_Counter  <= RING_OPP_LOST)
            { g_island.island_state = 1; }
        }
        if (g_island.island_state == 1)         // 状态1: 等待右边界起点上移
        {
            if (my_image.Boundry_Start_Right > 0 && my_image.Boundry_Start_Right < 50)
            { g_island.island_state = 2; g_island.state2_count = s_motion.drive_count_total; }
        }
        if (g_island.island_state == 2)
        {
            if (element_distance_from(g_island.state2_count) >= s_motion.ring_s2_cnt_r)
            {
                g_island.island_state = 3;
                g_island.state3_count = s_motion.drive_count_total;
                g_island.state3_angle = s_motion.element_yaw;
            }
        }
        if (g_island.island_state == 3)         // 状态3: 按元素转角沿环
        {
            if ((s_motion.element_yaw - g_island.state3_angle) <= -(float)s_motion.ring_angle)
            { g_island.island_state = 4; g_island.state4_count = s_motion.drive_count_total; }
        }
        if (g_island.island_state == 4)
        {
            if (element_distance_from(g_island.state4_count) >= RING_S4_CNT)
            { g_island.island_state = 5; g_island.state5_count = s_motion.drive_count_total; }
        }
        if (g_island.island_state == 5)
        {
            if (element_distance_from(g_island.state5_count) >= RING_S5_CNT)
            { g_island.island_state = 0; g_island.state5_count = 0; g_island.detect = 0; }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏移边线以生成避障中线
// 参数说明     void
// 返回参数     void
// 使用示例     obstacle_avoid_process();
//-------------------------------------------------------------------------------------------------------------------
static void obstacle_avoid_process(void)
{
    int i;

    for (i = IMG_H - 1; i >= IMG_H - my_image.Search_Stop_Line; i--)
    {
        if (g_obstacle.direction == 1)          // 左侧避障
            my_image.Left_Line[i] =
                (int)func_limit_ab(my_image.Left_Line[i] + s_motion.obs_line_offset, 0, IMG_W - 1);
        else if (g_obstacle.direction == 2)     // 右侧避障
            my_image.Right_Line[i] =
                (int)func_limit_ab(my_image.Right_Line[i] - s_motion.obs_line_offset, 0, IMG_W - 1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据赛宽收窄与边线连续性检测路障
// 参数说明     void
// 返回参数     void
// 使用示例     obstacle_detect();
//-------------------------------------------------------------------------------------------------------------------
static void obstacle_detect(void)
{
    int i;
    int top;
    int start;
    int end;

    g_obstacle.narrow_count = 0;
    top = IMG_H - (int)func_limit_ab(my_image.Search_Stop_Line, 0, IMG_H);
    start = (OBS_ROW_MIN > top) ? OBS_ROW_MIN : top;
    end = (OBS_ROW_MAX < IMG_H) ? OBS_ROW_MAX : (IMG_H - 1);

    // 仅在普通赛道状态进入路障
    if (g_obstacle.state == 0 &&
        g_island.island_state == 0 && g_order.cross == 0 && g_order.ramp == 0)
    {
        g_obstacle.direction = 0;
        if (my_image.continuity_change_flag_right)      g_obstacle.direction = 2;
        else if (my_image.continuity_change_flag_left)  g_obstacle.direction = 1;

        for (i = end; g_obstacle.direction != 0 && i >= start; i--)
        {
            if (!my_image.Left_Lost_Flag[i] && !my_image.Right_Lost_Flag[i] &&
                (float)my_image.Road_Wide[i] <=
                (float)Standard_Road_Wide[i] * s_motion.obs_narrow_ratio)
                g_obstacle.narrow_count++;
        }
        if (g_obstacle.narrow_count >= OBS_NARROW_CNT) g_obstacle.state = 1;
    }
    // 赛宽恢复后清除路障状态
    if (g_obstacle.state == 1 &&
        OBS_ROW_MIN >= top &&
        !my_image.Left_Lost_Flag[OBS_ROW_MIN] &&
        !my_image.Right_Lost_Flag[OBS_ROW_MIN] &&
        my_image.Road_Wide[OBS_ROW_MIN] >= Standard_Road_Wide[OBS_ROW_MIN] * OBS_RECOVER_RATIO)
    {
        g_obstacle.state = 0;
        g_obstacle.direction = 0;
    }

    if (g_obstacle.state == 1) obstacle_avoid_process();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化元素识别与控制状态
// 参数说明     void
// 返回参数     void
// 使用示例     element_init();
//-------------------------------------------------------------------------------------------------------------------
void element_init(void)
{
    memset((void *)&g_order,    0, sizeof(g_order));
    memset((void *)&g_island,   0, sizeof(g_island));
    memset((void *)&g_obstacle, 0, sizeof(g_obstacle));
    memset((void *)&s_motion,   0, sizeof(s_motion));
    s_guard_frames = 0;
    s_island_last_state = 0;
    g_elem_action.speed_scale = 1.0f;
    g_elem_action.active_elem = ELEM_NONE;
    g_elem_action.stop_request = 0;
    g_elem_action.ring_side_offset = RING_SIDE_OFFSET_DEFAULT;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新元素状态机使用的 CPU0 里程与姿态快照
// 参数说明     motion          总里程、元素角、Pitch 与基础偏差
// 返回参数     void
// 使用示例     element_set_motion(&motion);
//-------------------------------------------------------------------------------------------------------------------
void element_set_motion(const element_motion_t *motion)
{
    if (motion != 0)
        s_motion = *motion;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行单帧元素检测与控制量更新
// 参数说明     void
// 返回参数     void
// 使用示例     element_process();
//-------------------------------------------------------------------------------------------------------------------
void element_process(void)
{
    g_elem_action.speed_scale  = 1.0f;
    g_elem_action.active_elem  = ELEM_NONE;
    g_elem_action.stop_request = 0;
    g_elem_action.ring_side_offset = s_motion.ring_side_offset;

    if (s_guard_frames > 0) s_guard_frames--;               // 元素屏蔽期倒数

    // 每个元素由自己的使能位控制。关掉的元素不检测，并把自己的旗标清零，
    // 否则关的那一刻旗标会停在最后一次的值上，一直影响后面的互斥判据和补线。

    // 斑马线：终点线判据，不受屏蔽期限制
    if (s_motion.en_zebra)
    {
        zebra();
        if (g_order.zebra == 3)
        {
            g_elem_action.active_elem = ELEM_ZEBRA;
            g_elem_action.stop_request = 1;
        }
    }
    else
    {
        g_order.zebra = 0;
    }

    // 十字
    if (s_motion.en_cross && !element_guard_active())
    {
        Cross_Detect();
    }
    else
    {
        g_order.cross = 0;
        my_image.Left_Up_Find = 0;
        my_image.Right_Up_Find = 0;
    }

    // 环岛。屏蔽期只挡新进入，已经在环里的流程要跑完
    if (s_motion.en_ring)
    {
        if (!element_guard_active() || g_island.island_state != 0)
        {
            island_detect();
            if (g_island.detect == 1)      island_detect_left();
            else if (g_island.detect == 2) island_detect_right();
        }
        island_watchdog();
    }
    else
    {
        island_reset();
        s_island_last_state = 0;
    }
    g_order.island = (g_island.island_state != 0) ? 1 : 0;   // 环岛互斥标志

    // 坡道
    if (s_motion.en_ramp && !element_guard_active())
        Ramp_Detect();
    else
        g_order.ramp = 0;

    // 路障
    if (s_motion.en_obstacle && !element_guard_active())
    {
        obstacle_detect();
    }
    else
    {
        g_obstacle.state = 0;
        g_obstacle.direction = 0;
        g_obstacle.narrow_count = 0;
    }

    // 仅对检测到角点的一侧执行十字补线
    if (g_order.cross == 1)
    {
        if (my_image.Left_Up_Find  > 1) Lengthen_Left_Boundry (my_image.Left_Up_Find  - 1, IMG_H - 10);
        if (my_image.Right_Up_Find > 1) Lengthen_Right_Boundry(my_image.Right_Up_Find - 1, IMG_H - 10);
        g_elem_action.active_elem = ELEM_CROSS;
    }

    // 更新元素速度与类型
    if (g_order.ramp == 1)
    {
        g_elem_action.speed_scale = s_motion.speed_ramp_gain;
        g_elem_action.active_elem = ELEM_RAMP;
    }
    else if (g_island.island_state != 0)
    {
        g_elem_action.speed_scale = s_motion.speed_ring_gain;
        g_elem_action.active_elem = (g_island.detect == 1) ? ELEM_RING_LEFT : ELEM_RING_RIGHT;
    }
    else if (g_obstacle.state == 1)
    {
        g_elem_action.active_elem = ELEM_OBSTACLE;
    }
}

#pragma section all restore
