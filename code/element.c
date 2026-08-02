#include "element.h"
#include "image.h"
#include <math.h>
#include <string.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把元素编号转换为屏幕显示名称
// 参数说明     elem            元素编号，类型为 uint8
// 返回参数     const char*     定长 5 字符名称，未知编号返回 "NONE "
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
        default:              return "NONE ";
    }
}

#pragma section all "cpu1_dsram"

order_t         g_order      = {0};             // 元素调度状态
island_t        g_island     = {0};             // 环岛状态
elem_action_t   g_elem_action = {0};            // 元素控制量
static element_motion_t s_motion;                // CPU0 反馈快照
static uint32 s_guard_start_ms;                  // 元素屏蔽起点
static uint32 s_guard_duration_ms;               // 元素屏蔽时长
static int s_island_last_state;                  // 上一帧的环岛状态，用来识别状态切换并重新计时
static uint8 s_zebra_confirm;                    // 斑马线连续确认帧数
static uint8 s_zebra_release;                    // 斑马线离开确认帧数
static uint8 s_cross_confirm;                    // 十字连续确认帧数
static uint8 s_cross_release;                    // 十字双边恢复帧数
static uint8 s_cross_candidate_now;              // 当前帧双侧十字候选
static uint32 s_cross_since_ms;                  // 十字进入时间
static uint8 s_ring_candidate;                   // 环岛候选方向
static uint8 s_ring_confirm;                     // 环岛连续确认帧数
static uint8 s_ring_recover;                     // 环岛出口双边恢复帧数
static uint8 s_ramp_confirm;                     // 坡道连续确认帧数
static uint8 s_ramp_drive_frames;                // 坡道候选期间大驱动输出帧数
static int s_ramp_direction;                     // 坡道候选俯仰方向，-1/0/+1
static int32 s_ramp_probe_count;                 // 坡道候选开始时的总里程
static int32 s_ramp_count;                       // 进坡时的总里程，出坡按里程判
static uint32 s_ramp_since_ms;                   // 坡道进入时间
static elem_type_t s_owner;                      // 当前唯一元素所有者
static uint8 s_run_last;                         // 上一帧正式 Run 状态

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把元素参数的 10ms 计数换算成毫秒
// 参数说明     count           10ms 计数
// 返回参数     uint32          毫秒数
// 使用示例     timeout_ms = element_count_to_ms(RING_TIMEOUT_CNT);
//-------------------------------------------------------------------------------------------------------------------
static uint32 element_count_to_ms(int count)
{
    if (count <= 0) return 0;
    if (count > 0x0FFFFFFF) return 0xFFFFFFFFu;
    return (uint32)count * 10u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素退出时起屏蔽期，防止刚出环岛或十字就立刻被同一处特征再次触发
// 参数说明     void
// 返回参数     void
// 使用示例     element_guard_start();
//-------------------------------------------------------------------------------------------------------------------
static void element_guard_start(void)
{
    s_guard_start_ms = s_motion.uptime_ms;
    s_guard_duration_ms = element_count_to_ms(ELEM_GUARD_CNT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询是否处于元素屏蔽期
// 参数说明     void
// 返回参数     int              1=屏蔽中，本帧不许进入任何元素
// 使用示例     if (element_guard_active()) return;
//-------------------------------------------------------------------------------------------------------------------
static int element_guard_active(void)
{
    if (s_guard_duration_ms == 0u) return 0;
    return ((uint32)(s_motion.uptime_ms - s_guard_start_ms) < s_guard_duration_ms) ? 1 : 0;
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
    g_island.state_since_ms = 0;
    s_ring_candidate = 0;
    s_ring_confirm = 0;
    s_ring_recover = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     环岛状态超时看门狗
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
        g_island.state_since_ms = s_motion.uptime_ms;
        return;
    }
    if (g_island.island_state == 0) return;

    if (RING_TIMEOUT_CNT > 0 &&
        (uint32)(s_motion.uptime_ms - g_island.state_since_ms) >=
        element_count_to_ms(RING_TIMEOUT_CNT))
    {
        island_reset();
        s_island_last_state = 0;
        element_guard_start();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算当前总里程相对状态入口的绝对增量
// 参数说明     base_count      状态入口总里程
// 返回参数     uint32          绝对里程增量
// 使用示例     if (element_distance_from(base) >= threshold) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint32 element_distance_from(int32 base_count)
{
    int64 distance = (int64)s_motion.drive_count_total - (int64)base_count;

    if (distance < 0) distance = -distance;
    if (distance > 0xFFFFFFFFLL) return 0xFFFFFFFFu;
    return (uint32)distance;
}

//-------------------------------------------------------------------------------------------------------------------
// 重置坡道入口的连续姿态与驱动佐证。
//-------------------------------------------------------------------------------------------------------------------
static void ramp_probe_reset(void)
{
    s_ramp_confirm = 0;
    s_ramp_drive_frames = 0;
    s_ramp_direction = 0;
    s_ramp_probe_count = s_motion.drive_count_total;
}

//-------------------------------------------------------------------------------------------------------------------
// 俯仰和俯仰角速度同时显著时必须同向；返回 -1/0/+1。
//-------------------------------------------------------------------------------------------------------------------
static int ramp_motion_direction(void)
{
    int pitch_dir = 0;
    int rate_dir = 0;

    if (fabsf(s_motion.pitch) >= RAMP_PITCH_MIN)
        pitch_dir = (s_motion.pitch > 0.0f) ? 1 : -1;
    if (fabsf(s_motion.pitch_rate) >= RAMP_PITCH_RATE_MIN)
        rate_dir = (s_motion.pitch_rate > 0.0f) ? 1 : -1;
    if (pitch_dir != 0 && rate_dir != 0 && pitch_dir != rate_dir) return 0;
    return (pitch_dir != 0) ? pitch_dir : rate_dir;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检测图像中部三行的斑马线跳变特征
// 参数说明     void
// 返回参数     int              1=检测到 0=未检测到
// 使用示例     if (black_stop()) g_order.zebra = 1;
//-------------------------------------------------------------------------------------------------------------------
static int black_stop(void)
{
    int i;
    int hit_rows = 0;
    int jump_limit = ZEBRA_JUMP_CNT;
    int top = IMG_H - my_image.Search_Stop_Line;
    int bottom = my_image.Edge_Row_Bottom;

    if (top < 0) top = 0;
    if (bottom < 0 || bottom < top) return 0;
    if (bottom > IMG_H - 1) bottom = IMG_H - 1;
    if (bottom - top > ZEBRA_SCAN_ROWS) top = bottom - ZEBRA_SCAN_ROWS;
    for (i = bottom; i >= top; i--)
    {
        int left, right, col;
        int transitions = 0;
        int black_runs = 0;
        int first_black = -1;
        int last_black = -1;
        uint8 last;

        if (!image_get_track_envelope(i, &left, &right)) continue;
        left  += ZEBRA_EDGE_MARGIN;
        right -= ZEBRA_EDGE_MARGIN;
        if (right - left < ZEBRA_MIN_SPAN) continue;

        last = my_image.image_two_value[i][left];
        col = left;
        while (col <= right)
        {
            int start = col;
            uint8 color = my_image.image_two_value[i][col];
            int width;

            while (col <= right && my_image.image_two_value[i][col] == color) col++;
            width = col - start;
            if (start > left && color != last) transitions++;
            last = color;

            if (color == IMG_BLACK && width >= ZEBRA_RUN_MIN && width <= ZEBRA_RUN_MAX)
            {
                black_runs++;
                if (first_black < 0) first_black = start;
                last_black = col - 1;
            }
        }

        if (transitions >= jump_limit && black_runs >= ZEBRA_BLACK_RUNS &&
            first_black >= 0 &&
            (last_black - first_black) * 100 >= (right - left) * ZEBRA_COVER_PERCENT)
            hit_rows++;
    }
    return (hit_rows >= ZEBRA_HIT_ROWS) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     连续确认并锁存斑马线停车状态
// 参数说明     void
// 返回参数     void
// 使用示例     zebra();
//-------------------------------------------------------------------------------------------------------------------
static void zebra(void)
{
    if (black_stop())
    {
        if (s_zebra_confirm < ZEBRA_CONFIRM_FRAMES) s_zebra_confirm++;
        s_zebra_release = 0;
    }
    else
    {
        s_zebra_confirm = 0;
        if (g_order.zebra == 3 && s_zebra_release < ZEBRA_RELEASE_FRAMES)
            s_zebra_release++;
    }

    if (s_zebra_confirm >= ZEBRA_CONFIRM_FRAMES)
        g_order.zebra = 3;
    else if (g_order.zebra == 3 && s_zebra_release >= ZEBRA_RELEASE_FRAMES)
    {
        g_order.zebra = 0;
        s_zebra_release = 0;
        element_guard_start();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用四个角点检测十字并延伸缺失边线
// 参数说明     void
// 返回参数     void
// 使用示例     Cross_Detect();
//-------------------------------------------------------------------------------------------------------------------
static uint8 cross_side_evidence(const int *line, const int *lost,
                                 int corner, int sign)
{
    int row;
    int top = IMG_H - my_image.Search_Stop_Line;
    int near_sum = 0;
    int near_count = 0;
    int lost_run = 0;
    int far_line_run = 0;
    int gap_seen = 0;

    if (corner <= 0) return 0;
    if (top < 0) top = 0;
    for (row = corner + 1; row <= corner + 5 && row < IMG_H; row++)
        if (!lost[row]) { near_sum += line[row]; near_count++; }
    if (near_count == 0)
    {
        if (lost[corner]) return 0;
        near_sum = line[corner];
        near_count = 1;
    }

    for (row = corner - 1;
         row >= top && row >= corner - CROSS_EVIDENCE_ROWS; row--)
    {
        if (lost[row])
        {
            far_line_run = 0;
            if (++lost_run >= CROSS_FAR_LOST_ROWS) gap_seen = 1;
            continue;
        }

        if (sign * (near_sum / near_count - line[row]) >= CROSS_OUTWARD_MIN)
            return 1;
        lost_run = 0;
        if (gap_seen && ++far_line_run >= CROSS_FAR_LINE_ROWS) return 1;
    }
    return (uint8)gap_seen;
}

static void Cross_Detect(void)
{
    int corner[4];                              // [左下, 左上, 右下, 右上]
    int l_down, l_up, r_down, r_up;
    int i, top;
    int wide_rows = 0;
    int candidate;
    int paired_corner;
    int both_lost;
    uint8 left_evidence;
    uint8 right_evidence;
    int release_limit;
    int lost_limit = CROSS_LOST_CNT;

    Image_Find_Corners(corner);
    l_down = corner[0]; l_up = corner[1];
    r_down = corner[2]; r_up = corner[3];

    if (l_down > 0 && l_down < l_up) l_down = 0;
    if (r_down > 0 && r_down < r_up) r_down = 0;

    top = IMG_H - my_image.Search_Stop_Line;
    if (top < 0) top = 0;

    if (image_road_wide_ready())
        for (i = (my_image.Edge_Row_Bottom >= 0) ? my_image.Edge_Row_Bottom : IMG_H - 15;
             i >= top; i--)
        {
            if (my_image.Road_Perp_Wide[i] > 0 &&
                my_image.Road_Perp_Wide[i] - 2 * Road_Half_Wide[i] >= CROSS_WIDE_OVER)
                wide_rows++;
        }

    paired_corner = (l_up > 0 && r_up > 0 && func_abs(l_up - r_up) <= 24);
    both_lost = (my_image.Left_Lost_Counter >= lost_limit &&
                 my_image.Right_Lost_Counter >= lost_limit);
    left_evidence = cross_side_evidence(my_image.Left_Line,
                                        my_image.Left_Lost_Flag, l_up, 1);
    right_evidence = cross_side_evidence(my_image.Right_Line,
                                         my_image.Right_Lost_Flag, r_up, -1);
    candidate = (g_order.island == 0 &&
                 ((paired_corner &&
                   (both_lost || wide_rows >= CROSS_WIDE_ROWS ||
                    left_evidence || right_evidence)) ||
                  ((l_up > 0 || r_up > 0) && both_lost &&
                   (wide_rows >= CROSS_WIDE_ROWS ||
                    left_evidence || right_evidence))));
    s_cross_candidate_now = (uint8)candidate;

    if (g_order.cross == 0)
    {
        if (candidate)
        {
            if (s_cross_confirm < CROSS_CONFIRM_FRAMES) s_cross_confirm++;
        }
        else
        {
            s_cross_confirm = 0;
        }

        if (s_cross_confirm >= CROSS_CONFIRM_FRAMES)
        {
            g_order.cross = 1;
            s_cross_release = 0;
            s_cross_since_ms = s_motion.uptime_ms;
        }
    }
    else
    {
        // 双边恢复后退出；超时看门狗处理误触发或持续丢线。
        release_limit = lost_limit / 2;
        if (release_limit < 2) release_limit = 2;
        if (my_image.Left_Lost_Counter < release_limit &&
            my_image.Right_Lost_Counter < release_limit)
        {
            if (s_cross_release < CROSS_RELEASE_FRAMES) s_cross_release++;
        }
        else
        {
            s_cross_release = 0;
        }

        if (s_cross_release >= CROSS_RELEASE_FRAMES ||
            (uint32)(s_motion.uptime_ms - s_cross_since_ms) >=
            element_count_to_ms(CROSS_TIMEOUT_10MS))
        {
            g_order.cross = 0;
            s_cross_confirm = 0;
            s_cross_release = 0;
            s_cross_since_ms = 0;
            element_guard_start();
        }
    }

    if (g_order.cross != 1) return;

    // ---- 补线 ----
    if (l_up > 0 && l_down > 0 && (l_down - l_up) >= CROSS_CORNER_GAP)
        Image_Fill_Left(l_up, l_down);          // 正入：上下角点连直线
    else if (l_up > CROSS_FIT_ROWS)
        (void)Image_Extend_Left(l_up);          // 已在十字内：拟合上角点以远再顺下来

    if (r_up > 0 && r_down > 0 && (r_down - r_up) >= CROSS_CORNER_GAP)
        Image_Fill_Right(r_up, r_down);
    else if (r_up > CROSS_FIT_ROWS)
        (void)Image_Extend_Right(r_up);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检测坡道
// 参数说明     void
// 返回参数     void
// 使用示例     Ramp_Detect();
//-------------------------------------------------------------------------------------------------------------------
static void Ramp_Detect(void)
{
    int i;
    int direction;
    int wide_rows = 0;
    int top = IMG_H - my_image.Search_Stop_Line;
    uint8 drive_evidence;
    uint8 visual_evidence;

    if (g_order.ramp)
    {
        // 按里程正常退出，超时看门狗处理停车或误触发。
        if (element_distance_from(s_ramp_count) >= (uint32)RAMP_EXIT_CNT ||
            (uint32)(s_motion.uptime_ms - s_ramp_since_ms) >=
            element_count_to_ms(RAMP_TIMEOUT_10MS))
        {
            g_order.ramp = 0;
            ramp_probe_reset();
            s_ramp_since_ms = 0;
            element_guard_start();
        }
        return;
    }

    if (top < 0) top = 0;
    if (my_image.Search_Stop_Line > RAMP_STOP_ROW_MAX)
    {
        ramp_probe_reset();                     // 还能看很远，说明坡顶没挡住，不是坡
        return;
    }
    // 赛宽表收敛前不启用超宽判据。
    if (!image_road_wide_ready())
    {
        ramp_probe_reset();
        return;
    }

    // 使用垂直赛道方向的宽度，避免弯道水平投影造成误判。
    // 行带锚在实测双边区的近端，理由同 black_stop()：固定行带在低机位上全落在溢出区里
    {
        int band_bottom = (my_image.Edge_Row_Bottom >= 0)
                        ? my_image.Edge_Row_Bottom : RAMP_ROW_BOTTOM;
        int band_top = band_bottom - RAMP_BAND_ROWS;

        if (band_top < top) band_top = top;
        for (i = band_bottom; i >= band_top; i--)
        {
            if (my_image.Road_Perp_Wide[i] <= 0) continue;
            if (my_image.Road_Perp_Wide[i] - 2 * Road_Half_Wide[i] >= RAMP_WIDE_OVER)
                wide_rows++;
        }
    }

    direction = ramp_motion_direction();
    visual_evidence = (uint8)(wide_rows >= RAMP_WIDE_ROWS);
    if (!visual_evidence ||
        fabsf(s_motion.drive_speed_mps) < RAMP_SPEED_MIN_MPS ||
        direction == 0)
    {
        ramp_probe_reset();
        return;
    }

    if (s_ramp_confirm == 0 || s_ramp_direction != direction)
    {
        s_ramp_confirm = 1;
        s_ramp_direction = direction;
        s_ramp_probe_count = s_motion.drive_count_total;
        s_ramp_drive_frames =
            (fabsf(s_motion.drive_output) >= RAMP_DRIVE_OUT_MIN) ? 1u : 0u;
    }
    else
    {
        if (s_ramp_confirm < RAMP_CONFIRM_FRAMES) s_ramp_confirm++;
        if (fabsf(s_motion.drive_output) >= RAMP_DRIVE_OUT_MIN &&
            s_ramp_drive_frames < RAMP_DRIVE_FRAMES)
            s_ramp_drive_frames++;
    }

    drive_evidence = (uint8)(
        element_distance_from(s_ramp_probe_count) >= (uint32)RAMP_ENTRY_CNT_MIN ||
        s_ramp_drive_frames >= RAMP_DRIVE_FRAMES);
    if (s_ramp_confirm >= RAMP_CONFIRM_FRAMES && drive_evidence)
    {
        g_order.ramp = 1;
        s_ramp_count = s_motion.drive_count_total;
        s_ramp_since_ms = s_motion.uptime_ms;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据角点、丢线和对侧残差检测环岛方向
// 参数说明     void
// 返回参数     void
// 使用示例     island_detect();
//-------------------------------------------------------------------------------------------------------------------
static void island_detect(void)
{
    uint8 candidate = 0;

    if (g_island.detect == 0)
    {
        int   corner[4];
        float res_l, res_r;

        Image_Find_Corners(corner);
        res_l = image_edge_residual(my_image.Left_Line,  my_image.Left_Lost_Flag,
                                    my_image.Left_Row_Bottom);
        res_r = image_edge_residual(my_image.Right_Line, my_image.Right_Lost_Flag,
                                    my_image.Right_Row_Bottom);

        if (corner[2] > 0 &&                                    // 右边线下角点
            res_l >= 0.0f && res_l <= RING_STRAIGHT_RES &&      // 左边线是直的
            my_image.Boundry_Start_Left  >= RING_ENTRY_NEAR_ROW &&
            my_image.Boundry_Start_Right >= RING_ENTRY_NEAR_ROW &&
            my_image.Right_Lost_Counter >= RING_LOST_MIN &&
            my_image.Right_Lost_Counter <= RING_LOST_MAX &&
            my_image.Left_Lost_Counter  <= RING_OPP_LOST &&
            my_image.Search_Stop_Line   >= RING_VIEW &&
            my_image.Both_Lost_Counter  <= RING_OPP_LOST)
            candidate = 2;                      // 右环岛
        else if (corner[0] > 0 &&                               // 左边线下角点
                 res_r >= 0.0f && res_r <= RING_STRAIGHT_RES && // 右边线是直的
                 my_image.Boundry_Start_Left  >= RING_ENTRY_NEAR_ROW &&
                 my_image.Boundry_Start_Right >= RING_ENTRY_NEAR_ROW &&
                 my_image.Left_Lost_Counter  >= RING_LOST_MIN &&
                 my_image.Left_Lost_Counter  <= RING_LOST_MAX &&
                 my_image.Right_Lost_Counter <= RING_OPP_LOST &&
                 my_image.Search_Stop_Line   >= RING_VIEW &&
                 my_image.Both_Lost_Counter  <= RING_OPP_LOST)
            candidate = 1;                      // 左环岛

        if (candidate != 0 && candidate == s_ring_candidate)
        {
            if (s_ring_confirm < RING_CONFIRM_FRAMES) s_ring_confirm++;
        }
        else
        {
            s_ring_candidate = candidate;
            s_ring_confirm = (candidate != 0) ? 1u : 0u;
        }

        if (s_ring_confirm >= RING_CONFIRM_FRAMES)
            g_island.detect = s_ring_candidate;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断环岛出口处双边是否已经稳定恢复
// 参数说明     void
// 返回参数     uint8            1=双边恢复 0=尚未恢复
// 使用示例     if (island_edges_recovered()) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 island_edges_recovered(void)
{
    return (uint8)(my_image.Search_Stop_Line >= RING_VIEW &&
                   my_image.Left_Lost_Counter <= RING_OPP_LOST &&
                   my_image.Right_Lost_Counter <= RING_OPP_LOST &&
                   my_image.Both_Lost_Counter <= RING_OPP_LOST);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新左环岛六状态流程
// 参数说明     void
// 返回参数     void
// 使用示例     island_detect_left();
//-------------------------------------------------------------------------------------------------------------------
static void island_detect_left(void)
{
    if (g_order.cross == 0)
    {
        // 方向已经连续确认，状态0只检查当前偏差是否允许入环
        if (g_island.island_state == 0)
        {
            if (s_motion.track_valid && func_abs((int)s_motion.track_error) <= 20)
                g_island.island_state = 1;
            else
                island_reset();
            return;
        }
        if (g_island.island_state == 1)         // 状态1: 等待左边界起点上移
        {
            if (my_image.Boundry_Start_Left < 0 ||
                my_image.Boundry_Start_Left <= RING_ENTRY_FAR_ROW)
            { g_island.island_state = 2; g_island.state2_count = s_motion.drive_count_total; }
            return;
        }
        if (g_island.island_state == 2)         // 状态2: 按编码器累计进环
        {
            if (element_distance_from(g_island.state2_count) >= (uint32)s_motion.ring_s2_cnt_l)
            {
                g_island.island_state = 3;
                g_island.state3_count = s_motion.drive_count_total;
                g_island.state3_angle = s_motion.element_yaw;
            }
        }
        if (g_island.island_state == 3)         // 状态3: 按元素转角沿环
        {
            if ((s_motion.element_yaw - g_island.state3_angle) >= (float)s_motion.ring_angle)
            {
                g_island.island_state = 4;
                g_island.state4_count = s_motion.drive_count_total;
                s_ring_recover = 0;
            }
        }
        if (g_island.island_state == 4)         // 状态4: 等待双边恢复，里程作为兜底
        {
            if (island_edges_recovered())
            { if (s_ring_recover < 3u) s_ring_recover++; }
            else s_ring_recover = 0;
            if (s_ring_recover >= 3u ||
                element_distance_from(g_island.state4_count) >= (uint32)RING_S4_CNT)
            {
                g_island.island_state = 5;
                g_island.state5_count = s_motion.drive_count_total;
                s_ring_recover = 0;
                return;
            }
        }
        if (g_island.island_state == 5)         // 状态5: 双边稳定后完成出环
        {
            if (island_edges_recovered())
            { if (s_ring_recover < 5u) s_ring_recover++; }
            else s_ring_recover = 0;
            if (s_ring_recover >= 5u ||
                element_distance_from(g_island.state5_count) >= (uint32)RING_S5_CNT)
                island_reset();
        }
    }
    else if (g_island.island_state == 0)
    {
        island_reset();
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
    if (g_order.cross == 0)
    {
        if (g_island.island_state == 0)
        {
            if (s_motion.track_valid && func_abs((int)s_motion.track_error) <= 20)
                g_island.island_state = 1;
            else
                island_reset();
            return;
        }
        if (g_island.island_state == 1)         // 状态1: 等待右边界起点上移
        {
            if (my_image.Boundry_Start_Right < 0 ||
                my_image.Boundry_Start_Right <= RING_ENTRY_FAR_ROW)
            { g_island.island_state = 2; g_island.state2_count = s_motion.drive_count_total; }
            return;
        }
        if (g_island.island_state == 2)
        {
            if (element_distance_from(g_island.state2_count) >= (uint32)s_motion.ring_s2_cnt_r)
            {
                g_island.island_state = 3;
                g_island.state3_count = s_motion.drive_count_total;
                g_island.state3_angle = s_motion.element_yaw;
            }
        }
        if (g_island.island_state == 3)         // 状态3: 按元素转角沿环
        {
            if ((s_motion.element_yaw - g_island.state3_angle) <= -(float)s_motion.ring_angle)
            {
                g_island.island_state = 4;
                g_island.state4_count = s_motion.drive_count_total;
                s_ring_recover = 0;
            }
        }
        if (g_island.island_state == 4)
        {
            if (island_edges_recovered())
            { if (s_ring_recover < 3u) s_ring_recover++; }
            else s_ring_recover = 0;
            if (s_ring_recover >= 3u ||
                element_distance_from(g_island.state4_count) >= (uint32)RING_S4_CNT)
            {
                g_island.island_state = 5;
                g_island.state5_count = s_motion.drive_count_total;
                s_ring_recover = 0;
                return;
            }
        }
        if (g_island.island_state == 5)
        {
            if (island_edges_recovered())
            { if (s_ring_recover < 5u) s_ring_recover++; }
            else s_ring_recover = 0;
            if (s_ring_recover >= 5u ||
                element_distance_from(g_island.state5_count) >= (uint32)RING_S5_CNT)
                island_reset();
        }
    }
    else if (g_island.island_state == 0)
    {
        island_reset();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除元素识别和控制状态，保留最新运动快照
// 参数说明     void
// 返回参数     void
// 使用示例     element_state_reset();
//-------------------------------------------------------------------------------------------------------------------
static void element_state_reset(void)
{
    memset((void *)&g_order, 0, sizeof(g_order));
    memset((void *)&g_island, 0, sizeof(g_island));
    s_guard_start_ms = 0;
    s_guard_duration_ms = 0;
    s_island_last_state = 0;
    s_zebra_confirm = 0;
    s_zebra_release = 0;
    s_cross_confirm = 0;
    s_cross_release = 0;
    s_cross_candidate_now = 0;
    s_cross_since_ms = 0;
    s_ring_candidate = 0;
    s_ring_confirm = 0;
    s_ring_recover = 0;
    s_ramp_confirm = 0;
    s_ramp_drive_frames = 0;
    s_ramp_direction = 0;
    s_ramp_probe_count = 0;
    s_ramp_count = 0;
    s_ramp_since_ms = 0;
    s_owner = ELEM_NONE;
    g_elem_action.speed_limit_mps = RUN_SPEED_MAX_MPS;
    g_elem_action.active_elem = ELEM_NONE;
    g_elem_action.stop_request = 0;
    g_elem_action.ring_side_offset = RING_SIDE_OFFSET_DEFAULT;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化元素识别与控制状态
// 参数说明     void
// 返回参数     void
// 使用示例     element_init();
//-------------------------------------------------------------------------------------------------------------------
void element_init(void)
{
    memset((void *)&s_motion, 0, sizeof(s_motion));
    s_run_last = 0;
    element_state_reset();
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
    if (s_motion.run_active && !s_run_last)
        element_state_reset();
    s_run_last = s_motion.run_active;

    g_elem_action.speed_limit_mps = RUN_SPEED_MAX_MPS;
    g_elem_action.active_elem = s_owner;
    g_elem_action.stop_request = 0;
    g_elem_action.ring_side_offset = s_motion.ring_side_offset;

    if (s_owner == ELEM_CROSS && !s_motion.en_cross)
    {
        g_order.cross = 0;
        s_owner = ELEM_NONE;
    }
    if ((s_owner == ELEM_RING_LEFT || s_owner == ELEM_RING_RIGHT) && !s_motion.en_ring)
    {
        island_reset();
        s_owner = ELEM_NONE;
    }
    if (s_owner == ELEM_RAMP && !s_motion.en_ramp)
    {
        g_order.ramp = 0;
        ramp_probe_reset();
        s_owner = ELEM_NONE;
    }
    if (s_owner == ELEM_NONE && !s_motion.en_cross)
    {
        g_order.cross = 0;
        s_cross_confirm = 0;
        s_cross_release = 0;
        s_cross_since_ms = 0;
    }
    if (s_owner == ELEM_NONE && !s_motion.en_ring)
    {
        island_reset();
        s_island_last_state = 0;
    }
    if (s_owner == ELEM_NONE && !s_motion.en_ramp)
    {
        g_order.ramp = 0;
        ramp_probe_reset();
        s_ramp_since_ms = 0;
    }

    if (s_motion.en_zebra && (s_owner == ELEM_NONE || s_owner == ELEM_ZEBRA))
    {
        zebra();
        if (g_order.zebra == 3) s_owner = ELEM_ZEBRA;
        else if (s_owner == ELEM_ZEBRA) s_owner = ELEM_NONE;
    }
    else if (!s_motion.en_zebra)
    {
        g_order.zebra = 0;
        s_zebra_confirm = 0;
        s_zebra_release = 0;
        if (s_owner == ELEM_ZEBRA) s_owner = ELEM_NONE;
    }
    else
    {
        g_order.zebra = 0;
        s_zebra_confirm = 0;
        s_zebra_release = 0;
    }

    if ((s_owner != ELEM_NONE && s_owner != ELEM_RAMP) ||
        (s_owner == ELEM_NONE && element_guard_active()))
        ramp_probe_reset();

    if (s_owner == ELEM_NONE && !element_guard_active())
    {
        s_cross_candidate_now = 0;
        if (s_motion.en_cross)
        {
            Cross_Detect();
            if (g_order.cross) s_owner = ELEM_CROSS;
        }

        // 双侧十字候选优先于单侧环岛候选，避免环岛预确认阻塞十字。
        if (s_cross_candidate_now)
        {
            s_ring_candidate = 0;
            s_ring_confirm = 0;
            ramp_probe_reset();
            if (g_island.island_state == 0) g_island.detect = 0;
        }
        else if (s_owner == ELEM_NONE && s_motion.en_ring)
        {
            island_detect();
            if (g_island.detect == 1)      island_detect_left();
            else if (g_island.detect == 2) island_detect_right();
            island_watchdog();
            if (g_island.island_state != 0)
                s_owner = (g_island.detect == 1) ? ELEM_RING_LEFT : ELEM_RING_RIGHT;
            if (s_ring_candidate != 0 || g_island.island_state != 0)
                ramp_probe_reset();
        }

        g_order.island = (g_island.island_state != 0) ? 1 : 0;
        if (s_owner == ELEM_NONE && !s_cross_candidate_now &&
            s_ring_candidate == 0 && s_motion.en_ramp)
        {
            Ramp_Detect();
            if (g_order.ramp) s_owner = ELEM_RAMP;
        }
    }
    else if (s_owner == ELEM_CROSS)
    {
        Cross_Detect();
        if (!g_order.cross) s_owner = ELEM_NONE;
    }
    else if (s_owner == ELEM_RING_LEFT || s_owner == ELEM_RING_RIGHT)
    {
        if (g_island.detect == 1)      island_detect_left();
        else if (g_island.detect == 2) island_detect_right();
        island_watchdog();
        if (g_island.island_state == 0) s_owner = ELEM_NONE;
    }
    else if (s_owner == ELEM_RAMP)
    {
        Ramp_Detect();
        if (!g_order.ramp) s_owner = ELEM_NONE;
    }

    g_order.island = (g_island.island_state != 0) ? 1 : 0;
    g_elem_action.active_elem = s_owner;
    if (s_owner == ELEM_ZEBRA)
        g_elem_action.stop_request = 1;
    else if (s_owner == ELEM_CROSS)
        g_elem_action.speed_limit_mps = s_motion.speed_cross_mps;
    else if (s_owner == ELEM_RING_LEFT || s_owner == ELEM_RING_RIGHT)
        g_elem_action.speed_limit_mps = s_motion.speed_ring_mps;
    else if (s_owner == ELEM_RAMP)
        g_elem_action.speed_limit_mps = s_motion.speed_ramp_mps;
}

#pragma section all restore
