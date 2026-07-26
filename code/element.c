// #include "element.h"
// #include "image.h"
// #include <string.h>
//
// #pragma section all "cpu1_dsram"
//
// order_t         g_order      = {0};             // 元素调度状态
// island_t        g_island     = {0};             // 环岛状态
// obstacle_t      g_obstacle   = {0};             // 路障状态
// elem_action_t   g_elem_action = {0};            // 元素控制量
// static element_motion_t s_motion;                // CPU0 反馈快照
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     计算当前总里程相对状态入口的绝对增量
// // 参数说明     base_count      状态入口总里程
// // 返回参数     int32           绝对里程增量
// // 使用示例     if (element_distance_from(base) >= threshold) { ... }
// //-------------------------------------------------------------------------------------------------------------------
// static int32 element_distance_from(int32 base_count)
// {
//     int32 distance = s_motion.drive_count_total - base_count;
//     return (distance < 0) ? -distance : distance;
// }
//
// #if (ELEM_EN_ZEBRA)
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     检测近端图像的斑马线跳变特征
// // 参数说明     void
// // 返回参数     int              1=检测到 0=未检测到
// // 使用示例     if (black_stop()) g_order.zebra = 1;
// //-------------------------------------------------------------------------------------------------------------------
// static int black_stop(void)
// {
//     int i, j, count, best = 0;
//     for (i = IMG_H - 1; i >= IMG_H - 2; i--)
//     {
//         count = 0;
//         for (j = 30; j < IMG_W - 31; j++)
//             if (my_image.image_two_value[i][j] != my_image.image_two_value[i][j + 1])
//                 count++;
//         if (count > best) best = count;
//     }
//     return (best >= s_motion.zebra_jump_cnt) ? 1 : 0;
// }
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     更新斑马线三段确认状态
// // 参数说明     void
// // 返回参数     void
// // 使用示例     zebra();
// //-------------------------------------------------------------------------------------------------------------------
// static void zebra(void)
// {
//     if (black_stop() && g_order.zebra == 0)       g_order.zebra = 1;
//     if (g_order.zebra == 1 && black_stop() == 0)  g_order.zebra = 2;
//     if (g_order.zebra == 2 && black_stop())       g_order.zebra = 3;
// }
//
// #endif
//
// #if (ELEM_EN_CROSS)
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     根据双边丢线与上角点检测十字
// // 参数说明     void
// // 返回参数     void
// // 使用示例     Cross_Detect();
// //-------------------------------------------------------------------------------------------------------------------
// static void Cross_Detect(void)
// {
//     my_image.Left_Up_Find = 0;
//     my_image.Right_Up_Find = 0;
//     Find_Up_Point(IMG_H - 1, 0);
//
//     if (g_order.island == 0)
//     {
//         if (my_image.Left_Lost_Counter >= s_motion.cross_lost_cnt &&
//             my_image.Right_Lost_Counter >= s_motion.cross_lost_cnt)
//         {
//             if (my_image.Left_Up_Find != 0 || my_image.Right_Up_Find != 0)
//                 g_order.cross = 1;              // 双边丢线且存在角点
//             else
//                 g_order.cross = 0;
//         }
//         else
//             g_order.cross = 0;
//     }
//     else
//         g_order.cross = 0;
// }
//
// #endif
//
// #if (ELEM_EN_RAMP)
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     根据赛宽与中线偏差检测坡道
// // 参数说明     void
// // 返回参数     void
// // 使用示例     Ramp_Detect();
// //-------------------------------------------------------------------------------------------------------------------
// static void Ramp_Detect(void)
// {
//     int i, count = 0;
//
//     if (my_image.Search_Stop_Line >= RAMP_SEARCH_LINE)
//     {
//         for (i = IMG_H - 1; i > IMG_H - my_image.Search_Stop_Line; i--)
//             if (my_image.Road_Wide[i] - Standard_Road_Wide[i] > RAMP_WIDE_OVER)
//                 count++;                        // 统计超宽行
//     }
//
//     if (count >= RAMP_WIDE_ROWS &&
//         func_abs((int)s_motion.track_error) <= RAMP_ERR_LIMIT &&
//         g_order.cross == 0 &&
//         my_image.Right_Lost_Counter <= 15 && my_image.Left_Lost_Counter <= 15)
//         g_order.ramp = 1;
//     else
//         g_order.ramp = 0;
// }
//
// #endif
//
// #if (ELEM_EN_RING || ELEM_EN_OBSTACLE)
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     搜索右边线不连续位置
// // 参数说明     start/end        搜索起始/终止行
// // 返回参数     void
// // 使用示例     Continuity_Change_Right(30, IMG_H-1-10);
// //-------------------------------------------------------------------------------------------------------------------
// static void Continuity_Change_Right(int start, int end)
// {
//     int i, t;
//     if (my_image.Right_Lost_Counter >= (int)(0.9f * IMG_H)) { my_image.continuity_change_flag_right = 0; }
//     if (start >= IMG_H - 5) start = IMG_H - 5;
//     if (end <= 5) end = 5;
//     if (start < end) { t = start; start = end; end = t; }
//
//     for (i = start; i >= end; i--)
//     {
//         if (func_abs(my_image.Right_Line[i] - my_image.Right_Line[i - 5]) >= RING_CONTINUITY)
//         { my_image.continuity_change_flag_right = i; break; }
//         else
//             my_image.continuity_change_flag_right = 0;
//     }
// }
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     搜索左边线不连续位置
// // 参数说明     start/end        搜索起始/终止行
// // 返回参数     void
// // 使用示例     Continuity_Change_Left(30, IMG_H-1-10);
// //-------------------------------------------------------------------------------------------------------------------
// static void Continuity_Change_Left(int start, int end)
// {
//     int i, t;
//     if (my_image.Both_Lost_Counter >= (int)(0.9f * IMG_H)) my_image.continuity_change_flag_left = 0;
//     if (my_image.Search_Stop_Line <= 5)                    my_image.continuity_change_flag_left = 0;
//     if (start >= IMG_H - 1 - 5) start = IMG_H - 1 - 5;
//     if (end <= 5) end = 5;
//     if (start < end) { t = start; start = end; end = t; }
//
//     for (i = start; i >= end; i--)
//     {
//         if (func_abs(my_image.Left_Line[i] - my_image.Left_Line[i - 2]) >= RING_CONTINUITY)
//         { my_image.continuity_change_flag_left = i; break; }
//         else
//             my_image.continuity_change_flag_left = 0;
//     }
// }
//
// #endif
//
// #if (ELEM_EN_RING)
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     根据边线连续性与丢线状态检测环岛方向
// // 参数说明     void
// // 返回参数     void
// // 使用示例     island_detect();
// //-------------------------------------------------------------------------------------------------------------------
// static void island_detect(void)
// {
//     if (g_island.detect == 0)
//     {
//         my_image.continuity_change_flag_left = 0;
//         my_image.continuity_change_flag_right = 0;
//         Continuity_Change_Left(30, IMG_H - 1 - 10);
//         Continuity_Change_Right(30, IMG_H - 1 - 10);
//         if (my_image.continuity_change_flag_right >= 20 &&
//             my_image.continuity_change_flag_left  <= RING_OPP_LOST &&
//             my_image.Right_Lost_Counter >= RING_LOST_MIN &&
//             my_image.Right_Lost_Counter <= RING_LOST_MAX &&
//             my_image.Left_Lost_Counter  <= RING_OPP_LOST &&
//             my_image.Search_Stop_Line   >= RING_VIEW &&
//             my_image.Both_Lost_Counter  <= RING_OPP_LOST)
//             g_island.detect = 2;                // 右环岛
//         else if (my_image.continuity_change_flag_left >= 20 &&
//                  my_image.continuity_change_flag_right <= RING_OPP_LOST &&
//                  my_image.Left_Lost_Counter  >= RING_LOST_MIN &&
//                  my_image.Left_Lost_Counter  <= RING_LOST_MAX &&
//                  my_image.Right_Lost_Counter <= RING_OPP_LOST &&
//                  my_image.Search_Stop_Line   >= RING_VIEW &&
//                  my_image.Both_Lost_Counter  <= RING_OPP_LOST)
//             g_island.detect = 1;                // 左环岛
//     }
// }
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     更新左环岛六状态流程
// // 参数说明     void
// // 返回参数     void
// // 使用示例     island_detect_left();
// //-------------------------------------------------------------------------------------------------------------------
// static void island_detect_left(void)
// {
//     my_image.continuity_change_flag_left = 0;
//     my_image.continuity_change_flag_right = 0;
//     Continuity_Change_Right(30, IMG_H - 1 - 10);
//     Continuity_Change_Left(30, IMG_H - 1 - 10);
//     if (g_order.cross == 0 && g_order.ramp == 0)
//     {
//         // 状态0: 确认入环特征
//         if (g_island.island_state == 0 && func_abs((int)s_motion.track_error) <= 20)
//         {
//             if (my_image.continuity_change_flag_left >= 20 &&
//                 my_image.continuity_change_flag_right <= RING_OPP_LOST &&
//                 my_image.Left_Lost_Counter  >= RING_LOST_MIN &&
//                 my_image.Left_Lost_Counter  <= RING_LOST_MAX &&
//                 my_image.Right_Lost_Counter <= RING_OPP_LOST &&
//                 my_image.Search_Stop_Line   >= RING_VIEW &&
//                 my_image.Both_Lost_Counter  <= RING_OPP_LOST)
//             { g_island.island_state = 1; }
//         }
//         if (g_island.island_state == 1)         // 状态1: 等待左边界起点上移
//         {
//             if (my_image.Boundry_Start_Left < 50)
//             { g_island.island_state = 2; g_island.state2_count = s_motion.drive_count_total; }
//         }
//         if (g_island.island_state == 2)         // 状态2: 按编码器累计进环
//         {
//             if (element_distance_from(g_island.state2_count) >= s_motion.ring_s2_cnt_l)
//             {
//                 g_island.island_state = 3;
//                 g_island.state3_count = s_motion.drive_count_total;
//                 g_island.state3_angle = s_motion.element_yaw;
//             }
//         }
//         if (g_island.island_state == 3)         // 状态3: 按元素转角沿环
//         {
//             if ((s_motion.element_yaw - g_island.state3_angle) >= (float)s_motion.ring_angle)
//             { g_island.island_state = 4; g_island.state4_count = s_motion.drive_count_total; }
//         }
//         if (g_island.island_state == 4)         // 状态4: 按编码器累计出环
//         {
//             if (element_distance_from(g_island.state4_count) >= RING_S4_CNT)
//             { g_island.island_state = 5; g_island.state5_count = s_motion.drive_count_total; }
//         }
//         if (g_island.island_state == 5)         // 状态5: 完成出环
//         {
//             if (element_distance_from(g_island.state5_count) >= RING_S5_CNT)
//             { g_island.island_state = 0; g_island.state5_count = 0; g_island.detect = 0; }
//         }
//     }
// }
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     更新右环岛六状态流程
// // 参数说明     void
// // 返回参数     void
// // 使用示例     island_detect_right();
// //-------------------------------------------------------------------------------------------------------------------
// static void island_detect_right(void)
// {
//     my_image.continuity_change_flag_left = 0;
//     my_image.continuity_change_flag_right = 0;
//     Continuity_Change_Left(30, IMG_H - 1 - 10);
//     Continuity_Change_Right(30, IMG_H - 1 - 10);
//     if (g_order.cross == 0 && g_order.ramp == 0)
//     {
//         if (g_island.island_state == 0 && func_abs((int)s_motion.track_error) <= 20)
//         {
//             if (my_image.continuity_change_flag_right >= 20 &&
//                 my_image.continuity_change_flag_left  <= RING_OPP_LOST &&
//                 my_image.Right_Lost_Counter >= RING_LOST_MIN &&
//                 my_image.Right_Lost_Counter <= RING_LOST_MAX &&
//                 my_image.Left_Lost_Counter  <= RING_OPP_LOST &&
//                 my_image.Search_Stop_Line   >= RING_VIEW &&
//                 my_image.Both_Lost_Counter  <= RING_OPP_LOST)
//             { g_island.island_state = 1; }
//         }
//         if (g_island.island_state == 1)         // 状态1: 等待右边界起点上移
//         {
//             if (my_image.Boundry_Start_Right < 50)
//             { g_island.island_state = 2; g_island.state2_count = s_motion.drive_count_total; }
//         }
//         if (g_island.island_state == 2)
//         {
//             if (element_distance_from(g_island.state2_count) >= s_motion.ring_s2_cnt_r)
//             {
//                 g_island.island_state = 3;
//                 g_island.state3_count = s_motion.drive_count_total;
//                 g_island.state3_angle = s_motion.element_yaw;
//             }
//         }
//         if (g_island.island_state == 3)         // 状态3: 按元素转角沿环
//         {
//             if ((s_motion.element_yaw - g_island.state3_angle) <= -(float)s_motion.ring_angle)
//             { g_island.island_state = 4; g_island.state4_count = s_motion.drive_count_total; }
//         }
//         if (g_island.island_state == 4)
//         {
//             if (element_distance_from(g_island.state4_count) >= RING_S4_CNT)
//             { g_island.island_state = 5; g_island.state5_count = s_motion.drive_count_total; }
//         }
//         if (g_island.island_state == 5)
//         {
//             if (element_distance_from(g_island.state5_count) >= RING_S5_CNT)
//             { g_island.island_state = 0; g_island.state5_count = 0; g_island.detect = 0; }
//         }
//     }
// }
//
// #endif
//
// #if (ELEM_EN_OBSTACLE)
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     偏移边线以生成避障中线
// // 参数说明     void
// // 返回参数     void
// // 使用示例     obstacle_avoid_process();
// //-------------------------------------------------------------------------------------------------------------------
// static void obstacle_avoid_process(void)
// {
//     for (int i = IMG_H - 1; i >= IMG_H - my_image.Search_Stop_Line; i--)
//     {
//         if (g_obstacle.direction == 1)          // 左侧避障
//             my_image.Left_Line[i] = (my_image.Left_Line[i] > 20)
//                                   ? (my_image.Left_Line[i] + s_motion.obs_line_offset)
//                                   : 5;
//         else if (g_obstacle.direction == 2)     // 右侧避障
//             my_image.Right_Line[i] = (my_image.Right_Line[i] < IMG_W - 20)
//                                    ? (my_image.Right_Line[i] - s_motion.obs_line_offset)
//                                    : (IMG_W - 5);
//     }
// }
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     根据赛宽收窄与边线连续性检测路障
// // 参数说明     void
// // 返回参数     void
// // 使用示例     obstacle_detect();
// //-------------------------------------------------------------------------------------------------------------------
// static void obstacle_detect(void)
// {
//     g_obstacle.narrow_count = 0;
//
//     if (my_image.continuity_change_flag_right)      g_obstacle.direction = 2;   // 右侧避障
//     else if (my_image.continuity_change_flag_left)  g_obstacle.direction = 1;   // 左侧避障
//
//     // 仅在普通赛道状态检测路障
//     if (g_island.island_state == 0 && g_order.cross == 0 && g_order.ramp == 0)
//     {
//         for (int i = OBS_ROW_MAX; i >= OBS_ROW_MIN; i--)
//         {
//             if ((float)my_image.Road_Wide[i] <=
//                 (float)Standard_Road_Wide[i] * s_motion.obs_narrow_ratio)
//                 g_obstacle.narrow_count++;
//         }
//         if (g_obstacle.narrow_count >= OBS_NARROW_CNT) g_obstacle.state = 1;
//         else                                           g_obstacle.state = 0;
//     }
//     // 赛宽恢复后清除路障状态
//     if (g_obstacle.state == 1 &&
//         my_image.Road_Wide[OBS_ROW_MIN] >= Standard_Road_Wide[OBS_ROW_MIN] * OBS_RECOVER_RATIO)
//         g_obstacle.state = 0;
//
//     if (g_obstacle.state == 1) obstacle_avoid_process();
// }
//
// #endif
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     初始化元素识别与控制状态
// // 参数说明     void
// // 返回参数     void
// // 使用示例     element_init();
// //-------------------------------------------------------------------------------------------------------------------
// void element_init(void)
// {
//     memset((void *)&g_order,    0, sizeof(g_order));
//     memset((void *)&g_island,   0, sizeof(g_island));
//     memset((void *)&g_obstacle, 0, sizeof(g_obstacle));
//     memset((void *)&s_motion,   0, sizeof(s_motion));
//     g_elem_action.speed_scale = 1.0f;
//     g_elem_action.active_elem = ELEM_NONE;
//     g_elem_action.stop_request = 0;
//     g_elem_action.ring_side_offset = RING_SIDE_OFFSET_DEFAULT;
// }
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     更新元素状态机使用的 CPU0 里程与姿态快照
// // 参数说明     motion          总里程、元素角、Pitch 与基础偏差
// // 返回参数     void
// // 使用示例     element_set_motion(&motion);
// //-------------------------------------------------------------------------------------------------------------------
// void element_set_motion(const element_motion_t *motion)
// {
//     if (motion != 0)
//         s_motion = *motion;
// }
//
// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     执行单帧元素检测与控制量更新
// // 参数说明     void
// // 返回参数     void
// // 使用示例     element_process();
// //-------------------------------------------------------------------------------------------------------------------
// void element_process(void)
// {
//     g_elem_action.speed_scale  = 1.0f;
//     g_elem_action.active_elem  = ELEM_NONE;
//     g_elem_action.stop_request = 0;
//     g_elem_action.ring_side_offset = s_motion.ring_side_offset;
//
//     // 清除未启用元素的状态
// #if (!ELEM_EN_CROSS)
//     g_order.cross = 0;
// #endif
// #if (!ELEM_EN_RAMP)
//     g_order.ramp = 0;
// #endif
// #if (!ELEM_EN_RING)
//     g_order.island = 0; g_island.detect = 0; g_island.island_state = 0;
// #endif
// #if (!ELEM_EN_OBSTACLE)
//     g_obstacle.state = 0;
// #endif
//
// #if (ELEM_EN_ZEBRA)
//     // 斑马线检测
//     zebra();
//     if (g_order.zebra == 3) { g_elem_action.active_elem = ELEM_ZEBRA; g_elem_action.stop_request = 1; }
// #endif
//
// #if (ELEM_EN_CROSS)
//     // 十字检测
//     Cross_Detect();
// #endif
//
// #if (ELEM_EN_RING)
//     // 环岛检测与状态更新
//     island_detect();
//     if (g_island.detect == 1)      island_detect_left();
//     else if (g_island.detect == 2) island_detect_right();
//     g_order.island = (g_island.island_state != 0) ? 1 : 0;   // 环岛互斥标志
// #endif
//
// #if (ELEM_EN_RAMP)
//     // 坡道检测
//     Ramp_Detect();
// #endif
//
// #if (ELEM_EN_OBSTACLE)
//     // 路障检测
//     obstacle_detect();
// #endif
//
// #if (ELEM_EN_CROSS)
//     // 仅对检测到角点的一侧执行十字补线
//     if (g_order.cross == 1)
//     {
//         if (my_image.Left_Up_Find  > 1) Lengthen_Left_Boundry (my_image.Left_Up_Find  - 1, IMG_H - 10);
//         if (my_image.Right_Up_Find > 1) Lengthen_Right_Boundry(my_image.Right_Up_Find - 1, IMG_H - 10);
//         g_elem_action.active_elem = ELEM_CROSS;
//     }
// #endif
//
//     // 更新元素速度与类型
//     if (g_order.ramp == 1)
//     {
//         g_elem_action.speed_scale = s_motion.speed_ramp_gain;
//         g_elem_action.active_elem = ELEM_RAMP;
//     }
//     else if (g_island.island_state != 0)
//     {
//         g_elem_action.speed_scale = s_motion.speed_ring_gain;
//         g_elem_action.active_elem = (g_island.detect == 1) ? ELEM_RING_LEFT : ELEM_RING_RIGHT;
//     }
//     else if (g_obstacle.state == 1)
//     {
//         g_elem_action.active_elem = ELEM_OBSTACLE;
//     }
// }
//
// #pragma section all restore
