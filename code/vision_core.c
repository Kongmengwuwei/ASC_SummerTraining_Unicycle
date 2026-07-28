#include "vision_core.h"

#include <string.h>

#include "IfxStm.h"
#include "image.h"
#include "zf_device_mt9v03x.h"

typedef struct
{
    volatile uint32 seq;
    vision_result_t payload;
} vision_result_box_t;

typedef struct
{
    volatile uint32 seq;
    vision_feedback_t payload;
} vision_feedback_box_t;

typedef struct
{
    volatile uint32 seq;
    vision_command_t command;
    uint16 value;
} vision_command_box_t;

#pragma section all "vision_shared"
static vision_result_box_t   s_result_box;
static vision_feedback_box_t s_feedback_box;
static vision_command_box_t  s_command_box;
static volatile uint32       s_heartbeat;
static volatile uint8        s_display_request;
static volatile uint8        s_display_ready;
static volatile uint8        s_display_mode;
static volatile vision_core_state_t s_core_state;
static vision_display_frame_t s_display_frame;
#pragma section all restore

#pragma section all "cpu1_dsram"
static uint32 s_frame_seq;
static uint32 s_command_seq;
static uint32 s_process_max_us;
static uint16 s_camera_exposure;
static int32  s_road_wide_near;         // 已生效的近端赛宽，0=尚未应用过
static int32  s_road_wide_far;          // 已生效的远端赛宽，0=尚未应用过
static vision_feedback_t s_feedback_cache;
#pragma section all restore

#pragma section code "cpu1_psram"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 CPU1 STM 时钟差换算为微秒
// 参数说明     ticks            STM1 计数差
// 返回参数     uint32           经过四舍五入的微秒数
// 使用示例     elapsed_us = vision_ticks_to_us(now - start);
//-------------------------------------------------------------------------------------------------------------------
static inline uint32 vision_ticks_to_us(uint32 ticks)
{
    return (ticks + 50u) / 100u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将整数压缩到图像列坐标范围
// 参数说明     value           原始列坐标
// 返回参数     uint8           0 到 IMG_W-1 的列坐标
// 使用示例     frame->left_line[row] = vision_clip_col(my_image.Left_Line[row]);
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_clip_col(int value)
{
    if (value < 0) value = 0;
    if (value >= IMG_W) value = IMG_W - 1;
    return (uint8)value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU0 发布的反馈快照
// 参数说明     feedback        反馈结果输出地址
// 返回参数     uint8           1=读取成功 0=快照正在更新
// 使用示例     if (vision_feedback_read(&feedback)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_feedback_read(vision_feedback_t *feedback)
{
    uint32 seq_begin;
    uint32 seq_end;

    if (feedback == 0) return 0;
    seq_begin = s_feedback_box.seq;
    if (seq_begin & 1u) return 0;
    __dsync();
    *feedback = s_feedback_box.payload;
    __dsync();
    seq_end = s_feedback_box.seq;
    return (uint8)(seq_begin == seq_end && !(seq_end & 1u));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入 CPU1 在首份 CPU0 反馈到达前使用的视觉默认参数
// 参数说明     feedback        默认反馈输出地址
// 返回参数     void
// 使用示例     vision_feedback_defaults(&s_feedback_cache);
//-------------------------------------------------------------------------------------------------------------------
static void vision_feedback_defaults(vision_feedback_t *feedback)
{
    memset(feedback, 0, sizeof(*feedback));
    feedback->err_offset = 0.0f;
    feedback->speed_ramp_gain = SPEED_RAMP_GAIN_DEFAULT;
    feedback->speed_ring_gain = SPEED_RING_GAIN_DEFAULT;
    feedback->zebra_jump_cnt = ZEBRA_JUMP_CNT_DEFAULT;
    feedback->cross_lost_cnt = CROSS_LOST_CNT_DEFAULT;
    feedback->ring_angle = RING_ANGLE_DEFAULT;
    feedback->ring_s2_cnt_l = RING_S2_CNT_L_DEFAULT;
    feedback->ring_s2_cnt_r = RING_S2_CNT_R_DEFAULT;
    feedback->ring_side_offset = RING_SIDE_OFFSET_DEFAULT;
    feedback->ring_timeout_cnt = RING_TIMEOUT_CNT_DEFAULT;
    feedback->elem_guard_cnt = ELEM_GUARD_CNT_DEFAULT;
    feedback->road_wide_near = ROAD_WIDE_NEAR_DEFAULT;
    feedback->road_wide_far = ROAD_WIDE_FAR_DEFAULT;
    feedback->cam_exposure = CAM_EXPOSURE_DEFAULT;
    // 元素使能默认全关，CPU0 的首份快照到达前 CPU1 不跑任何元素
    feedback->elem_en_zebra = ELEM_EN_ZEBRA_DEFAULT;
    feedback->elem_en_cross = ELEM_EN_CROSS_DEFAULT;
    feedback->elem_en_ring = ELEM_EN_RING_DEFAULT;
    feedback->elem_en_ramp = ELEM_EN_RAMP_DEFAULT;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发布一份完整视觉结果并更新帧序号
// 参数说明     result          待发布结果
// 返回参数     void
// 使用示例     vision_result_publish(&result);
//-------------------------------------------------------------------------------------------------------------------
static void vision_result_publish(const vision_result_t *result)
{
    uint32 seq = s_result_box.seq;

    s_result_box.seq = seq + 1u;
    __dsync();
    s_result_box.payload = *result;
    __dsync();
    s_result_box.seq = seq + 2u;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU0 最新命令且跳过已执行命令
// 参数说明     command/value   命令与参数输出地址
// 返回参数     uint8           1=读取到新命令 0=无新命令
// 使用示例     if (vision_command_read(&command, &value)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_command_read(vision_command_t *command, uint16 *value)
{
    uint32 seq_begin;
    uint32 seq_end;

    seq_begin = s_command_box.seq;
    if (seq_begin == s_command_seq || (seq_begin & 1u)) return 0;
    __dsync();
    *command = s_command_box.command;
    *value = s_command_box.value;
    __dsync();
    seq_end = s_command_box.seq;
    if (seq_begin != seq_end || (seq_end & 1u)) return 0;
    s_command_seq = seq_end;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据当前图像结果生成 CPU0 屏幕快照
// 参数说明     mode            快照显示模式
// 返回参数     void
// 使用示例     vision_display_publish(VISION_DISPLAY_BIN_LINE);
//-------------------------------------------------------------------------------------------------------------------
static void vision_display_publish(vision_display_mode_t mode)
{
    int row;

    if (s_display_ready || !s_display_request) return;
    s_display_frame.frame_seq = s_frame_seq;
    s_display_frame.threshold = (uint16)my_image.Threshold;
    s_display_frame.search_stop_line = (uint16)my_image.Search_Stop_Line;
    s_display_frame.mode = (uint8)mode;
    s_display_frame.track_valid = (uint8)my_image.Track_Valid;
    s_display_frame.active_elem = (uint8)g_elem_action.active_elem;

    if (mode == VISION_DISPLAY_GRAY)
        image_copy_gray(&s_display_frame.pixels[0][0]);
    else
        memcpy(s_display_frame.pixels, my_image.image_two_value, sizeof(s_display_frame.pixels));

    for (row = 0; row < IMG_H; row++)
    {
        s_display_frame.left_line[row] = vision_clip_col(my_image.Left_Line[row]);
        s_display_frame.right_line[row] = vision_clip_col(my_image.Right_Line[row]);
        s_display_frame.mid_line[row] = vision_clip_col(my_image.Mid_Line[row]);
        s_display_frame.left_valid[row] = (uint8)!my_image.Left_Lost_Flag[row];
        s_display_frame.right_valid[row] = (uint8)!my_image.Right_Lost_Flag[row];
        s_display_frame.mid_valid[row] = (uint8)!my_image.Mid_Lost_Flag[row];
    }

    s_display_request = 0;
    __dsync();
    s_display_ready = 1;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行摄像头初始化并发布工作状态
// 参数说明     void
// 返回参数     uint8           1=初始化成功 0=初始化失败
// 使用示例     if (!vision_camera_start()) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_camera_start(void)
{
    uint8 ok;

    s_core_state = VISION_CORE_STARTING;
    s_process_max_us = 0;
    image_init();
    element_init();
    ok = (uint8)(mt9v03x_init() == 0u);
    if (ok)
    {
        (void)mt9v03x_set_exposure_time(s_camera_exposure);
        s_core_state = VISION_CORE_READY;
    }
    else
    {
        s_core_state = VISION_CORE_FAILED;
    }
    return ok;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一条 CPU0 摄像头命令
// 参数说明     void
// 返回参数     void
// 使用示例     vision_command_service();
//-------------------------------------------------------------------------------------------------------------------
static void vision_command_service(void)
{
    vision_command_t command;
    uint16 value;

    if (!vision_command_read(&command, &value)) return;
    if (command == VISION_CMD_SET_EXPOSURE)
    {
        s_camera_exposure = value;
        if (s_core_state == VISION_CORE_READY)
            (void)mt9v03x_set_exposure_time(value);
    }
    else if (command == VISION_CMD_RESTART_CAMERA)
    {
        (void)vision_camera_start();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在 CPU1 初始化摄像头与普通图像算法
// 参数说明     void
// 返回参数     void
// 使用示例     vision_core_init();
//-------------------------------------------------------------------------------------------------------------------
void vision_core_init(void)
{
    vision_feedback_t feedback;

    memset(&s_display_frame, 0, sizeof(s_display_frame));
    vision_feedback_defaults(&s_feedback_cache);
    if (vision_feedback_read(&feedback))
        s_feedback_cache = feedback;
    s_frame_seq = 0;
    s_command_seq = s_command_box.seq;
    s_process_max_us = 0;
    s_camera_exposure = s_feedback_cache.cam_exposure;
    // 置 0 表示赛宽表还没按运行参数建过，第一帧一定会重建一次
    s_road_wide_near = 0;
    s_road_wide_far = 0;
    s_heartbeat = 0;
    s_display_request = 0;
    s_display_ready = 0;
    s_display_mode = (uint8)VISION_DISPLAY_BIN_LINE;
    s_core_state = VISION_CORE_OFF;
    (void)vision_camera_start();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在 CPU1 主循环处理命令与一帧新图像
// 参数说明     void
// 返回参数     void
// 使用示例     vision_core_run();
//-------------------------------------------------------------------------------------------------------------------
void vision_core_run(void)
{
    vision_feedback_t feedback;
    vision_result_t result;
    element_motion_t motion;
    uint32 process_start;
    uint32 element_start;
    uint32 grab_us;
    uint32 element_us;
    uint32 process_us;
    float base_error = 0.0f;
    float final_error = 0.0f;

    s_heartbeat++;
    vision_command_service();
    if (s_core_state != VISION_CORE_READY) return;

    process_start = MODULE_STM1.TIM0.U;
    if (!image_grab()) return;
    grab_us = vision_ticks_to_us(MODULE_STM1.TIM0.U - process_start);

    if (vision_feedback_read(&feedback))
        s_feedback_cache = feedback;
    feedback = s_feedback_cache;

    // 赛宽表只在参数变化时重建，稳态下不做这 80 行插值。
    // 必须排在 image_process() 之前，本帧补线和元素判据要用同一张表。
    if (feedback.road_wide_near != s_road_wide_near ||
        feedback.road_wide_far  != s_road_wide_far)
    {
        s_road_wide_near = feedback.road_wide_near;
        s_road_wide_far  = feedback.road_wide_far;
        image_set_road_wide((int)s_road_wide_near, (int)s_road_wide_far);
    }

    image_process();
    if (my_image.Track_Valid)
        base_error = err_sum_average(ERR_FRONT_ROW, ERR_FRONT_ROW + ERR_AVG_ROWS) - feedback.err_offset;

    motion.drive_count_total = feedback.drive_count_total;
    motion.element_yaw = feedback.element_yaw;
    motion.pitch = feedback.pitch;
    motion.pitch_rate = feedback.pitch_rate;
    motion.track_error = base_error;
    motion.speed_ramp_gain = feedback.speed_ramp_gain;
    motion.speed_ring_gain = feedback.speed_ring_gain;
    motion.uptime_ms = feedback.uptime_ms;
    motion.zebra_jump_cnt = (int)feedback.zebra_jump_cnt;
    motion.cross_lost_cnt = (int)feedback.cross_lost_cnt;
    motion.ring_angle = (int)feedback.ring_angle;
    motion.ring_s2_cnt_l = (int)feedback.ring_s2_cnt_l;
    motion.ring_s2_cnt_r = (int)feedback.ring_s2_cnt_r;
    motion.ring_side_offset = (int)feedback.ring_side_offset;
    motion.ring_timeout_cnt = (int)feedback.ring_timeout_cnt;
    motion.elem_guard_cnt = (int)feedback.elem_guard_cnt;
    motion.en_zebra = feedback.elem_en_zebra;
    motion.en_cross = feedback.elem_en_cross;
    motion.en_ring = feedback.elem_en_ring;
    motion.en_ramp = feedback.elem_en_ramp;

    element_start = MODULE_STM1.TIM0.U;
    element_set_motion(&motion);
    element_process();
    if (g_order.cross == 1)
        Image_Build_Mid_Line();

    if (my_image.Track_Valid)
        final_error = err_sum_average(ERR_FRONT_ROW, ERR_FRONT_ROW + ERR_AVG_ROWS) - feedback.err_offset;
    element_us = vision_ticks_to_us(MODULE_STM1.TIM0.U - element_start);

    s_frame_seq++;
    vision_display_publish((vision_display_mode_t)s_display_mode);
    process_us = vision_ticks_to_us(MODULE_STM1.TIM0.U - process_start);
    if (process_us > s_process_max_us) s_process_max_us = process_us;

    memset(&result, 0, sizeof(result));
    result.frame_seq = s_frame_seq;
    result.input_seq = feedback.input_seq;
    result.heartbeat = s_heartbeat;
    result.camera_vsync_count = mt9v03x_vsync_count;
    result.camera_dma_count = mt9v03x_dma_count;
    result.camera_drop_count = mt9v03x_busy_drop_count;
    result.grab_us = grab_us;
    result.binarize_us = g_image_profile.binarize_us;
    result.border_us = g_image_profile.border_us;
    result.edge_us = g_image_profile.edge_us;
    result.midline_us = g_image_profile.midline_us;
    result.element_us = element_us;
    result.process_us = process_us;
    result.process_max_us = s_process_max_us;
    result.track_error = final_error;
    result.speed_scale = g_elem_action.speed_scale;
    result.threshold = (uint16)my_image.Threshold;
    result.search_stop_line = (uint16)my_image.Search_Stop_Line;
    result.left_lost = (uint16)my_image.Left_Lost_Counter;
    result.right_lost = (uint16)my_image.Right_Lost_Counter;
    result.both_lost = (uint16)my_image.Both_Lost_Counter;
    result.active_elem = g_elem_action.active_elem;
    result.island_state = (uint8)g_island.island_state;
    result.camera_ok = 1;
    result.track_valid = (uint8)my_image.Track_Valid;
    result.stop_request = g_elem_action.stop_request;
    vision_result_publish(&result);
}

#pragma section code restore

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原子读取 CPU1 发布的最新视觉结果
// 参数说明     result          视觉结果输出地址
// 返回参数     uint8           1=读取成功 0=结果正在更新
// 使用示例     if (vision_result_read(&result)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_result_read(vision_result_t *result)
{
    uint32 seq_begin;
    uint32 seq_end;

    if (result == 0) return 0;
    seq_begin = s_result_box.seq;
    if (seq_begin & 1u) return 0;
    __dsync();
    *result = s_result_box.payload;
    __dsync();
    seq_end = s_result_box.seq;
    return (uint8)(seq_begin == seq_end && !(seq_end & 1u));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向 CPU1 发布普通图像处理所需参数
// 参数说明     feedback        CPU0 反馈快照
// 返回参数     void
// 使用示例     vision_feedback_publish(&feedback);
//-------------------------------------------------------------------------------------------------------------------
void vision_feedback_publish(const vision_feedback_t *feedback)
{
    uint32 seq;

    if (feedback == 0) return;
    seq = s_feedback_box.seq;
    s_feedback_box.seq = seq + 1u;
    __dsync();
    s_feedback_box.payload = *feedback;
    __dsync();
    s_feedback_box.seq = seq + 2u;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 调整曝光或重新初始化摄像头
// 参数说明     command/value   命令类型与命令参数
// 返回参数     uint8           1=请求已发布 0=参数无效
// 使用示例     vision_command_request(VISION_CMD_SET_EXPOSURE, 512);
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_command_request(vision_command_t command, uint16 value)
{
    uint32 seq;

    if (command <= VISION_CMD_NONE || command > VISION_CMD_RESTART_CAMERA) return 0;
    seq = s_command_box.seq;
    s_command_box.seq = seq + 1u;
    __dsync();
    s_command_box.command = command;
    s_command_box.value = value;
    __dsync();
    s_command_box.seq = seq + 2u;
    __dsync();
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 在下一帧生成指定模式的屏幕快照
// 参数说明     mode            灰度、二值或二值加边线
// 返回参数     void
// 使用示例     vision_display_request(VISION_DISPLAY_BIN_LINE);
//-------------------------------------------------------------------------------------------------------------------
void vision_display_request(vision_display_mode_t mode)
{
    if (mode > VISION_DISPLAY_GRAY) mode = VISION_DISPLAY_BIN_LINE;
    s_display_mode = (uint8)mode;
    __dsync();
    s_display_request = 1;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取一份由 CPU1 锁定的屏幕图像快照
// 参数说明     frame           返回快照只读指针
// 返回参数     uint8           1=快照可用 0=尚未准备完成
// 使用示例     if (vision_display_read(&frame)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_display_read(const vision_display_frame_t **frame)
{
    if (frame == 0 || !s_display_ready) return 0;
    __dsync();
    *frame = &s_display_frame;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     释放屏幕快照并允许 CPU1 生成下一帧
// 参数说明     void
// 返回参数     void
// 使用示例     vision_display_release();
//-------------------------------------------------------------------------------------------------------------------
void vision_display_release(void)
{
    __dsync();
    s_display_ready = 0;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU1 摄像头工作状态
// 参数说明     void
// 返回参数     vision_core_state_t 当前工作状态
// 使用示例     if (vision_core_state() == VISION_CORE_READY) { ... }
//-------------------------------------------------------------------------------------------------------------------
vision_core_state_t vision_core_state(void)
{
    __dsync();
    return s_core_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU1 主循环心跳计数
// 参数说明     void
// 返回参数     uint32          心跳计数
// 使用示例     heartbeat = vision_core_heartbeat();
//-------------------------------------------------------------------------------------------------------------------
uint32 vision_core_heartbeat(void)
{
    __dsync();
    return s_heartbeat;
}
