#ifndef VISION_CORE_H_
#define VISION_CORE_H_

#include "zf_common_headfile.h"
#include "board_config.h"
// #include "element.h"                 // 元素识别暂不接入

typedef enum
{
    VISION_CORE_OFF = 0,
    VISION_CORE_STARTING,
    VISION_CORE_READY,
    VISION_CORE_FAILED,
} vision_core_state_t;

typedef enum
{
    VISION_CMD_NONE = 0,
    VISION_CMD_SET_EXPOSURE,
    VISION_CMD_RESTART_CAMERA,
} vision_command_t;

typedef enum
{
    VISION_DISPLAY_BIN_LINE = 0,
    VISION_DISPLAY_BIN,
    VISION_DISPLAY_GRAY,
} vision_display_mode_t;

typedef struct
{
    uint32      frame_seq;
    uint32      input_seq;
    uint32      heartbeat;
    float       track_error;
//  float       speed_scale;            // 完整跑车流程启用后恢复
    uint16      threshold;
    uint16      search_stop_line;
    uint16      left_lost;
    uint16      right_lost;
    uint16      both_lost;
//  elem_type_t active_elem;            // 元素识别启用后恢复
    uint8       camera_ok;
    uint8       track_valid;
//  uint8       stop_request;           // 元素识别启用后恢复
    uint8       reserved;
} vision_result_t;

typedef struct
{
    uint32 input_seq;
    uint32 uptime_ms;
    uint32 param_revision;
//  int32  drive_count_total;           // 元素状态机启用后恢复
//  float  element_yaw;
//  float  pitch;
//  float  pitch_rate;
    float  err_offset;
//  float  speed_ramp_gain;
//  float  speed_ring_gain;
//  float  obs_narrow_ratio;
//  int32  zebra_jump_cnt;
//  int32  cross_lost_cnt;
//  int32  ring_angle;
//  int32  ring_s2_cnt_l;
//  int32  ring_s2_cnt_r;
//  int32  ring_side_offset;
//  int32  obs_line_offset;
    uint16 cam_exposure;
    uint16 reserved;
} vision_feedback_t;

typedef struct
{
    uint32 frame_seq;
    uint16 threshold;
    uint16 search_stop_line;
    uint8  mode;
    uint8  track_valid;
    uint8  pixels[IMG_H][IMG_W];
    uint8  left_line[IMG_H];
    uint8  right_line[IMG_H];
    uint8  mid_line[IMG_H];
    uint8  left_valid[IMG_H];
    uint8  right_valid[IMG_H];
    uint8  mid_valid[IMG_H];
} vision_display_frame_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在 CPU1 初始化摄像头与普通图像算法
// 参数说明     void
// 返回参数     void
// 使用示例     vision_core_init();
//-------------------------------------------------------------------------------------------------------------------
void vision_core_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在 CPU1 主循环处理命令与一帧新图像
// 参数说明     void
// 返回参数     void
// 使用示例     vision_core_run();
//-------------------------------------------------------------------------------------------------------------------
void vision_core_run(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原子读取 CPU1 发布的最新视觉结果
// 参数说明     result          视觉结果输出地址
// 返回参数     uint8           1=读取成功 0=结果正在更新
// 使用示例     if (vision_result_read(&result)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_result_read(vision_result_t *result);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向 CPU1 发布普通图像处理所需参数
// 参数说明     feedback        CPU0 反馈快照
// 返回参数     void
// 使用示例     vision_feedback_publish(&feedback);
//-------------------------------------------------------------------------------------------------------------------
void vision_feedback_publish(const vision_feedback_t *feedback);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 调整曝光或重新初始化摄像头
// 参数说明     command/value   命令类型与命令参数
// 返回参数     uint8           1=请求已发布 0=参数无效
// 使用示例     vision_command_request(VISION_CMD_SET_EXPOSURE, 512);
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_command_request(vision_command_t command, uint16 value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 在下一帧生成指定模式的屏幕快照
// 参数说明     mode            灰度、二值或二值加边线
// 返回参数     void
// 使用示例     vision_display_request(VISION_DISPLAY_BIN_LINE);
//-------------------------------------------------------------------------------------------------------------------
void vision_display_request(vision_display_mode_t mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取一份由 CPU1 锁定的屏幕图像快照
// 参数说明     frame           返回快照只读指针
// 返回参数     uint8           1=快照可用 0=尚未准备完成
// 使用示例     if (vision_display_read(&frame)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_display_read(const vision_display_frame_t **frame);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     释放屏幕快照并允许 CPU1 生成下一帧
// 参数说明     void
// 返回参数     void
// 使用示例     vision_display_release();
//-------------------------------------------------------------------------------------------------------------------
void vision_display_release(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU1 摄像头工作状态
// 参数说明     void
// 返回参数     vision_core_state_t 当前工作状态
// 使用示例     if (vision_core_state() == VISION_CORE_READY) { ... }
//-------------------------------------------------------------------------------------------------------------------
vision_core_state_t vision_core_state(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU1 主循环心跳计数
// 参数说明     void
// 返回参数     uint32          心跳计数
// 使用示例     heartbeat = vision_core_heartbeat();
//-------------------------------------------------------------------------------------------------------------------
uint32 vision_core_heartbeat(void);

#endif
