#ifndef VISION_CORE_H_
#define VISION_CORE_H_

#include "zf_common_headfile.h"
#include "board_config.h"
#include "element.h"

// CPU0 与 CPU1 之间的双向信箱。
// CPU1 跑摄像头采集与整帧图像处理，CPU0 跑控制、菜单和波形。

// CPU1 摄像头工作状态
typedef enum
{
    VISION_CORE_OFF = 0,        // 尚未初始化
    VISION_CORE_STARTING,       // 正在初始化
    VISION_CORE_READY,          // 已就绪，正常出帧
    VISION_CORE_FAILED,         // 初始化失败
} vision_core_state_t;

// CPU0 发给 CPU1 的命令
typedef enum
{
    VISION_CMD_NONE = 0,
    VISION_CMD_SET_EXPOSURE,    // 改曝光时间
    VISION_CMD_RESTART_CAMERA,  // 重新初始化摄像头
    VISION_CMD_CALIB_IPM,       // 用当前这一帧直道图标定逆透视
    VISION_CMD_COUNT            // 合法命令上界，vision_command_request() 拿它做范围检查。
                                
} vision_command_t;

typedef enum
{
    VISION_DISPLAY_BIN_LINE = 0,    // 二值图，CPU0 再叠边线
    VISION_DISPLAY_BIN,             // 二值图
    VISION_DISPLAY_GRAY,            // 灰度图
} vision_display_mode_t;

// CPU1 每帧发布的循迹结果
typedef struct
{
    uint32      frame_seq;          // 视觉帧序号，单调递增
    uint32      input_seq;          // 本帧用到的 CPU0 参数快照序号
    uint32      heartbeat;          // CPU1 主循环心跳
    float       lateral_error;      // 横向误差，单位为半赛道宽，正=左
    float       heading_error;      // 航向误差(°)，正=赛道朝左
    float       curvature;          // 有符号归一化曲率，正=左弯
    float       quality;            // 视觉质量 0..1
    float       speed_limit_mps;    // 元素绝对限速(m/s)
    uint16      threshold;          // 本帧大津阈值
    uint16      search_stop_line;   // 有效前瞻行数
    uint8       ipm_calib_seq;      // CPU1 完成一次标定就 +1，CPU0 据此取走新矩阵
    uint8       ipm_ok;             // CPU1 侧逆透视是否可用
    float       ipm_h[9];           // CPU1 刚标定出来的矩阵，行优先
    uint16      left_lost;          // 前瞻区内左边线丢线行数
    uint16      right_lost;         // 前瞻区内右边线丢线行数
    uint16      both_lost;          // 前瞻区内双边丢线行数
    elem_type_t active_elem;        // 当前识别元素
    uint8       island_state;       // 环岛状态机状态号，0=空闲
    uint8       camera_ok;          // 摄像头出帧正常
    uint8       track_valid;        // 本帧循迹是否可信
    uint8       stop_request;       // 斑马线停车请求
    uint8       run_active;         // 本帧已应用正式 Run 快照
} vision_result_t;

// CPU0 发给 CPU1 的参数快照，视觉算法要用的运行参数都从这里过去
typedef struct
{
    uint32 input_seq;               // 发布序号
    uint32 uptime_ms;               // CPU0 运行时间(ms)
    uint32 param_revision;          // 参数修订号
    int32  drive_count_total;       // C 轮累计里程
    float  element_yaw;             // 独立元素累计转角(°)
    float  pitch;                   // 俯仰角(°)，坡道判据用
    float  pitch_rate;              // 俯仰角速度(°/s)
    float  drive_speed_mps;         // 行进轮实测速度(m/s)
    float  drive_output;            // Pitch 控制输出
    float  speed_cross_mps;         // 十字限速(m/s)
    float  speed_ring_mps;          // 环岛限速(m/s)
    float  speed_ramp_mps;          // 坡道限速(m/s)
    int32  ring_angle;              // 环岛转角阈值(°)
    int32  ring_s2_cnt_l;           // 左环状态2 里程阈值
    int32  ring_s2_cnt_r;           // 右环状态2 里程阈值
    int32  ring_side_offset;        // 环岛单边巡线横向补偿
    uint16 cam_exposure;            // 曝光时间
    uint8  elem_en_zebra;           // 斑马线使能
    uint8  elem_en_cross;           // 十字使能
    uint8  elem_en_ring;            // 环岛使能
    uint8  elem_en_ramp;            // 坡道使能
    uint8  run_active;              // 正式 Run 状态，CPU1 用上升沿复位元素
    int32  err_front_row;           // 前瞻行
    float  ipm_h[9];                // CPU0 保存的逆透视矩阵，行优先
    uint8  reserved;                // 结构对齐占位
} vision_feedback_t;

// 屏幕快照。CPU1 填完后置 ready，CPU0 画完再 release，同一时间只有一份
typedef struct
{
    uint32 frame_seq;               // 生成本快照时的视觉帧序号
    uint16 threshold;               // 本帧大津阈值
    uint16 search_stop_line;        // 有效前瞻行数
    uint8  ipm_row_near;            // 逆透视标定实际选中的近端行，0=没选到
    uint8  ipm_row_far;             // 逆透视标定实际选中的远端行，0=没选到
    uint8  ipm_pick;                // 取角点的结果，ipm_pick_t
    uint8  ipm_col[4];              // 四个角点列坐标：近左/近右/远左/远右
    uint8  mode;                    // 本快照的图像内容，vision_display_mode_t
    uint8  track_valid;             // 本帧循迹是否可信
    uint8  active_elem;             // 当前元素编号
    uint8  pixels[IMG_H][IMG_W];    // 灰度图或二值图
    uint8  left_line[IMG_H];        // 左边线列坐标
    uint8  right_line[IMG_H];       // 右边线列坐标
    uint8  mid_line[IMG_H];         // 中线列坐标
    uint8  left_valid[IMG_H];       // 该行左边线是否有效
    uint8  right_valid[IMG_H];      // 该行右边线是否有效
    uint8  mid_valid[IMG_H];        // 该行中线是否有效
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
