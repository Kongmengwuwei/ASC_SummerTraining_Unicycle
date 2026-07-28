#include "zf_common_headfile.h"
#include "vision_core.h"
#pragma section all "cpu1_dsram"

// CPU1：摄像头采集与整帧图像处理。
// 摄像头的三个中断（UART1 配置、ERU 通道 3 场同步、DMA 通道 5 搬运完成）也都在本核，
// 服务核在 isr_config.h 里指定，向量表号在 isr.c 的 IFX_INTERRUPT 第二个参数里指定，两处必须一致。
// 处理结果通过 vision_core.c 的信箱交给 CPU0，本核不碰任何电机。

void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);
    cpu_wait_event_ready();             // 等 CPU0 也初始化完
    vision_core_init();
    while (TRUE)
    {
        vision_core_run();
    }
}
#pragma section all restore
