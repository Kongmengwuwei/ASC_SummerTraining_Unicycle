#include "zf_common_headfile.h"
#include "vision_core.h"
#pragma section all "cpu1_dsram"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CPU1保留入口并等待双核同步
// 参数说明     void
// 返回参数     void
// 使用示例     core1_main();
//-------------------------------------------------------------------------------------------------------------------
void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);
    cpu_wait_event_ready();
    vision_core_init();
    while (TRUE)
    {
        vision_core_run();
    }
}
#pragma section all restore
