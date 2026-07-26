#include "zf_common_headfile.h"

#include "board_config.h"
#include "control.h"
#include "menu.h"
#include "vofa.h"

#pragma section all "cpu0_dsram"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CPU0初始化并持续运行菜单与VOFA通信
// 参数说明     void
// 返回参数     int             主程序返回值
// 使用示例     core0_main();
//-------------------------------------------------------------------------------------------------------------------
int core0_main(void)
{
    clock_init();
    debug_init();

    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_full(RGB565_BLACK);
    ips200_show_string(5, 5, "SYSTEM INIT");
    ips200_show_string(5, 25, "Keep car still...");

    control_init();
    menu_init();

    cpu_wait_event_ready();
    while (TRUE)
    {
        menu_run();
        vofa_cmd_poll();
        vofa_poll();
        vofa_tx_pump();
    }
}

#pragma section all restore
