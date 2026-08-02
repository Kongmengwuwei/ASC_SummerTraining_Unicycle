#include "zf_common_headfile.h"

#include "board_config.h"
#include "control.h"
#include "menu.h"
#include "vofa.h"

#pragma section all "cpu0_dsram"

// CPU0：屏幕菜单、按键、Run Test 命令与姿态波形。
// 控制在 CCU60_CH0 的 1ms 中断里跑，主循环只做不能进中断的事：刷屏和串口收发。
// 摄像头采集与整帧图像处理在 CPU1，见 cpu1_main.c。

int core0_main(void)
{
    clock_init();

    // 不调 debug_init()：UART0 已停用，Run Test 命令与姿态波形均走无线串口。
    // ips200_init() 内部会把 zf_assert / zf_log 的输出接管到屏幕上，断言照样看得到。
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_full(RGB565_BLACK);
    ips200_show_string(5, 5, "SYSTEM INIT");
    ips200_show_string(5, 25, "Keep car still...");

    control_init();                     // 内部会做 IMU 静止标定，这几秒车必须不动
    menu_init();

    cpu_wait_event_ready();             // 等 CPU1 也初始化完
    while (TRUE)
    {
        vofa_cmd_poll();                // 先接受 Run Test 命令，避免屏幕刷新增加转向基准延迟
        menu_run();                     // 按键与刷屏
        (void)control_ipm_flush();      // 标定完成后在主循环写 Flash
        vofa_poll();                    // 按分频把波形快照格式化进上行环
        // 串口字节的实际收发在 1ms 中断的 vofa_tick1ms() 里，不受上面刷屏拖累
    }
}

#pragma section all restore
