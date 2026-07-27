#include "zf_common_headfile.h"

#include "board_config.h"
#include "control.h"
#include "menu.h"
#include "vofa.h"

#pragma section all "cpu0_dsram"

// CPU0：屏幕菜单、按键、UART0 调参与波形。
// 控制在 CCU60_CH0 的 1ms 中断里跑，主循环只做不能进中断的事：刷屏和串口收发。
// 摄像头采集与整帧图像处理在 CPU1，见 cpu1_main.c。

int core0_main(void)
{
    clock_init();
    debug_init();                       // UART0，波形与调参通道

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
        menu_run();                     // 按键与刷屏
        vofa_cmd_poll();                // 收 UART0 调参命令
        vofa_poll();                    // 按分频把波形快照排进发送队列
        vofa_tx_pump();                 // 把发送队列灌进 UART0 硬件 FIFO
    }
}

#pragma section all restore
