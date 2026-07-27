#ifndef CPU0_MAIN_H
#define CPU0_MAIN_H

#include "Cpu/Std/Ifx_Types.h"

// 逐飞库 zf_common_clock.c 需要这两个类型和 g_AppCpu0，clock_init() 在里面填频率。

// 系统频率信息，单位 Hz
typedef struct
{
    float32 sysFreq;            // SPB 总线频率
    float32 cpuFreq;            // CPU 主频
    float32 pllFreq;            // PLL 输出频率
    float32 stmFreq;            // STM 计数频率
} AppInfo;

typedef struct
{
    AppInfo info;               // 频率信息
} App_Cpu0;

IFX_EXTERN App_Cpu0 g_AppCpu0;  // clock_init() 写，其余模块只读

#endif
