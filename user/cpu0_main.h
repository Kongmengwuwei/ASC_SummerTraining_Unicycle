#ifndef CPU0_MAIN_H
#define CPU0_MAIN_H

#include "Cpu/Std/Ifx_Types.h"

typedef struct
{
    float32 sysFreq;
    float32 cpuFreq;
    float32 pllFreq;
    float32 stmFreq;
} AppInfo;

typedef struct
{
    AppInfo info;
} App_Cpu0;

IFX_EXTERN App_Cpu0 g_AppCpu0;

#endif
