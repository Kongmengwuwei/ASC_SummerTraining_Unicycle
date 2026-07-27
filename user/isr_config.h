#ifndef _isr_config_h
#define _isr_config_h

// 中断服务核与优先级配置。
// INT_SERVICE 决定中断由哪个核响应。两个核共用 LCF_INTVEC0_START 这一张向量表，
// 所以 isr.c 里 IFX_INTERRUPT 的第二个参数一律是 0，改服务核只改本文件。
// 优先级必须全局唯一，跨核也不能重号，否则会落进同一个表项。
// CPU 服务优先级范围 1~255，DMA 服务优先级范围 0~47。

// CCU60_CH0：1ms 控制中断，运行在 CPU0
#define CCU6_0_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH0_ISR_PRIORITY 30

// CCU60_CH1、CCU61_CH0、CCU61_CH1 未使用，保留宏供 zf_driver_pit 编译
#define CCU6_0_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 31
#define CCU6_1_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 32
#define CCU6_1_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 33

// ERU 通道 0/4、1/5 未使用，保留宏供 zf_driver_exti 编译
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH0_CH4_INT_PRIO  	40
#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH1_CH5_INT_PRIO  	41

// ERU 通道 2：摄像头 PCLK，直接触发 DMA，不进 CPU，所以没有对应 ISR
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma
#define EXTI_CH2_CH6_INT_PRIO  	5

// ERU 通道 3：摄像头场同步，运行在 CPU1
#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu1
#define EXTI_CH3_CH7_INT_PRIO  	43

// DMA 通道 5：摄像头整场搬运完成，运行在 CPU1
#define	DMA_INT_SERVICE         IfxSrc_Tos_cpu1
#define DMA_DATA_CORE_ID        (1)                 // 预处理专用纯数字，DMA 状态放入 CPU1 DSRAM
#define DMA_INT_PRIO  	        60

// UART0：调参与波形，运行在 CPU0
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART0_TX_INT_PRIO       11
#define UART0_RX_INT_PRIO       10
#define UART0_ER_INT_PRIO       12

// UART1：摄像头配置串口，与摄像头其余中断同核，运行在 CPU1
#define	UART1_INT_SERVICE       IfxSrc_Tos_cpu1
#define UART1_TX_INT_PRIO       13
#define UART1_RX_INT_PRIO       14
#define UART1_ER_INT_PRIO       15

// UART2 未使用，保留宏供 zf_driver_uart 编译
#define	UART2_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO       16
#define UART2_RX_INT_PRIO       17
#define UART2_ER_INT_PRIO       18

// UART3：CYT2BL3 双路无刷驱动，运行在 CPU0
#define	UART3_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO       19
#define UART3_RX_INT_PRIO       20
#define UART3_ER_INT_PRIO       21

#endif
