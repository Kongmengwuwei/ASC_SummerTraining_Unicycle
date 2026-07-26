#ifndef _isr_config_h
#define _isr_config_h

// 各中断优先级保持唯一；CPU 服务优先级范围为 1~255，DMA 为 0~47。

// PIT
#define CCU6_0_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH0_ISR_PRIORITY 30

#define CCU6_0_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 31

#define CCU6_1_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 32

#define CCU6_1_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 33
// ERU
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH0_CH4_INT_PRIO  	40
#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH1_CH5_INT_PRIO  	41
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma
#define EXTI_CH2_CH6_INT_PRIO  	5
#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu1
#define EXTI_CH3_CH7_INT_PRIO  	43

// DMA
#define	DMA_INT_SERVICE         IfxSrc_Tos_cpu1
#define DMA_INT_PRIO  	        60

// UART
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART0_TX_INT_PRIO       11
#define UART0_RX_INT_PRIO       10
#define UART0_ER_INT_PRIO       12

#define	UART1_INT_SERVICE       IfxSrc_Tos_cpu1
#define UART1_TX_INT_PRIO       13
#define UART1_RX_INT_PRIO       14
#define UART1_ER_INT_PRIO       15

#define	UART2_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO       16
#define UART2_RX_INT_PRIO       17
#define UART2_ER_INT_PRIO       18

#define	UART3_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO       19
#define UART3_RX_INT_PRIO       20
#define UART3_ER_INT_PRIO       21
#endif
