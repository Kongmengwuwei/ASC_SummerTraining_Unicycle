#ifndef STATION_HOST_SHIM_H
#define STATION_HOST_SHIM_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
#define WIRELESS_UART_RTS_PIN 0
static struct { void *asclin; } uart2_handle;
static uint32 interrupt_global_disable(void) { return 0; }
static void interrupt_global_enable(uint32 state) { (void)state; }
static void __dsync(void) {}
static int wireless_uart_init(void) { return 0; }
static uint32 wireless_uart_read_buffer(uint8 *data, uint32 size) { (void)data; (void)size; return 0; }
static int gpio_get_level(int pin) { (void)pin; return 0; }
static int IfxAsclin_getTxFifoFillLevel(void *p) { (void)p; return 0; }
static void IfxAsclin_writeTxData(void *p, uint8 b) { (void)p; (void)b; }
#endif
