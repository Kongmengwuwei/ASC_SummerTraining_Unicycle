#ifndef MYMENU_H_
#define MYMENU_H_

#include "zf_common_headfile.h"
#include "menu.h"

void Menu_Init(void);
void Menu_Create(void);
void Menu_Show(void);
void Menu_Switch(void);

/* Call from the 5 ms timer interrupt. */
void Menu_KeyScan_5ms_ISR(void);

/* Call continuously from the CPU0 main loop. */
void Menu_Task(void);

#endif
