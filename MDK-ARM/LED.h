// LED.h
#ifndef __LED_H
#define __LED_H

#include "main.h"
#include <stdbool.h>

typedef enum {
    LED_STATE_IDLE,
    LED_STATE_ON,
    LED_STATE_OFF,
    LED_STATE_TOGGLE,
    LED_STATE_CHK_WAIT_0,
    LED_STATE_CHK_WAIT_1
} LedState;

void LED_SetCheckDoneCallback(void (*cb)(void));

void LED_Init(void);
void LED_ProcessCommand(const char *cmd);
void LED_TimerCallback(void);

bool LED_IsCheckDone(void);
bool LED_GetCheckResult(void);
#endif
