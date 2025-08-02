// BUZZER.h
#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"
#include <stdbool.h>
typedef enum {
    BUZZER_STATE_IDLE,
    BUZZER_STATE_SWEEP_UP,
    BUZZER_STATE_SWEEP_DOWN,
//    BUZZER_STATE_CHK_WAIT_0,
//		BUZZER_STATE_CHK_WAIT_DELAY,
//    BUZZER_STATE_CHK_WAIT_1,
	BUZZER_STATE_CHK_PWM_ON_PRE_DELAY,
	BUZZER_STATE_CHK_PWM_ON_SAMPLING,
	BUZZER_STATE_CHK_PWM_OFF_PRE_DELAY,
	BUZZER_STATE_CHK_PWM_OFF_SAMPLING
} BuzzerState;

void BUZZER_SetCheckDoneCallback(void (*cb)(void));


void BUZZER_Init(void);
void BUZZER_ProcessCommand(const char *cmd);
void BUZZER_TimerCallback(void);
void BUZZER_StartSweep(void);


bool BUZZER_IsCheckDone(void);
bool BUZZER_GetCheckResult(void);


#endif
