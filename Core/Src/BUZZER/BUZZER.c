// BUZZER.c
#include "BUZZER.h"
#include <stdio.h>
#include <string.h>

static BuzzerState buzzer_state = BUZZER_STATE_IDLE;
static uint16_t buzzer_tick = 0;
static uint16_t sweep_count = 0;
static uint16_t chk_tick_count = 0;
static uint16_t chk_result_0 = 0;
static uint16_t chk_result_1 = 0;


static volatile bool buzzer_chk_result = false;
static volatile bool buzzer_chk_done = true;

static void (*buzzer_done_cb)(void) = NULL;

void BUZZER_SetCheckDoneCallback(void (*cb)(void)) {
    buzzer_done_cb = cb;
}
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1; 
void BUZZER_Init(void) {
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	    uint32_t pwm_clock = 1000000;    // 1MHz
    uint32_t freq = 500;             // ?? ??? 500Hz
    uint32_t period = pwm_clock / freq;

    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  // PWM OFF

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

static uint16_t BUZZER_ReadADC(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}


void BUZZER_ProcessCommand(const char *cmd) {
    if (strcmp(cmd, "buzz sweep") == 0) {
        BUZZER_StartSweep();
    } else if (strcmp(cmd, "buzz chk") == 0) {
        // buzzer_state = BUZZER_STATE_CHK_WAIT_0;
				buzzer_state = BUZZER_STATE_CHK_PWM_ON_PRE_DELAY;
        chk_tick_count = 20;
        // printf("[BUZZER] Check started.\r\n");
    } else {
        // printf("[BUZZER] Unknown command: %s\r\n", cmd);
    }
}

void BUZZER_StartSweep(void) {
    if (buzzer_state != BUZZER_STATE_IDLE) {
        // printf("[BUZZER] Busy. Ignored.\r\n");
        return;
    }
    buzzer_state = BUZZER_STATE_SWEEP_UP;
    buzzer_tick = 300;
    sweep_count = 0;
    // printf("[BUZZER] Sweep started.\r\n");
}

void BUZZER_TimerCallback(void) {
    switch (buzzer_state) {
        case BUZZER_STATE_SWEEP_UP:
            if (buzzer_tick > 0) {
                buzzer_tick--;
                uint32_t pwm_clock = 1000000;
                uint32_t freq = 300 + sweep_count * 10;
                if (freq >= 750) {
                    buzzer_state = BUZZER_STATE_SWEEP_DOWN;
                    sweep_count = 0;
                } else {
                    uint32_t period = pwm_clock / freq;
                    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
                    sweep_count++;
                }
            } else {
                buzzer_state = BUZZER_STATE_IDLE;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                // printf("[BUZZER] Sweep finished.\r\n");
            }
            break;

        case BUZZER_STATE_SWEEP_DOWN:
            if (buzzer_tick > 0) {
                buzzer_tick--;
                uint32_t pwm_clock = 1000000;
                uint32_t freq = 750 - sweep_count * 10;
                if (freq <= 300) {
                    buzzer_state = BUZZER_STATE_SWEEP_UP;
                    sweep_count = 0;
                } else {
                    uint32_t period = pwm_clock / freq;
                    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
                    sweep_count++;
                }
            } else {
                buzzer_state = BUZZER_STATE_IDLE;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                // printf("[BUZZER] Sweep finished.\r\n");
            }
            break;

case BUZZER_STATE_CHK_PWM_ON_PRE_DELAY:
    if (chk_tick_count-- == 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim1)/2);
        chk_tick_count = 20; // 2ms delay (assuming 1ms timer)
        buzzer_state = BUZZER_STATE_CHK_PWM_ON_SAMPLING;
    }
    break;

case BUZZER_STATE_CHK_PWM_ON_SAMPLING:
    if (chk_tick_count-- == 0) {
        chk_result_0 = BUZZER_ReadADC();
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  // Stop PWM
        chk_tick_count = 20; // 2ms delay before OFF sampling
        buzzer_state = BUZZER_STATE_CHK_PWM_OFF_PRE_DELAY;
    }
    break;

case BUZZER_STATE_CHK_PWM_OFF_PRE_DELAY:
    if (chk_tick_count-- == 0) {
        buzzer_state = BUZZER_STATE_CHK_PWM_OFF_SAMPLING;
				chk_tick_count = 20;
    }
    break;

case BUZZER_STATE_CHK_PWM_OFF_SAMPLING:
	 if (chk_tick_count-- == 0){
		 
    chk_result_1 = BUZZER_ReadADC();
    // ?? ? ?? ??
//		char msg[64];
//		snprintf(msg, sizeof(msg), "ADC0=%d, ADC1=%d\r\n", chk_result_0, chk_result_1);
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    buzzer_chk_result = ( chk_result_0 >= 0 && chk_result_1 <= 100 );
    buzzer_chk_done = true;
    buzzer_state = BUZZER_STATE_IDLE;
    if (buzzer_done_cb) buzzer_done_cb();
		 
	 }
    break;
        case BUZZER_STATE_IDLE:
        default:
            break;
    }
}




bool BUZZER_IsCheckDone(void) {
    return buzzer_chk_done;
}

bool BUZZER_GetCheckResult(void) {
    buzzer_chk_done = false;  
    return buzzer_chk_result;
}

