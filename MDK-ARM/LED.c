// LED.c
#include "LED.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

static LedState led_state = LED_STATE_IDLE;
static uint16_t led_tick_count = 0;
static uint16_t toggle_tick_count = 0;
static uint8_t chk_tick_count = 0;
static uint8_t toggle_count = 0;
static bool toggle_flag = false;

static volatile bool led_chk_result = false;
static volatile bool led_chk_done = true;

static GPIO_PinState chk_result_0 = GPIO_PIN_RESET;
static GPIO_PinState chk_result_1 = GPIO_PIN_RESET;


static void (*led_done_cb)(void) = NULL;

void LED_SetCheckDoneCallback(void (*cb)(void)) {
    led_done_cb = cb;
}



void LED_Init(void) {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}

void LED_ProcessCommand(const char *cmd) {
    if (strcmp(cmd, "led off") == 0) {
        if (led_state == LED_STATE_OFF) {
            return;
        }
        led_state = LED_STATE_OFF;
        return;
    }

    if (led_state != LED_STATE_IDLE) {
        return;
    }

    if (strcmp(cmd, "led on") == 0) {
        led_state = LED_STATE_ON;
        led_tick_count = 300;		// 10ms standard
    } else if (strcmp(cmd, "led toggle") == 0) {
        led_state = LED_STATE_TOGGLE;
        led_tick_count = 300;		// 10ms standard

				 toggle_tick_count = 20;	
        toggle_count = 0;
        toggle_flag = false;
    } else if (strcmp(cmd, "led chk") == 0) {
				chk_tick_count = 20;
        led_state = LED_STATE_CHK_WAIT_0;
    } else if (strcmp(cmd, "led state") == 0) {
    } else {
    }
}

void LED_TimerCallback(void) {
    switch (led_state) {
        case LED_STATE_ON:
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            if (led_tick_count > 0) led_tick_count--;
            else {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                led_state = LED_STATE_IDLE;
            }
            break;

        case LED_STATE_OFF:
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            led_state = LED_STATE_IDLE;
            break;

        case LED_STATE_TOGGLE:
            if (led_tick_count > 0) {
                led_tick_count--;
                if (toggle_tick_count > 0) toggle_tick_count--;
                if (toggle_tick_count == 0 && toggle_count < 20) {
                    toggle_flag = !toggle_flag;
                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, toggle_flag ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    toggle_count++;
										toggle_tick_count	= 20;
                }
            } else {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                led_state = LED_STATE_IDLE;
            }
            break;

        case LED_STATE_CHK_WAIT_0:
            if (chk_tick_count > 0) chk_tick_count--;
            else {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                chk_result_0 = HAL_GPIO_ReadPin(LED_RED_INPUT_GPIO_Port, LED_RED_INPUT_Pin);
                chk_tick_count = 30;
                led_state = LED_STATE_CHK_WAIT_1;
            }
            break;

				case LED_STATE_CHK_WAIT_1:
            if (chk_tick_count > 0) chk_tick_count--;
            else {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                chk_result_1 = HAL_GPIO_ReadPin(LED_RED_INPUT_GPIO_Port, LED_RED_INPUT_Pin);

									if (chk_result_0 == GPIO_PIN_RESET && chk_result_1 == GPIO_PIN_SET) {
											led_chk_result = true;
									} else {
											led_chk_result = false;
									}

									led_chk_done = true;
									led_state = LED_STATE_IDLE;
									if (led_done_cb) led_done_cb(); 
            }
            break;


        case LED_STATE_IDLE:
        default:
            break;
    }
}




bool LED_IsCheckDone(void) {
    return led_chk_done;
}

bool LED_GetCheckResult(void) {
    led_chk_done = false;
    return led_chk_result;
}














