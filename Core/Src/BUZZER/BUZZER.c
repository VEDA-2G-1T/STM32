/**
 * @file    BUZZER.c
 * @brief   Source file for the Buzzer driver module.
 * @details This file implements the functions to control a buzzer, including
 * frequency sweeps and a self-check mechanism using ADC feedback.
 * It is designed to be driven by a periodic timer callback.
 */

/* Includes ------------------------------------------------------------------*/
#include "BUZZER.h"
#include <stdio.h>
#include <string.h>

/* Private Defines -----------------------------------------------------------*/
#define PWM_CLOCK_FREQ          1000000U // PWM timer clock frequency in Hz (1MHz)
#define DEFAULT_CHECK_FREQ      500U     // Default frequency for the self-check in Hz
#define SWEEP_START_FREQ        300U     // Sweep starting frequency in Hz
#define SWEEP_END_FREQ          750U     // Sweep ending frequency in Hz
#define SWEEP_FREQ_STEP         10U      // Frequency step for sweep
#define SWEEP_DURATION_TICKS    300      // Total duration of one sweep direction in timer ticks
#define CHECK_DELAY_TICKS       20       // Delay for self-check states in timer ticks (e.g., 20ms if tick is 1ms)
#define ADC_CHECK_THRESHOLD     100      // ADC value threshold for a successful check when PWM is off

/* External Handle Declarations ----------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1; 

/* Private Variables ---------------------------------------------------------*/
static BuzzerState buzzer_state = BUZZER_STATE_IDLE; /// Current state of the buzzer state machine.
static uint16_t buzzer_tick = 0;    /// Countdown timer for sweep duration.
static uint16_t sweep_count = 0;    /// Counter for calculating frequency steps during a sweep.
static uint16_t chk_tick_count = 0; /// Countdown timer for delays in the self-check routine.
static uint16_t chk_result_on = 0;  /// Stores ADC value when the buzzer is ON.
static uint16_t chk_result_off = 0; /// Stores ADC value when the buzzer is OFF.

// static uint16_t chk_result_0 = 0;
// static uint16_t chk_result_1 = 0;

static volatile bool buzzer_chk_result = false; /// Result of the last self-check (true=pass, false=fail).
static volatile bool buzzer_chk_done = true;    /// Flag indicating if the self-check is complete.

static void (*buzzer_done_cb)(void) = NULL; /// Function pointer for the check-done callback.

/* Private Function Prototypes -----------------------------------------------*/
static uint16_t BUZZER_ReadADC(void);

/**
 * @brief  Sets a callback function to be executed when the buzzer self-check is complete.
 * @param  cb: Pointer to the callback function.
 */
void BUZZER_SetCheckDoneCallback(void (*cb)(void)) {
    buzzer_done_cb = cb;
}

/**
 * @brief  Initializes the buzzer hardware (PWM Timer).
 * @details Sets the timer to a default frequency and starts the PWM channel with 0% duty cycle.
 */
void BUZZER_Init(void) {
    uint32_t period = PWM_CLOCK_FREQ / DEFAULT_CHECK_FREQ;

    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Start with PWM OFF (0% duty cycle)

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}


/**
 * @brief  Processes a string command to control the buzzer.
 * @param  cmd: A pointer to the null-terminated command string.
 */
void BUZZER_ProcessCommand(const char *cmd) {
    if (strcmp(cmd, "buzz sweep") == 0) {
        BUZZER_StartSweep();
    } else if (strcmp(cmd, "buzz chk") == 0) {
        buzzer_state = BUZZER_STATE_CHK_PWM_ON_PRE_DELAY;
        chk_tick_count = CHECK_DELAY_TICKS;
        buzzer_chk_done = false;
    }
}

/**
 * @brief  Starts the frequency sweep sound effect.
 * @details Does nothing if the buzzer is already busy.
 */
void BUZZER_StartSweep(void) {
    if (buzzer_state != BUZZER_STATE_IDLE) {
        return; // Buzzer is busy, ignore request.
    }
    buzzer_state = BUZZER_STATE_SWEEP_UP;
    buzzer_tick = SWEEP_DURATION_TICKS;
    sweep_count = 0;
}


/**
 * @brief  Periodic timer callback for the buzzer state machine.
 * @details This function must be called periodically to drive the buzzer logic.
 */
void BUZZER_TimerCallback(void) {
    switch (buzzer_state) {
        case BUZZER_STATE_SWEEP_UP:
            if (buzzer_tick > 0) {
                buzzer_tick--;
                // uint32_t pwm_clock = 1000000;
                uint32_t freq = SWEEP_START_FREQ  + sweep_count * SWEEP_FREQ_STEP;
                if (freq >= SWEEP_END_FREQ) {
                    buzzer_state = BUZZER_STATE_SWEEP_DOWN;
                    sweep_count = 0;
                } else {
                    uint32_t period = PWM_CLOCK_FREQ / freq;
                    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);  // 50% duty cycle
                    sweep_count++;
                }
            } else {
                buzzer_state = BUZZER_STATE_IDLE;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Turn off PWM
            }
            break;

        case BUZZER_STATE_SWEEP_DOWN:
            if (buzzer_tick > 0) {
                buzzer_tick--;
                uint32_t freq = SWEEP_END_FREQ  - sweep_count * SWEEP_FREQ_STEP;
                if (freq <= SWEEP_START_FREQ) {
                    buzzer_state = BUZZER_STATE_SWEEP_UP;
                    sweep_count = 0;
                } else {
                    uint32_t period = PWM_CLOCK_FREQ / freq;
                    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
                    sweep_count++;
                }
            } else {
                buzzer_state = BUZZER_STATE_IDLE;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            }
            break;

        case BUZZER_STATE_CHK_PWM_ON_PRE_DELAY:
            if (chk_tick_count-- == 0) {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim1)/2);
                chk_tick_count = CHECK_DELAY_TICKS; // 2ms delay (assuming 1ms timer)
                buzzer_state = BUZZER_STATE_CHK_PWM_ON_SAMPLING;
            }
            break;

        case BUZZER_STATE_CHK_PWM_ON_SAMPLING:
            if (chk_tick_count-- == 0) {
                chk_result_on = BUZZER_ReadADC();
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Turn PWM OFF
                chk_tick_count = CHECK_DELAY_TICKS; // Wait before next sample
                buzzer_state = BUZZER_STATE_CHK_PWM_OFF_PRE_DELAY;
            }
            break;

        case BUZZER_STATE_CHK_PWM_OFF_PRE_DELAY:
            if (chk_tick_count-- == 0) {
                chk_tick_count = CHECK_DELAY_TICKS;
                buzzer_state = BUZZER_STATE_CHK_PWM_OFF_SAMPLING;
            }
            break;
        case BUZZER_STATE_CHK_PWM_OFF_SAMPLING:
            if (chk_tick_count-- == 0) {
                chk_result_off = BUZZER_ReadADC();
            buzzer_chk_result = ( chk_result_on >= 0 && chk_result_off  <= 100 );
            buzzer_chk_done = true;
            buzzer_state = BUZZER_STATE_IDLE;
            if (buzzer_done_cb) buzzer_done_cb();
                
            }
            break;

        case BUZZER_STATE_IDLE:
        default:
            // Do nothing
            break;
    }
}



/**
 * @brief  Checks if the buzzer self-check procedure is finished.
 * @retval true if the check is complete, false otherwise.
 */
bool BUZZER_IsCheckDone(void) {
    return buzzer_chk_done;
}

/**
 * @brief  Gets the result of the last self-check and resets the 'done' flag.
 * @note   This function has a side effect: it resets the 'done' flag to false
 * after being called. It should only be called once per check.
 * @retval true if the check passed, false if it failed.
 */
bool BUZZER_GetCheckResult(void) {
    buzzer_chk_done = false; // Reset flag for the next check
    return buzzer_chk_result;
}



/**
 * @brief  Performs a single ADC conversion and returns the result.
 * @retval The 16-bit digital value from the ADC.
 */
static uint16_t BUZZER_ReadADC(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}
