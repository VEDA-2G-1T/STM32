/**
 * @file    BUZZER.h
 * @brief   Header file for the Buzzer driver module.
 * @details This module provides functionalities to control a buzzer, including
 * playing sounds, running frequency sweeps, and performing a self-check.
 */

#ifndef __BUZZER_H
#define __BUZZER_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"       
#include <stdbool.h>    



/* Exported Types ------------------------------------------------------------*/

/**
 * @brief Defines the operational states of the buzzer state machine.
 */
typedef enum {
    BUZZER_STATE_IDLE,                  /**< Buzzer is inactive or silent. */
    BUZZER_STATE_SWEEP_UP,              /**< Buzzer is performing a frequency sweep-up sound. */
    BUZZER_STATE_SWEEP_DOWN,            /**< Buzzer is performing a frequency sweep-down sound. */
    BUZZER_STATE_CHK_PWM_ON_PRE_DELAY,  /**< Self-check: Delay before turning PWM on for sampling. */
    BUZZER_STATE_CHK_PWM_ON_SAMPLING,   /**< Self-check: Sampling feedback while PWM is on. */
    BUZZER_STATE_CHK_PWM_OFF_PRE_DELAY, /**< Self-check: Delay before turning PWM off for sampling. */
    BUZZER_STATE_CHK_PWM_OFF_SAMPLING   /**< Self-check: Sampling feedback while PWM is off. */
} BuzzerState;

/**
 * @brief Sets a callback function to be executed when the buzzer self-check is complete.
 * @param cb Pointer to the callback function, which takes no arguments and returns nothing.
 */
void BUZZER_SetCheckDoneCallback(void (*cb)(void));

/**
 * @brief Initializes the buzzer module.
 * @details This function configures the necessary hardware (e.g., GPIO, Timer)
 * and should be called once at system startup.
 */
void BUZZER_Init(void);

/**
 * @brief Processes a string command to control the buzzer.
 * @details Useful for external control via an interface like UART.
 * @param cmd A pointer to the null-terminated command string.
 */
void BUZZER_ProcessCommand(const char *cmd);
/**
 * @brief Periodic timer callback for the buzzer state machine.
 * @details This function must be called periodically (e.g., from a timer interrupt)
 * to handle time-dependent operations like sweeps and delays.
 */
void BUZZER_TimerCallback(void);
/**
 * @brief Starts the frequency sweep sound effect.
 */
void BUZZER_StartSweep(void);
/**
 * @brief Checks if the buzzer self-check procedure is finished.
 * @return `true` if the check is complete, `false` otherwise.
 */
bool BUZZER_IsCheckDone(void);
/**
 * @brief Gets the result of the last self-check procedure.
 * @return `true` if the check passed, `false` if it failed.
 */
bool BUZZER_GetCheckResult(void);

#endif /* __BUZZER_H */
