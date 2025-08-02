/**
 * @file    LED.h
 * @brief   Header file for the LED driver module.
 * @details This module provides functionalities to control an LED, including
 * setting its state (on/off), toggling, and performing a self-check routine.
 */

#ifndef __LED_H
#define __LED_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"       // Main project header for MCU-specific definitions.
#include <stdbool.h>    // Standard library for boolean type (true, false).

/**
 * @brief Defines the operational states of the LED state machine.
 */
typedef enum {
    LED_STATE_IDLE,         /**< LED is not in an active, timed process (e.g., blinking). */
    LED_STATE_ON,           /**< LED is permanently ON. */
    LED_STATE_OFF,          /**< LED is permanently OFF. */
    LED_STATE_TOGGLE,       /**< LED is in a blinking/toggling state. */
    LED_STATE_CHK_WAIT_0,   /**< Self-check: First waiting state for the check routine. */
    LED_STATE_CHK_WAIT_1    /**< Self-check: Second waiting state for the check routine. */
} LedState;


/**
 * @brief Sets a callback function to be executed when the LED self-check is complete.
 * @param cb Pointer to the callback function, which takes no arguments and returns nothing.
 */
void LED_SetCheckDoneCallback(void (*cb)(void));

/**
 * @brief Initializes the LED module.
 * @details This function configures the GPIO pin associated with the LED
 * and should be called once at system startup.
 */
void LED_Init(void);

/**
 * @brief Processes a string command to control the LED.
 * @details Useful for external control via an interface like a command-line interpreter.
 * @param cmd A pointer to the null-terminated command string.
 */
void LED_ProcessCommand(const char *cmd);

/**
 * @brief Periodic timer callback for the LED state machine.
 * @details This function must be called periodically (e.g., from a timer interrupt)
 * to handle time-dependent operations like toggling.
 */
void LED_TimerCallback(void);

/**
 * @brief Checks if the LED self-check procedure is finished.
 * @return `true` if the check is complete, `false` otherwise.
 */
bool LED_IsCheckDone(void);

/**
 * @brief Gets the result of the last self-check procedure.
 * @return `true` if the check passed, `false` if it failed.
 */
bool LED_GetCheckResult(void);

#endif /* __LED_H */