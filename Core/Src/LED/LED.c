/**
 * @file    LED.c
 * @brief   Source file for the LED driver module.
 * @details This file implements a state machine to control an LED, supporting
 * on, off, toggle, and a hardware self-check functionality. It is
 * designed to be driven by a periodic timer callback.
 */

/* Includes ------------------------------------------------------------------*/
#include "LED.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Private Defines -----------------------------------------------------------*/
// Note: Assumes a 10ms tick from the timer callback for timing calculations.
#define LED_OP_DURATION_TICKS       30  // Default duration for ON/TOGGLE modes (30 * 10ms = 300ms)
#define LED_TOGGLE_INTERVAL_TICKS   20  // Interval for the toggle state (20 * 10ms = 200ms)
#define LED_TOGGLE_COUNT_MAX        10  // Number of times to toggle in one sequence
#define LED_CHECK_DELAY_TICKS       20  // Delay for the self-check states (20 * 10ms = 200ms)

/* Private Variables ---------------------------------------------------------*/
static LedState led_state = LED_STATE_IDLE; /// Current state of the LED state machine.

// Timers and counters for state machine logic
static uint16_t led_tick_count = 0;         /// General purpose countdown timer for state duration.
static uint16_t toggle_tick_count = 0;      /// Countdown timer for toggle intervals.
static uint8_t chk_tick_count = 0;          /// Countdown timer for self-check delays.
static uint8_t toggle_count = 0;            /// Counter for the number of toggles performed.
static bool toggle_flag = false;            /// Tracks the current on/off state during a toggle sequence.


static volatile bool led_chk_result = false;/// Stores the result of the self-check (true=pass, false=fail).
static volatile bool led_chk_done = true;   /// Flag indicating if the self-check is complete.
static GPIO_PinState chk_result_on = GPIO_PIN_RESET;  /// Stores GPIO readback when LED is supposed to be ON.
static GPIO_PinState chk_result_off = GPIO_PIN_RESET; /// Stores GPIO readback when LED is supposed to be OFF.

// Callback function pointer
static void (*led_done_cb)(void) = NULL;    /// Function pointer for the check-done callback.



/**
 * @brief  Sets a callback function to be executed when the LED self-check is complete.
 * @param  cb: Pointer to the callback function.
 */
void LED_SetCheckDoneCallback(void (*cb)(void)) {
    led_done_cb = cb;
}
/**
 * @brief  Initializes the LED GPIO pin.
 * @details Sets the LED to the OFF state. Assumes active-low configuration
 * where setting the pin HIGH turns the LED OFF.
 */
void LED_Init(void) {
    // Set pin HIGH to turn the LED OFF initially (active-low).
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    led_state = LED_STATE_IDLE;
}


/**
 * @brief  Processes a string command to control the LED.
 * @param  cmd: A pointer to the null-terminated command string.
 */
void LED_ProcessCommand(const char *cmd) {
    // The "led off" command can interrupt any state.
    if (strcmp(cmd, "led off") == 0) {
        if (led_state != LED_STATE_OFF) {
            led_state = LED_STATE_OFF;
        }
        return;
    }

    // Other commands can only be processed if the state machine is idle.
    if (led_state != LED_STATE_IDLE) {
        return; // Ignore command if busy
    }

    if (strcmp(cmd, "led on") == 0) {
        led_state = LED_STATE_ON;
        led_tick_count = LED_OP_DURATION_TICKS * LED_TOGGLE_COUNT_MAX;;		// 10ms standard
    } else if (strcmp(cmd, "led toggle") == 0) {
        led_state = LED_STATE_TOGGLE;
        led_tick_count = LED_OP_DURATION_TICKS * LED_TOGGLE_COUNT_MAX;;		// 10ms standard
        toggle_tick_count = LED_TOGGLE_INTERVAL_TICKS;	
        toggle_count = 0;
        toggle_flag = false;
    } else if (strcmp(cmd, "led chk") == 0) {
        led_state = LED_STATE_CHK_WAIT_0;
        chk_tick_count = LED_CHECK_DELAY_TICKS;
    } else if (strcmp(cmd, "led state") == 0) {
    } else {
        // Unknown command, do nothing.
    }
}




/**
 * @brief  Periodic timer callback for the LED state machine.
 * @details This function must be called periodically to drive the LED logic.
 */
void LED_TimerCallback(void) {
    switch (led_state) {
        case LED_STATE_ON:
            // Turn LED ON (active-low)
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            if (led_tick_count > 0) {
                led_tick_count--;
            } else {
                // Turn LED OFF (active-low)
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
                if (toggle_tick_count == 0 && toggle_count < LED_TOGGLE_INTERVAL_TICKS) {
                    toggle_flag = !toggle_flag;
                    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, toggle_flag ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    toggle_count++;
                    toggle_tick_count	= LED_TOGGLE_INTERVAL_TICKS;
                }
            } else {
                // Ensure LED is OFF at the end of the sequence
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                led_state = LED_STATE_IDLE;
            }
            break;

        case LED_STATE_CHK_WAIT_0:
            if (chk_tick_count > 0) chk_tick_count--;
            else {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
                chk_result_on = HAL_GPIO_ReadPin(LED_RED_INPUT_GPIO_Port, LED_RED_INPUT_Pin);
                chk_tick_count = LED_OP_DURATION_TICKS;
                led_state = LED_STATE_CHK_WAIT_1;
            }
            break;

        case LED_STATE_CHK_WAIT_1:
            if (chk_tick_count > 0) chk_tick_count--;
            else {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                chk_result_off = HAL_GPIO_ReadPin(LED_RED_INPUT_GPIO_Port, LED_RED_INPUT_Pin);

                if (chk_result_on == GPIO_PIN_RESET && chk_result_off == GPIO_PIN_SET) {
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
/**
 * @brief  Checks if the LED self-check procedure is finished.
 * @retval true if the check is complete, false otherwise.
 */
bool LED_IsCheckDone(void) {
    return led_chk_done;
}

/**
 * @brief  Gets the result of the last self-check and resets the 'done' flag.
 * @note   This function has a side effect: it resets the 'done' flag to false
 * after being called. It should only be called once per check.
 * @retval true if the check passed, false if it failed.
 */
bool LED_GetCheckResult(void) {
    led_chk_done = false; // Reset flag for the next check sequence
    return led_chk_result;
}













