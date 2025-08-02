/**
 * @file    PCF8591T.c
 * @brief   Implementation file for the PCF8591T I2C ADC/DAC driver.
 * @details This file contains the low-level I2C communication functions for the
 * PCF8591T chip, as well as the high-level application logic and state
 * machines for a CDS (light) sensor and an NTC (temperature) sensor.
 */

/* Includes ------------------------------------------------------------------*/
#include "PCF8591T.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>


/* Private Defines -----------------------------------------------------------*/
#define CDS_SAMPLE_COUNT        3   // Number of samples to average for the CDS sensor.
#define TEMP_SAMPLE_COUNT       3   // Number of samples to average for the TEMP sensor.
#define ADC_CHANNEL_CDS         0   // PCF8591T channel for the CDS sensor.
#define ADC_CHANNEL_TEMP        1   // PCF8591T channel for the NTC thermistor.
#define SAMPLING_DELAY_TICKS    20  // Ticks to wait between samples (e.g., 20 * 10ms = 200ms).

/* Private Types -------------------------------------------------------------*/
typedef enum {
    CDS_STATE_IDLE,
    CDS_STATE_READ_WAIT,
    CDS_STATE_REPORT,
} CDSState;

typedef enum {
    TEMP_STATE_IDLE,
    TEMP_STATE_READ_WAIT,
    TEMP_STATE_REPORT,
} TEMPState;

/* External Handle Declarations ----------------------------------------------*/
extern I2C_HandleTypeDef hi2c1; // Defined in main.c

/*--- Private Variables: CDS Sensor ---*/
static CDSState cds_state = CDS_STATE_IDLE; /// Current state of the CDS sensor state machine.
static uint16_t cds_tick_count = 0;         /// Countdown timer for sampling delays.
static uint16_t cds_sample_sum = 0;         /// Sum of ADC samples for averaging.
static uint8_t cds_sample_count = 0;        /// Number of samples taken so far.
static volatile uint16_t cds_chk_result = 0;/// Stores the final averaged ADC value.
static volatile bool cds_chk_done = true;   /// Flag indicating if the check is complete.
static void (*cds_done_cb)(void) = NULL;    /// Callback function for when a check is done.

static uint16_t cds_avg_value = 0;
static uint16_t cds_last_value = 0;


/*--- Private Variables: Temperature Sensor ---*/
static TEMPState temp_state = TEMP_STATE_IDLE; /// Current state of the TEMP sensor state machine.
static uint16_t temp_tick_count = 0;           /// Countdown timer for sampling delays.
static uint16_t temp_sample_sum = 0;           /// Sum of ADC samples for averaging.
static uint8_t temp_sample_count = 0;          /// Number of samples taken so far.
static volatile float temp_chk_result = 0.0f;  /// Stores the final calculated temperature.
static volatile bool temp_chk_done = true;     /// Flag indicating if the check is complete.
static void (*temp_done_cb)(void) = NULL;      /// Callback function for when a check is done.

static float temp_avg_value = 0.0f;
static uint16_t temp_last_adc_value = 0;

/* Private Function Prototypes -----------------------------------------------*/
static float calculate_thermistor_resistance(uint8_t adc_val);
static float convert_adc_to_temp_steinhart(uint8_t adc_val);
static inline bool process_adc_sampling(uint16_t *tick_count, uint16_t *sample_sum, uint8_t *sample_count, uint8_t channel, uint8_t sample_max, uint16_t *last_value);
/* Low-Level PCF8591T Driver Functions ---------------------------------------*/
/**
 * @brief  Reads a single analog channel from the PCF8591T.
 * @param  ch: The channel number to read (0-3).
 * @retval The 8-bit digital value from the ADC.
 */
uint8_t read_pcf8591_channel(uint8_t ch)
{
    if (ch > 3) ch = 3;
    uint8_t ctrl = 0x40 | (ch & 0x03); // Auto-increment OFF, select channel
    uint8_t dummy, data;

    HAL_I2C_Master_Transmit(&hi2c1, PCF8591T_ADDR, &ctrl, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, PCF8591T_ADDR, &dummy, 1, HAL_MAX_DELAY); 
    HAL_I2C_Master_Receive(&hi2c1, PCF8591T_ADDR, &data, 1, HAL_MAX_DELAY);

    return data;
}

/**
 * @brief  Writes a value to the PCF8591T's DAC output.
 * @param  value: The 8-bit value to write to the DAC.
 */
void write_pcf8591_dac(uint8_t value) {
    uint8_t tx_buffer[2] = {0x40, value}; // Control byte: DAC enable
    HAL_I2C_Master_Transmit(&hi2c1, PCF8591T_ADDR, tx_buffer, 2, HAL_MAX_DELAY);
}
/* CDS Sensor Module Functions -----------------------------------------------*/

/**
 * @brief  Sets the callback for when a CDS check is completed.
 * @param  cb: Pointer to the callback function.
 */
void CDS_SetCheckDoneCallback(void (*cb)(void)) {
    cds_done_cb = cb;
}

/**
 * @brief  Processes a command to start a CDS sensor check.
 * @param  cmd: The command string (expects "cds chk").
 */
void CDS_ProcessCommand(const char *cmd) {
    if (strcmp(cmd, "cds chk") == 0) {
        if (cds_state != CDS_STATE_IDLE) {
            return; // Ignore command if busy
        }
        cds_sample_sum = 0;
        cds_sample_count = 0;
        cds_tick_count = SAMPLING_DELAY_TICKS;
        cds_state = CDS_STATE_READ_WAIT;
        cds_chk_done = false;
    }
}
/**
 * @brief  Periodic timer callback for the CDS sensor state machine.
 */
void CDS_TimerCallback(void) {
    switch (cds_state) {
        case CDS_STATE_READ_WAIT:
            if (process_adc_sampling(&cds_tick_count, &cds_sample_sum, &cds_sample_count, 0, CDS_SAMPLE_COUNT, &cds_last_value)) {
                    cds_avg_value = cds_sample_sum / CDS_SAMPLE_COUNT;
                    cds_state = CDS_STATE_REPORT;
                    write_pcf8591_dac(255);
            }
            break;

        case CDS_STATE_REPORT:
            write_pcf8591_dac(0);
            cds_chk_result = cds_avg_value;
            cds_chk_done = true;
            cds_state = CDS_STATE_IDLE;
            if (cds_done_cb) cds_done_cb();
            break;

        case CDS_STATE_IDLE:
        default:
            break;
    }
}

/**
 * @brief  Checks if the CDS sensor check is finished.
 * @retval true if the check is complete, false otherwise.
 */
bool CDS_IsCheckDone(void) {
    return cds_chk_done;
}

/**
 * @brief  Gets the result of the last CDS check.
 * @note   Resets the 'done' flag to false upon being called.
 * @retval The averaged ADC value (0-255).
 */
uint16_t CDS_GetCheckResult(void) {
    cds_chk_done = false;
    return cds_chk_result;
}

/* Temperature Sensor Module Functions ---------------------------------------*/

/**
 * @brief  Sets the callback for when a TEMP check is completed.
 * @param  cb: Pointer to the callback function.
 */
void TEMP_SetCheckDoneCallback(void (*cb)(void)) {
    temp_done_cb = cb;
}

/**
 * @brief Initializes the TEMP sensor module variables.
 */
void TEMP_Init(void) {
    temp_state = TEMP_STATE_IDLE;
    temp_sample_sum = 0;
    temp_sample_count = 0;
    temp_avg_value = 0;
}

/**
 * @brief  Processes a command to start a TEMP sensor check.
 * @param  cmd: The command string (expects "temp chk").
 */
void TEMP_ProcessCommand(const char *cmd) {
    if (strcmp(cmd, "temp chk") == 0) {
        if (temp_state != TEMP_STATE_IDLE) {
            return; // Ignore command if busy
        }
        temp_sample_sum = 0;
        temp_sample_count = 0;
        temp_tick_count = SAMPLING_DELAY_TICKS; // 200ms
        temp_state = TEMP_STATE_READ_WAIT;
    } 
}

/**
 * @brief  Periodic timer callback for the TEMP sensor state machine.
 */
void TEMP_TimerCallback(void) {
    switch (temp_state) {
        case TEMP_STATE_READ_WAIT:
            if (process_adc_sampling(&temp_tick_count, &temp_sample_sum, &temp_sample_count, 1, TEMP_SAMPLE_COUNT, &temp_last_adc_value)) {
                temp_avg_value = temp_sample_sum / TEMP_SAMPLE_COUNT;
                temp_state = TEMP_STATE_REPORT;
                write_pcf8591_dac(255);
            }
            break;
        case TEMP_STATE_REPORT:
            write_pcf8591_dac(0);            
            temp_chk_result =  convert_adc_to_temp_steinhart(temp_avg_value);
            temp_chk_done = true;
            temp_state = TEMP_STATE_IDLE;
            if (temp_done_cb) temp_done_cb(); 
            break;

        case TEMP_STATE_IDLE:
        default:
            break;
    }
}

/**
 * @brief  Checks if the TEMP sensor check is finished.
 * @retval true if the check is complete, false otherwise.
 */
bool TEMP_IsCheckDone(void) {
    return temp_chk_done;
}
/**
 * @brief  Gets the result of the last TEMP check.
 * @note   Resets the 'done' flag to false upon being called.
 * @retval The calculated temperature in degrees Celsius.
 */
float TEMP_GetCheckResult(void) {
    temp_chk_done = false;
    return temp_chk_result;
}



static inline bool process_adc_sampling(uint16_t *tick_count, uint16_t *sample_sum, uint8_t *sample_count, uint8_t channel, uint8_t sample_max, uint16_t *last_value) {
    if ((*tick_count) > 0) {
        (*tick_count)--;
        return false;
    }

    uint16_t adc_value = read_pcf8591_channel(channel);
    (*sample_sum) += adc_value;
    (*sample_count)++;

    if ((*sample_count) < sample_max) {
        (*tick_count) = 20;
        return false;
    } else {
        *last_value = adc_value;
        return true;
    }
}

/**
 * @brief  Calculates the resistance of the NTC thermistor.
 * @param  adc_val: The 8-bit ADC value from the voltage divider.
 * @retval The calculated resistance in Ohms.
 */
static float calculate_thermistor_resistance(uint8_t adc_val) {
    float voltage_out = ((float)adc_val / 255.0f) * VCC;
    if (voltage_out <= 0.001f) voltage_out = 0.001f;
    return R_PULLUP * (voltage_out / (VCC - voltage_out));
}

/**
 * @brief  Converts an ADC value to temperature using the Steinhart-Hart equation.
 * @param  adc_val: The 8-bit ADC value from the thermistor circuit.
 * @retval The calculated temperature in degrees Celsius.
 */
static float convert_adc_to_temp_steinhart(uint8_t adc_val) {
    float resistance = calculate_thermistor_resistance(adc_val);
    float lnR = logf(resistance);
    float inv_T = A_EFF + (B_EFF * lnR) + (C_EFF * lnR * lnR * lnR);
    return (1.0f / inv_T) - 273.15f;
}

