/**
 * @file    pcf8591t.h
 * @brief   Header file for the PCF8591T I2C ADC/DAC driver.
 * @details This file provides low-level functions to interface with the PCF8591T chip.
 * It also includes the application-level interface for a CDS (light) sensor
 * and an NTC (temperature) sensor connected to the chip's analog inputs.
 */

#ifndef PCF8591T_H
#define PCF8591T_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"       // Main project header for MCU-specific definitions.
#include <stdbool.h>    // Standard library for boolean type (true, false).

/* Hardware and Chip Definitions ---------------------------------------------*/

/**
 * @brief PCF8591T I2C device address.
 * @note  The 7-bit address is 0x48. This is left-shifted by 1 to form
 * the 8-bit address format required by some HAL libraries.
 */
#define PCF8591T_ADDR (0x48 << 1)

/**
 * @brief Analog Input Channel Mapping for PCF8591T
 *
 * - **AIN0:** CDS Photoresistor (Model: 5537) for light sensing.
 * Value ranges from ~0 (bright) to ~255 (dark).
 * - **AIN1:** NTC Thermistor (Model: MF58-103F3950) for temperature sensing.
 * - **AIN2:** (Not Used)
 * - **AIN3:** Potentiometer / Variable Resistor.
 */

/* NTC Thermistor Calculation Constants --------------------------------------*/
#define VCC                 5.0f    // ADC reference voltage.
#define R_PULLUP            1000.0f // Pull-up resistor value in Ohms (1kOhm).
#define R0                  10000.0f// Nominal resistance of the NTC at T0 (10kOhm @ 25°C).
#define BETA                3950.0f // Beta coefficient of the NTC thermistor.
#define T0_K                298.15f // Nominal temperature (25°C) in Kelvin.

// Steinhart-Hart equation coefficients for more precise temperature calculation.
#define A_EFF               1.106836861e-03f
#define B_EFF               2.384641754e-04f
#define C_EFF               0.6507394466e-7f

/* Low-Level PCF8591T Driver Functions ---------------------------------------*/

/**
 * @brief Reads a single analog channel from the PCF8591T.
 * @param ch The channel number to read (0-3).
 * @return The 8-bit digital value from the ADC.
 */
uint8_t read_pcf8591_channel(uint8_t ch);

/**
 * @brief Reads multiple analog channels sequentially.
 * @param data Pointer to a buffer to store the read values.
 * @param count The number of channels to read.
 */
void read_pcf8591_channels(uint8_t *data, uint8_t count);

/**
 * @brief Writes a value to the PCF8591T's DAC output.
 * @param value The 8-bit value to write to the DAC.
 */
void write_pcf8591_dac(uint8_t value);


/* CDS Sensor Module Functions -----------------------------------------------*/

/**
 * @brief Sets a callback function for when the CDS sensor check is done.
 * @param cb Pointer to the callback function.
 */
void CDS_SetCheckDoneCallback(void (*cb)(void));

/**
 * @brief Initializes the CDS sensor module.
 */
void CDS_Init(void);

/**
 * @brief Processes a string command for the CDS sensor.
 * @param cmd The command string.
 */
void CDS_ProcessCommand(const char *cmd);

/**
 * @brief Periodic timer callback for the CDS sensor state machine.
 */
void CDS_TimerCallback(void);

/**
 * @brief Checks if the CDS sensor check is complete.
 * @return `true` if the check is done, `false` otherwise.
 */
bool CDS_IsCheckDone(void);

/**
 * @brief Gets the result of the CDS sensor check.
 * @return The result data from the check (e.g., raw ADC value).
 */
uint16_t CDS_GetCheckResult(void);


/* Temperature Sensor Module Functions ---------------------------------------*/

/**
 * @brief Sets a callback function for when the TEMP sensor check is done.
 * @param cb Pointer to the callback function.
 */
void TEMP_SetCheckDoneCallback(void (*cb)(void));

/**
 * @brief Initializes the TEMP sensor module.
 */
void TEMP_Init(void);

/**
 * @brief Processes a string command for the TEMP sensor.
 * @param cmd The command string.
 */
void TEMP_ProcessCommand(const char *cmd);

/**
 * @brief Periodic timer callback for the TEMP sensor state machine.
 */
void TEMP_TimerCallback(void);

/**
 * @brief Checks if the TEMP sensor check is complete.
 * @return `true` if the check is done, `false` otherwise.
 */
bool TEMP_IsCheckDone(void);

/**
 * @brief Gets the result of the TEMP sensor check.
 * @return The calculated temperature in degrees Celsius.
 */
float TEMP_GetCheckResult(void);


#endif /* PCF8591T_H */