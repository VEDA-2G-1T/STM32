#include "PCF8591T.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define CDS_SAMPLE_COUNT 3
#define CDS_THRESHOLD_DAY   170

#define TEMP_SAMPLE_COUNT 3



extern I2C_HandleTypeDef hi2c1;

typedef enum {
	CDS_STATE_IDLE,
	CDS_STATE_READ_WAIT,
	CDS_STATE_REPORT,
	
	
}CDSState;

typedef enum {
    TEMP_STATE_IDLE,
    TEMP_STATE_READ_WAIT,
    TEMP_STATE_REPORT,
} TEMPState;


static CDSState cds_state = CDS_STATE_IDLE;
static uint16_t cds_tick_count = 0;
static uint16_t cds_sample_sum = 0;
static uint8_t cds_sample_count = 0;
static uint16_t cds_avg_value = 0;
static uint16_t cds_last_value = 0;


static TEMPState temp_state = TEMP_STATE_IDLE;
static uint16_t temp_tick_count = 0;
static uint16_t temp_sample_sum = 0;
static uint8_t temp_sample_count = 0;
static float temp_avg_value = 0.0f;
static uint16_t temp_last_adc_value = 0;


static volatile uint16_t cds_chk_result = 0;
static volatile bool cds_chk_done = true;

static volatile float temp_chk_result = 0.0f;
static volatile bool temp_chk_done = true;


static void (*cds_done_cb)(void) = NULL;

void CDS_SetCheckDoneCallback(void (*cb)(void)) {
    cds_done_cb = cb;
}

static void (*temp_done_cb)(void) = NULL;

void TEMP_SetCheckDoneCallback(void (*cb)(void)) {
    temp_done_cb = cb;
}



uint8_t read_pcf8591_channel(uint8_t ch)
{
    if (ch > 3) ch = 3;
    uint8_t ctrl = 0x40 | (ch & 0x03);
    uint8_t dummy, data;

    HAL_I2C_Master_Transmit(&hi2c1, PCF8591T_ADDR, &ctrl, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, PCF8591T_ADDR, &dummy, 1, HAL_MAX_DELAY); // ? ??? ??
    HAL_I2C_Master_Receive(&hi2c1, PCF8591T_ADDR, &data, 1, HAL_MAX_DELAY);

    return data;
}


void write_pcf8591_dac(uint8_t value)
{
    uint8_t control = 0x40;  // DAC enable (bit 6 = 1), analog output only
    uint8_t data[2] = { control, value };

    HAL_I2C_Master_Transmit(&hi2c1, PCF8591T_ADDR, data, 2, HAL_MAX_DELAY);
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
            // printf("[CDS] Avg: %d", cds_avg_value);
							cds_chk_result = cds_avg_value;
							cds_chk_done = true;
//						if (cds_avg_value <= CDS_THRESHOLD_DAY) {
//							printf(" DAY\r\n");
//						} else{
//							printf(" NIGHT\r\n");
//						}
            cds_state = CDS_STATE_IDLE;
					if (cds_done_cb) cds_done_cb();
            break;

        case CDS_STATE_IDLE:
        default:
            break;
    }
}


void CDS_ProcessCommand(const char *cmd) {
    if (strcmp(cmd, "cds chk") == 0) {
        if (cds_state != CDS_STATE_IDLE) {
           // printf("[WARN] CDS busy. Command ignored: %s\r\n", cmd);
            return;
        }
        cds_sample_sum = 0;
        cds_sample_count = 0;
        cds_tick_count = 20; // 200ms (assuming 10ms base tick)
        cds_state = CDS_STATE_READ_WAIT;
        //printf("[ACTION] CDS check start\r\n");
    } else {
        // printf("[ERROR] Unknown CDS command: %s\r\n", cmd);

    }
}


void TEMP_Init(void) {
    temp_state = TEMP_STATE_IDLE;
    temp_sample_sum = 0;
    temp_sample_count = 0;
    temp_avg_value = 0;
}
void TEMP_ProcessCommand(const char *cmd) {
    if (strcmp(cmd, "temp chk") == 0) {
        if (temp_state != TEMP_STATE_IDLE) {
           // printf("[WARN] TEMP busy. Command ignored: %s\r\n", cmd);
            return;
        }
        temp_sample_sum = 0;
        temp_sample_count = 0;
        temp_tick_count = 20; // 200ms
        temp_state = TEMP_STATE_READ_WAIT;
        // printf("[ACTION] TEMP check start\r\n");
    } else {
        // printf("[ERROR] Unknown TEMP command: %s\r\n", cmd);
    }
}



float calculate_thermistor_resistance(uint8_t adc_val) {
    float voltage_out = ((float)adc_val / 255.0f) * VCC;
    if (voltage_out <= 0.001f) voltage_out = 0.001f;
    return R_PULLUP * (voltage_out / (VCC - voltage_out));
}

float convert_adc_to_temp_steinhart(uint8_t adc_val) {
    float resistance = calculate_thermistor_resistance(adc_val);
    float lnR = logf(resistance);

    // MF58-103F3950 ??
//    const float A = 1.106836861e-03f;
//    const float B = 2.384641754e-04f;
//    const float C = 0.6507394466e-7f;
    const float A = A_EFF;
    const float B = B_EFF;
    const float C = C_EFF;
    float inv_T = A + B * lnR + C * lnR * lnR * lnR;
    return (1.0f / inv_T) - 273.15f;
}


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
            // printf("[TEMP] Avg Temp: %.2f C\r\n", convert_adc_to_temp_steinhart(temp_avg_value));
            
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


bool CDS_IsCheckDone(void)
	{
    return cds_chk_done;
}

uint16_t CDS_GetCheckResult(void) 
	{
    cds_chk_done = false;  
    return cds_chk_result;
}


bool TEMP_IsCheckDone(void)
	{
    return temp_chk_done;
}

float TEMP_GetCheckResult(void) 
	{
    temp_chk_done = false;  
    return temp_chk_result;
}

