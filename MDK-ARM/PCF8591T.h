// pcf8591t.h
#ifndef PCF8591T_H
#define PCF8591T_H

#include "main.h"
#include <stdbool.h>

#define PCF8591T_ADDR (0x48 << 1)

#define VCC         5.0f     
#define R_PULLUP    1000.0f   // ??? ???? 1kO
#define R0          10000.0f  // MF58-103F3950 ?? ??? (25??? 10kO)
#define BETA        3950.0f   
#define T0_K        298.15f   // 25? in Kelvin
#define A_EFF				1.106836861e-03f
#define B_EFF				2.384641754e-04f
#define C_EFF				0.6507394466e-7f

/*
1. AIN0 :	CDS sensor (ex : 0 : bright  , 255 : dark							photoresistor 5537
2. AIN1 : TEMP sensor				// R : 2.4 kohm											MF58-103F3950 model
3. AIN3 : variable resistor 
*/

uint8_t read_pcf8591_channel(uint8_t ch);
void read_pcf8591_channels(uint8_t *data, uint8_t count);
void write_pcf8591_dac(uint8_t value);

void CDS_SetCheckDoneCallback(void (*cb)(void));
void TEMP_SetCheckDoneCallback(void (*cb)(void));

void CDS_Init(void);
void CDS_ProcessCommand(const char *cmd);
void CDS_TimerCallback(void);



void TEMP_Init(void);
void TEMP_ProcessCommand(const char *cmd);
void TEMP_TimerCallback(void);


bool CDS_IsCheckDone(void);
uint16_t CDS_GetCheckResult(void);


bool TEMP_IsCheckDone(void);
float TEMP_GetCheckResult(void);

#endif












