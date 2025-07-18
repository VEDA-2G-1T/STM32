/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "LED.h"
#include "BUZZER.h"
#include "PCF8591T.h"

#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    STATE_IDLE,
    STATE_PARSE,
    STATE_EXECUTE,
    STATE_SEND_RESPONSE
} SystemState;

typedef void (*CommandHandler)(uint8_t *data, uint16_t length);
typedef struct {
    uint8_t command;
    CommandHandler handler;
} CommandEntry;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE 64
#define PCF8591_ADDR (0x48 << 1) // HAL I2C ?? (7?? << 1)

#define DMA_RX_BUF_SIZE 128
#define TX_BUF_SIZE 128
#define CMD_TOGGLE 0x10
#define CMD_CHK    0x11
#define CMD_IWDG_REQ 0x12

#define SOF_BYTE       0x7E
#define CMD_TOGGLE     0x10
#define CMD_CHK        0x11
#define TYPE_REQ       0x00
#define TYPE_RSP       0x01
#define MAX_FRAME_LEN  64


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint8_t dmaRxBuf[DMA_RX_BUF_SIZE];
uint8_t txBuf[TX_BUF_SIZE];
volatile uint16_t dmaRxHead = 0;

static bool chk_led_done = false;
static bool chk_buzzer_done = false;
static bool chk_temp_done = false;
static bool chk_cds_done = false;
static uint8_t chk_seq_pending;
static bool chk_pending = false;

// RX state
static uint8_t  rx_buf[MAX_FRAME_LEN];
static uint8_t  rx_len, rx_pos;
static enum { ST_WAIT_SOF, ST_WAIT_LEN, ST_WAIT_BODY } rx_state = ST_WAIT_SOF;
// TX sequence counter
static uint8_t tx_seq = 0;


volatile bool system_healthy = false;
volatile bool iwdg_reset_requested = false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void process_command(const char *cmd);


void handle_toggle(uint8_t *data, uint16_t length);
void handle_chk(uint8_t *data, uint16_t length);
void handle_iwdg_req(uint8_t *data, uint16_t length);
void process_received_data(uint8_t *data, uint16_t length);
void start_uart_dma_rx(void);
static void send_frame(uint8_t cmd, uint8_t seq, uint8_t type,
                       const uint8_t *data, uint8_t data_len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
	uint8_t temp[1] ={ch};
	
	 //HAL_UART_Transmit (&huart2, temp,1,2);
	//HAL_UART_Transmit_DMA(&huart2, &temp, 1);
	return (ch);
}
void try_send_chk_rsp(void) {
    if (!chk_pending) return;
    if (!(chk_led_done && chk_buzzer_done && chk_temp_done && chk_cds_done)) return;

    uint8_t payload[1+1+2+4];
    uint8_t *p = payload;
    *p++ = LED_GetCheckResult();
    *p++ = BUZZER_GetCheckResult();
    uint16_t cds = CDS_GetCheckResult();
    *p++ = cds & 0xFF;
    *p++ = (cds >> 8) & 0xFF;
    float temp = TEMP_GetCheckResult();
    memcpy(p, &temp, sizeof(temp));

    send_frame(CMD_CHK, chk_seq_pending, TYPE_RSP, payload, sizeof(payload));
    chk_pending = false;
}

void on_chk_led_done(void)    { chk_led_done = true; try_send_chk_rsp(); }
void on_chk_buzzer_done(void) { chk_buzzer_done = true; try_send_chk_rsp(); }
void on_chk_temp_done(void)   { chk_temp_done = true; try_send_chk_rsp(); }
void on_chk_cds_done(void)    { chk_cds_done = true; try_send_chk_rsp(); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
				
        CDS_TimerCallback();
        TEMP_TimerCallback();
        LED_TimerCallback();
        BUZZER_TimerCallback();
        if (!iwdg_reset_requested) {
            system_healthy = true;
        }

    }
}


CommandEntry cmd_table[] = {
    { CMD_TOGGLE, handle_toggle },
    { CMD_CHK, handle_chk },
		 { CMD_IWDG_REQ, handle_iwdg_req } 
};

void handle_toggle(uint8_t *data, uint16_t length) {
    LED_ProcessCommand("led toggle");
    BUZZER_ProcessCommand("buzz sweep");
	
	  uint8_t seq = rx_buf[1];
    uint8_t ack[] = { 0x01 };
    send_frame(CMD_TOGGLE, seq, TYPE_RSP, ack, sizeof(ack));
		
}


void handle_chk(uint8_t *data, uint16_t length) {
    chk_led_done = chk_buzzer_done = chk_temp_done = chk_cds_done = false;

    LED_SetCheckDoneCallback(on_chk_led_done);
    BUZZER_SetCheckDoneCallback(on_chk_buzzer_done);
    TEMP_SetCheckDoneCallback(on_chk_temp_done);
    CDS_SetCheckDoneCallback(on_chk_cds_done);

    LED_ProcessCommand("led chk");
    BUZZER_ProcessCommand("buzz chk");
    TEMP_ProcessCommand("temp chk");
    CDS_ProcessCommand("cds chk");

    chk_seq_pending = rx_buf[1];
    chk_pending = true;
}

void handle_iwdg_req(uint8_t *data, uint16_t length) {
    iwdg_reset_requested = true;
    system_healthy = false;
	
	  uint8_t seq = rx_buf[1];  // ??? ??? ?? ??
    uint8_t ack[] = { 0x01 }; // ?? ?? ??? ("OK")
    send_frame(CMD_IWDG_REQ, seq, TYPE_RSP, ack, sizeof(ack));
		
		
}


void process_received_data(uint8_t *data, uint16_t length) {
    if (length < 1) return;
    uint8_t cmd = data[0];
    for (int i = 0; i < sizeof(cmd_table)/sizeof(cmd_table[0]); i++) {
        if (cmd_table[i].command == cmd) {
            cmd_table[i].handler(&data[1], length - 1);
            return;
        }
    }
    snprintf((char *)txBuf, TX_BUF_SIZE, "UNKNOWN CMD: 0x%02X\r\n", cmd);
    HAL_UART_Transmit_DMA(&huart2, txBuf, strlen((char *)txBuf));
}

void start_uart_dma_rx(void) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, dmaRxBuf, DMA_RX_BUF_SIZE);
}

static uint16_t calc_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; ++b) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

// ??? ??
static void send_frame(uint8_t cmd, uint8_t seq, uint8_t type,
                       const uint8_t *data, uint8_t data_len)
{
    uint8_t frame[1+1+1+1+1+data_len+2];
    uint8_t *p = frame;

    *p++ = SOF_BYTE;
    *p++ = (1+1+1+data_len+2);  // LEN = CMD+SEQ+TYPE+DATA+CRC
    *p++ = cmd;
    *p++ = seq;
    *p++ = type;
    if (data_len) {
        memcpy(p, data, data_len);
        p += data_len;
    }
    uint16_t crc = calc_crc16(&frame[2], 3 + data_len);
    *p++ = crc & 0xFF;
    *p++ = (crc >> 8) & 0xFF;

    HAL_UART_Transmit_DMA(&huart2, frame, p - frame);
}


// ??? ?? ??
void parse_byte(uint8_t b) {
    switch (rx_state) {
    case ST_WAIT_SOF:
        if (b == SOF_BYTE) {
            rx_state = ST_WAIT_LEN;
        }
        break;
    case ST_WAIT_LEN:
        rx_len  = b;
        rx_pos  = 0;
        rx_state = ST_WAIT_BODY;
        break;
    case ST_WAIT_BODY:
        if (rx_pos < rx_len && rx_pos < MAX_FRAME_LEN) {
            rx_buf[rx_pos++] = b;
        }
        if (rx_pos == rx_len) {
            // ?? ??
            uint16_t recv_crc = rx_buf[rx_len-2] | (rx_buf[rx_len-1] << 8);
            uint16_t calc     = calc_crc16(rx_buf, rx_len-2);
            if (recv_crc == calc) {
                uint8_t cmd  = rx_buf[0];
                uint8_t seq  = rx_buf[1];
                uint8_t type = rx_buf[2];
                uint8_t *data = &rx_buf[3];
                uint8_t data_len = rx_len - (1+1+1+2);

                if (type == TYPE_REQ) {
                    // ?? ??
                    if (cmd == CMD_TOGGLE) {
                        handle_toggle(data, data_len);
                    } else if (cmd == CMD_CHK) {
                        handle_chk(data, data_len);
                    } else if (cmd == CMD_IWDG_REQ){
												handle_iwdg_req(data, data_len);
										}
                }
            }
            // ?? ???
            rx_state = ST_WAIT_SOF;
        }
        break;
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	// printf("STM32 ready. Type 'hi' from PC.\r\n");
	
	LED_Init();
	BUZZER_Init();
	
	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	 start_uart_dma_rx();  


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (iwdg_reset_requested) {
        // IWDG ?? ?? ??: ?? ????? ??
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(200);
    } else {
        // ?? ??: ?? ??? + IWDG Refresh
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);

        if (system_healthy) {
            HAL_IWDG_Refresh(&hiwdg);
            system_healthy = false;
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 249;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_INPUT_Pin */
  GPIO_InitStruct.Pin = LED_RED_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LED_RED_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
