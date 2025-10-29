/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main body code for joystick transmitter
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
#include "stdlib.h"
#include <stdio.h>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NRF24_CMD_R_REGISTER 0x00
#define NRF24_CMD_W_REGISTER 0x20


#define CSN_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_6

#define CE_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_8

#define CSN_HIGH() HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET)
#define CSN_LOW()  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET)
#define CE_HIGH()  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET)
#define CE_LOW()   HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET)

#define BLUE_LOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)
#define BLUE_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)
#define GREEN_LOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
#define GREEN_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define YELLOW_LOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)
#define YELLOW_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)
#define RED_LOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)
#define RED_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET) //Legacy helper functions from subsystem 2, here for reference if needed.

/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
int16_t UD = 0;
int16_t LR = 0;
char uart_buf[50];
int uart_buf_len;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

//SPI Transmission Helper Function
uint8_t SPI_Transfer(uint8_t data) {
    uint8_t received = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &received, 1, HAL_MAX_DELAY);
    char debug_buf[30];
//    sprintf(debug_buf, "SPI Sent: %02X, Got: %02X\n", data, received);
//    HAL_UART_Transmit(&huart2, (uint8_t *)debug_buf, strlen(debug_buf), HAL_MAX_DELAY);
    return received;
}

void nRF24_WriteRegister(uint8_t reg, uint8_t value) {
    CSN_LOW();  // Select the nRF24L01
    SPI_Transfer(0x20 | (reg & 0x1F));  // Write command (0x20 + Register Address)
    SPI_Transfer(value);  // Send data
    CSN_HIGH(); // Deselect
}

void nRF24_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length) {
    CSN_LOW();  // Select nRF24L01
    SPI_Transfer(0x20 | (reg & 0x1F));  // Write command
    HAL_SPI_Transmit(&hspi1, data, length, HAL_MAX_DELAY);
    CSN_HIGH();  // Deselect
}

uint8_t nRF24_ReadRegister(uint8_t reg) {
    CSN_LOW();  // Select the nRF24L01
    SPI_Transfer(reg & 0x1F);  // Read command (0x00 + Register Address)
    uint8_t value = SPI_Transfer(0xFF);  // Receive data
    CSN_HIGH();  // Deselect
    return value;
}

void nRF24_WritePayload(uint8_t *data, uint8_t length) {
    CSN_LOW();  // Select the nRF24L01
    SPI_Transfer(0xA0);  // Write Payload Command
    HAL_SPI_Transmit(&hspi1, data, length, HAL_MAX_DELAY);
    CSN_HIGH();  // Deselect
}

void nRF24_ReadPayload(uint8_t *data, uint8_t length) {
    CSN_LOW();  // Select nRF24L01
    SPI_Transfer(0x61);  // Read Payload Command
    HAL_SPI_Receive(&hspi1, data, length, HAL_MAX_DELAY);
    CSN_HIGH();  // Deselect
}

//Initialize transmitter as sender
void nRF24_SetTXMode() {
	uint8_t tx_addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

    CE_LOW();  // Ensure CE is low
    nRF24_WriteRegister(0x00, 0x0E);  // Set PWR_UP & PRIM_TX (CONFIG register)
    nRF24_WriteRegisterMulti(0x10, tx_addr, 5);  // Set TX address (must match RX)
    HAL_Delay(2);  // Wait for power-up
}

//Clear send queue
void nRF24_FlushTX() {
    CSN_LOW();           // Select the nRF24L01
    HAL_SPI_Transmit(&hspi1, 0xE1, 1, HAL_MAX_DELAY);  // Send command
    CSN_HIGH();          // Deselect
}

//Transmitter main loop
void nRF24_SendData(uint8_t *data, uint8_t length) {
    CE_LOW();
    nRF24_WritePayload(data, length);  // Load payload
    CE_HIGH();  // Start transmission
    HAL_Delay(1);  // Ensure minimum CE pulse width
    CE_LOW();  // Stop transmission

    // Check if transmission failed (MAX_RT reached)
    uint8_t status = nRF24_ReadRegister(0x07);
    if (status & (1 << 4)) {
        nRF24_WriteRegister(0x07, (1 << 4));  // Clear MAX_RT flag
        nRF24_FlushTX();  // Flush TX FIFO to reset stuck transmission
        return;
    }

    // Check if data was sent successfully (ACK received)
    if (status & (1 << 5)) {
        nRF24_WriteRegister(0x07, (1 << 5));  // Clear TX_DS flag
    }
}

//Initialize Transmitter
void nRF24_Init() {
    CE_LOW();  // Disable radio
    CSN_HIGH();  // Deselect SPI

    nRF24_WriteRegister(0x00, 0x0A);  // Set PWR_UP, CRC enabled
    nRF24_WriteRegister(0x01, 0x01);  // Enable Auto Acknowledgment
    nRF24_WriteRegister(0x02, 0x01);  // Enable only Pipe 0
    nRF24_WriteRegister(0x05, 0x02);  // Set RF Channel (Change as needed)
    nRF24_WriteRegister(0x06, 0x07);  // Set RF Power 0dBm, 1Mbps

    HAL_Delay(2);  // Wait for power-up
}

void nRF24_SetRXMode() {
    uint8_t rx_addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };  // Must match TX

    CE_LOW();
    nRF24_WriteRegister(0x00, 0x0F);  // PWR_UP | PRIM_RX
    nRF24_WriteRegister(0x02, 0x01);  // Enable Pipe 0
    nRF24_WriteRegisterMulti(0x0A, rx_addr, 5);  // Set RX address
    nRF24_WriteRegister(0x11, 1);  // Set static payload length
    nRF24_WriteRegister(0x1C, 0x00);  // Disable dynamic payload
    nRF24_WriteRegister(0x1D, 0x00);  // Disable features
    HAL_Delay(2);
    CE_HIGH();  // Enable RX mode
}

uint8_t nRF24_DataAvailable() {
    uint8_t status = nRF24_ReadRegister(0x07);  // Read STATUS register
    if (status & (1 << 6)) {  // RX_DR is set
        nRF24_WriteRegister(0x07, (1 << 6));  // Clear RX_DR flag
        return 1;  // Data available
    }
    return 0;  // No data
}
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
 /* Configure the peripherals common clocks */
 PeriphCommonClock_Config();
 /* USER CODE BEGIN SysInit */
 /* USER CODE END SysInit */
 /* Initialize all configured peripherals */
 MX_GPIO_Init();
 MX_USART2_UART_Init();
 MX_ADC1_Init();
 MX_ADC2_Init();
 MX_SPI1_Init();
 /* USER CODE BEGIN 2 */
 /* USER CODE END 2 */
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */



 while (1)
 {
   /* USER CODE END WHILE */
   /* USER CODE BEGIN 3 */
	  nRF24_Init();  // Initialize nRF24L01
	  nRF24_SetTXMode();  // Set TX Mode
	  HAL_Delay(25);

	  //Use ADC to get readings from joystick
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  UD = HAL_ADC_GetValue(&hadc1);
	  LR = HAL_ADC_GetValue(&hadc2);

	  //Calculate values for up / down / left / right states
	  UD = UD - 2048;
	  LR = LR - 2048;
	  int deadzone = 1100;
	  int val;

	  if (abs(UD) < deadzone && abs(LR) < deadzone) {
		  val = 0; //If all values in deadzone, stop motion
	  } else if (abs(UD) > abs(LR)) { //If up/down has a higher variance, send the corresponding value
		  if (UD > 0) {		//UP
			  val = 3;
		  } else if (UD < 0) {	//DOWN
			  val = 4;
		  }
	  } else { //If left/right has a higher variance, send the corresponding value
		  if (LR > 0) {		//RIGHT
			  val = 2;
		  } else if (LR < 0) {		//LEFT
			  val = 1;
		  }
	  }

	  uint8_t txData[1] = {val};

	  uint8_t config = nRF24_ReadRegister(0x00);

//	  HAL_Delay(25);
	  //Send current state
	  nRF24_SendData(txData, 1);

	  //Print joystick values to serial monitor
	  uart_buf_len= sprintf(uart_buf, "Up/Down: %d and Left/Right: %d -- VAL %d\r\n", UD,LR,val,config);
	  HAL_UART_Transmit(&huart2,(uint8_t*)uart_buf, uart_buf_len, 100);
	  //Print config debug to serial monitor
	  sprintf(uart_buf, "CONFIG: %02X\n\r", config);
	  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
//
//	  HAL_Delay(50);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
