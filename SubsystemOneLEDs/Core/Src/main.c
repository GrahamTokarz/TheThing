/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main body code for 'hand' motion
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SPI_HandleTypeDef hspi1;
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


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int mode = 0; //Receiver Output
int testingMode = 0;
float min1 = .096; //Minimum angle of servo 1
float max1 = .124; //Max angle of servo 1
float min2 = .096; //Minimum angle of servo 2
float max2 = .124; //Max angle of servo 2
int period = 19999; //PWM period for servos
float m1_dc = .035; //Initial position for servo 1
float m2_dc = .035; //Initial position for servo 2
float mod_1 = .0025; //Factor of change for servo 1
float mod_2 = -.0025; //Factor of change for servo 2
int m1_factor = 0; //Multiplicative factor to allow for stopping of motion of servo 1
int m2_factor = 0; //Multiplicative factor to allow for stopping of motion of servo 2
int lastState = 0; //Last receiver value
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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

void nRF24_SetTXMode() {
	uint8_t tx_addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

    CE_LOW();  // Ensure CE is low
    nRF24_WriteRegister(0x00, 0x0E);  // Set PWR_UP & PRIM_TX (CONFIG register)
    nRF24_WriteRegisterMulti(0x10, tx_addr, 5);  // Set TX address (must match RX)
    HAL_Delay(2);  // Wait for power-up
}

void nRF24_SendData(uint8_t *data, uint8_t length) {
    CE_LOW();
	nRF24_WritePayload(data, length);  // Load payload
    CE_HIGH();  // Start transmission
    HAL_Delay(1);  // Ensure minimum CE pulse width
    CE_LOW();  // Stop transmission
}


//Initialize Transmitter
void nRF24_Init() {
    CE_LOW();  // Disable radio
    CSN_HIGH();  // Deselect SPI

    nRF24_WriteRegister(0x00, 0x0A);  // Set PWR_UP, CRC enabled
    nRF24_WriteRegister(0x01, 0x3F);  // Enable Auto Acknowledgment
    nRF24_WriteRegister(0x02, 0x01);  // Enable only Pipe 0
    nRF24_WriteRegister(0x05, 0x02);  // Set RF Channel (Change as needed)
    nRF24_WriteRegister(0x06, 0x07);  // Set RF Power 0dBm, 1Mbps

    HAL_Delay(2);  // Wait for power-up
}

//Establish Transmitter as Receiver
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

//Stop motion
void noInput() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	m1_dc = .035;
	m2_dc = .035;
	mod_1 = .0025;
	mod_2 = .0025;
	m1_factor = 0;
	m2_factor = 0;
}

//'Wave' Left
void leftInput() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	m1_dc = .095;
	m2_dc = .095;
	mod_1 = .0;
	mod_2 = .0025;
	m1_factor = 0;
	m2_factor = 1;
}

//'Wave' Right
void rightInput() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	m1_dc = .095;
	m2_dc = .095;
	mod_1 = .0025;
	mod_2 = 0;
	m1_factor = 1;
	m2_factor = 0;
}

//Move Forward
void upInput() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 110);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	m1_dc = .035;
	m2_dc = .095;
	mod_1 = .0025;
	mod_2 = .0025;
	m1_factor = 0;
	m2_factor = 0;
}

//Move Backwards
void downInput() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 80);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	m1_dc = .035;
	m2_dc = .095;
	mod_1 = .0025;
	mod_2 = .0025;
	m1_factor = 0;
	m2_factor = 0;
}

//Receiver main loop
void nRF24_ReceiveData() {
    uint8_t rxData[1];

    if (nRF24_DataAvailable()) {
        nRF24_ReadPayload(rxData, 1);
        nRF24_WriteRegister(0x07, (1 << 6));  // Clear RX_DR

        char uart_buf[50];
        sprintf(uart_buf, "Received: %02X\n\r", rxData[0]);
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY); //Print received value to serial monitor

        if (rxData[0] != lastState) { //Don't call the same case multiple times in a row or no motion would occur
        	switch(rxData[0]) {
				case 0:
					noInput();
					break;
				case 1:
					leftInput();
					break;
				case 2:
					rightInput();
					break;
				case 3:
					upInput();
					break;
				case 4:
					downInput();
					break;
			}
        	lastState = rxData[0]; //Set last state
        }



    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Blue button callback to allow for motor testing without using transmitter
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	testingMode++;
	testingMode = testingMode % 2;

	if (testingMode == 0){
		noInput();
	} else {
		upInput();
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  nRF24_Init();  // Initialize nRF24L01
  nRF24_SetRXMode();  // Set RX Mode

  //Turn DC motor on, set to locked state
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 110);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  while (1)
  {
	  HAL_Delay(50);

	  //Set new values for servos based on duty cycle and PWM period
	  float pulse_1 = m1_dc * period;
	  float pulse_2 = m2_dc * period;
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int)pulse_1);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (int)pulse_2);

	  //Check if servo 1 has reached an endpoint
	  if (m1_dc >= max1) {
		  mod_1 = -0.0025;
	  }
	  if (m1_dc <= min1) {
		  mod_1 = 0.0025;
	  }

	  //Check if servo 2 has reached an endpoint
	  if (m2_dc >= max2) {
		  mod_2 = -0.0025;
	  }
	  if (m2_dc <= min2) {
		  mod_2 = 0.0025;
	  }

	  //Increment servo duty cycle for motion
	  m1_dc += mod_1 * m1_factor;
	  m2_dc += mod_2 * m2_factor;

	  nRF24_ReceiveData();  // Continuously check for incoming data

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
