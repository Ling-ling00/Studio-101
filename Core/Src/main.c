/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
uint16_t x_floor[5] = {0, 0, 0, 0, 0};
uint16_t z_floor[5] = {0, 0, 0, 0, 0};
uint8_t RxBuffer[10];
uint8_t sensor1 = 0;
uint8_t sensor2 = 0;
uint8_t motor1_INA = 0;
uint8_t motor1_INB = 0;
uint16_t motor1_PWM = 50;
uint16_t z_current_position = 0;
uint16_t z_target_position = 0;
uint16_t x_target_position = 0;
int16_t z_diff_position = 0;
float target_position = 0;
typedef struct
{
	// for record New / Old value to calculate dx / dt
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float QEIPostion_1turn;
	float QEIAngularVelocity;
}QEI_StructureTypeDef;
QEI_StructureTypeDef QEIdata = {0};
uint64_t _micros = 0;
enum
{
	NEW,OLD
};
float velocity = 0;
float position = 0;
float angular_position;
float angular_velocity;
int position_round = 0;
int counter = 0;
int n = 0;
int mode = 0;

//trajectory
float trajec_position;
float trajec_velocity;
float trajec_acceleration;
float trajec_target;
uint8_t trajec_state;
float p0;

//position PID
float position_Kp = 0;
float position_Ki = 0;
float position_Kd = 0;
float position_Ts = 0.005;
float position_PID_output = 0;

//velocity PID
float velocity_Kp = 0.008;
float velocity_Ki = 0.005;
float velocity_Kd = 0.00001;
float velocity_Ts = 0.001;
float velocity_PID_output = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void z_transition();
void UARTDMAConfig();
void update_position();
uint64_t micros();
void QEIEncoderPosVel_Update();

void setMotor();
//trajectory
void trajectory();

//PID
void velocity_PID();
void position_PID();
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  UARTDMAConfig();
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  z_transition();
//	  update_position();
	  sensor1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	  sensor2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
		  mode = 1;
		  target_position = 600;
	  }
	  else{
		  target_position = 0;
	  }
//	  if(mode == 1){
//		  if(sensor1 == 0 && n == 0){
//			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
//			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999*4/5.0);
//		  }
//		  else if(sensor1 == 1){
//			  n = 1;
//		  }
//		  else if(sensor2 == 0 && n == 1){
//			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
//			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999*4/5.0);
//		  }
//		  else if(sensor2 == 1){
//			  mode = 0;
//			  n = 0;
//			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
//			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//		  }
//	  }

	  trajectory();

	  static uint64_t timestamp_velocity_PID = 0;
	  uint64_t currentTime = micros();
	  if(currentTime > timestamp_velocity_PID)
	  {
		  timestamp_velocity_PID = currentTime + 1000;//us
		  QEIEncoderPosVel_Update();
		  velocity = angular_velocity*14/2.0/M_PI;
		  velocity_PID();
		  setMotor();
	  }

	  static uint64_t timestamp_position_PID = 0;
	  currentTime = micros();
	  if(currentTime > timestamp_position_PID)
	  {
		  timestamp_position_PID = currentTime + 5000;//us
		  position_PID();
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 63487;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4.294967295E9;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
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
void z_transition()
{
	if (z_diff_position > 0){
		if (sensor1 == 0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)(motor1_PWM/100.0*999));
		}
		else if (sensor1 == 1){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			z_target_position = 680;
			z_current_position = 680;
		}
	}
	else if (z_diff_position < 0){
		if (sensor2 == 0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)(motor1_PWM/100.0*999));
		}
		else if (sensor2 == 1){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			z_target_position = 80;
			z_current_position = 80;
		}
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	z_diff_position = z_target_position - position;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
		{
			//(for string only) Add string stop symbol \0 to end string
			RxBuffer[9] = '\0';
		}
}

void UARTDMAConfig()
{
	//start UART in DMA mode
	HAL_UART_Receive_DMA(&huart1, RxBuffer,9);
}
update_position()
{
	int a = 0;
	for(int i = 0; i<= 8; i++){
		if (i == '\n'){
			a = i;
			break;
		}
	}
	x_target_position = (RxBuffer[(a+1)%9]-'0')*100+(RxBuffer[(a+2)%9]-'0')*10+(RxBuffer[(a+3)%9]-'0');
	z_target_position = (RxBuffer[(a+4)%9]-'0')*100+(RxBuffer[(a+5)%9]-'0')*10+(RxBuffer[(a+6)%9]-'0');
	x_floor[RxBuffer[(a+7)%9]-'0'-1] = x_target_position;
	z_floor[RxBuffer[(a+7)%9]-'0'-1] = z_target_position;
}

//MicroSecondTimer Implement
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		_micros += UINT32_MAX;
	}
}

uint64_t micros()
{
	return __HAL_TIM_GET_COUNTER(&htim5)+_micros;
}

void QEIEncoderPosVel_Update()
{
	//collect data
	QEIdata.TimeStamp[NEW] = micros();
	QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim4);
	//Postion 1 turn calculation
	QEIdata.QEIPostion_1turn = QEIdata.Position[NEW] % 2048;
	//calculate dx
	int32_t diffPosition = QEIdata.Position[NEW] - QEIdata.Position[OLD];
	//Handle Warp around
	if(diffPosition > 31744){
		diffPosition -=63488;
		counter--;
	}
	if(diffPosition < -31744){
		diffPosition +=63488;
		counter++;
	}
	//calculate dt
	float diffTime = (QEIdata.TimeStamp[NEW]-QEIdata.TimeStamp[OLD]) * 0.000001;
	//calculate anglar velocity
	QEIdata.QEIAngularVelocity = diffPosition / diffTime;

	angular_velocity = QEIdata.QEIAngularVelocity/2048.0*2*M_PI;
	angular_position = (QEIdata.Position[NEW]%2048)/2048.0*2*M_PI;
	position_round = (counter*31)+(int)(QEIdata.Position[NEW]/2048.0);
	position = ((angular_position)/(2.0*M_PI)*14)+(14*position_round);
	//store value for next loop
	QEIdata.Position[OLD] = QEIdata.Position[NEW];
	QEIdata.TimeStamp[OLD]=QEIdata.TimeStamp[NEW];
}

void trajectory(){
	static uint32_t Timestamp;
	if(trajec_target != 0 && trajec_state == 0){
		trajec_state = 1;
		Timestamp = HAL_GetTick();
	}
	else if(trajec_state == 1 && trajec_target >= 0){
		float t = (HAL_GetTick() - Timestamp)*0.001;
		float time = (-100 + sqrt(10000 + (2000*trajec_target)))/1000;
		if(HAL_GetTick() - Timestamp <= (time*1000)){
			trajec_acceleration = 500.0;
			trajec_velocity = 500*t;
			trajec_position = (250*t*t)+p0;
		}
		else if(HAL_GetTick() - Timestamp <= ((time+0.2)*1000)){
			trajec_acceleration = 0;
			trajec_velocity = 500*time;
			trajec_position = ((500*time*(t-time))+(250*time*time))+p0;
		}
		else if(HAL_GetTick() - Timestamp <= (((time*2)+0.2)*1000)){
			trajec_acceleration = -500.0;
			trajec_velocity = (-500*(t-time-0.2))+(500*time);
			trajec_position = ((-250*(t-time-0.2)*(t-time-0.2))+(500*time*(t-time-0.2))+(250*time*time)+(100*time))+p0;
		}
		else{
			trajec_acceleration = 0;
//			trajec_velocity = 0;
//			trajec_position = 0;
			trajec_target = 0;
			trajec_state = 0;
		}
	}
	else if(trajec_state == 1 && trajec_target < 0){
		float t = (HAL_GetTick() - Timestamp)*0.001;
		float time = (-100 + sqrt(10000 + (-2000*trajec_target)))/1000;
		if(HAL_GetTick() - Timestamp <= (time*1000)){
			trajec_acceleration = -500.0;
			trajec_velocity = (500*t)*-1;
			trajec_position = ((250*t*t)*-1)+p0;
		}
		else if(HAL_GetTick() - Timestamp <= ((time+0.2)*1000)){
			trajec_acceleration = 0;
			trajec_velocity = (500*time)*-1;
			trajec_position = (((500*time*(t-time))+(250*time*time))*-1)+p0;
		}
		else if(HAL_GetTick() - Timestamp <= (((time*2)+0.2)*1000)){
			trajec_acceleration = 500.0;
			trajec_velocity = ((-500*(t-time-0.2))+(500*time))*-1;
			trajec_position = (((-250*(t-time-0.2)*(t-time-0.2))+(500*time*(t-time-0.2))+(250*time*time)+(100*time))*-1)+p0;
		}
		else{
			trajec_acceleration = 0;
//			trajec_velocity = 0;
//			trajec_position = 0;
			trajec_target = 0;
			trajec_state = 0;
		}
	}
	else if(trajec_state == 0 && trajec_target == 0){
		trajec_target = target_position-trajec_position;
		p0 = trajec_position;
	}
}

void velocity_PID(){
	static float u_n;
	static float u_n1 = 0;
	static float u_n2 = 0;
	static float y_n;
	static float y_n1 = 0;
	float one = (2*velocity_Ts*velocity_Kp)+(velocity_Ki*velocity_Ts*velocity_Ts)+(2*velocity_Kd);
	float two = (-2*velocity_Ts*velocity_Kp)+(velocity_Ki*velocity_Ts*velocity_Ts)-(4*velocity_Kd);
	float three = 2*velocity_Kd;
	float four = 2*velocity_Ts;
	u_n = trajec_velocity + position_PID_output - velocity;
	y_n = ((one*u_n)+(two*u_n1)+(three*u_n2)+(four*y_n1))/four;

	velocity_PID_output += y_n;
	u_n2 = u_n1;
	u_n1 = u_n;
	y_n1 = y_n;
}

void position_PID(){
	static float u_n;
	static float u_n1 = 0;
	static float u_n2 = 0;
	static float y_n;
	static float y_n1 = 0;
	float one = (2*position_Ts*position_Kp)+(position_Ki*position_Ts*position_Ts)+(2*position_Kd);
	float two = (-2*position_Ts*position_Kp)+(position_Ki*position_Ts*position_Ts)-(4*position_Kd);
	float three = 2*position_Kd;
	float four = 2*position_Ts;
	u_n = trajec_position - position;
	y_n = ((one*u_n)+(two*u_n1)+(three*u_n2)+(four*y_n1))/four;

	position_PID_output += y_n;
	u_n2 = u_n1;
	u_n1 = u_n;
	y_n1 = y_n;
}

void setMotor()
{
	if(velocity_PID_output > 24){
		velocity_PID_output = 24;
	}
	else if(velocity_PID_output < -24){
		velocity_PID_output = -24;
	}
	if(velocity_PID_output > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)velocity_PID_output*999/24.0);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)velocity_PID_output*(-999)/24.0);
	}
}
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
