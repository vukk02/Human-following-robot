/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define R 0.12
#define r 0.0325
#define pi 3.14159
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t Rx_buffer[11],Tx_buffer[10],timer_cnt = 10,cnt = 0;
int16_t enc_cnt_L=0,enc_cnt_pre_L=0,enc_cnt_R=0,enc_cnt_pre_R=0,pulse,rate_L,set_rate_L,rate_R,set_rate_R;
float Kp_L = 1.5, Kp_R = 1.4,Ki_L = 0.1 ,Ki_R = 0.1,Kd_L = 0.01,Kd_R = 0.01,T_sampling = 0.01;
int x_coord, y_coord;

float c1 = 0.3, c2 = 0.8, c3 = 0.6, c4 = 0.95, c5 = 0.4, c6 = 0.8, K1 = 600, K2 = 600,Ku_v = 200, Ku_omega = 0.1;
float e_x, e_x_norm, e_y, e_y_norm, u, u_dot, u_norm, beta_v[5],beta_omega[5];
float x1_NB, x1_NS, x1_ZE, x1_PS, x1_PB, x2_NB, x2_NS, x2_ZE, x2_PS, x2_PB;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
float HamLienThuocHinhThang(float x_ht, float L_ht, float C1_ht, float C2_ht, float R_ht);
float HamLienThuocHinhTamGiac(float x_htg, float L_htg, float C_htg, float R_htg);
float GiaiMo(float *beta,float y_NB, float y_NS, float y_PS, float y_PB);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3); 
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	__HAL_TIM_SetCounter(&htim2,0x8000);
	__HAL_TIM_SetCounter(&htim3,0x8000);
	HAL_UART_Receive_DMA(&huart2,Rx_buffer,11);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		cnt++;
		HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
	//Cau hinh lai prescaler 
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 359;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==htim4.Instance)//Interrupt Timer4 10ms
	{		 //PID	
			int e_L,e_R,u_L,u_R;
			float pid_L,pid_R;
			static float pid_pre_L=0,pid_pre_R=0;
			static int e_pre_pre_L=0,e_pre_L=0,e_pre_pre_R=0,e_pre_R=0;
			set_rate_L = 30;
			set_rate_R = 100;
			//PID Left Motor
			enc_cnt_pre_L = enc_cnt_L;
			enc_cnt_L = __HAL_TIM_GetCounter(&htim2);
			pulse = enc_cnt_L - enc_cnt_pre_L;
			if (pulse > 32767) pulse = pulse - 65536;
			if (pulse < -32768) pulse = pulse + 65536;
			rate_L = pulse*1500/330;
			e_L = set_rate_L - rate_L;
			pid_L = pid_pre_L + Kp_L*(e_L-e_pre_L)+Ki_L*T_sampling*(e_L+e_pre_L)/2.0+Kd_L*(e_L-2*e_pre_pre_L+e_pre_L)/T_sampling;
			u_L = pid_L;
			e_pre_pre_L = e_pre_L;
			e_pre_L = e_L;
			pid_pre_L = pid_L;
			//PID Right Motor
			enc_cnt_pre_R = enc_cnt_R;
			enc_cnt_R = __HAL_TIM_GetCounter(&htim3);
			pulse = enc_cnt_R - enc_cnt_pre_R;
			if (pulse > 32767) pulse = pulse - 65536;
			if (pulse < -32768) pulse = pulse + 65536;
			rate_R = pulse*1500/330;
			e_R = set_rate_R - rate_R;
			pid_R = pid_pre_R + Kp_R*(e_R-e_pre_R)+Ki_R*T_sampling*(e_R+e_pre_R)/2.0+Kd_R*(e_R-2*e_pre_pre_R+e_pre_R)/T_sampling;
			u_R = pid_R;
			e_pre_pre_R = e_pre_R;
			e_pre_R = e_R;
			pid_pre_R = pid_R;
			//PWM Output
			if (u_L > 499) u_L = 499;
			else if (u_L < -499) u_L = -499;
			if (u_R > 499) u_R = 499;
			else if (u_R < -499) u_R = -499;
			if (u_L > 0)
			{
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,u_L);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
			}
			else
			{
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,-u_L);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
			}
			if (u_R > 0)
			{
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,u_R);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
			}
			else
			{
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,-u_R);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
			}
			timer_cnt++;
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		HAL_UART_Receive_DMA(&huart2,Rx_buffer,11);
		if ((Rx_buffer[0]!=0xFF)||(Rx_buffer[1]!=0x02)||(Rx_buffer[10]!=0xAA))
		{}
		else
		{
			x_coord = (Rx_buffer[2]-48)*1000+(Rx_buffer[3]-48)*100+(Rx_buffer[4]-48)*10+(Rx_buffer[5]-48);
			y_coord = (Rx_buffer[6]-48)*1000+(Rx_buffer[7]-48)*100+(Rx_buffer[8]-48)*10+(Rx_buffer[9]-48);
			//Fuzzy controller calculating v and omega
			static int e_pre = 0;
			static float set_v_norm = 0,set_v_dot_pre = 0;
			float set_v,set_v_dot, set_omega;
			e_x = 600-x_coord;
			e_y = 600-y_coord;
			//Chuan hoa
		e_x_norm = e_x/K1;
		e_y_norm = e_y/K2;
		if (e_x_norm > 1) e_x_norm = 1;
		else if (e_x_norm < -1) e_x_norm = -1;
		if (e_y_norm > 1) e_y_norm = 1;
		else if (e_y_norm < -1) e_y_norm = -1;
		//e_x_norm = 0.5;
		//e_y_norm = 0;	
		//Mo hoa
		x1_NB = HamLienThuocHinhThang(e_x_norm,-3,-2,-c2,-c1);
		x1_NS = HamLienThuocHinhTamGiac(e_x_norm,-c2,-c1,0);
		x1_ZE = HamLienThuocHinhTamGiac(e_x_norm,-c1,0,c1);
		x1_PS = HamLienThuocHinhTamGiac(e_x_norm,0,c1,c2);
		x1_PB = HamLienThuocHinhThang(e_x_norm,c1,c2,2,3);
	
		x2_NB = HamLienThuocHinhThang(e_y_norm,-3,-2,-c4,-c3);
		x2_NS = HamLienThuocHinhTamGiac(e_y_norm,-c4,-c3,0);
		x2_ZE = HamLienThuocHinhTamGiac(e_y_norm,-c3,0,c3);
		x2_PS = HamLienThuocHinhTamGiac(e_y_norm,0,c3,c4);
		x2_PB = HamLienThuocHinhThang(e_y_norm,c3,c4,2,3);
		
		beta_omega[0] = x1_NB;
		beta_omega[1] = x1_NS;
		beta_omega[2] = x1_ZE;
		beta_omega[3] = x1_PS;
		beta_omega[4] = x1_PB;
		
		beta_v[0] = x2_NB;
		beta_v[1] = x2_NS;
		beta_v[2] = x2_ZE;
		beta_v[3] = x2_PS;
		beta_v[4] = x2_PB;
		
		//Luat dieu khien
		set_v_dot = GiaiMo(beta_v,-1,-0.4,0.2,0.5);
		set_omega = GiaiMo(beta_omega,-1,-0.4,0.4,1);
		set_v_norm = set_v_norm+(set_v_dot+set_v_dot_pre)/2*T_sampling;
		if (set_v_norm < 0.0) set_v_norm = 0.0;
		else if(set_v_norm > 2.0) set_v_norm = 2.0;
		set_v = set_v_norm * Ku_v;
		set_omega = Ku_omega * set_omega;
		set_rate_L = (set_v-set_omega*R)/r * 60/(2*pi);
		set_rate_R = (set_v+set_omega*R)/r * 60/(2*pi); 
		}
		}
}

float Max_Value(float *array, uint8_t a, uint8_t b)
{
	float max;
	max = array[a];
	for (int i = a; i < b;i++)
	{
		if (array[i] < array[i+1])
		{
			max = array[i+1];
		}
	}
	return max;
}

float HamLienThuocHinhThang(float x_ht, float L_ht, float C1_ht, float C2_ht, float R_ht)
{
	float y;
	if (x_ht <= L_ht) y = 0;
	else if(x_ht < C1_ht) y = (x_ht-L_ht)/(C1_ht - L_ht);
	else if(x_ht <= C2_ht) y = 1;
	else if(x_ht < R_ht) y = (R_ht-x_ht)/(R_ht-C2_ht);
	else y = 0;
	return y;
}

float HamLienThuocHinhTamGiac(float x_htg, float L_htg, float C_htg, float R_htg)
{
	float y;
	if (x_htg <= L_htg) y = 0;
	else if(x_htg < C_htg) y = (x_htg-L_htg)/(C_htg - L_htg);
	else if(x_htg < R_htg) y = (R_htg-x_htg)/(R_htg-C_htg);
	else y = 0;
	return y;
}


float GiaiMo(float *beta,float y_NB, float y_NS, float y_PS, float y_PB)
{
	float y;
	float Num, Den;
	Num = beta[0]*y_NB+beta[1]*y_NS+beta[2]*0+beta[3]*y_PS+beta[4]*y_PB;
	Den = beta[0]+beta[1]+beta[2]+beta[3]+beta[4];
	y = Num/Den;
	return y;
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
