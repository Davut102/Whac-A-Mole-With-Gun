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
#include "stdlib.h"
#include <stdio.h> // sprintf komutunu kullanabilmek için
#include "LCD.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t leds[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_PIN GPIO_PIN_11
#define BUTTON_PIN_2 GPIO_PIN_10
#define TIM2_MAX_PERIOD 63999
#define POTENTIOMETER_MIN_VALUE 0
#define POTENTIOMETER_MAX_VALUE 4095
#define UART_BUFFER_SIZE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
static uint16_t previous_adc_value = 0xFFFF;
static uint16_t current_prescaler = 0xFFFF;
static uint32_t current_period = 0xFFFFFFFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void temp_conv(uint16_t temp_var);
void print_char(uint32_t num_var);
char *Game_Over = {"   Game over!!!\r\n"};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char colors[4][16] = {
    "  Red   ",   // 0. eleman
    "  Purple",     // 1. eleman
    "  Green ",      // 2. eleman
    "  Blue  ",      // 3. eleman
    
};


bool is_array_Not_Empty(uint8_t array[], size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (array[i] != 0) {
            return true;
        }
    }
    return false;
}
	
	uint16_t ADC_val; 
	uint8_t USER1[10];
	uint8_t USER2[10];
	char transmitBuffer[50];
	int colorNumber_keeper;
	int ledNumber_keeper;
	int score;
	int score2;
	int flag;
	char score_str[20];
	char score_str2[20];
	int game_time=0;
	int temp_game_time=0;
	char *WinnerText = {" Plyr1 won:\r\n"};
	char *WinnerText2 = {" Plyr2 won:\r\n"};
	char *draw = {" Draw\r\n"};
	char *Equal = {"Scores are equal!(%d)\r\n"};
	
	int winnerScore;
	char winner[40];
	uint8_t *Welcome_msg = {"Write name of player1.\r\n"};
	uint8_t *new_line = {"\r\n"};
	uint8_t *Welcome_msg2 = {"Write name of player2.\r\n"};
	uint8_t *No_Player={"Player names cannot be null!.\r\n"};
	uint8_t data_index = 0;

		 

	float voltage;
	float temperature;
	float temperature2;
	char yazi[32] = " ";
	
	int strike = 0;
	int strike2 = 0;
	


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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
		while(!is_array_Not_Empty(USER1, sizeof(USER1))){
			
		HAL_UART_Transmit(&huart1, No_Player, 30, 2000);
		HAL_UART_Transmit(&huart1, Welcome_msg, 25, 5000);
		HAL_UART_Receive(&huart1, (uint8_t*)USER1, 25, 5000);
		HAL_UART_Transmit(&huart1, new_line, 2, 1);
			
	}
		
	while(!is_array_Not_Empty(USER2, sizeof(USER2))){
		HAL_UART_Transmit(&huart1, No_Player, 30, 2000);
		HAL_UART_Transmit(&huart1, Welcome_msg2, 25, 5000);
		HAL_UART_Receive(&huart1, (uint8_t*)USER2, 35, 5000);
		HAL_UART_Transmit(&huart1, new_line, 2, 1);
		
	}
	
	HAL_TIM_Base_Start_IT(&htim2);
	MX_ADC1_Init();
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	lcd_init(_LCD_4BIT, _LCD_FONT_5x10, _LCD_2LINE);



		
	uint8_t data[16];
	
	
	//FOR BUZZER
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_ADCEx_Calibration_Start(&hadc2);


	

	
	//FOR UART

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		

		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 21;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim2.Init.Prescaler = 124;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin|LCD_D4_Pin|LCD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D6_Pin|LCD_D7_Pin|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_EN_Pin LCD_D4_Pin LCD_D5_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_D4_Pin|LCD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D6_Pin LCD_D7_Pin PB12 PB13
                           PB14 PB15 LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_D6_Pin|LCD_D7_Pin|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int previousLed = -1;
int user_taken=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		
	
    if (htim->Instance == TIM2)
    {
			
				flag =0;
        if (previousLed != -1) {
            HAL_GPIO_WritePin(GPIOB, leds[previousLed], GPIO_PIN_RESET);
				
        }
        
        int random = rand() % 4; 
				int random2 = rand() % 4;
				
			
        HAL_GPIO_WritePin(GPIOB, leds[random], GPIO_PIN_SET); 
				colorNumber_keeper = random2;
				ledNumber_keeper = random;
				
			
				sprintf(score_str, " %s ", colors[random2]);
				lcd_print(1, 1, score_str);
				sprintf(score_str, " %d ", score);
				lcd_print(2, 1, score_str);
				sprintf(score_str2, " %d ", score2);
				lcd_print(2, 6, score_str2);
				
				
        previousLed = random; 
				
				game_time += temp_game_time;
				if(game_time==60000||(temperature==50.0 && temperature2==50.0)){
					HAL_TIM_Base_Stop_IT(&htim2); // Stop Timer 2 interrupt
					HAL_ADC_Stop_IT(&hadc1);      // Stop ADC1 interrupt
					HAL_TIM_Base_Stop_IT(&htim1);
					HAL_GPIO_WritePin(GPIOB, leds[0], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, leds[1], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, leds[2], GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, leds[3], GPIO_PIN_RESET);
					lcd_cmd(_CLEAR);
					lcd_print(1,1, Game_Over);
					
			
					
					if(score>score2){
						winnerScore=score;
						sprintf(transmitBuffer, "Winner is %s and the score is: %d\r\n", USER1,score);
						HAL_UART_Transmit_DMA(&huart1, (uint8_t*)transmitBuffer, strlen(transmitBuffer));
	
						
				
						
					}else if(score<score2){
						winnerScore=score2;
						sprintf(transmitBuffer, "Winner is %s and the score is: %d\r\n", USER2,score2);
						HAL_UART_Transmit_DMA(&huart1, (uint8_t*)transmitBuffer, strlen(transmitBuffer));
						
						
					
						
					}else if (score==score2){
						
						winnerScore=score;
						sprintf(transmitBuffer, "Draw! Scores: %d\r\n",winnerScore);
						HAL_UART_Transmit_DMA(&huart1, (uint8_t*)transmitBuffer, strlen(transmitBuffer));

						
					}
	
					
				}
				
				
    }
		
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 
    if (GPIO_Pin == BUTTON_PIN && temperature<50.0) 
    {	
			  

			  if(ledNumber_keeper==colorNumber_keeper && flag==0){
				score++;	
			  strike++;
				flag=1;
					if(strike==3){
						temperature-=10;
						strike =0;
						snprintf(yazi, sizeof(yazi), "Gun_1 temp decreased:%.2fC\r\n", temperature);
						HAL_UART_Transmit_DMA(&huart1, (uint8_t*)yazi, strlen(yazi));
					}
					
				}else if(!(ledNumber_keeper==colorNumber_keeper) && flag==0){
					score--;
					strike=0;
					temperature+=10;
					flag=1;
					snprintf(yazi, sizeof(yazi), "Gun_1 temp increased:%.2fC\r\n", temperature);
					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)yazi, strlen(yazi));

				}
    } 
		if (GPIO_Pin == BUTTON_PIN_2 && temperature2<50.0) 
    {	
				

			  if(ledNumber_keeper==colorNumber_keeper && flag==0){
				score2++;	
				flag=1;
				strike2++;
					if(strike2==3){
						temperature2-=10;
						strike2 =0;
						snprintf(yazi, sizeof(yazi), "Gun_2 temp decreased:%.2fC\r\n", temperature2);
						HAL_UART_Transmit_DMA(&huart1, (uint8_t*)yazi, strlen(yazi));
					}
				}else if(!(ledNumber_keeper==colorNumber_keeper)&& flag==0){
					score2--;
					strike2=0;
					temperature2+=10;
					flag=1;
					snprintf(yazi, sizeof(yazi), "Gun_2 temp increased:%.2fC\r\n", temperature2);
					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)yazi, strlen(yazi));
				}
    }
		
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	
	

    uint16_t adc_value = 0;

    adc_value = HAL_ADC_GetValue(hadc);

    uint16_t new_prescaler;
    uint32_t new_period;

    // ADC degerine göre yeni prescaler ve periyot degerlerini belirleyin
    if (adc_value < 0x555) { // ADC degeri 1.1 V'nin altindaysa
        new_prescaler = 159;
        new_period = 62499;
				game_time=1250;
				temp_game_time=1250;
    } else if (adc_value < 0x6C2) { // ADC degeri 2.2 V'nin altindaysa
        new_prescaler = 191;
        new_period = 62499;
			  game_time=1500;
			temp_game_time=1500;
    } else { // Diger durumlar
        new_prescaler = 249;
        new_period = 63999;
				game_time=2000;
			temp_game_time=2000;
    }

    // Eger yeni ayarlar mevcut ayarlardan farkliysa, timer'i yeniden yapilandirin
    if (new_prescaler != current_prescaler || new_period != current_period) {
        htim2.Init.Prescaler = new_prescaler;
        htim2.Init.Period = new_period;

        // Timer'i durdurun
        HAL_TIM_Base_Stop(&htim2);

        // Timer'i yeniden yapilandirin
        if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
            // Hata durumunda buraya ek kod ekleyin
        }

        // Timer'i tekrar baslatin
        if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
            // Hata durumunda buraya ek kod ekleyin
        }

        // Güncellenen prescaler ve periyot degerlerini kaydedin
        current_prescaler = new_prescaler;
        current_period = new_period;
    }

    // Önceki ADC degerini güncelleyin
    previous_adc_value = adc_value;
		
		if (hadc->Instance == ADC1) {
        voltage = (HAL_ADC_GetValue(hadc) * 3.3) / 4096.0;  // ADC degerini volta ?evir
        temperature = voltage * 100.0; // Voltaji sicakliga ?evir
				temperature2=temperature;
        snprintf(yazi, sizeof(yazi), "Gun temperature is: %.2f C\n\r", temperature);
        
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*)yazi, strlen(yazi));
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
