/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>     /* atoi */
#include <string.h>
#include "dwt_stm32_delay.h"
#include "pzem004t.h"
#include "DHT11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_RANGE 24

#define ADDR_A (uint8_t)0x01
#define ADDR_B (uint8_t)0xC1 
#define ADDR_C (uint8_t)0xA1

#define SIZE_VOLTAGE_PZEM 3
#define SIZE_CURRENT_PZEM 2
#define SIZE_POWER_PZEM 2
#define SIZE_ENERGY_PZEM 3

#define DHT_AVAILABLE 0

#define RAINY (uint8_t)0xA6
#define NO_RAIN (uint8_t)0xC0

#define UPDATE_REQUEST (uint8_t)0x96

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char SonicData[6];
uint8_t PzemData[7];
uint8_t LoraData[5];
uint32_t RainData;
volatile uint8_t isrCounter;
volatile uint8_t isrPzem = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
  isrCounter = 1; 
  HAL_UART_Receive_IT(&huart1, (uint8_t*) LoraData , sizeof(LoraData));

  }
  if(huart->Instance == USART2)
  {
  HAL_UART_Receive_IT(&huart2, (uint8_t*) SonicData , sizeof(SonicData));
  }
  if(huart->Instance == USART3)
  {
  isrPzem = 0;
  HAL_UART_Receive_IT(&huart3, (uint8_t*) PzemData , sizeof(PzemData));
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == hadc1.Instance)
	{
		RainData = HAL_ADC_GetValue(hadc);
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float RH, TEMP;
uint8_t buffer[5];
uint8_t* pzemDataCplt;
uint32_t temp = 0;
uint8_t dimData = 0;
/*
volatile int i = 0, zeroCrossing = 0, debug = 0;
volatile uint8_t dim = 100; // dim around: 5 - 128 :))) // 128 is turn off Lamp
*/
// Detect Zero crossing
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ 
  /*
	if(GPIO_Pin == GPIO_PIN_8)
	{
		// Peridically = 10ms
		zeroCrossing = 1;
		i = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET ); // Turn off dimmer
	}
  */
}

// Timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

/*	
	if(htim->Instance == htim2.Instance){  // Timer with frequence 75us
	 if(zeroCrossing == 1){
	   if(i >= dim){
	     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // Turn on dimmer
	     i = 0;
	     zeroCrossing = 0;
	   }
	   else i++;
	 }
	}
*/
}
uint32_t Distance(uint8_t* data1, uint8_t* data2, uint8_t* data3, uint8_t* data4)
{
  uint32_t result = 0;
    if(SonicData[0] == 'R')
    {
      *data1 = SonicData[1];
      *data2 = SonicData[2];
      *data3 = SonicData[3];
      *data4 = SonicData[4];
      for (uint8_t i = 1; i < 5; i++)
      {

        result = result*10 + (uint32_t)( SonicData[i] - '0' );
      }
    }
  return result;
}

// uint32_t AverageDistance()
// {
//  uint32_t result = 0;
// if(cou)
// }

uint8_t sumData(uint8_t* data, uint8_t size)
{
  uint16_t crc  = 0x0000;
  for(uint8_t i=0; i< (size - 1); i++)
      crc += *(data + i);
  return (uint8_t)(crc & 0xFF);
} 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t x = 0;
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t*) LoraData , sizeof(LoraData));
	HAL_UART_Receive_IT(&huart2, (uint8_t*) SonicData , sizeof(SonicData));
  HAL_UART_Receive_IT(&huart3, (uint8_t*) PzemData , sizeof(PzemData));
	HAL_ADC_Start_IT(&hadc1);
	DWT_Delay_Init ();
//	HAL_TIM_Base_Start_IT(&htim2); //HALOGEN BULD dimmer

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2); // LED dimmer
	htim2.Instance->CCR2 = 175;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (sumData(LoraData, 5) == LoraData[4] && isrCounter && LoraData[0] == ADDR_A && LoraData[1] == ADDR_B && LoraData[2] == ADDR_C )
		{
      isrCounter = 0;
      if (LoraData[3] == UPDATE_REQUEST)
      {          
        uint8_t loraDataSend[DATA_RANGE] = {0};
        loraDataSend[0] = ADDR_A;
        loraDataSend[1] = ADDR_B;
        loraDataSend[2] = ADDR_C;
            
        pzemDataCplt =  getPzemData(&huart3);
        for(uint8_t i = 3; i< 13; i++)
        {
          loraDataSend[i] = *(pzemDataCplt + (i - 3));
        }

        temp = Distance(&loraDataSend[13], &loraDataSend[14], &loraDataSend[15], &loraDataSend[16]);

        if (dht11_read_data(buffer) == DHT_AVAILABLE) 
          {
            loraDataSend[17] = buffer[0];
            loraDataSend[18] = buffer[1];
            loraDataSend[19] = buffer[2];
            loraDataSend[20] = buffer[3];
            RH = buffer[0] + buffer[1] / 10.0;
            TEMP = buffer[2] + buffer[3] / 10.0;
          }

        if (RainData < 1500)
        {
          loraDataSend[21] = RAINY;
        }
        else loraDataSend[21] = NO_RAIN;
        loraDataSend[22] = dimData;
        loraDataSend[23] = sumData(loraDataSend, DATA_RANGE);
        HAL_UART_Transmit(&huart1, loraDataSend, sizeof(loraDataSend), 1000);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }
			else 
        {
          x++;
          uint8_t loraFeedback[4] = {0};
          for (int i = 0; i < 3; ++i)
          {
            loraFeedback[i] = LoraData[i];
          }  
          loraFeedback[3] = LoraData[4];       
          HAL_UART_Transmit(&huart1, loraFeedback, sizeof(loraFeedback), 1000);
          dimData = LoraData[3];
          htim2.Instance->CCR2 = dimData*2;
        }   
    }	

		// uint8_t VOLTAGE[7] = {0xB0, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1A};
		// HAL_UART_Transmit(&huart3, VOLTAGE, sizeof(VOLTAGE), 1000);
    // HAL_Delay(500);
		// uint8_t CURRENT[7] = {0xB1, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1B};
		// HAL_UART_Transmit(&huart3, CURRENT, sizeof(CURRENT), 1000);
		// HAL_Delay(500);
		
		// uint8_t POWER[7] = {0xB2, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1C};
		// HAL_UART_Transmit(&huart3, POWER, sizeof(POWER), 1000);
		// HAL_Delay(5000);
		// uint8_t ENERGY[7] = {0xB3, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1D};
		// HAL_UART_Transmit(&huart3, ENERGY, sizeof(ENERGY), 1000);
		// HAL_Delay(500);		
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Dimmer_GPIO_Port, Dimmer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT_Pin */
  GPIO_InitStruct.Pin = DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Zero_Crossing_Pin */
  GPIO_InitStruct.Pin = Zero_Crossing_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Zero_Crossing_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Dimmer_Pin */
  GPIO_InitStruct.Pin = Dimmer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Dimmer_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
