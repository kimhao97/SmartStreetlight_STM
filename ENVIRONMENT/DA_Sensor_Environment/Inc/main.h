/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ULTRASONIC_TX_Pin GPIO_PIN_2
#define ULTRASONIC_TX_GPIO_Port GPIOA
#define ULTRASONIC_RX_Pin GPIO_PIN_3
#define ULTRASONIC_RX_GPIO_Port GPIOA
#define PZEM_TX_Pin GPIO_PIN_10
#define PZEM_TX_GPIO_Port GPIOB
#define PZEM_RX_Pin GPIO_PIN_11
#define PZEM_RX_GPIO_Port GPIOB
#define DHT_Pin GPIO_PIN_8
#define DHT_GPIO_Port GPIOA
#define LORA_TX_Pin GPIO_PIN_9
#define LORA_TX_GPIO_Port GPIOA
#define LORA_RX_Pin GPIO_PIN_10
#define LORA_RX_GPIO_Port GPIOA
#define DIMER_Pin GPIO_PIN_3
#define DIMER_GPIO_Port GPIOB
#define Zero_Crossing_Pin GPIO_PIN_8
#define Zero_Crossing_GPIO_Port GPIOB
#define Zero_Crossing_EXTI_IRQn EXTI9_5_IRQn
#define Dimmer_Pin GPIO_PIN_9
#define Dimmer_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
