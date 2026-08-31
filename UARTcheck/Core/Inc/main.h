/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ZE_Yphase_Pin GPIO_PIN_1
#define ZE_Yphase_GPIO_Port GPIOC
#define ZE_Bphase_Pin GPIO_PIN_2
#define ZE_Bphase_GPIO_Port GPIOC
#define ZE_UPS_Pin GPIO_PIN_3
#define ZE_UPS_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_4
#define CS2_GPIO_Port GPIOC
#define CS3_Pin GPIO_PIN_5
#define CS3_GPIO_Port GPIOC
#define R_YauxRelay_Pin GPIO_PIN_7
#define R_YauxRelay_GPIO_Port GPIOC
#define B_RauxRelay_Pin GPIO_PIN_8
#define B_RauxRelay_GPIO_Port GPIOA
#define Y_BauxRelay_Pin GPIO_PIN_9
#define Y_BauxRelay_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_10
#define CS1_GPIO_Port GPIOA
#define B_YauxRelay_Pin GPIO_PIN_3
#define B_YauxRelay_GPIO_Port GPIOB
#define R_BauxRelay_Pin GPIO_PIN_4
#define R_BauxRelay_GPIO_Port GPIOB
#define Y_RauxRelay_Pin GPIO_PIN_5
#define Y_RauxRelay_GPIO_Port GPIOB
#define B_mainRelay_Pin GPIO_PIN_6
#define B_mainRelay_GPIO_Port GPIOB
#define R_mainRelay_Pin GPIO_PIN_8
#define R_mainRelay_GPIO_Port GPIOB
#define Y_mainRelay_Pin GPIO_PIN_9
#define Y_mainRelay_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
