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
#define SW2_Pin GPIO_PIN_13
#define SW2_GPIO_Port GPIOC
#define EX_tap1_sig_Pin GPIO_PIN_1
#define EX_tap1_sig_GPIO_Port GPIOF
#define EX_tap2_sig_Pin GPIO_PIN_10
#define EX_tap2_sig_GPIO_Port GPIOG
#define Digital_in_Pin GPIO_PIN_0
#define Digital_in_GPIO_Port GPIOC
#define R_ZX_Pin GPIO_PIN_3
#define R_ZX_GPIO_Port GPIOC
#define R_LEAK_Pin GPIO_PIN_0
#define R_LEAK_GPIO_Port GPIOA
#define Y_LEAK_Pin GPIO_PIN_1
#define Y_LEAK_GPIO_Port GPIOA
#define Y_ZX_Pin GPIO_PIN_2
#define Y_ZX_GPIO_Port GPIOA
#define B_ZX_Pin GPIO_PIN_3
#define B_ZX_GPIO_Port GPIOA
#define B_LEAK_Pin GPIO_PIN_4
#define B_LEAK_GPIO_Port GPIOA
#define R_sel_Pin GPIO_PIN_4
#define R_sel_GPIO_Port GPIOC
#define B_sel_Pin GPIO_PIN_5
#define B_sel_GPIO_Port GPIOC
#define UPS_sel_Pin GPIO_PIN_0
#define UPS_sel_GPIO_Port GPIOB
#define USP_LEAK_Pin GPIO_PIN_1
#define USP_LEAK_GPIO_Port GPIOB
#define UPS_ZX_Pin GPIO_PIN_2
#define UPS_ZX_GPIO_Port GPIOB
#define ESP_TX_Pin GPIO_PIN_10
#define ESP_TX_GPIO_Port GPIOB
#define ESP_RX_Pin GPIO_PIN_11
#define ESP_RX_GPIO_Port GPIOB
#define CF1_Pin GPIO_PIN_12
#define CF1_GPIO_Port GPIOB
#define CF1_EXTI_IRQn EXTI15_10_IRQn
#define CF2_Pin GPIO_PIN_13
#define CF2_GPIO_Port GPIOB
#define CF2_EXTI_IRQn EXTI15_10_IRQn
#define CF3_Pin GPIO_PIN_14
#define CF3_GPIO_Port GPIOB
#define CF3_EXTI_IRQn EXTI15_10_IRQn
#define CF4_Pin GPIO_PIN_15
#define CF4_GPIO_Port GPIOB
#define CF4_EXTI_IRQn EXTI15_10_IRQn
#define CF5_Pin GPIO_PIN_6
#define CF5_GPIO_Port GPIOC
#define CF5_EXTI_IRQn EXTI9_5_IRQn
#define CF6_Pin GPIO_PIN_7
#define CF6_GPIO_Port GPIOC
#define CF6_EXTI_IRQn EXTI9_5_IRQn
#define CF7_Pin GPIO_PIN_8
#define CF7_GPIO_Port GPIOC
#define CF7_EXTI_IRQn EXTI9_5_IRQn
#define Y_sel_Pin GPIO_PIN_9
#define Y_sel_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define UPS_AUX_sig_Pin GPIO_PIN_15
#define UPS_AUX_sig_GPIO_Port GPIOA
#define UPS_main_sig_Pin GPIO_PIN_10
#define UPS_main_sig_GPIO_Port GPIOC
#define R_main_sig_Pin GPIO_PIN_11
#define R_main_sig_GPIO_Port GPIOC
#define R_AUX1_sig_Pin GPIO_PIN_12
#define R_AUX1_sig_GPIO_Port GPIOC
#define R_AUX2_sig_Pin GPIO_PIN_2
#define R_AUX2_sig_GPIO_Port GPIOD
#define Y_main_sig_Pin GPIO_PIN_3
#define Y_main_sig_GPIO_Port GPIOB
#define Y_AUX1_sig_Pin GPIO_PIN_4
#define Y_AUX1_sig_GPIO_Port GPIOB
#define Y_AUX2_sig_Pin GPIO_PIN_5
#define Y_AUX2_sig_GPIO_Port GPIOB
#define B_main_sig_Pin GPIO_PIN_6
#define B_main_sig_GPIO_Port GPIOB
#define B_AUX1_sig_Pin GPIO_PIN_7
#define B_AUX1_sig_GPIO_Port GPIOB
#define B_AUX2_sig_Pin GPIO_PIN_9
#define B_AUX2_sig_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
