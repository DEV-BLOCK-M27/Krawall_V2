/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define POTI1_Pin GPIO_PIN_0
#define POTI1_GPIO_Port GPIOA
#define POTI2_Pin GPIO_PIN_1
#define POTI2_GPIO_Port GPIOA
#define POTI3_Pin GPIO_PIN_2
#define POTI3_GPIO_Port GPIOA
#define POTI4_Pin GPIO_PIN_3
#define POTI4_GPIO_Port GPIOA
#define SP_Pin GPIO_PIN_5
#define SP_GPIO_Port GPIOA
#define LED_Page_Pin GPIO_PIN_6
#define LED_Page_GPIO_Port GPIOA
#define PUSH1_Pin GPIO_PIN_0
#define PUSH1_GPIO_Port GPIOB
#define PUSH2_Pin GPIO_PIN_1
#define PUSH2_GPIO_Port GPIOB
#define PUSH3_Pin GPIO_PIN_2
#define PUSH3_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI2_LATCH_Pin GPIO_PIN_13
#define SPI2_LATCH_GPIO_Port GPIOB
#define SPI2_SHDN_Pin GPIO_PIN_15
#define SPI2_SHDN_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOD
#define PUSH4_Pin GPIO_PIN_3
#define PUSH4_GPIO_Port GPIOB
#define LFO_WAVE_1_Pin GPIO_PIN_4
#define LFO_WAVE_1_GPIO_Port GPIOB
#define LFO_WAVE_2_Pin GPIO_PIN_5
#define LFO_WAVE_2_GPIO_Port GPIOB
#define LFO_RANGE_1_Pin GPIO_PIN_6
#define LFO_RANGE_1_GPIO_Port GPIOB
#define LFO_RANGE_2_Pin GPIO_PIN_7
#define LFO_RANGE_2_GPIO_Port GPIOB
#define ADSR_Pin GPIO_PIN_8
#define ADSR_GPIO_Port GPIOB
#define TRIGGER_Pin GPIO_PIN_9
#define TRIGGER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
