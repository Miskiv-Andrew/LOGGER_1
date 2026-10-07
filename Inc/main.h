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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/*
* Включает передачу отладочных сообщений
* о переходах между состояниями по RadioLine.
*
* Для релизной версии закомментировать.
*/
#define DEBUG_MODE
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BDBG_TX_Pin GPIO_PIN_0
#define BDBG_TX_GPIO_Port GPIOA
#define BDBG_RX_Pin GPIO_PIN_1
#define BDBG_RX_GPIO_Port GPIOA
#define RADIO_TX_Pin GPIO_PIN_2
#define RADIO_TX_GPIO_Port GPIOA
#define RADIO_RX_Pin GPIO_PIN_3
#define RADIO_RX_GPIO_Port GPIOA
#define SD_CARD_CS_Pin GPIO_PIN_4
#define SD_CARD_CS_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_4
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_5
#define GPS_RX_GPIO_Port GPIOC
#define LRX_LiDAR_Pin GPIO_PIN_10
#define LRX_LiDAR_GPIO_Port GPIOB
#define LTX_LiDAR_Pin GPIO_PIN_11
#define LTX_LiDAR_GPIO_Port GPIOB
#define RE485_Pin GPIO_PIN_6
#define RE485_GPIO_Port GPIOC
#define DE485_Pin GPIO_PIN_7
#define DE485_GPIO_Port GPIOC
#define GPS_RESET_Pin GPIO_PIN_7
#define GPS_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
